#include <SPI.h>
#include <SD.h>
#include <WiFi.h>

extern "C" {
  #include "i8080.h"
}

// Replace with your network credentials
const char* ssid = "XXXXXXXXXX";
const char* password = "XXXXXXXXXXXXX";

//#define DEBUG
//#define DEBUGIO
//#define DISK_DEBUG

//Pin Connection Defines
//Pin connected to RClk (Pin 12) of 74HC595
#define LlatchPin 2
//Pin connected to SRCk (Pin 11) of 74HC595
#define LclockPin 3
////Pin connected to SER (Pin 14) of 1st 74HC595
#define LdataPin 1
//Pin connected to PL! (Pin 1) of 74HCT165
#define SlatchPin 6
//Pin connected to CP (Pin 2) of 74HCT165
#define SclockPin 5
//Pin connected to Q7 (Pin 9) of 1st 74HCT165
#define SdataPin 4

//Pin connected to enable SD Card
#define SDenablePin 10

//14 is Serial RX
#define Serial2RX 14
//15 is Serial TX
#define Serial2TX 15

//Pin 9 is AnalogOut
#define AnalogOut 7

//GPIO21 is Builtin LED
#define BUILTIN_LED 21

//8 is SDA
//9 is SCL
//11 is MOSI
//13 is MISO
//12 is SClk

// Switch Clock and Latch Delay
#define Cdelay 4
#define Ldelay 1

//MEMR--The memory bus will be used for memory read data.
//INP--The address bus contains the address of an input device
//M1--The CPU is processing the first machine cycle of an instruction.
//OUT--The address contains the address of an output device.
//HLTA--A HALT instruction has been executed and acknowledged.
//STACK--The address bus holds the Stack Pointer's pushdown stack address
//WO--Operation in the current machine cycle will be a WRITE memory or OUTPUT function
//.   otherwise it's a READ memory or INPUT
//INT--An interrupt request has been acknowledged.
//INTE--An interrupt has been enabled.
//PROT—The memory is protected.
//WAIT--The CPU is in a WAIT state.
//HLDA--A HOLD has been acknowledged.

/*Original 
enum State { 
  HLDA, WAIT, WO, STACK, MEMR, INP, M1, IOUT, HLTA, PROT,
  INTE, INT };
*/

//IMASI 8080
enum State { 
  INT, WO, STACK, HLTA, IOUT, M1, INP, MEMR, INTE, PROT,
  WAIT, HLDA, USER4, USER3, USER2, USER1 };

/*Original
enum Control { STOP, SINGLE_STEP, EXAMINE, DEPOSIT, 
     RUN, SINGLE_STEP_, EXAMINE_NEXT, DEPOSIT_NEXT, 
     AUX2_UP, AUX1_UP, PROTECT, RESET, AUX2_DOWN, 
     AUX1_DOWN, UNPROTECT, CLR };
*/
/*ALTAIR 8800
enum Control { STOP, RUN, SINGLE_STEP, SINGLE_STEP_, 
     EXAMINE, EXAMINE_NEXT, DEPOSIT, DEPOSIT_NEXT, 
     RESET, CLR, PROTECT, UNPROTECT, AUX1_UP, AUX1_DOWN,\
     AUX2_UP, AUX2_DOWN };
*/
//IMASI 8080
enum Control { EXAMINE, EXAMINE_NEXT, DEPOSIT, DEPOSIT_NEXT, 
    RESET, CLR, RUN, STOP, SINGLE_STEP, SINGLE_STEP_, 
    PROTECT, UNPROTECT, AUX1_UP, AUX1_DOWN, AUX2_UP, AUX2_DOWN };

struct {
  uint16_t address;
  uint8_t data;
  uint16_t state;
} bus;

struct {
  uint16_t address;
  uint16_t control,prev_control;
} switches;

enum UState { RDRF,TDRE,DCD,CTS,FERR,OVRN,PERR,IRQ };
       // Bit 0 RX Full
       // Bit 1 TX Full
       // Bit 2 DCD
       // Bit 3 CTS
       // Bit 4 Framing Error
       // Bit 5 Overrun
       // Bit 6 Parity Error
       // Bit 7 Interupt Request

enum UControl { CDS1,CDS2,WS1,WS2,WS3,TC1,TC2,RIE};
      //Bit 0 Counter Divide Select 1
      //Bit 1 Counter Divide Select 2
      //Bit 2 Word Select 1 
      //Bit 3 Word Select 2
      //Bit 4 Word Select 3
      //Bit 5 Transmit Control 1
      //Bit 6 Transmit Control 2
      //Bit 7 Recieve Interupt Enable  

struct {
  uint8_t ucontrol;
  uint8_t ustate;
  uint8_t rxdata;
  uint8_t txdata;
} uartx00,uartx04,uartx10,uartx12,uartx20,uartx22;

//Disk Subsystem based on: https://github.com/dankar/altair8800/
#define STATUS_ENWD			1
#define STATUS_MOVE_HEAD	2
#define STATUS_HEAD			4
#define STATUS_IE			32
#define STATUS_TRACK_0		64
#define STATUS_NRDA			128

#define CONTROL_STEP_IN		1
#define CONTROL_STEP_OUT	2
#define CONTROL_HEAD_LOAD	4
#define CONTROL_HEAD_UNLOAD 8
#define CONTROL_IE			16
#define CONTROL_ID			32
#define CONTROL_HCS			64
#define CONTROL_WE			128

#define SECTOR 137UL
#define TRACK (32UL*SECTOR)

typedef struct
{
	File fp;
	uint8_t track;
	uint8_t sector;
	uint8_t status;
	uint8_t write_status;
} disk_t;

typedef struct
{
	disk_t disk1;
	disk_t disk2;
	disk_t nodisk;
	disk_t *current;
} disks;

disks disk_drive;

//Echo for SIO 
//https://www.penguinstew.ca/Writings/60/Computers/Programming/Projects/Altair%208800/Serial%20Echo
//Size is 13
const byte EchoSIO[] PROGMEM = {
0xDB, 0x00, 0x0F, 0xDA, 0x00, 0x00, 0xDB, 0x01, 0xD3, 0x01, 0xC3, 0x00, 0x00
};

//Size is 64
const byte EchoSIOInt[] PROGMEM = {
0x31, 0x00, 0x01, 0x3E, 0x01, 0xD3, 0x00, 0xFB, 0x00, 0x00, 0x00, 0xC3, 0x08, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF5, 0xDB, 0x01, 0xD3, 0x01, 0xF1, 0xFB, 0xC9
};

//Size is 21
const byte Echo2SIO[] PROGMEM = {
0x3E, 0x03, 0xD3, 0x10, 0x3E, 0x15, 0xD3, 0x10, 0xDB, 0x10, 0x0F, 0xD2, 0x08, 0x00, 0xDB, 0x11, 
0xD3, 0x11, 0xC3, 0x08, 0x00
};

const byte Echo2SIOInt[] PROGMEM = {
0x31, 0x00, 0x01, 0x3E, 0x03, 0xD3, 0x10, 0x3E, 0x95, 0xD3, 0x10, 0xFB, 0x00, 0x00, 0x00, 0xC3, 
0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF5, 0xDB, 0x11, 0xD3, 0x11, 0xF1, 0xFB, 0xC9 
};

//Disk Boot ROM at FF00
const byte DBLROM[] PROGMEM = { 
0x21, 0x13, 0xFF, 0x11, 0x00, 0x2C, 0x0E, 0xEB, 0x7E, 0x12, 0x23, 0x13, 0x0D, 0xC2, 0x08, 0xFF,
0xC3, 0x00, 0x2C, 0xF3, 0xAF, 0xD3, 0x22, 0x2F, 0xD3, 0x23, 0x3E, 0x2C, 0xD3, 0x22, 0x3E, 0x03,
0xD3, 0x10, 0xDB, 0xFF, 0xE6, 0x10, 0x0F, 0x0F, 0xC6, 0x10, 0xD3, 0x10, 0x31, 0x79, 0x2D, 0xAF,
0xD3, 0x08, 0xDB, 0x08, 0xE6, 0x08, 0xC2, 0x1C, 0x2C, 0x3E, 0x04, 0xD3, 0x09, 0xC3, 0x38, 0x2C,
0xDB, 0x08, 0xE6, 0x02, 0xC2, 0x2D, 0x2C, 0x3E, 0x02, 0xD3, 0x09, 0xDB, 0x08, 0xE6, 0x40, 0xC2,
0x2D, 0x2C, 0x11, 0x00, 0x00, 0x06, 0x00, 0x3E, 0x10, 0xF5, 0xD5, 0xC5, 0xD5, 0x11, 0x86, 0x80,
0x21, 0xEB, 0x2C, 0xDB, 0x09, 0x1F, 0xDA, 0x50, 0x2C, 0xE6, 0x1F, 0xB8, 0xC2, 0x50, 0x2C, 0xDB,
0x08, 0xB7, 0xFA, 0x5C, 0x2C, 0xDB, 0x0A, 0x77, 0x23, 0x1D, 0xCA, 0x72, 0x2C, 0x1D, 0xDB, 0x0A,
0x77, 0x23, 0xC2, 0x5C, 0x2C, 0xE1, 0x11, 0xEE, 0x2C, 0x01, 0x80, 0x00, 0x1A, 0x77, 0xBE, 0xC2,
0xCB, 0x2C, 0x80, 0x47, 0x13, 0x23, 0x0D, 0xC2, 0x79, 0x2C, 0x1A, 0xFE, 0xFF, 0xC2, 0x90, 0x2C,
0x13, 0x1A, 0xB8, 0xC1, 0xEB, 0xC2, 0xC2, 0x2C, 0xF1, 0xF1, 0x2A, 0xEC, 0x2C, 0xCD, 0xE5, 0x2C,
0xD2, 0xBB, 0x2C, 0x04, 0x04, 0x78, 0xFE, 0x20, 0xDA, 0x44, 0x2C, 0x06, 0x01, 0xCA, 0x44, 0x2C,
0xDB, 0x08, 0xE6, 0x02, 0xC2, 0xAD, 0x2C, 0x3E, 0x01, 0xD3, 0x09, 0xC3, 0x42, 0x2C, 0x3E, 0x80,
0xD3, 0x08, 0xC3, 0x00, 0x00, 0xD1, 0xF1, 0x3D, 0xC2, 0x46, 0x2C, 0x3E, 0x43, 0x01, 0x3E, 0x4D,
0xFB, 0x32, 0x00, 0x00, 0x22, 0x01, 0x00, 0x47, 0x3E, 0x80, 0xD3, 0x08, 0x78, 0xD3, 0x01, 0xD3,
0x11, 0xD3, 0x05, 0xD3, 0x23, 0xC3, 0xDA, 0x2C, 0x7A, 0xBC, 0xC0, 0x7B, 0xBD, 0xC9, 0x00, 0x00
};

//Disk Boot ROM at FD00
const byte UBMON[] PROGMEM = { 
0x3E, 0x03, 0xD3, 0x10, 0x3E, 0x11, 0xD3, 0x10, 0x31, 0x00, 0xFC, 0x01, 0x08, 0xFD, 0xC5, 0xCD,
0xA1, 0xFD, 0x3E, 0x2E, 0xCD, 0xF4, 0xFD, 0x21, 0x00, 0xFC, 0xCD, 0xE8, 0xFD, 0xD6, 0x4A, 0xCA,
0x81, 0xFD, 0x80, 0xCC, 0xAC, 0xFD, 0xCA, 0x86, 0xFD, 0x0E, 0x7F, 0xC5, 0x3C, 0xC8, 0x26, 0xFF,
0x82, 0xC8, 0x25, 0xFE, 0x12, 0xC8, 0xC1, 0x84, 0xC0, 0x5D, 0xCD, 0xAC, 0xFD, 0xEB, 0xCD, 0xAC,
0xFD, 0x23, 0x47, 0xCD, 0x75, 0xFD, 0xAF, 0xCD, 0x75, 0xFD, 0x05, 0x7D, 0x93, 0x4F, 0x7C, 0x9A,
0xC2, 0x54, 0xFD, 0x41, 0xD5, 0x1E, 0x3C, 0x50, 0xCD, 0xA4, 0xFD, 0xD1, 0xCD, 0xA4, 0xFD, 0x83,
0x4F, 0x1A, 0xCD, 0xF4, 0xFD, 0x81, 0x13, 0x05, 0xC2, 0x60, 0xFD, 0xCD, 0xF4, 0xFD, 0x7B, 0x95,
0x7A, 0x9C, 0xDA, 0x4A, 0xFD, 0x06, 0x3C, 0xCD, 0xF4, 0xFD, 0x05, 0xC2, 0x77, 0xFD, 0xC9, 0x34,
0xC8, 0xCC, 0xAC, 0xFD, 0xE9, 0x23, 0xCD, 0xA1, 0xFD, 0xE5, 0xCD, 0xC8, 0xFD, 0xCD, 0xAD, 0xFD,
0x7D, 0x21, 0x85, 0xFD, 0xE3, 0xD0, 0x77, 0xBE, 0xC8, 0x3E, 0x3F, 0xCD, 0xF4, 0xFD, 0xC3, 0x08,
0xFD, 0x11, 0x0D, 0x0A, 0x7B, 0xCD, 0xF4, 0xFD, 0x7A, 0xC3, 0xF4, 0xFD, 0x06, 0x06, 0x03, 0x65,
0xCD, 0xE8, 0xFD, 0xC8, 0xD6, 0x30, 0xFE, 0x08, 0xD2, 0x99, 0xFD, 0x29, 0x29, 0x29, 0xB5, 0x6F,
0x05, 0xC2, 0xB0, 0xFD, 0x37, 0xC3, 0xE3, 0xFD, 0x4E, 0x06, 0x06, 0xAF, 0xCD, 0xD4, 0xFD, 0xAF,
0x06, 0x03, 0x29, 0x17, 0x29, 0x17, 0xC6, 0x30, 0xCD, 0xF4, 0xFD, 0xAF, 0x29, 0x17, 0x05, 0xC2,
0xD2, 0xFD, 0x61, 0x3E, 0x20, 0xC3, 0xF4, 0xFD, 0xDB, 0x10, 0x0F, 0xD2, 0xE8, 0xFD, 0xDB, 0x11,
0xE6, 0x7F, 0xFE, 0x20, 0xF5, 0xDB, 0x10, 0xE6, 0x02, 0xCA, 0xF5, 0xFD, 0xF1, 0xD3, 0x11, 0xC9
};

//Turnkey Monitor ROM at FD00
const byte TURNMON[] PROGMEM = { 
0x3E, 0x03, 0xD3, 0x10, 0x3E, 0x11, 0xD3, 0x10, 0x31, 0x00, 0xFC, 0xCD, 0x96, 0xFD, 0xCD, 0x96,
0xFD, 0x3E, 0x2E, 0xCD, 0xF2, 0xFD, 0xCD, 0xE8, 0xFD, 0xFE, 0x4D, 0xCA, 0x2C, 0xFD, 0xFE, 0x44,
0xCA, 0x54, 0xFD, 0xFE, 0x4A, 0xC2, 0x08, 0xFD, 0xCD, 0xA0, 0xFD, 0xE9, 0xCD, 0xA0, 0xFD, 0xC3,
0x33, 0xFD, 0x23, 0xCD, 0x96, 0xFD, 0x54, 0x5D, 0xCD, 0xC6, 0xFD, 0x1A, 0x67, 0xCD, 0xCC, 0xFD,
0xCD, 0xA5, 0xFD, 0xEB, 0xDA, 0x32, 0xFD, 0x77, 0xBE, 0xCA, 0x32, 0xFD, 0x3E, 0x3F, 0xCD, 0xF2,
0xFD, 0xC3, 0x08, 0xFD, 0xCD, 0xA0, 0xFD, 0xEB, 0xD4, 0xE3, 0xFD, 0xCD, 0xA0, 0xFD, 0xE5, 0x62,
0x6B, 0xCD, 0x96, 0xFD, 0xCD, 0xC6, 0xFD, 0xCD, 0xE3, 0xFD, 0x01, 0x10, 0x00, 0x1A, 0x67, 0xC5,
0x3E, 0x08, 0xB9, 0xC2, 0x7E, 0xFD, 0x3E, 0x2D, 0xCD, 0xF2, 0xFD, 0xCD, 0xE3, 0xFD, 0xCD, 0xCC,
0xFD, 0xC1, 0xE1, 0x7C, 0xBA, 0xC2, 0x8D, 0xFD, 0x7D, 0xBB, 0xCA, 0x08, 0xFD, 0xE5, 0x13, 0x0D,
0xC2, 0x6D, 0xFD, 0xC3, 0x5F, 0xFD, 0x3E, 0x0D, 0xCD, 0xF2, 0xFD, 0x3E, 0x0A, 0xC3, 0xF2, 0xFD,
0x06, 0x06, 0xC3, 0xA7, 0xFD, 0x06, 0x03, 0x21, 0x00, 0x00, 0xCD, 0xE8, 0xFD, 0x4F, 0xFE, 0x20,
0x37, 0xC8, 0xE6, 0xB8, 0xEE, 0x30, 0xC2, 0x4C, 0xFD, 0x79, 0xE6, 0x07, 0x29, 0x29, 0x29, 0x85,
0x6F, 0x05, 0xC2, 0xAA, 0xFD, 0xC9, 0x06, 0x06, 0xAF, 0xC3, 0xD6, 0xFD, 0x06, 0x03, 0xAF, 0xC3,
0xD3, 0xFD, 0x29, 0x17, 0x29, 0x17, 0x29, 0x17, 0xE6, 0x07, 0xF6, 0x30, 0xCD, 0xF2, 0xFD, 0x05,
0xC2, 0xD2, 0xFD, 0x3E, 0x20, 0xC3, 0xF2, 0xFD, 0xDB, 0x10, 0x0F, 0xD2, 0xE8, 0xFD, 0xDB, 0x11,
0xE6, 0x7F, 0xF5, 0x81, 0x4F, 0xDB, 0x10, 0x0F, 0x0F, 0xD2, 0xF5, 0xFD, 0xF1, 0xD3, 0x11, 0xC9
};

byte CPUMemory[65536];

void readPage(uint16_t tmp16addr) {
    Serial.print("0x");
    Serial.print(tmp16addr, HEX);
    Serial.print(": ");
    for (unsigned long i = tmp16addr; i < (tmp16addr + 0x200); i++) {
      if (CPUMemory[i] < 0x10) {
        Serial.print("0");
      }
      Serial.print(CPUMemory[i],HEX);
      if ((i & 31) != 31) {
        // Print 32 bytes per line, space between them, (31 spaces, fence post)
        Serial.print(" ");
      } else {
        // Last one of a line gets a newline printed after it instead.
        Serial.println();
        Serial.print("0x");
        Serial.print(i+1,HEX);
        Serial.print(": ");
      }
    }
    Serial.println();
}

void CPUStatus() {
  Serial.print("PC:");
  Serial.print(i8080_pc(),HEX);
  Serial.print(" BUS:");         
  Serial.print(bus.address,HEX);
  Serial.print("  SP:");
  Serial.print(i8080_regs_sp(),HEX);        
  Serial.print(" Bus Data:");
  Serial.print(bus.data,HEX);
  Serial.print(" Memory Data:");
  Serial.print(CPUMemory[i8080_pc()],HEX);
  Serial.print(" AF:");
  Serial.print(i8080_regs_af(),HEX);
  Serial.print(" BC:");
  Serial.print(i8080_regs_bc(),HEX);
  Serial.print("  DE:");
  Serial.print(i8080_regs_de(),HEX);
  Serial.print("  HL:");
  Serial.print(i8080_regs_hl(),HEX);
    Serial.println();
}

void writeLEDs() {
  // take the latchPin low so
  // the LEDs don't change while you're sending in bits:
  digitalWrite(LlatchPin, LOW);

  // Now push data to 74HC595
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.state >> 8);
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.address);
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.address >> 8);
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.state);
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.data);
  // take the latch pin high so the LEDs will light up:
  digitalWrite(LlatchPin, HIGH);
}

void readSwitches() {
   byte al = 0;
   byte ah = 0;
   byte cl = 0;
   byte ch = 0;
   boolean bitVal = LOW;
   
  //Write Pulse to Latch Pin
   digitalWrite(SlatchPin, LOW);
   delayMicroseconds(Ldelay);
   digitalWrite(SlatchPin, HIGH);
   delayMicroseconds(Ldelay);

  // Now get data from 74HC165
   for(int i = 0; i < 8; i++)
   {
        bitVal = digitalRead(SdataPin);
        al |= (bitVal << (7 - i));

        digitalWrite(SclockPin, HIGH);
        delayMicroseconds(Cdelay);
        digitalWrite(SclockPin, LOW);
    }
   for(int i = 0; i < 8; i++)
   {
        bitVal = digitalRead(SdataPin);
        ah |= (bitVal << (7 - i));

        digitalWrite(SclockPin, HIGH);
        delayMicroseconds(Cdelay);
        digitalWrite(SclockPin, LOW);
    }
   for(int i = 0; i < 8; i++)
   {
        bitVal = digitalRead(SdataPin);
        cl |= (bitVal << (7 - i));

        digitalWrite(SclockPin, HIGH);
        delayMicroseconds(Ldelay);
        digitalWrite(SclockPin, LOW);
    }

   for(int i = 0; i < 8; i++)
   {
        bitVal = digitalRead(SdataPin);
        ch |= (bitVal << (7 - i));

        digitalWrite(SclockPin, HIGH);
        delayMicroseconds(Ldelay);
        digitalWrite(SclockPin, LOW);
    }
  switches.prev_control = switches.control; ////remember previous value
  switches.address = (ah<<8) + al;
  switches.control = (ch<<8) + cl;
}

bool onRelease(int c) {
  return (switches.prev_control & (1<<c)) > 0 && (switches.control & (1<<c)) == 0;
}

bool isDown(int c) {
  return switches.control & (1<<c);
}

byte sense() {
  return switches.address >> 8;
}

extern "C" {
  //uint8_t Flash.writeWord(uint32_t address, uint16_t data)
  //uint8_t Flash.writeByte(uint32_t address, uint8_t data)
  //uint8_t Flash.erasePage(uint32_t address, uint8_t size = 1);

  unsigned char i8080_hal_memory_read_byte(short unsigned int tmp16addr) {    
     bitSet(bus.state,MEMR);
     bitClear(bus.state,WO);  
     return CPUMemory[tmp16addr];
     }

  void i8080_hal_memory_write_byte(unsigned short int tmp16addr, unsigned char vale) {
    bitSet(bus.state,MEMR);
    bitSet(bus.state,WO);
    CPUMemory[tmp16addr] = vale;
  } 

  unsigned short int i8080_hal_memory_read_word(unsigned short int tmp16addr) {
     #ifdef DEBUG
     Serial.print(" Read Word ");
     Serial.print(tmp16addr,HEX);
     Serial.print("=");
     Serial.print((CPUMemory[tmp16addr+1] << 8) + CPUMemory[tmp16addr],HEX);
     #endif
     bitSet(bus.state,MEMR);
     bitClear(bus.state,WO);
     return (CPUMemory[tmp16addr+1] << 8) + CPUMemory[tmp16addr];
  }

  void i8080_hal_memory_write_word(unsigned short int tmp16addr, unsigned short int vale) {
     #ifdef DEBUG
     Serial.print("Write Word ");       
     Serial.print(tmp16addr,HEX);
     Serial.print("=");
     Serial.print(vale,HEX);
     #endif
     bitSet(bus.state,MEMR);
     bitSet(bus.state,WO);     
     CPUMemory[tmp16addr+1] = vale >> 8;
     CPUMemory[tmp16addr] = vale;
     return;
  }
  
// Copyright (C) 2020 David Hansel
// Ports used by emulated devices (ports with "*" are available on Arduino MEGA)
// 00-01  * MITS 88-SIO
// 02     * printer (OkiData, C700 or generic)
// 03     * printer (OkiData, C700 or generic, output only)
// 04       Cromemco disk controller
// 04-05    ProTec VDM1 keyboard (input only)
// 06-07  * MITS 88-ACR
// 08-0A    MITS 88-DCDD disk controller
// 0E       Cromemco Dazzler
// 0F       Cromemco Dazzler (output only)
// 10-11  * MITS 88-2SIO port A
// 12-13  * MITS 88-2SIO port B
// 14-15  * second MITS 88-2SIO port A
// 16-17  * second MITS 88-2SIO port B
// 18       Cromemco D+7A board (Dazzler support, input only)
// 19-1C    Cromemco D+7A board (Dazzler support)
// 1D-1F    Cromemco D+7A board (Dazzler support, output only)
// 30-34    Cromemco disk controller
// 40       Cromemco disk controller (output only)
// A0-A8    MITS hard disk controller (4PIO interface board)
// C8       ProTec VDM-1 (output only)
// F0       Cromemco disk controller (input only)
// F8-FD    Tarbell disk controller
// FE     * MITS 88-VI interrupt controller (output only)
// FF     * front-panel sense switches (input only)

  // From i8080_hal
  unsigned char i8080_hal_io_input(unsigned char port) {

    #ifdef DEBUGIO
    Serial.print("In Port:");
    Serial.print(port,HEX);
    Serial.print(" Data:");
    Serial.println(i8080_regs_a(),HEX);
    #endif

    bitSet(bus.state,INP);
    bitClear(bus.state,IOUT);
    bitClear(bus.state,WO);
    bitClear(bus.state,MEMR);

    unsigned int seek;
	  unsigned char ret_val;
  
    switch (port) {
      case 0x00:     // CPU Board 88-SIO Status --> Serial1
       return uartx00.ustate;
      case 0x01:     // CPU Board 6850 Serial Port read/RX--> Serial1      
        //Reading a character, so set RDRF unless there is more to get
        if (Serial2.available()==0) {
          bitSet(uartx00.ustate,RDRF);
        }
        return uartx00.rxdata;
      case 0x04: //Protec VDM1 Keyboard??
        return uartx04.ustate;
      case 0x05: //Protect VDM1 Keyboard??
        return 0xFF;
      case 0x06: // 88-ACR Board Cassette
        Serial.print("Cassette Port In 0x06, Data:");
        Serial.print(i8080_regs_a(),HEX);
        Serial.println();
        return 0xFF;
      case 0x07: // 88-ACR Board Cassette
        Serial.print("Cassette Port In 0x07, Data:");
        Serial.print(i8080_regs_a(),HEX);
        Serial.println();
        return 0xFF;
      case 0x08: //88-DCDD Disk Status
        #ifdef DISK_DEBUG
	      Serial.print("Returning status ");
	      Serial.print(disk_drive.current->status);
	      Serial.println(" for disk");
        #endif
	      return disk_drive.current->status;
      case 0x09: //88-DCDD Disk Sector
      	if(disk_drive.current->sector == 32) disk_drive.current->sector = 0;
      	//current_sector = current_sector % 32;
	      seek = disk_drive.current->track * TRACK + disk_drive.current->sector * (SECTOR);
	      disk_drive.current->fp.seek(seek);
	      ret_val = disk_drive.current->sector << 1;
        #ifdef DISK_DEBUG  
        Serial.print("Current sector: ");
	      Serial.print(disk_drive.current->sector);
	      Serial.print(" (");
	      Serial.print(ret_val, HEX);
	      Serial.print(") (bytes per track: ");
	      Serial.print(TRACK);
	      Serial.println(")");
        #endif
	      disk_drive.current->sector++;
	      return ret_val;
      case 0x0A: //88-DCDD Disk Read
        ret_val = disk_drive.disk1.fp.read();
	      //ret_val = disk_drive.current->fp.read();
	      seek++;
        #ifdef DISK_DEBUG
        Serial.print("Reading byte ");
        Serial.print(seek);
        Serial.print(" (");
        Serial.print(ret_val,HEX);
        Serial.println(")");
        #endif
	      return ret_val;
      case 0x10: // 88-2SIO or 88-ACR port 0, Status --> Serial
       Serial2.print("Port 10 In:");
       Serial2.println(uartx10.ustate,HEX);
       return uartx10.ustate;
      case 0x11:     // CPU Board 6850 Serial Port read/RX--> Serial1      
        //Reading a character, so set RDRF unless there is more to get
        if (Serial.available()==0) {
          bitClear(uartx10.ustate,RDRF);
        }
        Serial2.print("Port 11 In:");
        Serial2.println(uartx10.rxdata,HEX);
        return uartx10.rxdata;
      case 0x12: // 88-2SIO or 88-ACR port 1, Status
        return uartx12.ustate;
      case 0x13: // 88-2SIO or 88-ACR port 1, read/RX
        bitClear(uartx12.ustate,RDRF);
        return uartx12.rxdata;
      case 0x18: // 88-ACR Board I/O 2ndary
        Serial.print("Cassette Port In 0x18, Data:");
        Serial.print(i8080_regs_a(),HEX);
        Serial.println();
        return 0xFF;
      case 0x19: // 88-ACR Board I/O 2ndary
        Serial.print("Cassette Port In 0x19, Data:");
        Serial.print(i8080_regs_a(),HEX);
        Serial.println();
        return 0xFF;
      case 0x20: // 88-2SIO or 88-ACR Alternate port 0, Status --> WiFi Serial
        return uartx20.ustate;
      case 0x21: // 88-2SIO or 88-ACR Alternate port 0, read/RX --> WiFi Serial
        return uartx20.rxdata;
      case 0x22: // 88-2SIO or 88-ACR Alternate port 1, Status --> WiFi Serial
        return uartx22.ustate;
      case 0x23: // 88-2SIO or 88-ACR Alternate port 1, read/RX --> WiFi Serial
        return uartx22.rxdata;
      case 0xfe: // Get realtime from time.gov
        Serial.print("88-VI/RTC In 0xFE, Data:");
        Serial.print(i8080_regs_a(),HEX);
        Serial.println();
        return 0xFF;
      case 0xff: // Front panel switches
        //bitSet(bus.state,USER0);
        //Serial.print("Address PI: ");
        readSwitches();
        //Serial.printHex(switches.address >> 8);
        //Serial.println();
        return (switches.address >> 8);
      default:
        #ifdef DEBUGIO
        Serial.print("In Port:");
        Serial.print(port,HEX);
        Serial.print(" Data:");
        CPUStatus();
        #endif
        return 0xFF;
    }
    //Should never get here....
    return 0xFF;
  }

  // From i8080_hal 
  void i8080_hal_io_output(unsigned char port, unsigned char vale) {
    //This is the Output handler
    bitClear(bus.state,INP);
    bitSet(bus.state,IOUT);
    bitSet(bus.state,WO);
    bitClear(bus.state,MEMR);

    #ifdef DEBUGIO
    Serial.print("Out Port:");
    Serial.print(port,HEX);
    Serial.print(" Data:");
    Serial.println(i8080_regs_a(),HEX);
    #endif

    //Put the data on the bus
    bus.data = vale;
    unsigned int seek;
	  unsigned char ret_val;

    switch (port) {
    case 0x00: // CPU Board 6850 Serial Port Status --> Serial
      //Handle what could be sent to the UART
      //Bit 0-4 Speed Select
      //Bit 5-6 TX Control
      //Bit 7 - RX Interrupt Enable
      //It's usually 0x43
      //Should check if Bit 7 is set 
      if (vale==3) {
        Serial2.println("88-SIO Reset");
      } else {
        Serial2.print("88-SIO: ");
        Serial2.println(vale,HEX);
      }
      return;
    case 0x01: // CPU Board 6850 Serial write/TX --> Serial
      Serial2.write(vale & 0x7F);
      return;
    case 0x06: // 88-ACR Board Cassette
      Serial.print("Cassette Port Out 0x06, Data:");
      Serial.print(i8080_regs_a(),HEX);
      Serial.println();
    case 0x07: // 88-ACR Board Cassette
      Serial.print("Cassette Port Out 0x07, Data:");
      Serial.print(i8080_regs_a(),HEX);
      Serial.println();
    case 0x08: //88-DCDD Disk Select
      ret_val = vale & 0xf;
	    if(ret_val == 0) {
	      disk_drive.current = &disk_drive.disk1;
	    } else if(ret_val == 1) {
		    disk_drive.current = &disk_drive.disk2;
	    } else {
		    disk_drive.current = &disk_drive.nodisk;
	    }
      return;
    case 0x09: //88-DCDD Disk Function
      #ifdef DISK_DEBUG
	    Serial.print("Disk function ");
	    Serial.println(vale);
      #endif
	    if(vale & CONTROL_STEP_IN) {
		    disk_drive.current->track++;
		    if(disk_drive.current->track != 0) {
          disk_drive.current->status |= STATUS_TRACK_0;
        }
		    disk_drive.current->fp.seek(TRACK * disk_drive.current->track);
        #ifdef DISK_DEBUG
		    Serial.print("Track seek to : ");
		    Serial.println(TRACK * disk_drive.current->track);
        #endif
    	}
	    if(vale & CONTROL_STEP_OUT) {
		    if(disk_drive.current->track > 0) disk_drive.current->track--;
		    if(disk_drive.current->track == 0) disk_drive.current->status &= ~STATUS_TRACK_0;
		    disk_drive.current->fp.seek(TRACK * disk_drive.current->track);
        #ifdef DISK_DEBUG
        Serial.print("Track seek to : ");
        Serial.println(TRACK * disk_drive.current->track);
        #endif
    	}
	    if(vale & CONTROL_HEAD_LOAD) {
    		disk_drive.current->status &= ~STATUS_HEAD;
		    disk_drive.current->status &= ~STATUS_NRDA;
    	}
	    if(vale & CONTROL_HEAD_UNLOAD) disk_drive.current->status |= STATUS_HEAD; 
	    if(vale & CONTROL_IE) { }
    	if(vale & CONTROL_ID) { }
    	if(vale & CONTROL_HCS) { }
    	if(vale & CONTROL_WE) {
    		disk_drive.current->status &= ~STATUS_ENWD;
		    disk_drive.current->write_status = 0;
	    }
      return;
    case 0x0A: //88-DCDD Disk Write
      #ifdef DISK_DEBUG
      Serial.print("Write ");
      Serial.print(vale);
      Serial.print(" (byte in sector: ");
      Serial.print(disk_drive.current->write_status);
      Serial.println(")");
      #endif
	    disk_drive.current->fp.write(&vale, 1);
	    if(disk_drive.current->write_status == 137) {
    		disk_drive.current->write_status = 0;
		    disk_drive.current->status |= STATUS_ENWD;
        #ifdef DISK_DEBUG
		    Serial.println("Disabling clear");
        #endif
	    }	else disk_drive.current->write_status++;
      Serial.println("Disk Write");
      return;
    case 0x10: // 88-2SIO or 88-ACR port 0, Control Serial1
      if (vale==3) {
        Serial2.println("88-2SIO Reset");
      } else {
        Serial2.print("88-2SIO: ");
        Serial2.println(vale,HEX);
      }
      return;
    case 0x11: // 88-2SIO or 88-ACR port 0, Write/TX Serial1
      Serial.write(vale & 0x7F);
      return;
    case 0x12: // 88-2SIO or 88-ACR port 1, Control
      return;
    case 0x13: // 88-2SIO or 88-ACR port 1, Write/TX
      return;
    case 0x20: // 88-2SIO or 88-ACR Alternate port 0, Control
      return;
    case 0x21: // 88-2SIO or 88-ACR Alternate port 0, Write/TX
      return;
    case 0x22: // 88-2SIO or 88-ACR Alternate port 1, Control
      return;
    case 0x23: // 88-2SIO or 88-ACR Alternate port 1, Write/TX
      return;
    case 0xFE:
      Serial.print("88-VI/RTC Out, Data:");
      Serial.print(i8080_regs_a(),HEX);
      Serial.println();
      return;
    case 0xFF: // panel LEDs
      bus.state = (bus.state & 0x0FFF) + (vale << 12);      
      return;
    default:
      #ifdef DEBUGIO 
      Serial.print("Port Out: ");
      Serial.print(port,HEX);
      Serial.print(" Data:");
      Serial.print(vale,HEX);
      Serial.println();
      #endif
      return;
    }
  }

  //From i8080_hal
  void i8080_hal_halt() {
    bitSet(bus.state,HLTA);
    bitClear(bus.state,WAIT);
    bitClear(bus.state,HLDA);
    return;
  }

  // From i8080_hal 
  //extern void i8080_hal_iff(int on);
  void i8080_hal_iff(unsigned char on) {
    //This routine processes the EI and DI instruction
    //Each peripheral needs to do the interupt in the main loop
    //When doing an interrtupt: Push the PC onto the stack SP -= 2; WR_WORD(SP, (reg));
    //xC7 RST(0) - 0x0000
    //xCF RST(1) - 0x0008
    //xD7 RST(2) - 0x0010
    //xDF RST(3) - 0x0018
    //xE7 RST(4) - 0x0020
    //xEF RST(5) - 0x0028
    //xF7 RST(6) - 0x0030
    //xFF RST(7) - 0x0038
    if (on) {
      //Enable interrupts
      bitSet(bus.state,INTE);
    } else{
      bitClear(bus.state,INTE);
    }
    //Interrupt Vectors
    return;
  }

  // From i8080_hal 
  extern unsigned char* i8080_hal_memory(void) {
    //Not really sure what this is for, set M1 just in case
    bitSet(bus.state,M1);
    return 0;
  }
}

void examine(uint16_t tmp16addr) {
     i8080_jump(tmp16addr); //set program counter
     bus.data = CPUMemory[tmp16addr];
     writeLEDs();
}

void deposit(uint16_t tmp16addr, uint8_t vale) {
     i8080_jump(tmp16addr); //set program counter
     CPUMemory[tmp16addr] = vale;
     bus.data = CPUMemory[tmp16addr];
     writeLEDs();
}

uint8_t hexcToInt(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  } else if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  } else if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  } else {
    return 0;
  }
}

int loadFile(const char filename[], int offset) {
  File file = SD.open(filename);
  if (!file) {
    Serial.println("ERR:Load");
    return -2; 
  }
  while (file.available()) {
    i8080_hal_memory_write_byte(offset++, file.read());
    //examine(offset);
    //writeLEDs();
  }
  file.close();
  return 0;
}

void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void setup() {
  Serial.begin();
  pinMode(LlatchPin, OUTPUT);
  pinMode(LclockPin, OUTPUT);
  pinMode(LdataPin, OUTPUT);
  pinMode(SlatchPin, OUTPUT);
  pinMode(SclockPin, OUTPUT);
  pinMode(SdataPin, INPUT);
  pinMode(SDenablePin,OUTPUT);

  //pinMode(BUILTIN_LED,OUTPUT);
  rgbLedWrite(BUILTIN_LED, 0, 64, 0); 

  // Connect to Wi-Fi network with SSID and password
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.print("\n\rIP Address: ");
  
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
  
  Serial2.begin(38400, SERIAL_8N1, Serial2RX, Serial2TX);
  #ifdef DEBUG
  Serial.println("Output on Serial");
  Serial2.println("Output on Serial2");
  #endif

  // Let's start with the SIO cards returning 0xFF if empty
  //88-SIO Bit 7 is low for Ready to Send
  uartx00.ustate = 0x0D; //RDRF-off TDRE-on, DCD-on, CTS-on, IRQ=off
  uartx04.ustate = 0xFF;
  uartx10.ustate = 0x8E; //RDRF-off TDRE-on, DCD-on, CTS-off, IRQ=off
  uartx12.ustate = 0xFF;
  uartx20.ustate = 0xFF;
  uartx22.ustate = 0xFF;

  if (!SD.begin(SDenablePin)) {
    Serial.println("SD-Card init failed!");
  } else {
    Serial.println("SD-Card init OK!");  
  }
  rgbLedWrite(BUILTIN_LED, 0, 0, 64); 

  //All Flags off
  bus.state = 0x00;
  bitSet(bus.state,WAIT);
  bitSet(bus.state,USER3);
  
  //Copy TURNMON and DBLROM ROMs to RAM
  Serial.println("Loading UBMON/DBLROM");
  for (int i=0; i < 256; i++) {
    //CPUMemory[i+0xFD00] = TURNMON[i];
    CPUMemory[i+0xFD00] = UBMON[i];
    CPUMemory[i+0xFF00] = DBLROM[i];
  }
  //Init clears flags and sets PC to $F800
  i8080_init();
  rgbLedWrite(BUILTIN_LED, 64, 0, 0);

}

void loop() {
  // put your main code here, to run repeatedly

  if (i8080_regs_sp()==bus.address) {
    bitSet(bus.state,STACK);
  } else bitClear(bus.state,STACK);

  //A Halt is signified via UN3_FLAG = 1;
  //if (i8080_regs_af && 0x08) {
  //  Serial.print("Halted: ");
  //  CPUStatus();
  //  bitSet(bus.state,HLTA);
  //  bitSet(bus.state,WAIT);
  //}

  bus.address = i8080_pc();
  bus.data = CPUMemory[i8080_pc()];
  
  //Look for Serial Port Data -- 88-SIO
  //Bit 0 is low that Data has been recieved
  //Check to see if there is already data, don't read again
  if (Serial2.available()) {
    uartx00.rxdata = Serial2.read();
    bitClear(uartx00.ustate,RDRF);
  }
  //Look for Serial Port Data -- 88-2SIO
  //Bit 0 is high that Data has been recieved
  //Check to see if there is already data, don't read
  //again if buffer is still full
  if (Serial.available() && !(bitRead(uartx10.ustate,RDRF)) ) {
    if bitRead(bus.state,INTE) {
      //When doing an interrtupt
      int clkcycles = i8080_interrupt(0xFF);
    }
    uartx10.rxdata = Serial.read();
    bitSet(uartx10.ustate,RDRF);
  }

  readSwitches();

  if (onRelease(EXAMINE)) {
      examine(switches.address);
      if (!bitRead(bus.state,USER4)) {
        Serial.print("Examine: ");
        Serial.print(i8080_pc(),HEX);
        Serial.print(":");
        Serial.print(CPUMemory[i8080_pc()],HEX);
        Serial.println();
      }
  }
  if (onRelease(EXAMINE_NEXT)) {
     examine(bus.address+1);
      if (!bitRead(bus.state,USER4)) {
        Serial.print("Examine Next: ");
        Serial.print(i8080_pc(),HEX); 
        Serial.print(":");
        Serial.print(CPUMemory[i8080_pc()],HEX);
        Serial.println();
      }
  } 
  if (onRelease(DEPOSIT)) {
     Serial.println("DEPOSIT");
     uint8_t tmpvale = 0x00FF & switches.address;
     deposit(i8080_pc(),tmpvale);
  }
  if (onRelease(DEPOSIT_NEXT)) {
     Serial.println("DEPOSIT_NEXT");  
     deposit(i8080_pc()+1,switches.address);
  }
  if (onRelease(RUN))  {
     Serial.println("RUN");
     bitClear(bus.state,WAIT);
  }
  if (onRelease(STOP)) {
     Serial.println("STOP");  
     bitSet(bus.state,WAIT);
  } 
  if (onRelease(RESET)) {
     Serial.println("RESET");  
     bus.address = 0;
     bus.data = 0;
     //Init address is F800
     i8080_init();
     examine(0);
  }
  //CLR is a CLEAR command for external input/output equipment.
  if (onRelease(CLR)) {
    Serial.println("CLR");
    bitClear(bus.state,PROT);
    bitClear(bus.state,INTE);
    bitClear(bus.state,HLDA);
    bitSet(bus.state,WAIT);
    //File root;
    //root = SD.open("/");
    //printDirectory(root, 0);
  }

  if (onRelease(SINGLE_STEP)) {
    Serial.print("SSTEP. ");
    i8080_instruction();
    CPUStatus();
  }
  if (onRelease(SINGLE_STEP_)) {
     if (!bitRead(bus.state,USER4)) {
        bitSet(bus.state,USER4);
        Serial.println("SSTep/Debug On");
     } else {
        bitClear(bus.state,USER4);
        Serial.println("SSTep/Debug Off");
     }
  }
  if (onRelease(PROTECT)) {
     Serial.println("PROTECT");  
     bitSet(bus.state,PROT);
     Serial.println("Loading 88-2SIO Int Echo");
     for (int i=0; i < 64; i++) {
       CPUMemory[i] = Echo2SIOInt[i];
     }
     examine(0x0000);
     bitClear(bus.state,WAIT);
  }
  if (onRelease(UNPROTECT)) {
    Serial.println("UNPROTECT");
    bitClear(bus.state,PROT);
    Serial.println("Loading 88-SIO Echo");
    for (int i=0; i < 13; i++) {
       CPUMemory[i] = EchoSIO[i];
    }
    examine(0x0000);
    bitClear(bus.state,WAIT);
  }
  if (onRelease(AUX1_UP)) {
     Serial.println("AUX1_UP");
     if (bitRead(bus.state,USER3)) {
        Serial.println("LED Update Off");
        bitClear(bus.state,USER3);
        writeLEDs();
     } else {
        Serial.println("LEDs Update On");
        bitSet(bus.state,USER3);
     }

  }
  if (onRelease(AUX1_DOWN)) {
     Serial.println("AUX1_DOWN");  
     Serial.println("Loading 8K Basic...");
     loadFile("/8KBAS_E0.BIN",0xE000);
     loadFile("/8KBAS_E8.BIN",0xE800);
     loadFile("/8KBAS_F0.BIN",0xF000);
     loadFile("/8KBAS_F8.BIN",0xF800);
     examine(0xE000);
     bitClear(bus.state,WAIT);
  }
  if (onRelease(AUX2_UP)) {
    Serial.println("AUX2_UP");  
    Serial.println("Loading Altair Basic..");
    disk_drive.disk1.fp = SD.open("/ALTAIRDOS.DSK", FILE_READ);
    disk_drive.disk2.fp = SD.open("/BDSC.DSK", FILE_READ);
    if(!disk_drive.disk2.fp || !disk_drive.disk1.fp) {
      Serial.println("ERR:dsk");
      disk_drive.nodisk.status = 0xff;
    } else { 
      examine(0xff00);
      disk_drive.current = &disk_drive.disk1;
      bitClear(bus.state,WAIT);
    }
  }
  if (onRelease(AUX2_DOWN)) {
    Serial.println("AUX2_DOWN");
    disk_drive.disk1.fp = SD.open("/CPM63K.DSK", FILE_READ);
    disk_drive.disk2.fp = SD.open("/ZORK.DSK", FILE_READ);
    if(!disk_drive.disk2.fp || !disk_drive.disk1.fp) {
      Serial.println("ERR:dsk");
      disk_drive.nodisk.status = 0xff;
    } else {
      disk_drive.current = &disk_drive.disk1;
      examine(0xff00);
      bitClear(bus.state,WAIT);
    }
  }  
  if (!bitRead(bus.state,WAIT)) {
      i8080_instruction();
    //  }
  }
  if (bitRead(bus.state,USER4)) {
    //i8080_instruction();
    CPUStatus();
    //delay(1000);
  }
  if (bitRead(bus.state,USER3)) writeLEDs();
}
