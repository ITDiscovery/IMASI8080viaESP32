extern "C" {
  #include "i8080.h"
}

//#define DISK_DEBUG
//#define DEBUG
//#define DEBUGIO

#include <SPI.h>
#include <SD.h>
#include "disk.h"

//Pin connected to RClk (Pin 12) of 74HC595
#define LlatchPin PIN_PD2
//Pin connected to SRCk (Pin 11) of 74HC595
#define LclockPin PIN_PD3
////Pin connected to SER (Pin 14) of 1st 74HC595
#define LdataPin PIN_PD1
//Pin connected to PL! (Pin 1) of 74HCT165
#define SlatchPin PIN_PD6
//Pin connected to CP (Pin 2) of 74HCT165
#define SclockPin PIN_PD4
//Pin connected to Q7 (Pin 9) of 1st 74HCT165
#define SdataPin PIN_PD5

//Pin connected to SD Card Select
#define SDenablePin PIN_PC3
//PIN_PC0 is SD Card DataIn
//PIN_PC1 is SD Card DataOut
//PIN_PC2 is SD Card Clk

#define SSerialRX PIN_PA3
#define SSerialTX PIN_PA2
//PIN_PA3 is SSerial RX
//PIN_PA2 is SSerial TX
//PIN_PA1 is Serial RX
//PIN_PA0 is Serial TX

//PIN_PA7 is H3-29 AnalogOut
//PIN_PF1 is H3-26

/*
  Microchip 23LC512--AVRXXDB28
  1  /CS --- PF0
  2  SO  --- PA5
  3  NC  --- GND
  4  Vss --- GND
  5  SI  --- PA4
  6  SCK --- PA6
  7  HOLD--- Vcc
  8  Vcc --- Vcc
*/
#define RAMInPin PIN_PA5
#define RAMOutPin PIN_PA4
#define RAMClkPin PIN_PA6
#define RAMCS PIN_PF1

// Switch Clock Delay
#define Cdelay 10

/*Original 
enum State { 
  HLDA, WAIT, WO, STACK, MEMR, INP, M1, IOUT, HLTA, PROT,
  INTE, INT };
*/
//IMASI 8080
enum State { 
  INTA, WO, STACK, HLTA, IOUT, M1, INP, MEMR, INTE,
  RUNM, WAIT, HLDA, USER1, USER2, USER3, USER4 };

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
    RESET, CLR, RUNC, STOP, SINGLE_STEP, SINGLE_STEP_, 
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

void writeLEDs() {
  // take the latchPin low so
  // the LEDs don't change while you're sending in bits:
  digitalWrite(LlatchPin, LOW);

  // Now push data to 74HC595
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.state >> 8);
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.address >> 8);
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.address);
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
   delayMicroseconds(5);
   digitalWrite(SlatchPin, HIGH);
   delayMicroseconds(5);

  // Now get data from 74HC165
   for(int i = 0; i < 8; i++)
   {
        bitVal = digitalRead(SdataPin);
        al |= (bitVal << (7 - i));

        digitalWrite(SclockPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(SclockPin, LOW);
    }
   for(int i = 0; i < 8; i++)
   {
        bitVal = digitalRead(SdataPin);
        ah |= (bitVal << (7 - i));

        digitalWrite(SclockPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(SclockPin, LOW);
    }
   for(int i = 0; i < 8; i++)
   {
        bitVal = digitalRead(SdataPin);
        cl |= (bitVal << (7 - i));

        digitalWrite(SclockPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(SclockPin, LOW);
    }

   for(int i = 0; i < 8; i++)
   {
        bitVal = digitalRead(SdataPin);
        ch |= (bitVal << (7 - i));

        digitalWrite(SclockPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(SclockPin, LOW);
    }

   #ifdef DEBUG 
   Serial.print ("AL:");
   Serial.print(al,HEX);
   Serial.print (" AH:");
   Serial.print(ah,HEX);
   Serial.print (" CL:");
   Serial.print(cl,HEX);
   Serial.print (" CH:");
   Serial.print(ch,HEX);
   Serial.println();
   #endif

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
  
  // From i8080_hal
  unsigned char i8080_hal_memory_read_byte(short unsigned int tmp16addr) {
  unsigned char retval = 0;

  bitClear(bus.state,INP);
  bitClear(bus.state,IOUT);
  bitClear(bus.state,WO);
  bitSet(bus.state,MEMR);

  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x03);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,((tmp16addr >> 8) & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(tmp16addr & 255));
  retval = shiftIn(RAMInPin,RAMClkPin,MSBFIRST);

  digitalWrite(RAMCS, HIGH);

   #ifdef DEBUG
  Serial.print("Read Byte From:");
  Serial.printHex(tmp16addr);
  Serial.print(" Returns:");
  Serial.printHex(retval);
  Serial.println();
  #endif //DEBUG

  // Memory puts the data on the bus
  bus.data = retval;
  return retval;
  }

  // From i8080_hal
  void i8080_hal_memory_write_byte(unsigned short int tmp16addr, unsigned char vale) {

  bitClear(bus.state,INP);
  bitClear(bus.state,IOUT);
  bitClear(bus.state,MEMR);
  bitSet(bus.state,WO);
  //Set the bus state and put the written data on it
  bus.data = vale;
  
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x02);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,((tmp16addr >> 8) & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(tmp16addr & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,vale);
  digitalWrite(RAMCS, HIGH);
  return;
  }

  // From i8080_hal
  unsigned short int i8080_hal_memory_read_word(unsigned short int tmp16addr) {

  bitClear(bus.state,INP);
  bitClear(bus.state,IOUT);
  bitClear(bus.state,WO);
  bitSet(bus.state,MEMR);

  // MSB in the lower byte
  uint8_t pgaddr = (tmp16addr >> 8) & 0xFF;
  // See if this is a read across a page
  if ((tmp16addr &  0x00FF) == 0xFF) pgaddr++; 

  unsigned char bretval = 0;
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x03);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(tmp16addr & 0x00FF));
  bretval = shiftIn(RAMInPin,RAMClkPin,MSBFIRST);
  digitalWrite(RAMCS, HIGH);

  #ifdef DEBUG
    Serial.print("Read Word From:");
    Serial.printHex(tmp16addr);
    Serial.print(" Low Byte:");
    Serial.printHex(bretval);
  #endif //DEBUG

  unsigned short int retval = bretval;
  //delayMicroseconds(Cdelay);
  
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x03);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(tmp16addr & 0x00FF)+1);
  bretval = shiftIn(RAMInPin,RAMClkPin,MSBFIRST);
  digitalWrite(RAMCS, HIGH);
  retval = retval + (bretval << 8);

  #ifdef DEBUG
    Serial.print(" High Byte:");
    Serial.printHex(bretval);
    Serial.print(" Word:");
    Serial.printHex(retval);
    Serial.println();
  #endif //DEBUG
   
  return retval;
  }

  // From i8080_hal  
  void i8080_hal_memory_write_word(unsigned short int tmp16addr, unsigned short int vale) {

  bitClear(bus.state,INP);
  bitClear(bus.state,IOUT);
  bitClear(bus.state,MEMR);
  bitSet(bus.state,WO);
    
  // MSB in the lower byte
    
  uint8_t pgaddr = (tmp16addr >> 8) & 0xFF;
  // See if this is a write across a page
  if ((tmp16addr &  0x00FF) == 0xFF) pgaddr++; 

    #ifdef DEBUG
    Serial.print("Write Word To:");
    Serial.printHex(tmp16addr);
    Serial.print(" Vale:");
    Serial.printHex(vale);
    Serial.print(" Low Byte:");
    Serial.printHex(vale & 0xFF);
    Serial.print(" High Byte:");
    Serial.printHex(vale >> 8);
    Serial.println();
    #endif

    digitalWrite(RAMCS, LOW);
    shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x02);
    shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
    shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(tmp16addr & 0xFF));
    shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(vale & 0xFF));
    digitalWrite(RAMCS, HIGH);

    delayMicroseconds(Cdelay);
    digitalWrite(RAMCS, LOW);
    shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x02);
    shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
    shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(tmp16addr & 0x00FF)+1);
    shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(vale >> 8));
    digitalWrite(RAMCS, HIGH);

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

/*
#define SST_RDRF     0x01 // receive register full (character received)
#define SST_TDRE     0x02 // send register empty (ready for next byte)
#define SST_FNF      0x04 // used in simulator to signal file-not-found in CLOAD
#define SST_OVRN     0x20 // data overrun (received character when previous not read)
#define SST_OVRN2    0x40 // used in the simulator to signal that OVRN is upcoming
#define SST_INT      0x80 // interrupt signaled

#define SSC_SIOTP0   0x01 // revision of SIO board, bit 0
#define SSC_SIOTP1   0x02 // revision of SIO board, bit 1
#define SSC_INTTX    0x20 // transmit interrupt enabled
#define SSC_REALTIME 0x40 // force real-(simulation-)time operation (always use baud rate)
#define SSC_INTRX    0x80 // receive  interrupt enabled
*/

  // From i8080_hal
  unsigned char i8080_hal_io_input(unsigned char port) {
  #ifdef DEBUGIO
  Serial.print("In Port ");
  Serial.printHex(port);
  Serial.println();
  #endif
  
  bitSet(bus.state,INP);
  bitClear(bus.state,IOUT);
  bitClear(bus.state,WO);
  bitClear(bus.state,MEMR);
  
  unsigned char retval;

  switch (port) {
     case 0x00:
       return 0x03;
     case 0x1:     // CPU Board Serial Port
      if (Serial.available() == 0) {
        retval = Serial.read();
      } else {
        retval = 0;
      }
      return retval;
    case 0x06: // 88-ACR Board Cassette
      Serial.print("Cassette Port In 0x06, Data:");
      Serial.printHex(i8080_regs_a());
      Serial.println();
      return 0xFF;
    case 0x07: // 88-ACR Board Cassette
      Serial.print("Cassette Port In 0x07, Data:");
      Serial.printHex(i8080_regs_a());
      Serial.println();
      return 0xFF;
    case 0x08: 
      return disk_status();
    case 0x09:
      return disk_sector();
    case 0x0a: 
      return disk_read();
    case 0x10: // 88-2SIO or 88-ACR port 0, Status
        return 0x03;
    case 0x11: // 88-2SIO or 88-ACR port 0, read/RX
      if (Serial.available() == 0) {
        return 0x00;
      } else {
        return Serial.read();
      }
      break;
    case 0x12: // 88-2SIO or 88-ACR port 1, Status
        return 0x03;
    case 0x13: // 88-2SIO or 88-ACR port 1, read/RX
      if (Serial1.available() == 0) {
        return 0x00; 
      } else {
        return Serial1.read();
      }
      break;
    case 0x18: // 88-ACR Board I/O 2ndary
      Serial.print("Cassette Port In 0x18, Data:");
      Serial.printHex(i8080_regs_a());
      Serial.println();
      retval = 0xFF;
      break;
    case 0x19: // 88-ACR Board I/O 2ndary
      Serial.print("Cassette Port In 0x19, Data:");
      Serial.printHex(i8080_regs_a());
      Serial.println();
      retval = 0xFF;
      break;
    case 0x20: // 88-2SIO or 88-ACR Alternate port 0, Status
        return 0x03;
    case 0x21: // 88-2SIO or 88-ACR Alternate port 0, read/RX
      if (Serial.available() == 0) {
        return 0x00;
      } else {
        return Serial.read();
      }
      break;
    case 0x22: // 88-2SIO or 88-ACR Alternate port 1, Status
        return 0x03;
    case 0x23: // 88-2SIO or 88-ACR Alternate port 1, read/RX
      if (Serial.available() == 0) {
        return 0x00;
      } else {
        return Serial.read();
      }
      break;
    case 0xfe:
      Serial.print("88-VI/RTC In 0xFE, Data:");
      Serial.printHex(i8080_regs_a());
      Serial.println();
      return 0xFF;
    case 0xff: // Front panel switches
      //Serial.print("Address PI: ");
      readSwitches();
      //Serial.printHex(switches.address >> 8);
      //Serial.println();
      return (switches.address >> 8);
    default:
      Serial.print("In Port:");
      Serial.printHex(port);
      Serial.print(" Data:");
      Serial.printHex(i8080_regs_a());
      Serial.println();
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
    bitClear(bus.state,WO);
    bitClear(bus.state,MEMR);

    #ifdef DEBUGIO
    Serial.print("Out Port ");
    Serial.printHex(port);
    Serial.print(" Value:");
    Serial.printHex(vale);
    Serial.println();
    #endif

    //Put the data on the bus
    bus.data = vale;

    switch (port) {
    case 0x00:
      Serial.write(vale);
      return;
    case 0x01: 
      Serial.write(vale);
      return;
    case 0x06: // 88-ACR Board Cassette
      Serial.print("Cassette Port Out 0x06, Data:");
      Serial.printHex(i8080_regs_a());
      Serial.println();
    case 0x07: // 88-ACR Board Cassette
      Serial.print("Cassette Port Out 0x07, Data:");
      Serial.printHex(i8080_regs_a());
      Serial.println();
    case 0x08:
      disk_select(vale);
      return;
    case 0x09:
      disk_function(vale);
      return;
    case 0x0a:
      disk_write(vale);
      return;
    case 0x10: // 88-2SIO or 88-ACR port 0, Control
      return;
    case 0x11: // 88-2SIO or 88-ACR port 0, Write/TX
      Serial.write(vale);
      return;
    case 0x12: // 88-2SIO or 88-ACR port 1, Control
      return;
    case 0x13: // 88-2SIO or 88-ACR port 1, Write/TX
      Serial1.write(vale);
      return;
    case 0x20: // 88-2SIO or 88-ACR Alternate port 0, Control
      return;
    case 0x21: // 88-2SIO or 88-ACR Alternate port 0, Write/TX
      Serial.write(vale);
      return;
    case 0x22: // 88-2SIO or 88-ACR Alternate port 1, Control
      return;
    case 0x23: // 88-2SIO or 88-ACR Alternate port 1, Write/TX
      Serial.write(vale);
      return;
    case 0xFE:
      Serial.print("88-VI/RTC Out, Data:");
      Serial.printHex(i8080_regs_a());
      Serial.println();
      return;
    case 0xFF: // panel LEDs
      bus.state = (bus.state & 0x0FFF) + (vale << 12);      
      return;
    default:
      Serial.print("Port Out: ");
      Serial.printHex(port);
      Serial.print(" Data:");
      Serial.printHex(vale);
      Serial.println();
      return;
    }
  }

  //From i8080_hal
  void i8080_hal_halt() {
    bitSet(bus.state,HLTA);
    bitClear(bus.state,RUNM);
    bitSet(bus.state,WAIT);
    bitSet(bus.state,HLDA);
    return;
  }

  // From i8080_hal 
  //extern void i8080_hal_iff(int on);
  void i8080_hal_iff(unsigned char on) {
    //This is the interupt handler, and is currently not implemented
    bitSet(bus.state,INTE);
    return;
  }
  
  // From i8080_hal 
  extern unsigned char* i8080_hal_memory(void) {
    //Not really sure what this is for, set M1 just in case
    bitSet(bus.state,M1);
    return 0;
  }
}

void dumpregs() {
  Serial.print("PC: ");
  Serial.printHex(i8080_pc());
  Serial.print(" Data Bus:");
  Serial.printHex(bus.data);
  Serial.print(" A:");
  Serial.printHex(i8080_regs_a());
  Serial.print(" Flags:");
  Serial.printHex(i8080_regs_flags());
  Serial.print(" BC:");
  Serial.printHex(i8080_regs_bc()); 
  Serial.print(" DE:");
  Serial.printHex(i8080_regs_de());
  Serial.print(" HL:");
  Serial.printHex(i8080_regs_hl());
  Serial.print(" Stack:");
  Serial.printHex(i8080_regs_sp());
  Serial.println();  
}

void examine(uint16_t tmp16addr) {
     Serial.print(" Addr ");
     Serial.printHex(tmp16addr);
     Serial.print(" Value:");
     Serial.print(i8080_hal_memory_read_byte(tmp16addr),HEX);
     Serial.println();
     i8080_jump(tmp16addr); //set program counter
     bus.data = i8080_hal_memory_read_byte(tmp16addr);
     dumpregs();       
     writeLEDs();
}

void deposit(uint16_t tmp16addr, uint8_t vale) {
     i8080_jump(tmp16addr); //set program counter
     Serial.print(" Addr:");
     Serial.printHex(tmp16addr);
     Serial.print(" Out:");    
     i8080_hal_memory_write_byte(tmp16addr,vale);
     Serial.printHex(i8080_hal_memory_read_byte(tmp16addr));
     Serial.println();
     bus.data = i8080_hal_memory_read_byte(tmp16addr);
     dumpregs();
     writeLEDs();
}

void readPage(uint16_t tmp16addr) {
    Serial.print("0x");
    Serial.print(tmp16addr, HEX);
    Serial.println(":");
    for (uint16_t i = tmp16addr; i < (tmp16addr +  0x200); i++) {
      Serial.printHex(i8080_hal_memory_read_byte(i)); // DxCore helper function - does the leading 0.
      if ((i & 31) != 31) {
        // Print 32 bytes per line, space between them, (31 spaces, fence post)
        Serial.print(' ');
      } else {
        // Last one of a line gets a newline printed after it instead.
        Serial.println();
      }
    }
    Serial.println();
}

void loadHexFile(const char filename[]) {
  File hexFile = SD.open(filename);
  
  // Read the HEX file line by line
  while (hexFile.available()) {
    String line = hexFile.readStringUntil('\n');
    uint8_t dataLen = line.charAt(0);    
    uint8_t recordType = hexsToInt(line.substring(7,9));
    if (recordType == 0x01) {
          hexFile.close();
          return;
    }
    if ((dataLen == 0x3A) && (recordType == 0)) {      
      // Parse the line
      dataLen = hexsToInt(line.substring(1,3));
      uint16_t ldaddress = hexsToInt(line.substring(3,7));
      uint8_t checksum = hexsToInt(line.substring(line.length() - 3));

      #ifdef DEBUG
      Serial.print("Data Length: ");
      Serial.printHex(dataLen);
      Serial.print(" Load Address: ");
      Serial.printHex(ldaddress);
      Serial.print(" Record Type:");
      Serial.printHex(recordType);
      Serial.print(" Checksum:");
      Serial.printHex(checksum);
      Serial.println();    
      #endif
      // Verify the checksum
      uint8_t computedChecksum = computeChecksum(line.substring(1, line.length() - 2));
      if (computedChecksum != checksum) {
        Serial.print("Computed Error!");
        return;
      }
      // Load the data into memory
      if (recordType == 0) {
        Serial.print("Addr ");
        Serial.printHex(ldaddress);
        Serial.print(" ");
        dataLen = 9 + dataLen * 2;
        checksum = 0;
        for (uint8_t i = 9; i < dataLen; i=i+2) {
          uint8_t dataval = hexsToInt(line.substring(i, i + 2));
          i8080_hal_memory_write_byte((ldaddress + checksum) , dataval);
          checksum++;
          Serial.printHex(dataval);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
  }

  // Close the HEX file
  hexFile.close();
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

uint16_t hexsToInt(String s) {
  uint16_t result = 0;
  uint8_t sLen = s.length();
  for (uint8_t i = 0; i < sLen; i++) {
    result = result | (hexcToInt(s.charAt(i)) << ((sLen-i-1) * 4));
  }
  return result;
}

uint8_t computeChecksum(String s) {
  uint16_t result = 0;
  for (uint8_t i = 0; i < s.length()-1; i += 2) {
    result += hexsToInt(s.substring(i, i + 2));
  }
  #ifdef DEBUG
  Serial.print(" Sum:");
  Serial.printHex(result);
  Serial.println();
  #endif
  uint8_t lsb = result & 0xFF;
  return ~lsb + 1;
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
  
  Serial.begin(38400);
  Serial1.swap(1);
  Serial1.begin(38400);

  //Setup the 74HCT595s
  pinMode(LlatchPin, OUTPUT);
  pinMode(LclockPin, OUTPUT);
  pinMode(LdataPin, OUTPUT);

  //Setup the 74HCT165s
  pinMode(SlatchPin, OUTPUT);
  pinMode(SclockPin, OUTPUT);
  pinMode(SdataPin, INPUT);

  //Optional Setups
  pinMode(PIN_PA7,OUTPUT);
  pinMode(PIN_PD7,INPUT);
  pinMode(PIN_PF1, OUTPUT);

  // Setup the 23LC512
  pinMode(RAMCS, OUTPUT);
  pinMode(RAMInPin, INPUT); 
  pinMode(RAMOutPin, OUTPUT);
  pinMode(RAMClkPin, OUTPUT);
  digitalWrite(RAMCS,HIGH);
  digitalWrite(RAMCS,LOW);    
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0xFF); //Reset Mode
  digitalWrite(RAMCS,HIGH);

  digitalWrite(RAMCS,LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x01); //Write to register
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x00); //Mode 0 to register
  digitalWrite(RAMCS,HIGH);

  //SDCard Output Enable
  pinMode(SDenablePin,OUTPUT);

  //SDCard Setup
  SPI.swap(SPI1_SWAP_DEFAULT);
  if (!SD.begin(SDenablePin)) {
    Serial.println("SD-Card init failed!");
    } else {
    Serial.println("SD-Card init OK!");  
    }  

  //All Flags off except Stop
  bitClear(bus.state,INTA);
  bitSet(bus.state,WO); //flag CLEARED by writeByte() inverted logic
  bitClear(bus.state, STACK); //set by readByte and writeByte if addr==SP
  bitClear(bus.state,HLTA);
  bitClear(bus.state,IOUT); //flag set by output()
  bitClear(bus.state,M1);  //flag set by step()
  bitClear(bus.state,INP); //flag set by input()
  bitClear(bus.state,MEMR); //flag set by readByte()
  bitClear(bus.state,INTE); //flag set by iff (interupt enable);
  bitClear(bus.state,RUNM); //Run Mode
  bitSet(bus.state,WAIT); //Tripped by a HLT or breakpoint
  bitClear(bus.state,HLDA); //Step Mode

  //Turn on USER1-4
  bitSet(bus.state,USER1);
  bitSet(bus.state,USER2);
  bitSet(bus.state,USER3);
  bitSet(bus.state,USER4); 

  bus.address = 0;
  bus.data = 0;
  i8080_init();
  examine(0);

}

void loop() {
  bus.address = i8080_pc();
  readSwitches();

  if (onRelease(EXAMINE)) {
     Serial.print("EXAMINE ");
     examine(switches.address);
  }
  if (onRelease(EXAMINE_NEXT)) {
     Serial.print("EXAMINE NEXT ");
     examine(switches.address+1);
  } 
  if (onRelease(DEPOSIT)) {
     Serial.print("DEPOSIT ");
     uint8_t tmpvale = 0x00FF & switches.address;
     deposit(i8080_pc(),tmpvale);
  }
  if (onRelease(DEPOSIT_NEXT)) {
     Serial.print("DEPOSIT NEXT");
     uint8_t tmpvale = 0x00FF & switches.address;
     deposit(i8080_pc()+1,tmpvale);
  }
  if (onRelease(RUNC))  {
     Serial.println("RUN");
     bitSet(bus.state,RUNM);
     bitClear(bus.state,WAIT);
     bitClear(bus.state,HLTA);
     writeLEDs();
  }
  if (onRelease(STOP)) {
     Serial.println("STOP");
     bitClear(bus.state,HLTA);
     bitClear(bus.state,RUNM);
     bitSet(bus.state,WAIT);
     writeLEDs();
  } 
  if (onRelease(RESET)) {
     Serial.println("RESET");  
     examine(0);
  }
  if (onRelease(CLR)) {

    Serial.println("CLR - Read Page");
    readPage(i8080_pc());
    
    //Change this to wiping the memory chip
    //Serial.println("Clearing Memory!!");
    //for(unsigned short int rmaddr = 0; rmaddr < 65536; rmaddr++ ) {
    //  i8080_hal_memory_write_word(rmaddr,0xFF);
    //  if ((rmaddr & 0x00FF) == 0) {
    //    bus.address = rmaddr;
    //    writeLEDs();
    //  }
    //}
    //Serial.println("Cleared Memory!!");
  }
  if (onRelease(SINGLE_STEP)) {
     bitClear(bus.state,HLTA);
     bitClear(bus.state,RUNM);
     bitSet(bus.state,WAIT);
     bitSet(bus.state,HLDA);
     i8080_instruction();
     Serial.print("SSTEP After:");
     dumpregs();
  }
  if (onRelease(SINGLE_STEP_)) {
     if (bitRead(bus.state,HLDA)) {
       Serial.println("SStep Mode Off");
       bitClear(bus.state,HLDA);
     } else {
       Serial.println("SStep Mode On");
       bitSet(bus.state,HLDA);
     }
  }
  if (onRelease(PROTECT)) {
     Serial.println("Robot Protect mode, reset to exit");
     while (HIGH) {
      bus.address = random(65536);
      bus.data = random(256);
      bus.state = random(65536);
      writeLEDs();
      delay(500);
     }
  }
  if (onRelease(UNPROTECT)) {    
     Serial.println("Load Very Tiny Basic");
     loadFile("vtl-2.BIN", 0xF800);
     examine(0xF800);  
  
     //Serial.println("Load 8K Basic E0 ROM");
     //loadFile("8kBas_e0.BIN", 0xE000);
     //Serial.println("Load 8K Basic E8 ROM");
     //loadFile("8kBas_e8.BIN", 0xE800);
     //Serial.println("Load 8K Basic F0 ROM");
     //loadFile("8kBas_f0.BIN", 0xF000);
     //Serial.println("Load 8K Basic F8 ROM");
     //loadFile("8kBas_f8.BIN", 0xF800);
     //examine(0xE000);
     //Serial.println("Set Address PI to 0001 0000 for 2SIO or 0010 0000 for SIO, then RUN.");
     //Address PI 0100 0000 for Port 20, 1000 0000 for Port 5
  }
  if (onRelease(AUX1_UP)) {
     dumpregs();
     readPage(i8080_pc());
  }
  if (onRelease(AUX1_DOWN)) {
     Serial.println("CLR - SD Card LIsting");
     File root;
     root = SD.open("/");
     printDirectory(root, 0);
  }
  if (onRelease(AUX2_UP)) {
     Serial.println("AUX2_UP");  
     Serial.println("Loading CPM...");
     loadFile("88DSKROM.BIN", 0xff00);
     //loadHexFile("CDBL.HEX");
     disk_drive.disk1.fp = SD.open("ALTCPM.DSK", FILE_WRITE);
     disk_drive.disk2.fp = SD.open("MEMTST.DSK", FILE_WRITE);
     if(!disk_drive.disk2.fp || !disk_drive.disk1.fp)
        Serial.println("ERR:dsk");
     disk_drive.nodisk.status = 0xff;
     examine(0xFF00);
  }
  if (onRelease(AUX2_DOWN)) {
     Serial.println("AUX2_DOWN - LED and Switch Test");
     for (byte i = 0; i < 200; i++) {
        readSwitches();
        digitalWrite(LlatchPin, LOW);
        // Now push data to 74HC595
        shiftOut(LdataPin,LclockPin,MSBFIRST,switches.control >> 8);
        shiftOut(LdataPin,LclockPin,MSBFIRST,switches.address >> 8);
        shiftOut(LdataPin,LclockPin,MSBFIRST,switches.address);
        shiftOut(LdataPin,LclockPin,MSBFIRST,switches.control);
        shiftOut(LdataPin,LclockPin,MSBFIRST,switches.address);
        // take the latch pin high so the LEDs will light up:
        digitalWrite(LlatchPin, HIGH);
        Serial.print ("Address:");
        Serial.print(switches.address,HEX);
        Serial.print (" Control:");
        Serial.print(switches.control,HEX);
        Serial.println();
        delay(1000);
     }
  }

  //Lights and Switch Check every 100 Instructions
  if ((bitRead(bus.state,RUNM)) && (!bitRead(bus.state,HLDA))) {
    for (int i=0; i < 100; i++) i8080_instruction();
  } 
  if ((bitRead(bus.state,RUNM)) && (bitRead(bus.state,HLDA))) {
    writeLEDs();
    i8080_instruction();
    //dumpregs();
  }
  
}
