extern "C" {
  #include "i8080.h"
}
#include <Flash.h>
#include <SPI.h>
#include <SD.h>
#include "disk.h"

#define FlashOff 0x0C000
//#define DEBUG
//#define DISK_DEBUG

//Pin Connection Defines
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
//Pin connected to CE! (Pin 15) of 74HCT165
#define SenablePin PIN_PD7
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

//PIN_PA7 is AnalogOut
//PIN_PF0 is H3-9
//PIN_PF1 is H3-8

// Switch Clock Delay
#define Cdelay 10

/*Original 
enum State { 
  HLDA, WAIT, WO, STACK, MEMR, INP, M1, IOUT, HLTA, PROT,
  INTE, INT };
*/
//IMASI 8080
enum State { 
  INT, WO, STACK, HLTA, IOUT, M1, INP, MEMR, PROT, INTE,
  HLDA, WAIT, USER2, USER1, USER4, USER3 };

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


byte term_in() {
  return Serial.read(); 
}

void term_out(char c) {
  Serial.write(c & 0x7f);
}

int input(int port) {
  bitSet(bus.state,INP);
  static uint8_t character = 0;
// add more "simulated" hardware by adding more code around I/O ports
  switch (port) {
    case 0x00:
      return 0;
    case 0x01: //serial read
      return term_in();
    case 0x08: 
      return disk_status();
    case 0x09:
      return disk_sector();
    case 0x0a: 
      return disk_read();
    case 0x10: //2SIO port 1 status
      if (!character) {
        // printf("2SIO port 1 status\n");
        character = term_in();
      }
      return (character ? 0b11 : 0b10); 
    case 0x11: //2SIO port 1, read
      if (character) {
        int tmp = character; 
        character = 0; 
        return tmp; 
      } else {
        return term_in();
      }
    case 0xff: //sense switches
      return switches.address >> 8;
    default:
//      Serial.print("in ");
      Serial.println(port);
      while(1);
  }
  return 0xff;
}

void output(int port, byte vale) {
  // add more "simulated" hardware by adding more code around I/O ports
  bitSet(bus.state,IOUT);
  switch (port) {
    case 0x01: 
      //Serial.print((char)(value & 0x7f));
      term_out(vale);
      break;
    case 0x08:
      disk_select(vale);
      break;
    case 0x09:
      disk_function(vale);
      break;
    case 0x0a:
      disk_write(vale);
      break;
    case 0x10: // 2SIO port 1 control
      //nothing
      break;
    case 0x11: // 2SIO port 1 write  
      //Serial.print((char)(value & 0x7f));
      term_out(vale);
      break;
    case 0x12: // ????
      break;
    default:
//      Serial.print("out ");
      Serial.println(port);
      while(1);
      break;
  }
}

void loadData(byte program[], int s, int offset) {
   for (int i=0; i<s; i++) {
     Flash.writeByte(FlashOff+i+offset,program[i]);
   }
   readPage(offset);
}

int loadFile(const char filename[], int offset) {
  File file = SD.open(filename);
  int i=0;
  if (!file) {
    Serial.println("ERR:Load");
    return -2; 
  }
  while (file.available()) {
    Flash.writeByte(FlashOff + offset + (i++), file.read());
    #ifdef DEBUG
      Serial.print(i,HEX);
      Serial.print(" ");
    #endif
  }
  file.close();
  return 0;
}

void readPage(uint16_t tmp16addr) {
    uint32_t tmp32addr = tmp16addr + FlashOff;
    Serial.print("0x");
    Serial.print(tmp32addr, HEX);
    Serial.print(": ");
    for (unsigned long i = tmp32addr; i < (tmp32addr + 0x200); i++) {
      Serial.printHex(Flash.readByte(i)); // DxCore helper function - does the leading 0.
      if ((i & 31) != 31) {
        // Print 32 bytes per line, space between them, (31 spaces, fence post)
        Serial.print(" ");
      } else {
        // Last one of a line gets a newline printed after it instead.
        Serial.println();
        Serial.print("0x");
        Serial.print(i,HEX);
        Serial.print(": ");
      }
    }
    Serial.println();
}

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
  //Flash.readByte(uint32_t address)
  //Flash.readWord(uint32_t address)
  
  // From i8080_hal
  unsigned char i8080_hal_memory_read_byte(short unsigned int tmp16addr) {
  //int i8080_hal_memory_read_byte(int tmp16addr) {
     uint32_t tmp32addr = 0xFFFF0000 & tmp16addr;
     return Flash.readByte(tmp32addr + FlashOff);
     }

  // From i8080_hal
  void i8080_hal_memory_write_byte(unsigned short int tmp16addr, unsigned char vale) {
  //void i8080_hal_memory_write_byte(int tmp16addr, int vale) {
     uint32_t tmp32addr = 0xFFFF0000 & tmp16addr;
     uint8_t retval = Flash.writeByte(tmp32addr + FlashOff, vale);
     if (retval) {
        Serial.print("Error: ");
        Serial.println(retval,HEX);
     }
  }

  // From i8080_hal
  //int i8080_hal_memory_read_word(int addr);
  unsigned short int i8080_hal_memory_read_word(unsigned short int tmp16addr) {
     uint32_t tmp32addr = 0xFFFF0000 & tmp16addr;      
     return Flash.readWord(tmp32addr + FlashOff);
  }

  // From i8080_hal  
  //void i8080_hal_memory_write_word(int addr, int vale);
  void i8080_hal_memory_write_word(unsigned short int tmp16addr, unsigned short int vale) {
     uint32_t tmp32addr = 0xFFFF0000 & tmp16addr;      
     uint8_t retval = Flash.writeWord(tmp32addr + FlashOff, vale);
     if (retval) {
        Serial.print("Error: ");
     }   
  }

  // From i8080_hal 
  //int i8080_hal_io_input(int port);
  unsigned char i8080_hal_io_input(unsigned char port) {
    //This sends the port number off to a IO Input handler
    return input(port);
  }
  
  // From i8080_hal 
  //void i8080_hal_io_output(int port, int vale);
  void i8080_hal_io_output(unsigned char port, unsigned char vale) {
    //This sends the port number off to a IO Output handler
    output(port,vale);
  }

  // From i8080_hal 
  //extern void i8080_hal_iff(int on);
  void i8080_hal_iff(unsigned char on) {
    //This is the interupt handler, and is currently not implemented
  }
  // From i8080_hal 
  //extern unsigned char* i8080_hal_memory(void);
  //Not really sure what this is for
}

void examine(uint16_t tmp16addr) {
     uint32_t tmp32addr =  tmp16addr;
     i8080_jump(tmp16addr); //set program counter
     #ifdef DEBUG 
       Serial.print("Examine: Addr ");
       Serial.printHex(tmp32addr);
       Serial.print(" (Flash:");
       Serial.printHex(tmp32addr+FlashOff);
       Serial.print(") Value:");
       Serial.print(Flash.readByte(tmp32addr + FlashOff),HEX);
       Serial.print(" Address Bus: ");
       Serial.printHex(i8080_pc());
       Serial.print(" BC:" );
       Serial.printHex(i8080_regs_bc());
       Serial.println();
     #endif
     bus.data = Flash.readByte(tmp32addr+FlashOff);
     writeLEDs();
}

void deposit(uint16_t tmp16addr, uint8_t vale) {
     i8080_jump(tmp16addr); //set program counter
     uint32_t tmp32addr = (0x0000FFFF & tmp16addr) + FlashOff;
     uint8_t retval = Flash.writeByte(tmp32addr,vale);
     if (retval) {
         Serial.print("Flash Write Error: ");
         Serial.print(tmp32addr,HEX);
         Serial.print("/");
         Serial.println(retval,HEX);
     }
     #ifdef DEBUG
       Serial.print("Deposit: Addr:");
       Serial.print(tmp16addr,HEX);
       Serial.print(" (Flash:");
       Serial.print(tmp32addr,HEX);
       Serial.print(") Value In:");
       Serial.print(vale,HEX);
       Serial.print(" Out:");
       Serial.println(Flash.readByte(tmp32addr),HEX);
     #endif
     bus.data = Flash.readByte(tmp32addr);
     writeLEDs();
}

void setup() {
  Serial.begin(115200);
  Serial1.swap(1);
  Serial1.begin(38400);

  pinMode(LlatchPin, OUTPUT);
  pinMode(LclockPin, OUTPUT);
  pinMode(LdataPin, OUTPUT);
  pinMode(SlatchPin, OUTPUT);
  pinMode(SclockPin, OUTPUT);
  pinMode(SenablePin, OUTPUT);
  pinMode(SdataPin, INPUT);
  pinMode(SDenablePin,OUTPUT);
  pinMode(PIN_PF0,OUTPUT);
  pinMode(PIN_PF1, OUTPUT);

  //uint8_t Flash.erasePage(uint32_t address, uint8_t size = 1);
  uint8_t retval = Flash.checkWritable();
  switch (retval) {
      case 0:
        Serial.println("Flash OK!");
      break;
      case 1:
        Serial.println("Bootloader is pre-1.3.0");
      break;
      case 14:
        Serial.println("Sketch mismatch.");
      break;
   }
   // Erase First 16k of flash on an AVR128DA/DB, using a maximum size multi-page erase
   retval = Flash.erasePage(FlashOff,32);
   if (retval) {
     Serial.println("Error on Erase Pages!");
   } else {
     Serial.println("Success on Erase Page!");
   }

  // The 74HCT165 Enable should just be at GND
  digitalWrite(SenablePin, LOW);

  SPI.swap(SPI1_SWAP_DEFAULT);
  if (!SD.begin(SDenablePin)) {
    Serial.println("SD-Card init failed!");
    } else {
    Serial.println("SD-Card init OK!");  
  }

  //All Flags off except Stop
  bitSet(bus.state,WAIT);
  bitClear(bus.state,INT);
  bitClear(bus.state,WO);
  bitClear(bus.state,STACK);
  bitClear(bus.state,HLTA);
  bitClear(bus.state,IOUT);
  bitClear(bus.state,M1);
  bitClear(bus.state,INP);
  bitClear(bus.state,MEMR);
  bitClear(bus.state,PROT);
  bitClear(bus.state,INTE);
  bitClear(bus.state,HLDA);
  
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
  // put your main code here, to run repeatedly:
  bitClear(bus.state,MEMR); //flag set by readByte()
  bitClear(bus.state,M1);  //flag set by step()
  bitClear(bus.state,IOUT); //flag set by output()
  bitClear(bus.state,INP); //flag set by input()
  bitSet(bus.state,WO); //flag CLEARED by writeByte() inverted logic
  bitClear(bus.state, STACK); //set by readByte and writeByte if addr==SP
  bus.address = i8080_pc();

  readSwitches();

  if (onRelease(EXAMINE)) {
     Serial.println("EXAMINE"); 
     examine(switches.address);
  }
  if (onRelease(EXAMINE_NEXT)) {
     Serial.println("EXAMINE NEXT");  
     examine(switches.address+1);
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
     i8080_init();
     examine(0);
  }
  if (onRelease(CLR)) {
     Serial.println("CLR");
     if (!bitRead(bus.state,USER4)) {
         bitSet(bus.state,USER4);
     } else {
         bitClear(bus.state,USER4);
     }
  }
  if (onRelease(SINGLE_STEP) || onRelease(SINGLE_STEP_)) {
     Serial.println("SSTEP");  
     i8080_instruction();   
  }
  if (onRelease(PROTECT)) {
     Serial.println("PROTECT");  
     if (!bitRead(bus.state,PROT)) {
         bitSet(bus.state,PROT);
     } else {
         bitClear(bus.state,PROT);
     }
  }
  if (onRelease(UNPROTECT)) {
     Serial.println("UNPROTECT");
     byte killbits[] = {
       0x21,0x00,0x00,
       0x16,0x80,
       0x01,0x00,0x20,
       0x1a,
       0x1a,
       0x1a,
       0x1a,
       0x09,
       0xd2,0x08,0x00,
       0xdb,0xff,
       0xaa,
       0x0f,
       0x57,
       0xc3,0x08,0x00 };
    loadData(killbits,sizeof(killbits),0);
  }
  if (onRelease(AUX1_UP)) {
     Serial.println("AUX1_UP");
     Serial.print("Bus Addr: ");
     Serial.print(bus.address,HEX);
     Serial.print(" Switch Addr:");
     Serial.print(switches.address,HEX);
     Serial.print(" Data:");
     Serial.print(bus.data);
     Serial.print(" State:");
     Serial.println(bus.state,BIN);
     readPage(switches.address);
  }
  if (onRelease(AUX1_DOWN)) {
     Serial.println("AUX1_DOWN");  
     Serial.println("Loading 4K Basic...");
     loadFile("4KBAS32.BIN", 0);
  }
  if (onRelease(AUX2_UP)) {
     Serial.println("AUX2_UP");  
     Serial.println("Loading CPM...");
     loadFile("88DSKROM.BIN", 0xff00);
     disk_drive.disk1.fp = SD.open("cpm63k.dsk", FILE_WRITE);
     disk_drive.disk2.fp = SD.open("zork.dsk", FILE_WRITE);
     if(!disk_drive.disk2.fp || !disk_drive.disk1.fp)
        Serial.println("ERR:dsk");
     disk_drive.nodisk.status = 0xff;
     examine(0xff00);
     bitClear(bus.state,WAIT);
  }
  if (onRelease(AUX2_DOWN)) {
     Serial.println("AUX2_DOWN");  
  }  
  if (!bitRead(bus.state,WAIT)) {
    for (int i=0; i < 50; i++)
      i8080_instruction();
      if (!bitRead(bus.state,USER4)) {
        Serial.printHex(bus.address);
        Serial.print(":");
        Serial.printHex(bus.data);
        Serial.println();
      }

  }
  writeLEDs();
}
