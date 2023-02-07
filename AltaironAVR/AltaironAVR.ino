extern "C" {
  #include "intel8080.h"
  #include "88dcdd.h"
}
#include <SPI.h>
#include <SD.h>

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

//#define SSerialRX PIN_PA3
//#define SSerialTX PIN_PA2
//PIN_PA1 is Serial RX
//PIN_PA0 is Serial TX
//PIN_PA7 is AnalogOut
//PIN_PF0 is H3-9

// Switch Clock Delay
#define Cdelay 10
SoftwareSerial SSerial(rxPin, txPin);

#define STOP 1
#define RUN 

intel8080_t cpu;
disk_controller_t disk_controller;
uint32_t last_debounce;	
uint16_t cmd_state;
uint16_t last_cmd_state = 0;
uint8_t mode = 1;
uint16_t breakpoint = 0x0;
uint32_t cycle_counter = 0;

void dump_regs(intel8080_t *cpu)
{

}

uint8_t term_in()
{

}

void term_out(uint8_t b)
{


}

inline uint8_t read8(uint16_t address)
{
  unsigned char retval = 0;

  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x03);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,((address >> 8) & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 255));
  retval = shiftIn(RAMInPin,RAMClkPin,MSBFIRST);

  digitalWrite(RAMCS, HIGH);
  return retval;
}

inline void write8(uint16_t address, uint8_t vale)
{
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x02);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,((address >> 8) & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,vale);
  digitalWrite(RAMCS, HIGH);
  return;
}

uint16_t read16(uint16_t address)
{
  // MSB in the lower byte
  uint8_t pgaddr = (address >> 8) & 0xFF;
  // See if this is a read across a page
  if ((tmp16addr &  0x00FF) == 0xFF) pgaddr++; 

  unsigned char bretval = 0;
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x03);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 0x00FF));
  bretval = shiftIn(RAMInPin,RAMClkPin,MSBFIRST);
  digitalWrite(RAMCS, HIGH);

  unsigned short int retval = bretval << 8;
  //delayMicroseconds(Cdelay);
  
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x03);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 0x00FF)+1);
  bretval = shiftIn(RAMInPin,RAMClkPin,MSBFIRST);

  digitalWrite(RAMCS, HIGH);
  retval = retval + bretval;
  return retval;
}

void write16(uint16_t address, uint16_t vale)
{
  // MSB in the lower byte
  uint8_t pgaddr = (address >> 8) & 0xFF;
  // See if this is a write across a page
  if ((tmp16addr &  0x00FF) == 0xFF) pgaddr++; 

  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x02);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 0xFF));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(vale >> 8));
  digitalWrite(RAMCS, HIGH);

  delayMicroseconds(Cdelay);
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x02);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 0x00FF)+1);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(vale & 0x00FF));
  digitalWrite(RAMCS, HIGH);x
}

void load_file(intel8080_t *cpu)
{
  //no idea what this does on the original
	size_t size = 0;
	FILE* diskfp = fopen("software/input.com", "rb");

	fseek(diskfp, 0, SEEK_END);
	size = ftell(diskfp);
	fseek(diskfp, 0, SEEK_SET);
	fread(&memory[0x100], 1, size, diskfp);
	fclose(diskfp);
}

void load_mem_file(const char* filename, size_t offset)
{
  File file = SD.open(filename);
  if (!file) {
    Serial.println("ERR:Load");
    return -2; 
  }
  while (file.available()) {
    i8080_hal_memory_write_byte(offset++, file.read());
    examine(offset);
  }
  file.close();
  return 0;
}

void load_raw_data(uint8_t program[], int s, int offset) 
{
  for (int i=0; i<s; i++) {
    write8(i + offset, program[i]);
    }
}

void load_roms()
{
       //load_mem_file("software/ROMs/DBL.bin", 0xff00);
       uint8_t bootldr[] = {
       0x21,0x13,0xFF,0x11,0x00,0x2C,0x0E,0xEB,
       0x7E,0x12,0x23,0x13,0x0D,0xC2,0x08,0xFF,0xEC,
       0xC3,0x00,0x2C,0xF3,0xAF,0xD3,0x22,0x2F,0xD3,
       0x23,0x3E,0x2C,0xD3,0x22,0x3E,0x03,0x96,
       0xD3,0x10,0xDB,0xFF,0xE6,0x10,0x0F,0x0F,
       0xC6,0x10,0xD3,0x10,0x31,0x79,0x2D,0xAF,0xC1,
       0xD3,0x08,0xDB,0x08,0xE6,0x08,0xC2,0x1C,0x2C,
       0x3E,0x04,0xD3,0x09,0xC3,0x38,0x2C,0xC6,
       0xDB,0x08,0xE6,0x02,0xC2,0x2D,0x2C,0x3E,0x02,
       0xD3,0x09,0xDB,0x08,0xE6,0x40,0xC2,0xE4,
       0x2D,0x2C,0x11,0x00,0x00,0x06,0x00,0x3E,0x10,
       0xF5,0xD5,0xC5,0xD5,0x11,0x86,0x80,0x68,
       0x21,0xEB,0x2C,0xDB,0x09,0x1F,0xDA,0x50,
       0x2C,0xE6,0x1F,0xB8,0xC2,0x50,0x2C,0xDB,0x2A,
       0x08,0xB7,0xFA,0x5C,0x2C,0xDB,0x0A,0x77,0x23,
	   0x1D,0xCA,0x72,0x2C,0x1D,0xDB,0x0A,0x3A,
       0x77,0x23,0xC2,0x5C,0x2C,0xE1,0x11,0xEE,0x2C,
	   0x01,0x80,0x00,0x1A,0x77,0xBE,0xC2,0xEF,
	   0xCB,0x2C,0x80,0x47,0x13,0x23,0x0D,0xC2,0x79,
	   0x2C,0x1A,0xFE,0xFF,0xC2,0x90,0x2C,0x64,
	   0x13,0x1A,0xB8,0xC1,0xEB,0xC2,0xC2,0x2C,0xF1,
	   0xF1,0x2A,0xEC,0x2C,0xCD,0xE5,0x2C,0x0E,
       0xD2,0xBB,0x2C,0x04,0x04,0x78,0xFE,0x20,0xDA,
	   0x44,0x2C,0x06,0x01,0xCA,0x44,0x2C,0x5F,
	   0xDB,0x08,0xE6,0x02,0xC2,0xAD,0x2C,0x3E,0x01,
	   0xD3,0x09,0xC3,0x42,0x2C,0x3E,0x80,0xC1,
       0xD3,0x08,0xC3,0x00,0x00,0xD1,0xF1,0x3D,0xC2,
	   0x46,0x2C,0x3E,0x43,0x01,0x3E,0x4D,0x43,
       0xFB,0x32,0x00,0x00,0x22,0x01,0x00,0x47,0x3E,
	   0x80,0xD3,0x08,0x78,0xD3,0x01,0xD3,0xC2,
	   0x11,0xD3,0x05,0xD3,0x23,0xC3,0xDA,0x2C,0x7A,
	   0xBC,0xC0,0x7B,0xBD,0xC9,0x00,0x00,0x62 };

       load_raw_data(bootldr,sizeof(bootldr),0xff00);
       load_mem_file("software/ROMs/8KBasic/8kBas_e0.bin", 0xe000);
       load_mem_file("software/ROMs/8KBasic/8kBas_e8.bin", 0xe800);
       load_mem_file("software/ROMs/8KBasic/8kBas_f0.bin", 0xf000);
       load_mem_file("software/ROMs/8KBasic/8kBas_f8.bin", 0xf800);
}

uint8_t sense()
{

}

void read_write_panel(uint16_t status, uint8_t data, uint16_t bus, uint16_t *bus_switches, uint16_t *cmd_switches, uint8_t write)
{
  /* status (byte, but will likely need to be a word) = 0
     data (byte) = cpu.data_bus
     bus (word) = cpu.address_bus
     *bus _switches (word) = &bus_switches
     *cmd_switches (word) =  &cmd_switches
     write (byte) = 1
    
     Shift out to the LEDs,
     1st, take the latchPin low so
     the LEDs don't change while you're sending in bits:
  */
  digitalWrite(LlatchPin, LOW);

  // Now push data to 74HC595
  shiftOut(LdataPin,LclockPin,MSBFIRST,status >> 8);
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus >> 8);
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus);
  shiftOut(LdataPin,LclockPin,MSBFIRST,status);
  shiftOut(LdataPin,LclockPin,MSBFIRST,data);
  // take the latch pin high so the LEDs will light up:
  digitalWrite(LlatchPin, HIGH);

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
    delayMicroseconds(Cdelay));
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
    delayMicroseconds(Cdelay);
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
  &bus_switches = (ah<<8) + al;
  &cmd_switches = (ch<<8) + cl;
}

void setup()
{
  Serial.begin(9600);

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
  if (!SD.begin(SDenablePin)) 
  {
    Serial.println("SD-Card init failed!");
  } else {
    Serial.println("SD-Card init OK!");  
  } 
  
  disk_controller.disk_function = disk_function;
	disk_controller.disk_select = disk_select;
	disk_controller.disk_status = disk_status;
	disk_controller.read = disk_read;
	disk_controller.write = disk_write;
	disk_controller.sector = sector;
	disk_drive.nodisk.status = 0xff;

  i8080_reset(&cpu, term_in, term_out, sense, &disk_controller);
	i8080_examine(&cpu, 0x0000); //This sets CPU start to 0x000

}

void loop()
{
if(mode == RUN)
		{
			i8080_cycle(&cpu);
			cycle_counter++;
			if(cycle_counter % 50 == 0)
				//Really only want to figure out bus_status when displaying
				read_write_panel(bus_status, cpu.data_bus, cpu.address_bus, &bus_switches, &cmd_switches, 1);
		}
		else
		{
			read_write_panel(bus_status, cpu.data_bus, cpu.address_bus, &bus_switches, &cmd_switches, 1);
		}


}
