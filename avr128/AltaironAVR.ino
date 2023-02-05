extern "C" {
  #include "../src/intel8080.h"
  #include "../src/88dcdd.h"
}
#include <SPI.h>
#include <SD.h>
#

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
        uint8_t data;
        digitalWrite(MEMORY_CS, LOW);

        SPI.transfer(3); // read byte
        SPI.transfer(0);
        SPI.transfer((address >> 8) & 0xff);
        SPI.transfer(address & 0xff); // 24 bit address
        data = SPI.transfer(0x00); // data
        digitalWrite(MEMORY_CS, HIGH);

	led_out(address, data, 0x00);

        return data;
}

inline void write8(uint16_t address, uint8_t val)
{
        digitalWrite(MEMORY_CS, LOW);

        SPI.transfer(2); // write byte
        SPI.transfer(0);
        SPI.transfer((address >> 8) & 0xff);
        SPI.transfer(address & 0xff); // 24 bit address
        SPI.transfer(val); // data
        digitalWrite(MEMORY_CS, HIGH);

	led_out(address, val, 0x00);
}

void load_file(intel8080_t *cpu)
{

}

void load_mem_file(const char* filename, size_t offset)
{

}

void load_raw_data(uint8_t program[], int s, int offset) 
{

}

uint8_t sense()
{

}

void setup()
{
Serial.begin(115200);
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

}

void loop()
{

}

