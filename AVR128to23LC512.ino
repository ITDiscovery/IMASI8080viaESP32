/*
  Microchip 23LC512--Arduino Uno
  1  /CS --- PA7
  2  SO  --- PA5
  3  NC  --- GND
  4  Vss --- GND
  5  SI  --- PA4
  6  SCK --- PA6
  7  HOLD--- Vcc
  8  Vcc --- Vcc
*/

#include <SPI.h>
#define RAMCS PIN_PA7

unsigned char i8080_hal_memory_read_byte(short unsigned int tmp16addr) {
  digitalWrite(RAMCS, LOW);
  SPI.transfer(0x03); //Read Data Mode
  SPI.transfer((tmp16addr >> 8) & 255); //16 bit address
  SPI.transfer((tmp16addr) & 255);   
  unsigned char retval = SPI.transfer(0x00); //Data
  digitalWrite(RAMCS, HIGH);
  return retval;
}

void i8080_hal_memory_write_byte(unsigned short int tmp16addr, unsigned char vale) {
  digitalWrite(RAMCS, LOW);
  SPI.transfer(0x02);      //Write Data Mode
  SPI.transfer((tmp16addr >> 8) & 255);
  SPI.transfer((tmp16addr) & 255);
  SPI.transfer(vale);
  digitalWrite(RAMCS, HIGH);
  return;
}

void setup() {

  uint16_t address = 0x001A;
  byte bval = 0x5A;
  byte retval = 0x00;
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(RAMCS, OUTPUT);
  digitalWrite(RAMCS,HIGH);
  SPI.begin();
  
  SPI.setClockDivider(SPI_CLOCK_DIV2);    //SPI clock at 3MHx
  SPI.setDataMode(SPI_MODE3);             //CPOL=1, CPHA=1
  SPI.setBitOrder(MSBFIRST);              //MSB first

  digitalWrite(RAMCS,LOW);    
  SPI.transfer(0xFF);      //Reset Mode
  digitalWrite(RAMCS,HIGH);

  digitalWrite(RAMCS,LOW);
  SPI.transfer(0x01);        //Write to register
  SPI.transfer(0x00);        //Mode 0 to register
  digitalWrite(RAMCS,HIGH);

  Serial.print("Status Register:");
  digitalWrite(RAMCS,LOW);
  SPI.transfer(0x05);        //Read from register
  Serial.printHex(SPI.transfer(0xFF));
  Serial.println();
  digitalWrite(RAMCS,HIGH);
   
  //digitalWrite(RAMCS, LOW);
  //SPI.transfer(0x02);      //Write Data Mode
  //SPI.transfer((address >> 8) & 255);
  //SPI.transfer((address) & 255);
  //SPI.transfer(bval);
  //digitalWrite(RAMCS, HIGH);
  i8080_hal_memory_write_byte(address,bval);
    
  delay(10);

  //digitalWrite(RAMCS, LOW);
  //SPI.transfer(0x03); //Read Data Mode
  //SPI.transfer((address >> 8) & 255); //16 bit address
  //SPI.transfer((address) & 255);   
  //retval = SPI.transfer(0x00); //Data
  //digitalWrite(RAMCS, HIGH);

  retval = i8080_hal_memory_read_byte(address);
   
   Serial.print("Memory Check at: ");
   Serial.printHex(address);
   Serial.print(" Written:");
   Serial.printHex(bval);
   Serial.print(" Read:");
   Serial.printHex(retval);
   Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly
  Serial.println("Starting Full Test");
  for (unsigned short int tstaddr = 0x0000; tstaddr < 0xFFFF; tstaddr++) {
    unsigned char bval = random(255);
    i8080_hal_memory_write_byte(tstaddr,bval);
    unsigned char retval = i8080_hal_memory_read_byte(tstaddr);
    if (bval != retval ) {
       Serial.print("Memory Fail at: ");
       Serial.printHex(tstaddr);
       Serial.print(" Written:");
       Serial.printHex(bval);
       Serial.print(" Read:");
       Serial.printHex(retval);
       Serial.println();      
    }
    delay(1);
  }
  
}
