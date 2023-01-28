/*
  Microchip 23LC512--AVRXXDB28
  1  /CS --- PA7
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

unsigned char i8080_hal_memory_read_byte(short unsigned int tmp16addr) {
  boolean bitVal = LOW;
  unsigned char retval = 0;

  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x03);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,((tmp16addr >> 8) & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(tmp16addr & 255));
  retval = shiftIn(RAMInPin,RAMClkPin,MSBFIRST);

  digitalWrite(RAMCS, HIGH);
  return retval;
}

void i8080_hal_memory_write_byte(unsigned short int tmp16addr, unsigned char vale) {
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x02);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,((tmp16addr >> 8) & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(tmp16addr & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(tmp16addr & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,vale);
  digitalWrite(RAMCS, HIGH);
  return;
}

void setup() {

  uint16_t address = 0x001A;
  byte bval = 0x5A;
  byte retval = 0x00;
  boolean bitVal = 0;
  // put your setup code here, to run once:
  Serial.begin(115200);
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

  Serial.print("Status Register:");
  digitalWrite(RAMCS,LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x05);   //Read from register
  retval = shiftIn(RAMInPin,RAMClkPin,MSBFIRST);
  
  Serial.print(retval);
  Serial.println();
  digitalWrite(RAMCS,HIGH);

  delay(10);
   
  i8080_hal_memory_write_byte(address,bval);   
  delay(10);
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
