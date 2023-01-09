#include <SPI.h>
#include <SD.h>
File root;

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


void sendLED(byte ldata, byte lstatus, byte lowaddr, byte upaddr, byte lstate) {
    // take the latchPin low so
    // the LEDs don't change while you're sending in bits:
    digitalWrite(LlatchPin, LOW);
    // shift out the first byte:
    shiftOut(LdataPin, LclockPin, MSBFIRST, lstate);
    // shift out the second byte:
    shiftOut(LdataPin, LclockPin, MSBFIRST, upaddr);
    // shift out the third byte:
    shiftOut(LdataPin, LclockPin, MSBFIRST, lowaddr);
    // shift out the fourth byte:
    shiftOut(LdataPin, LclockPin, MSBFIRST, lstatus);
    // shift out the fifth byte:
    shiftOut(LdataPin, LclockPin, MSBFIRST, ldata);
    //take the latch pin high so the LEDs will light up:    
    digitalWrite(LlatchPin, HIGH);  
}

void readSwitch() {
   byte al = 0;
   byte ah = 0;
   byte cl = 0;
   byte ch = 0;
   boolean bitVal = LOW;
 
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

   Serial.print ("AL:");
   Serial.print(al,HEX);
   Serial.print (" AH:");
   Serial.print(ah,HEX);
   Serial.print (" CL:");
   Serial.print(cl,HEX);
   Serial.print (" CH:");
   Serial.print(ch,HEX);
   Serial.println();

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
  Serial.begin(115200);
  //set pins to output so you can control the shift register
  pinMode(LlatchPin, OUTPUT);
  pinMode(LclockPin, OUTPUT);
  pinMode(LdataPin, OUTPUT);
  pinMode(SlatchPin, OUTPUT);
  pinMode(SclockPin, OUTPUT);
  pinMode(SenablePin, OUTPUT);
  pinMode(SdataPin, INPUT);
  pinMode(SDenablePin,OUTPUT);
  SPI.swap(SPI1_SWAP_DEFAULT);
  if (!SD.begin(PIN_PC3)) {
    Serial.println("SD Card initialization failed!");
    //while (1);
    } else {
    Serial.println("SD Card initialization succeeded!");  
    }
  // The 74HCT165 Enable should just be at GND
  digitalWrite(SenablePin, LOW);
}

void loop() {
    Serial.println("Shifting On/Off Light test.");
    for(byte i = 0; i < 10; i++) {
       sendLED(255,255,255,255,255);
       delay(1000);
       sendLED(0,0,0,0,0);
       delay(1000);    
    }
    Serial.print("Low Data-");
    for(byte i = 0; i < 10; i++) {
       sendLED(255,0,0,0,0);
       delay(1000);
       sendLED(0,0,0,0,0);
       delay(1000);    
    }
    Serial.print("Status-");
    for(byte i = 0; i < 10; i++) {
       sendLED(0,255,0,0,0);
       delay(1000);
       sendLED(0,0,0,0,0);
       delay(1000);    
    }
    Serial.print("Address Bus-");
    for(byte i = 0; i < 10; i++) {
       sendLED(0,0,255,255,0);
       delay(1000);
       sendLED(0,0,0,0,0);
       delay(1000);    
    }
    Serial.println("State");
    for(byte i = 0; i < 10; i++) {
       sendLED(0,0,0,0,255);
       delay(1000);
       sendLED(0,0,0,0,0);
       delay(1000);    
    }
    Serial.println("Count Up");
    for(int i=0; i<256; i++) {
      sendLED(i,i,i,i,i);
      delay(100);
    }
    Serial.println("Switch Test");
    for (int j = 0; j < 100; j++) {
       readSwitch();
       delay(100);
    }
    root = SD.open("/");
    printDirectory(root, 0);
    Serial.println("SD Card Done!");
    // file 88DSKROM.BIN should be there, it's CPM
    
 }
