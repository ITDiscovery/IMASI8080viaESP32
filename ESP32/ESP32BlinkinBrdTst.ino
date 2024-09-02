#include <SPI.h>
#include <SD.h>
File root;

//Pin connected to RClk (Pin 12) of 74HC595
#define LlatchPin 2
//Pin connected to SRCk (Pin 11) of 74HC595
#define LclockPin 3
////Pin connected to SER (Pin 14) of 1st 74HC595
#define LdataPin 1
//Pin connected to PL! (Pin 1) of 74HCT165
#define SlatchPin 6
//Pin connected to CP (Pin 2) of 74HCT165
#define SclockPin 4
//Pin connected to Q7 (Pin 9) of 1st 74HCT165
#define SdataPin 5

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

void setup() {
  Serial.begin(115200);
  //set pins to output so you can control the shift register
  pinMode(LlatchPin, OUTPUT);
  pinMode(LclockPin, OUTPUT);
  pinMode(LdataPin, OUTPUT);
  pinMode(SlatchPin, OUTPUT);
  pinMode(SclockPin, OUTPUT);
  pinMode(SdataPin, INPUT);
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
 }
