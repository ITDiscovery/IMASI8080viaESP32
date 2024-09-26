//**************************************************************//
  //  Name    : shiftOut 16 bits
//  Notes   : Code for using two 74HC595 Shift Register           // 
//          : to count from 0 to 65535
//****************************************************************
//Pin connected to ST_CP of 74HC595
int latchPin = 2;
//Pin connected to SH_CP of 74HC595
int clockPin = 3;
////Pin connected to DS of 74HC595
int dataPin = 1;

void setup() {
//set pins to output so you can control the shift register
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  Serial.begin(115200);
  }

void loop() {
    byte firstbyte = random(256);
    byte secondbyte = random(256);
    byte thirdbyte = random(256);
    byte fourthbyte = random(256);

    Serial.print(firstbyte,HEX);
    Serial.print(secondbyte,HEX);
    Serial.print(" - ");
    Serial.print(thirdbyte,HEX);
    Serial.println(fourthbyte,HEX);
    // take the latchPin low so
    // the LEDs don't change while you're sending in bits:
    digitalWrite(latchPin, LOW);
    // shift out the first byte:
    shiftOut(dataPin, clockPin, MSBFIRST, firstbyte);
    // shift out the second byte:
    shiftOut(dataPin, clockPin, MSBFIRST, secondbyte);
    // shift out the third byte:
    shiftOut(dataPin, clockPin, MSBFIRST, thirdbyte);
    // shift out the fourth byte:
    shiftOut(dataPin, clockPin, MSBFIRST, fourthbyte);
   //take the latch pin high so the LEDs will light up:
   digitalWrite(latchPin, HIGH);
   // pause before next value:
   delay(100);
}
