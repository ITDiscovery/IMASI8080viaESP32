//Pin connected to RClk (Pin 12) of 74HC595
#define LlatchPin 2
//Pin connected to SRCk (Pin 11) of 74HC595
#define LclockPin 3
////Pin connected to SER (Pin 14) of 1st 74HC595
#define LdataPin 1
//Pin connected to PL! (Pin 1) of 74HCT165
#define SlatchPin 7
//Pin connected to CP (Pin 2) of 74HCT165
#define SclockPin 5
//Pin connected to Q7 (Pin 9) of 1st 74HCT165
#define SdataPin 4

enum State { 
  HLDA, WAIT, WO /*was INTE*/, STACK /*PROT*/, MEMR, INP, 
  M1, OUT, HLTA, PROT /*was STACK*/, INTE /*was WO*/, INT };

enum Control { STOP, SINGLE_STEP, EXAMINE, DEPOSIT, 
    RUN, SINGLE_STEP_, EXAMINE_NEXT, DEPOSIT_NEXT, 
    AUX2_UP, AUX1_UP, PROTECT, RESET, AUX2_DOWN, 
    AUX1_DOWN, UNPROTECT, CLR };

struct {
  int address;
  byte data;
  byte state;
  byte option;
} bus;

struct {
  int address;
  int control,prev_control;
} switches;

#define Cdelay 10

void writeLEDs() {
  // take the latchPin low so
  // the LEDs don't change while you're sending in bits:
  // Now push data to 74HC595
  digitalWrite(LlatchPin, LOW);
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.option);
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
  bus.address = switches.address;
  bus.state = ch;
  bus.option = cl;
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
  readSwitches();
  Serial.print(" Switch Addr:");
  Serial.print(switches.address,HEX);
  Serial.print(" Switches Control:");
  Serial.println(switches.control,HEX);
  writeLEDs();
  delay(100);
 }
