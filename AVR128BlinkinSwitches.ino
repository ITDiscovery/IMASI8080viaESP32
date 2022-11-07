//Pin connected to ST_CP of 74HCT165
int latchPin = PIN_PD6;
//Pin connected to SH_CP of 74HCT165
int clockPin = PIN_PD4;
//Pin connected to DS of 74HCT165
int dataPin = PIN_PD5;
//Pin connected to EN of 74HCT165
int enablePin  = PIN_PD7;

#define DATA_WIDTH 16
#define Cdelay 10

unsigned long pinValues;

void setup() {
//set pins to output so you can control the shift register
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  pinMode(enablePin, OUTPUT);
  Serial.begin(115200);

  digitalWrite(clockPin, LOW);
  digitalWrite(latchPin, HIGH);
}

void loop()
{
   long bitVal;
   unsigned long bytesVal = 0;

   digitalWrite(enablePin, HIGH);
   digitalWrite(latchPin, LOW);
   delayMicroseconds(Cdelay);
   digitalWrite(latchPin, HIGH);
   digitalWrite(enablePin, LOW);

   for(int i = 0; i < DATA_WIDTH; i++)
   {
        bitVal = digitalRead(dataPin);
        bytesVal |= (bitVal << ((DATA_WIDTH-1) - i));

        digitalWrite(clockPin, HIGH);
        delayMicroseconds(Cdelay);
        digitalWrite(clockPin, LOW);
    }
    Serial.println(bytesVal,BIN);
    delay(50);
}

void print_byte() { 

  Serial.println("*Shift Register Values:*\r\n");

  for(byte i=0; i<=DATA_WIDTH-1; i++) 
  { 
    Serial.print("P");
    Serial.print(i+1);
    Serial.print(" "); 
  }
  Serial.println();
  for(byte i=0; i<=DATA_WIDTH-1; i++) 
  { 
    Serial.print(pinValues >> i & 1, BIN); 
    
    if(i>8){Serial.print(" ");}
    Serial.print("  "); 
    
  } 
  
  Serial.print("\n"); 
  Serial.println();Serial.println();

}
