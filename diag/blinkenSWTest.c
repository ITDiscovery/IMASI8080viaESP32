#include <stdint.h>
#include <wiringPi.h>
#include <stdbool.h>
#include <stdio.h>

#define LEDdPIN  27
#define LEDlPIN  28
#define LEDcPIN  29
#define SWdPIN  24
#define SWlPIN  23
#define SWcPIN  25
#define lDelay 5
#define MSBFIRST 1

void rpi_init()
{
	wiringPiSetup();
	pinMode(LEDlPIN,OUTPUT);
	pinMode(LEDlPIN,OUTPUT);
	pinMode(LEDcPIN,OUTPUT);
	pinMode(SWdPIN,INPUT);
	pinMode(SWcPIN,OUTPUT);
	pinMode(SWlPIN,OUTPUT);
}

void shiftOut(uint8_t dpin,uint8_t cpin,uint8_t order,uint8_t idata) {
  //MSBOrder is 1 always for us,someday update this code to do either
  for(int i=7; i > -1; i--){
    digitalWrite(cpin,LOW);
    digitalWrite(dpin, (idata >> i) & 0x01 );
    digitalWrite(cpin,HIGH);
  }
}

uint8_t shiftIn(uint8_t dpin,uint8_t cpin,uint8_t order) {
    //LSBOrder is 0 always for us, someday update this code to do either
    uint8_t retVal = 0;
    bool bitVal = 0;
    for(int i=0; i < 8; i++ ){
        bitVal = digitalRead(dpin);
        retVal |= (bitVal << (7 - i));
        digitalWrite(cpin,HIGH);
        delay(lDelay);
        digitalWrite(cpin,LOW);
    }
    return retVal;
}

int main ()
{
    rpi_init();
    uint16_t bus_switches = 0;
    uint16_t cmd_switches = 0;
    uint16_t status = 0x0000;
    uint8_t data = 0x00;
    uint16_t bus = 0x0000;

    while (1) {
        // Get Switches by shifting in from 74HCT165
        // Write Pulse to Latch Pin
        digitalWrite(SWlPIN,LOW);
        delay(lDelay);
        digitalWrite(SWlPIN,HIGH);
        delay(lDelay);
        // Get Address Low, Address High, Control Low, Control High
        bus_switches = shiftIn(SWdPIN,SWcPIN,MSBFIRST);
        bus_switches = bus_switches + (shiftIn(SWdPIN,SWcPIN,MSBFIRST) << 8);
        cmd_switches = shiftIn(SWdPIN,SWcPIN,MSBFIRST);
        cmd_switches = cmd_switches + (shiftIn(SWdPIN,SWcPIN,MSBFIRST) << 8);
        //digitalWrite(SWlPIN ,HIGH);
        printf("Address: %x   Control: %x \n",bus_switches,cmd_switches);

        // Now push data to 74HC595
        digitalWrite(LEDlPIN, LOW);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,cmd_switches >> 8);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,bus_switches);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,bus_switches >> 8);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,cmd_switches);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,cmd_switches);
        digitalWrite(LEDlPIN, HIGH);
        delay(lDelay);
    }
}
