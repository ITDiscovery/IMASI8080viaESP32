#include <wiringPi.h>
#include <stdbool.h>
#include "types.h"
#include "pi_panel.h"

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

void rpi_init()
{
	wiringPiSetup ();
	pinMode(LEDlPIN,OUTPUT);
	pinMode(LEDlPIN,OUTPUT);
	pinMode(LEDcPIN,OUTPUT);
	pinMode(SWdPIN,INPUT);
	pinMode(SWcPIN,OUTPUT);
	pinMode(SWlPIN,OUTPUT);
}

void read_write_panel(uint16_t status, uint8_t data, uint16_t bus, uint16_t *bus_switches, uint16_t *cmd_switches, uint8_t write)
{
    // status (byte, but will likely need to be a word) = 0
    // data (byte) = cpu.data_bus
    // bus (word) = cpu.address_bus
    // *bus _switches (word) = &bus_switches
    // *cmd_switches (byte, but will likely need to be a word)  &cmd_switches
    // write (byte) = 1

	if(write) { // Write data to the 74HCT595s for the LEDs
       // Take the latchPin low so the LEDs don't change while you're sending in bits:
        digitalWrite(LEDlPIN, LOW);
       // Now push data to 74HCT595s
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,status >> 8);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,bus);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,bus >> 8);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,status);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,data);
        // Take the latchPin high to display the LEDs
        digitalWrite(LEDlPIN, HIGH);
    }

    //Read from the 74HCT165s to get the Switches
    //Write Pulse to Latch Pin
    digitalWrite(SWlPIN, LOW);
    delay(lDelay);
    digitalWrite(SWlPIN, HIGH);
    delay(lDelay);

    // Now get data from 74HC165: AL, AH, CL, CH
    uint8_t al = shiftIn(SWdPIN,SWcPIN,LSBFIRST);
    uint8_t ah = shiftIn(SWdPIN,SWcPIN,LSBFIRST);
    uint8_t cl = shiftIn(SWdPIN,SWcPIN,LSBFIRST);
    uint8_t ch = shiftIn(SWdPIN,SWcPIN,LSBFIRST);

    *bus_switches = (ah<<8) + al;
    *cmd_switches = (ch<<8) + cl;
}
