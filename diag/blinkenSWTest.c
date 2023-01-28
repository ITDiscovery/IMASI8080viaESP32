#include <pigpio.h>
#include <stdbool.h>
#include <stdio.h>

#define LEDdPIN  16
#define LEDlPIN  20
#define LEDcPIN  21
#define SWdPIN  19
#define SWlPIN  13
#define SWcPIN  26
#define MSBFIRST 1
#define LSBFIRST 0

void shiftOut(uint8_t dpin,uint8_t cpin,uint8_t order,uint8_t idata) {
  //MSBOrder is 1 always for us
  for(int i=7; i > -1; i--){
    gpioWrite(cpin,PI_LOW);
    gpioWrite(dpin, (idata >> i) & 0x01 );
    gpioWrite(cpin,PI_HIGH);
  }
}

uint8_t shiftIn(uint8_t dpin,uint8_t cpin,uint8_t order) {
    //LSBOrder is 0 always for us
    uint8_t retVal = 0;
    bool bitVal = 0;
    for(int i=0; i < 8; i++ ){
        bitVal = gpioRead(dpin);
        retVal |= (bitVal << (7 - i));
        gpioWrite(cpin,PI_HIGH);
        gpioDelay(100);
        gpioWrite(cpin,PI_LOW);
    }
    return retVal;
}

void rpi_init()
{
    gpioInitialise();     //Setup the library
    gpioSetMode(LEDlPIN,PI_OUTPUT);
    gpioSetMode(LEDlPIN,PI_OUTPUT);
    gpioSetMode(LEDcPIN, PI_OUTPUT);
    gpioSetMode(SWdPIN, PI_INPUT);
    gpioSetMode(SWcPIN, PI_INPUT);
    gpioSetMode(SWlPIN, PI_OUTPUT);

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
        gpioWrite(SWlPIN,PI_LOW);
        gpioDelay(50);
        gpioWrite(SWlPIN,PI_HIGH);
        gpioDelay(50);
        // Get Address Low, Address High, Control Low, Control High
        bus_switches = shiftIn(SWdPIN,SWcPIN,LSBFIRST);
        bus_switches = bus_switches + (shiftIn(SWdPIN,SWcPIN,LSBFIRST) << 8);
        cmd_switches = shiftIn(SWdPIN,SWcPIN,LSBFIRST);
        cmd_switches = cmd_switches + (shiftIn(SWdPIN,SWcPIN,LSBFIRST) << 8);
        gpioWrite(SWlPIN ,PI_HIGH);
        printf("Address: %x   Control: %x \n",bus_switches,cmd_switches);

        // Now push data to 74HC595
        gpioWrite(LEDlPIN, PI_LOW);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,cmd_switches >> 8);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,bus_switches);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,bus_switches >> 8);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,cmd_switches);
        shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,cmd_switches);
        gpioWrite(LEDlPIN, PI_HIGH);
        gpioDelay(50000);
    }
}
