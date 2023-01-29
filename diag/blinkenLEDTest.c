#include <stdint.h>
#include <stdlib.h>
#include <wiringPi.h>

#define LEDdPIN  27
#define LEDlPIN  28
#define LEDcPIN  29
#define SWdPIN  24
#define SWlPIN  23
#define SWcPIN  25
#define MSBFIRST 1
#define lDelay 5

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

int main ()
{
    rpi_init();
    uint16_t status = 0xA5A5;
    uint8_t data = 0xA5;
    uint16_t bus = 0xA5A5;
    //uint16_t *bus_switches input from panel
    //uint16_t *cmd_switches input from panel

    //uint8_t shiftIn (uint8_t dPin, uint8_t cPin, uint8_t order) ;
    //void shiftOut (uint8_t dPin, uint8_t cPin, uint8_t order, uint8_t val) ;

    while (1) {
       digitalWrite(LEDlPIN, LOW);
       // Now push data to 74HC595
       shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,status >> 8);
       shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,bus >> 8);
       shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,bus);
       shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,status);
       shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,data);
       // take the latch pin high so the LEDs will light up:
       digitalWrite(LEDlPIN, HIGH);
       delay(1000);
       status = rand() % 65536;
       data = rand() % 256;
       bus = rand() % 65536;
    }
}
