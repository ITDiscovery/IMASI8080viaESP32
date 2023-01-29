#include <wiringPi.h>

#define LEDdPIN  27
#define LEDlPIN  28
#define LEDcPIN  29
#define SWdPIN  24
#define SWlPIN  23
#define SWcPIN  25
#define MSBFIRST  1

void shiftOut(uint8_t dpin,uint8_t cpin,uint8_t order,uint8_t idata) {
  //MSBOrder is 1 always for us
  for(int i=7; i > -1; i--){
    gpioWrite(cpin,PI_LOW);
    gpioWrite(dpin, (idata >> i) & 0x01 );
    gpioWrite(cpin,PI_HIGH);
  }
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
    uint16_t status = 0xA5A5;
    uint8_t data = 0xA5;
    uint16_t bus = 0xA5A5;
    //uint16_t *bus_switches input from panel
    //uint16_t *cmd_switches input from panel

    while (1) {
       gpioWrite(LEDlPIN, PI_LOW);
       // Now push data to 74HC595
       shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,status >> 8);
       shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,bus >> 8);
       shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,bus);
       shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,status);
       shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,data);
       // take the latch pin high so the LEDs will light up:
       gpioWrite(LEDlPIN, PI_HIGH);
       gpioDelay(500000);
      status = rand() % 65536;
      data = rand() % 256;
      bus = rand() % 65536;
    }

}
