#ifndef _PI_PANEL_H_
#define _PI_PANEL_H_

#include "types.h"

//Define pins using wiringPi numbering
#define LEDdPIN  27
#define LEDlPIN  28
#define LEDcPIN  29
#define SWdPIN   24
#define SWlPIN   23
#define SWcPIN   25
//Latch Delay
#define lDelay 5

#define MSBFIRST 1
#define LSBFIRST 0

#define RUN 1
#define STOP 2

#define STOP_CMD 0x80
#define RUN_CMD 0x40
#define SINGLE_STEP 0x0100
#define SSTEP_DOWN 0x200
#define EXAMINE 0x01
#define EXAMINE_NEXT 0x02
#define DEPOSIT 0x04
#define DEPOSIT_NEXT 0x08
#define RESET_CMD 0x10
#define CLR_CMD 0x20
#define PROTECT 0x400
#define UNPROTECT 0x800
#define AUX1_UP 0x1000
#define AUX1_DOWN 0x2000
#define AUX2_UP 0x4000
#define AUX2_DOWN 0x8000

void rpi_init();
void read_write_panel(uint8_t status, uint8_t data, uint16_t bus, uint16_t *bus_switches, uint16_t *cmd_switches, uint8_t write);

#endif
