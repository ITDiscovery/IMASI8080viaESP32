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

#define EXAMINE 0x01
#define EXAMINE_NEXT 0x02
#define DEPOSIT 0x04
#define DEPOSIT_NEXT 0x08
#define RESET_CMD 0x10
#define CLR_CMD 0x20
#define RUN_CMD 0x40
#define STOP_CMD 0x80
#define SINGLE_STEP 0x0100
#define SSTEP_DOWN 0x200
#define PROTECT 0x400
#define UNPROTECT 0x800
#define AUX1_UP 0x1000
#define AUX1_DOWN 0x2000
#define AUX2_UP 0x4000
#define AUX2_DOWN 0x8000

#define INTA 0x01
#define WO 0x02
#define STACK 0x04
#define HLTA 0x08
#define IOUT 0x10
#define MI 0x20
#define INP 0x40
#define MEMR 0x80
#define INTEN 0x100
#define RUNM 0x200
#define WAIT 0x400
#define HOLD 0x800
#define USR4 0x1000
#define USR3 0x2000
#define USR2 0x4000
#define USR1 0x8000

void rpi_init();
void read_write_panel(uint16_t status, uint8_t data, uint16_t bus, uint16_t *bus_switches, uint16_t *cmd_switches, uint8_t write);

#endif
