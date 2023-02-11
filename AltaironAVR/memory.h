#ifndef _MEMORY_H_
#define _MEMORY_H_

#include "types.h"

extern uint8_t cmd_switches;
extern uint16_t bus_switches;
extern uint16_t bus_status;

#ifdef __AVR__AVR128__

inline uint8_t read8(uint16_t address)
{
  unsigned char retval = 0;

  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x03);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,((address >> 8) & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 255));
  retval = shiftIn(RAMInPin,RAMClkPin,MSBFIRST);

  digitalWrite(RAMCS, HIGH);
  return retval;
}

inline void write8(uint16_t address, uint8_t vale)
{
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x02);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,((address >> 8) & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 255));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,vale);
  digitalWrite(RAMCS, HIGH);
  return;
}

uint16_t read16(uint16_t address)
{
  // MSB in the lower byte
  uint8_t pgaddr = (address >> 8) & 0xFF;
  // See if this is a read across a page
  if ((tmp16addr &  0x00FF) == 0xFF) pgaddr++; 

  unsigned char bretval = 0;
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x03);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 0x00FF));
  bretval = shiftIn(RAMInPin,RAMClkPin,MSBFIRST);
  digitalWrite(RAMCS, HIGH);

  unsigned short int retval = bretval << 8;
  //delayMicroseconds(Cdelay);
  
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x03);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 0x00FF)+1);
  bretval = shiftIn(RAMInPin,RAMClkPin,MSBFIRST);

  digitalWrite(RAMCS, HIGH);
  retval = retval + bretval;
  return retval;
}

void write16(uint16_t address, uint16_t vale)
{
  // MSB in the lower byte
  uint8_t pgaddr = (address >> 8) & 0xFF;
  // See if this is a write across a page
  if ((tmp16addr &  0x00FF) == 0xFF) pgaddr++; 

  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x02);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 0xFF));
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(vale >> 8));
  digitalWrite(RAMCS, HIGH);

  delayMicroseconds(Cdelay);
  digitalWrite(RAMCS, LOW);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,0x02);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,pgaddr);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(address & 0x00FF)+1);
  shiftOut(RAMOutPin,RAMClkPin,MSBFIRST,(vale & 0x00FF));
  digitalWrite(RAMCS, HIGH);x
}

#endif //__AVR__AVR128__

#include "pi_panel.h"

extern uint8_t memory[64*1024];

uint8_t read8(uint16_t address)
{
	uint8_t data;
        if(address < 64*1024)
                data =  memory[address];
	else
		data = 0;
        return data;
}

void write8(uint16_t address, uint8_t val)
{
        if(address < 64*1024)
                memory[address] = val;
}

uint16_t read16(uint16_t address)
{
        uint16_t result = 0;
        result = read8(address);
        result |= read8(address+1) << 8;
        return result;
}

void write16(uint16_t address, uint16_t val)
{
        write8(address, val & 0xff);
        write8(address+1, (val >> 8) & 0xff);
}

#endif //_MEMORY_H_
