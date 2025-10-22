#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h> // Use standard integer types

// Re-define the UART state and control enums here so this module is self-contained
enum UState { RDRF,TDRE,DCD,CTS,FERR,OVRN,PERR,IRQ };
enum UControl { CDS1,CDS2,WS1,WS2,WS3,TC1,TC2,RIE};

// Structure to hold the state for a single UART device
typedef struct {
  uint8_t ucontrol;
  uint8_t ustate;
  uint8_t rxdata;
  uint8_t txdata;
} uart_device_t;

#ifdef __cplusplus
extern "C" {
#endif

// --- Public Function Declarations ---

// Initializes the serial module and its devices
void serial_init(void);

// Can pass an interrupt back if a character comes in.
int serial_update(void);

// Handles IN instruction for Port 00H (Status)
uint8_t serial_port00_in(void);

// Handles IN instruction for Port 01H (Data)
uint8_t serial_port01_in(void);

// Handles OUT instruction for Port 00H (Control)
void serial_port00_out(uint8_t vale);

// Handles OUT instruction for Port 01H (Data)
void serial_port01_out(uint8_t vale);

// Handles IN instruction for Port 10H (Status)
uint8_t serial_port10_in(void);

// Handles IN instruction for Port 11H (Data)
uint8_t serial_port11_in(void);

// Handles OUT instruction for Port 10H (Control)
void serial_port10_out(uint8_t vale);

// Handles OUT instruction for Port 11H (Data)
void serial_port11_out(uint8_t vale);

#ifdef __cplusplus
}
#endif

#endif // SERIAL_H