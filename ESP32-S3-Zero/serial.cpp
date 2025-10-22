#include "serial.h"
#include <Arduino.h>

// This file implements a flexible serial I/O mapping for the Altair emulator.
// The mapping is controlled by the CONSOLE_IS_2SIO macro defined in the main .ino file.
//
// --- Configuration Summary ---
// - CONSOLE_IS_2SIO = 1: Primary Console is 88-2SIO (Port 0x10) on USB Serial.
//                        Secondary port is 88-SIO (Port 0x00) on Serial2.
// - CONSOLE_IS_2SIO = 0: Primary Console is 88-SIO (Port 0x00) on USB Serial.
//                        Secondary port is 88-2SIO (Port 0x10) on Serial2.
#define CONSOLE_IS_2SIO 1

// --- Private Module Variables ---
static uart_device_t sio_port_10; // Emulates the 88-2SIO card on ports 10H/11H
static uart_device_t sio_port_00; // Emulates the 88-SIO card on ports 00H/01H

void serial_init(void) {
    sio_port_10.ustate = 0x82; // TDRE is high (ready), RDRF is low (not ready)
    sio_port_00.ustate = 0x0D; // TDRE is high, RDRF is high (not ready)
}

int serial_update(void) {
    #if CONSOLE_IS_2SIO == 1
        // --- CONFIGURATION: 88-2SIO is Primary Console ---
        // Primary Console (88-2SIO, Port 0x10) is mapped to USB Serial for input.
        if (Serial.available() && !(sio_port_10.ustate & (1 << RDRF))) {
            sio_port_10.rxdata = Serial.read();
            sio_port_10.ustate |= (1 << RDRF); // Set RDRF bit (data ready)
        }
        // Secondary Port (88-SIO, Port 0x00) is mapped to Serial2 for input.
        if (Serial2.available() && (sio_port_00.ustate & (1 << RDRF))) {
            sio_port_00.rxdata = Serial2.read();
            sio_port_00.ustate &= ~(1 << RDRF); // Clear RDRF bit (data ready)
        }
    #else
        // --- CONFIGURATION: 88-SIO is Primary Console ---
        // Primary Console (88-SIO, Port 0x00) is mapped to USB Serial for input.
        if (Serial.available() && (sio_port_00.ustate & (1 << RDRF))) {
            sio_port_00.rxdata = Serial.read();
            sio_port_00.ustate &= ~(1 << RDRF); // Clear RDRF bit (data ready)
        }
        // Secondary Port (88-2SIO, Port 0x10) is mapped to Serial2 for input.
        if (Serial2.available() && !(sio_port_10.ustate & (1 << RDRF))) {
            sio_port_10.rxdata = Serial2.read();
            sio_port_10.ustate |= (1 << RDRF);
            return 1; // Signal potential interrupt for main loop
        }
    #endif

    return 0; // No interrupt generated
}

// --- I/O Handlers for 88-2SIO (Ports 10H/11H) ---
uint8_t serial_port10_in(void) {
    return sio_port_10.ustate;
}

uint8_t serial_port11_in(void) {
    sio_port_10.ustate &= ~(1 << RDRF); // CRITICAL: Reading data resets the Ready flag.
    return sio_port_10.rxdata;
}

void serial_port10_out(uint8_t vale) {
    sio_port_10.ucontrol = vale;
}

void serial_port11_out(uint8_t vale) {
    #if CONSOLE_IS_2SIO == 1
      // 88-2SIO is primary: Output to main USB Serial.
      Serial.write(vale & 0x7F);
    #else
      // 88-2SIO is secondary: Output to hardware Serial2.
      Serial2.write(vale & 0x7F);
    #endif
}

// --- I/O Handlers for 88-SIO (Ports 00H/01H) ---
uint8_t serial_port00_in(void) {
    return sio_port_00.ustate;
}

uint8_t serial_port01_in(void) {
    sio_port_00.ustate |= (1 << RDRF); // CRITICAL: Reading data resets the Ready flag.
    return sio_port_00.rxdata;
}

void serial_port00_out(uint8_t vale) {
    sio_port_00.ucontrol = vale;
}

void serial_port01_out(uint8_t vale) {
    #if CONSOLE_IS_2SIO == 1
      // 88-SIO is secondary: Output to hardware Serial2.
      Serial2.write(vale & 0x7F);
    #else
      // 88-SIO is primary: Output to main USB Serial.
      Serial.write(vale & 0x7F);
    #endif
}