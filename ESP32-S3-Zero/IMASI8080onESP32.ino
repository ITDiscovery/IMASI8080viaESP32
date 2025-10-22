// This file assumes a microcontroller environment (like ESP32/Arduino)
// where 'byte', 'boolean', 'digitalWrite', 'digitalRead', and 'delayMicroseconds' are available.
// This is specifically for a Waveshare ESP32-S3-Mini
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

extern "C" {
  #include "i8080.h"
}
#include "serial.h"
#include "disk.h"

// Replace with your network credentials
const char* ssid = "skylernet";
const char* password = "tilt0waitsplunDger";

//#define DEBUG
//#define DEBUGIO

//Pin Connection Defines
//Pin connected to RClk (Pin 12) of 74HC595
#define LlatchPin 2
//Pin connected to SRCk (Pin 11) of 74HC595
#define LclockPin 3
////Pin connected to SER (Pin 14) of 1st 74HC595
#define LdataPin 1
//Pin connected to PL! (Pin 1) of 74HCT165
#define SlatchPin 6
//Pin connected to CP (Pin 2) of 74HCT165
#define SclockPin 5
//Pin connected to Q7 (Pin 9) of 1st 74HCT165
#define SdataPin 4

//14 is Serial RX
#define Serial2RX 14
//15 is Serial TX
#define Serial2TX 15

//Pin 9 is AnalogOut
#define AnalogOut 7

// SD Card Setup
// Make sure pins_arduino.h in ~/Library/Arduino15/packages/esp32/hardware/esp32/3.1.1/variants/waveshare_esp32_s3_zero
#define SD_CS_PIN 10     // Chip Select
#define SD_CLK_PIN 12    // SCK (Clock)
#define SD_MOSI_PIN 11   // MOSI (Master Out Slave In)
#define SD_MISO_PIN 13   // MISO (Master In Slave Out) 

// Switch Clock and Latch Delay
#define Cdelay 4
#define Ldelay 1
#define KeyDelay 20

// GPIO 21 is BUILTIN_LED (from your existing code) and WS_RGB
// We define the parameters needed for the NeoPixel object:
#define LED_COUNT 1 

Adafruit_NeoPixel pixel(LED_COUNT, BUILTIN_LED, NEO_GRB + NEO_KHZ800);

//IMASI 8080
enum Control { EXAMINE, EXAMINE_NEXT, DEPOSIT, DEPOSIT_NEXT, 
    RESET, CLR, RUN, STOP, SINGLE_STEP, SINGLE_STEP_, 
    PROTECT, UNPROTECT, AUX1_UP, AUX1_DOWN, AUX2_UP, AUX2_DOWN };
// Funciton locks
// Global single-shot flags for momentary switch debounce/logic protection
bool examine_lock = false;
bool examine_next_lock = false;
bool deposit_lock = false;
bool deposit_next_lock = false;
bool reset_lock = false;
bool clr_lock = false;
bool single_step_lock = false;
bool aux1_up_lock = false;
bool aux1_down_lock = false;
bool aux2_up_lock = false;
bool aux2_down_lock = false;
// CRITICAL FIX: These new internal flags replace the USER3 and USER4 status bits.
bool continuousLedUpdates = true; // Replaces USER3 (starts ON)
bool continuousDebugOutput = false; // Replaces USER4 (starts OFF)

// --- DEBUG & BREAKPOINT CONFIGURATION ---
bool cpu_halted_by_breakpoint = false;
// Use an enum for clarity instead of magic numbers (0=A, 1=B, etc.)
enum BreakpointRegister { BP_REG_A, BP_REG_B, BP_REG_C, BP_REG_D, BP_REG_E, BP_REG_H, BP_REG_L };
BreakpointRegister bp_target_register = BP_REG_A;
uint8_t bp_target_value = 0x00;

// Debug Mode
bool debugStepModeActive = false; // Toggled by the SINGLE_STEP_ button

struct {
  uint16_t address;
  uint16_t control,prev_control;
} switches;

Bus bus;

// Size is 26
// Double Loop to show performance of emulation
const byte BenchMark[] PROGMEM = {
    0x01, 0x00, 0x00, // LXI B, 0000H
    0x06, 0xFF,       // MVI B, FFH
    0x0E, 0xFF,       // MVI C, FFH
    0x00,             // NOP (0007H)
    0x0D,             // DCR C
    0x79,             // MOV A, C     <-- FIX: Ensures ZF is set correctly
    0xB7,             // ORA A        <-- FIX: Ensures ZF is set correctly
    0xC2, 0x07, 0x00, // JNZ 0007H    <-- Jump Address for inner loop
    0x05,             // DCR B
    0xC2, 0x05, 0x00, // JNZ 0005H    <-- Jump Address for outer loop
    0x3E, 0x0A,       // MVI A, 0AH
    0xDB, 0xF0,       // IN F0H       <-- Trap Instruction
    0xC3, 0x00, 0x00, // JMP 0000H
    0x00              // Final byte to ensure size is 26
};
//Tests 88-SIO Emulation (Polling)
//https://www.penguinstew.ca/Writings/60/Computers/Programming/Projects/Altair%208800/Serial%20Echo
//Size is 21
const byte Echo2SIO[] PROGMEM = {
0x3E, 0x03, 0xD3, 0x10, 0x3E, 0x15, 0xD3, 0x10, 0xDB, 0x10, 0x0F, 0xD2, 0x08, 0x00, 0xDB, 0x11, 
0xD3, 0x11, 0xC3, 0x08, 0x00
};
//Tests 88-2SIO Emulation (Interrupt)
//Sie is 35
const byte Echo2SIOInt[] PROGMEM = {
0x31, 0x00, 0x01, 0x3E, 0x03, 0xD3, 0x10, 0x3E, 0x95, 0xD3, 0x10, 0xFB, 0x00, 0x00, 0x00, 0xC3, 
0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF5, 0xDB, 0x11, 0xD3, 0x11, 0xF1, 0xFB, 0xC9 
};
// Size is 65
// Verify that a call to CONIN (BDOS Call 1) blocks and waits for a character
const byte CONIN_TEST[] = PROGMEM {
  0x2E, 0x01, 0x7D, 0xCD, 0x30, 0x01, 0x11, 0x2D, 0x01, 0x0E, 0x09, 0xCD,
  0x05, 0x00, 0xC5, 0xD5, 0x0E, 0x01, 0xCD, 0x05, 0x00, 0x47, 0xD1, 0xC1,
  0x58, 0x0E, 0x02, 0xCD, 0x05, 0x00, 0x11, 0x2A, 0x01, 0x0E, 0x09, 0xCD,
  0x05, 0x00, 0x2C, 0xC3, 0x02, 0x01, 0x0D, 0x0A, 0x24, 0x3A, 0x20, 0x24,
  0xC5, 0xD5, 0x0E, 0x30, 0xB9, 0xDA, 0x3E, 0x01, 0x5F, 0x0E, 0x02, 0xCD,
  0x05, 0x00, 0xD1, 0xC1, 0xC9
};
//Disk Boot ROM at FF00
const byte DBLROM[] PROGMEM = { 
0x21, 0x13, 0xFF, 0x11, 0x00, 0x2C, 0x0E, 0xEB, 0x7E, 0x12, 0x23, 0x13, 0x0D, 0xC2, 0x08, 0xFF,
0xC3, 0x00, 0x2C, 0xF3, 0xAF, 0xD3, 0x22, 0x2F, 0xD3, 0x23, 0x3E, 0x2C, 0xD3, 0x22, 0x3E, 0x03,
0xD3, 0x10, 0xDB, 0xFF, 0xE6, 0x10, 0x0F, 0x0F, 0xC6, 0x10, 0xD3, 0x10, 0x31, 0x79, 0x2D, 0xAF,
0xD3, 0x08, 0xDB, 0x08, 0xE6, 0x08, 0xC2, 0x1C, 0x2C, 0x3E, 0x04, 0xD3, 0x09, 0xC3, 0x38, 0x2C,
0xDB, 0x08, 0xE6, 0x02, 0xC2, 0x2D, 0x2C, 0x3E, 0x02, 0xD3, 0x09, 0xDB, 0x08, 0xE6, 0x40, 0xC2,
0x2D, 0x2C, 0x11, 0x00, 0x00, 0x06, 0x00, 0x3E, 0x10, 0xF5, 0xD5, 0xC5, 0xD5, 0x11, 0x86, 0x80,
0x21, 0xEB, 0x2C, 0xDB, 0x09, 0x1F, 0xDA, 0x50, 0x2C, 0xE6, 0x1F, 0xB8, 0xC2, 0x50, 0x2C, 0xDB,
0x08, 0xB7, 0xFA, 0x5C, 0x2C, 0xDB, 0x0A, 0x77, 0x23, 0x1D, 0xCA, 0x72, 0x2C, 0x1D, 0xDB, 0x0A,
0x77, 0x23, 0xC2, 0x5C, 0x2C, 0xE1, 0x11, 0xEE, 0x2C, 0x01, 0x80, 0x00, 0x1A, 0x77, 0xBE, 0xC2,
0xCB, 0x2C, 0x80, 0x47, 0x13, 0x23, 0x0D, 0xC2, 0x79, 0x2C, 0x1A, 0xFE, 0xFF, 0xC2, 0x90, 0x2C,
0x13, 0x1A, 0xB8, 0xC1, 0xEB, 0xC2, 0xC2, 0x2C, 0xF1, 0xF1, 0x2A, 0xEC, 0x2C, 0xCD, 0xE5, 0x2C,
0xD2, 0xBB, 0x2C, 0x04, 0x04, 0x78, 0xFE, 0x20, 0xDA, 0x44, 0x2C, 0x06, 0x01, 0xCA, 0x44, 0x2C,
0xDB, 0x08, 0xE6, 0x02, 0xC2, 0xAD, 0x2C, 0x3E, 0x01, 0xD3, 0x09, 0xC3, 0x42, 0x2C, 0x3E, 0x80,
0xD3, 0x08, 0xC3, 0x00, 0x00, 0xD1, 0xF1, 0x3D, 0xC2, 0x46, 0x2C, 0x3E, 0x43, 0x01, 0x3E, 0x4D,
0xFB, 0x32, 0x00, 0x00, 0x22, 0x01, 0x00, 0x47, 0x3E, 0x80, 0xD3, 0x08, 0x78, 0xD3, 0x01, 0xD3,
0x11, 0xD3, 0x05, 0xD3, 0x23, 0xC3, 0xDA, 0x2C, 0x7A, 0xBC, 0xC0, 0x7B, 0xBD, 0xC9, 0x00, 0x00
};
//Disk Boot ROM at FD00
const byte UBMON[] PROGMEM = { 
0x3E, 0x03, 0xD3, 0x10, 0x3E, 0x11, 0xD3, 0x10, 0x31, 0x00, 0xFC, 0x01, 0x08, 0xFD, 0xC5, 0xCD,
0xA1, 0xFD, 0x3E, 0x2E, 0xCD, 0xF4, 0xFD, 0x21, 0x00, 0xFC, 0xCD, 0xE8, 0xFD, 0xD6, 0x4A, 0xCA,
0x81, 0xFD, 0x80, 0xCC, 0xAC, 0xFD, 0xCA, 0x86, 0xFD, 0x0E, 0x7F, 0xC5, 0x3C, 0xC8, 0x26, 0xFF,
0x82, 0xC8, 0x25, 0xFE, 0x12, 0xC8, 0xC1, 0x84, 0xC0, 0x5D, 0xCD, 0xAC, 0xFD, 0xEB, 0xCD, 0xAC,
0xFD, 0x23, 0x47, 0xCD, 0x75, 0xFD, 0xAF, 0xCD, 0x75, 0xFD, 0x05, 0x7D, 0x93, 0x4F, 0x7C, 0x9A, 
0xC2, 0x54, 0xFD, 0x41, 0xD5, 0x1E, 0x3C, 0x50, 0xCD, 0xA4, 0xFD, 0xD1, 0xCD, 0xA4, 0xFD, 0x83,
0x4F, 0x1A, 0xCD, 0xF4, 0xFD, 0x81, 0x13, 0x05, 0xC2, 0x60, 0xFD, 0xCD, 0xF4, 0xFD, 0x7B, 0x95,
0x7A, 0x9C, 0xDA, 0x4A, 0xFD, 0x06, 0x3C, 0xCD, 0xF4, 0xFD, 0x05, 0xC2, 0x77, 0xFD, 0xC9, 0x34,
0xC8, 0xCC, 0xAC, 0xFD, 0xE9, 0x23, 0xCD, 0xA1, 0xFD, 0xE5, 0xCD, 0xC8, 0xFD, 0xCD, 0xAD, 0xFD,
0x7D, 0x21, 0x85, 0xFD, 0xE3, 0xD0, 0x77, 0xBE, 0xC8, 0x3E, 0x3F, 0xCD, 0xF4, 0xFD, 0xC3, 0x08,
0xFD, 0x11, 0x0D, 0x0A, 0x7B, 0xCD, 0xF4, 0xFD, 0x7A, 0xC3, 0xF4, 0xFD, 0x06, 0x06, 0x03, 0x65,
0xCD, 0xE8, 0xFD, 0xC8, 0xD6, 0x30, 0xFE, 0x08, 0xD2, 0x99, 0xFD, 0x29, 0x29, 0x29, 0xB5, 0x6F,
0x05, 0xC2, 0xB0, 0xFD, 0x37, 0xC3, 0xE3, 0xFD, 0x4E, 0x06, 0x06, 0xAF, 0xCD, 0xD4, 0xFD, 0xAF,
0x06, 0x03, 0x29, 0x17, 0x29, 0x17, 0xC6, 0x30, 0xCD, 0xF4, 0xFD, 0xAF, 0x29, 0x17, 0x05, 0xC2, 
0xD2, 0xFD, 0x61, 0x3E, 0x20, 0xC3, 0xF4, 0xFD, 0xDB, 0x10, 0x0F, 0xD2, 0xE8, 0xFD, 0xDB, 0x11,
0xE6, 0x7F, 0xFE, 0x20, 0xF5, 0xDB, 0x10, 0xE6, 0x02, 0xCA, 0xF5, 0xFD, 0xF1, 0xD3, 0x11, 0xC9
};
//Turnkey Monitor ROM at FD00
const byte TURNMON[] PROGMEM = { 
0x3E, 0x03, 0xD3, 0x10, 0x3E, 0x11, 0xD3, 0x10, 0x31, 0x00, 0xFC, 0xCD, 0x96, 0xFD, 0xCD, 0x96,
0xFD, 0x3E, 0x2E, 0xCD, 0xF2, 0xFD, 0xCD, 0xE8, 0xFD, 0xFE, 0x4D, 0xCA, 0x2C, 0xFD, 0xFE, 0x44,
0xCA, 0x54, 0xFD, 0xFE, 0x4A, 0xC2, 0x08, 0xFD, 0xCD, 0xA0, 0xFD, 0xE9, 0xCD, 0xA0, 0xFD, 0xC3,
0x33, 0xFD, 0x23, 0xCD, 0x96, 0xFD, 0x54, 0x5D, 0xCD, 0xC6, 0xFD, 0x1A, 0x67, 0xCD, 0xCC, 0xFD, 
0xCD, 0xA5, 0xFD, 0xEB, 0xDA, 0x32, 0xFD, 0x77, 0xBE, 0xCA, 0x32, 0xFD, 0x3E, 0x3F, 0xCD, 0xF2,
0xFD, 0xC3, 0x08, 0xFD, 0xCD, 0xA0, 0xFD, 0xEB, 0xD4, 0xE3, 0xFD, 0xCD, 0xA0, 0xFD, 0xE5, 0x62,
0x6B, 0xCD, 0x96, 0xFD, 0xCD, 0xC6, 0xFD, 0xCD, 0xE3, 0xFD, 0x01, 0x10, 0x00, 0x1A, 0x67, 0xC5,
0x3E, 0x08, 0xB9, 0xC2, 0x7E, 0xFD, 0x3E, 0x2D, 0xCD, 0xF2, 0xFD, 0xCD, 0xE3, 0xFD, 0xCD, 0xCC,
0xFD, 0xC1, 0xE1, 0x7C, 0xBA, 0xC2, 0x8D, 0xFD, 0x7D, 0xBB, 0xCA, 0x08, 0xFD, 0xE5, 0x13, 0x0D,
0xC2, 0x6D, 0xFD, 0xC3, 0x5F, 0xFD, 0x3E, 0x0D, 0xCD, 0xF2, 0xFD, 0x3E, 0x0A, 0xC3, 0xF2, 0xFD,
0x06, 0x06, 0xC3, 0xA7, 0xFD, 0x06, 0x03, 0x21, 0x00, 0x00, 0xCD, 0xE8, 0xFD, 0x4F, 0xFE, 0x20,
0x37, 0xC8, 0xE6, 0xB8, 0xEE, 0x30, 0xC2, 0x4C, 0xFD, 0x79, 0xE6, 0x07, 0x29, 0x29, 0x29, 0x85,
0x6F, 0x05, 0xC2, 0xAA, 0xFD, 0xC9, 0x06, 0x06, 0xAF, 0xC3, 0xD6, 0xFD, 0x06, 0x03, 0xAF, 0xC3,
0xD3, 0xFD, 0x29, 0x17, 0x29, 0x17, 0x29, 0x17, 0xE6, 0x07, 0xF6, 0x30, 0xCD, 0xF2, 0xFD, 0x05,
0xC2, 0xD2, 0xFD, 0x3E, 0x20, 0xC3, 0xF2, 0xFD, 0xDB, 0x10, 0x0F, 0xD2, 0xE8, 0xFD, 0xDB, 0x11,
0xE6, 0x7F, 0xFE, 0x20, 0xF5, 0xDB, 0x10, 0xE6, 0x02, 0xCA, 0xF5, 0xFD, 0xF1, 0xD3, 0x11, 0xC9
};
byte CPUMemory[65536];

void readPage(uint16_t tmp16addr) {
    Serial.print("0x");
    Serial.print(tmp16addr, HEX);
    Serial.print(": ");
    for (unsigned long i = tmp16addr; i < (tmp16addr + 0x200); i++) {
      if (CPUMemory[i] < 0x10) {
        Serial.print("0");
      }
      Serial.print(CPUMemory[i],HEX);
      if ((i & 31) != 31) {
        // Print 32 bytes per line, space between them, (31 spaces, fence post)
        Serial.print(" ");
      } else {
        // Last one of a line gets a newline printed after it instead.
        Serial.println();
        Serial.print("0x");
        Serial.print(i+1,HEX);
        Serial.print(": ");
      }
    }
    Serial.println();
}

void CPUStatus() {
  Serial.print("PC:");
  Serial.print(i8080_pc(),HEX);
  Serial.print(" BUS:");         
  Serial.print(bus.address,HEX);
  Serial.print("  SP:");
  Serial.print(i8080_regs_sp(),HEX);        
  Serial.print(" Bus Data:");
  Serial.print(bus.data,HEX);
  Serial.print(" Memory Data:");
  Serial.print(CPUMemory[i8080_pc()],HEX);
  Serial.print(" AF:");
  Serial.print(i8080_regs_af(),HEX);
  Serial.print(" BC:");
  Serial.print(i8080_regs_bc(),HEX);
  Serial.print("  DE:");
  Serial.print(i8080_regs_de(),HEX);
  Serial.print("  HL:");
  Serial.print(i8080_regs_hl(),HEX);
    Serial.println();
}

void writeLEDs() {
  // take the latchPin low so
  // the LEDs don't change while you're sending in bits:
  digitalWrite(LlatchPin, LOW);
  // Now push data to 74HC595
  // Shift order: Status (High) -> Status (Low) -> Address (High) -> Address (Low) -> Data
  
  // R1: Status High (Bits 15-8).
  // Mask out USER3 (bit 13) and USER4 (bit 12) positions.
  // The mask 0xCF (1100 1111b) clears bits 5 and 4 of the high byte, ensuring they are always OFF.
  uint8_t status_high = bus.state >> 8;
  status_high = status_high & 0xCF; 
  shiftOut(LdataPin,LclockPin,MSBFIRST,status_high); 

  // R2: Status Low (Bits 7-0)
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.state);
  // R3: Address High (Bits 15-8)
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.address >> 8);

  // R4: Address Low (Bits 7-0)
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.address);
  // R5: Data (8 bits)
  shiftOut(LdataPin,LclockPin,MSBFIRST,bus.data);
  
  // take the latch pin high so the LEDs will light up:
  digitalWrite(LlatchPin, HIGH);
}

void rgbLedWrite(int pin, uint8_t red, uint8_t green, uint8_t blue) {
    // The 'pin' argument is ignored as 'pixel' controls the single onboard LED.
    // Set the color for the single NeoPixel (LED index 0)
    pixel.setPixelColor(0, pixel.Color(red, green, blue));
    // Push the color data to the LED
    pixel.show();
}

// -----------------------------------------------------------------------------
// CORE ROUTINE: Read a single 8-bit byte from the shift register chain
// -----------------------------------------------------------------------------

/**
 * @brief Reads one 8-bit byte from the serial data line (MSB first).
 *
 * @param dataPin The pin connected to the serial output (Qh).
 * @param clockPin The pin connected to the clock.
 * @return byte The assembled 8-bit value.
 */
byte readByte(int dataPin, int clockPin) {
    byte result = 0;
    // Use 'int' for digitalRead result to avoid common type conversion warnings
    int bitVal;
    for (int i = 0; i < 8; i++) {
        // 1. Read the current bit value (0 or 1)
        bitVal = digitalRead(dataPin);
        // 2. Assemble the byte: The first bit read (i=0) goes into position 7 (MSB).
        result |= ((byte)bitVal << (7 - i)); 
        
        // 3. Pulse the clock HIGH/LOW to shift the next bit
        digitalWrite(clockPin, HIGH);
        delayMicroseconds(Cdelay); 
        digitalWrite(clockPin, LOW);
    }
    return result;
}

void readSwitches() {
   byte al, ah, cl, ch;
   // 1. LATCH PHASE: Pulse the Latch pin LOW (PL!) to parallel load switch states.
   digitalWrite(SlatchPin, LOW);
   delayMicroseconds(Ldelay);
   digitalWrite(SlatchPin, HIGH);
   delayMicroseconds(Ldelay);

  // 2. SHIFT PHASE: Read 4 bytes sequentially (32 bits total)
  // Read order: AL -> AH -> CL -> CH
  al = readByte(SdataPin, SclockPin); // Address Low (A0-A7)
  ah = readByte(SdataPin, SclockPin); // Address High (A8-A15)
  cl = readByte(SdataPin, SclockPin); // Control Low (D0-D7, includes EXAMINE/DEPOSIT)
  ch = readByte(SdataPin, SclockPin); // Control High (e.g., RUN/STOP, RESET)
  
  // 3. Update the global state
  switches.prev_control = switches.control; // Save previous control value

  // Assemble the 16-bit address word
  switches.address = ((unsigned int)ah << 8) | al;

  // Assemble the 16-bit control word (Active High, NO INVERSION)
  switches.control = ((unsigned int)ch << 8) | cl;
}

bool onRelease(int c) {
  return (switches.prev_control & (1<<c)) > 0 && (switches.control & (1<<c)) == 0;
}

bool isDown(int c) {
  return switches.control & (1<<c);
}

byte sense() {
  return switches.address >> 8;
}

extern "C" {
  //uint8_t Flash.writeWord(uint32_t address, uint16_t data)
  //uint8_t Flash.writeByte(uint32_t address, uint8_t data)
  //uint8_t Flash.erasePage(uint32_t address, uint8_t size = 1);
  unsigned char i8080_hal_memory_read_byte(short unsigned int tmp16addr) {    
     bitSet(bus.state,MEMR);
     bitClear(bus.state,WO);  
     return CPUMemory[tmp16addr];
  }

  void i8080_hal_memory_write_byte(unsigned short int tmp16addr, unsigned char vale) {
  bitSet(bus.state,MEMR);
  bitSet(bus.state,WO);
  CPUMemory[tmp16addr] = vale;
}

  unsigned short int i8080_hal_memory_read_word(unsigned short int tmp16addr) {
     #ifdef DEBUG
     Serial.print(" Read Word ");
     Serial.print(tmp16addr,HEX);
     Serial.print("=");
     Serial.print((CPUMemory[tmp16addr+1] << 8) + CPUMemory[tmp16addr],HEX);
     #endif
     bitSet(bus.state,MEMR);
     bitClear(bus.state,WO);
     return (CPUMemory[tmp16addr+1] << 8) + CPUMemory[tmp16addr];
  }

  void i8080_hal_memory_write_word(unsigned short int tmp16addr, unsigned short int vale) {
     #ifdef DEBUG
     Serial.print("Write Word ");
     Serial.print(tmp16addr,HEX);
     Serial.print("=");
     Serial.print(vale,HEX);
     #endif
     bitSet(bus.state,MEMR);
     bitSet(bus.state,WO);     
     CPUMemory[tmp16addr+1] = vale >> 8;
     CPUMemory[tmp16addr] = vale;
     return;
  }
  
// Copyright (C) 2020 David Hansel
// Ports used by emulated devices (ports with "*" are available on Arduino MEGA)
// 00-01  * MITS 88-SIO
// 02     * printer (OkiData, C700 or generic)
// 03     * printer (OkiData, C700 or generic, output only)
// 04       Cromemco disk controller
// 04-05    ProTec VDM1 keyboard (input only)
// 06-07  * MITS 88-ACR
// 08-0A    MITS 88-DCDD disk controller
// 0E       Cromemco Dazzler
// 0F       Cromemco Dazzler (output only)
// 10-11  * MITS 88-2SIO port A
// 12-13  * MITS 88-2SIO port B
// 14-15  * second MITS 88-2SIO port A
// 16-17  * second MITS 88-2SIO port B
// 18       Cromemco D+7A board (Dazzler support, input only)
// 19-1C    Cromemco D+7A board (Dazzler support)
// 1D-1F    Cromemco D+7A board (Dazzler support, output only)
// 30-34    Cromemco disk controller
// 40       Cromemco disk controller (output only)
// A0-A8    MITS hard disk controller (4PIO interface board)
// C8       ProTec VDM-1 (output only)
// F0       Cromemco disk controller (input only)
// F8-FD    Tarbell disk controller
// FE     * MITS 88-VI interrupt controller (output only)
// FF     * front-panel sense switches (input only)

  // From i8080_hal
  unsigned char i8080_hal_io_input(unsigned char port) {

    #ifdef DEBUGIO
    Serial.print("In Port:");
    Serial.print(port,HEX);
    Serial.print(" Data:");
    Serial.println(i8080_regs_a(),HEX);
    #endif

    bitSet(bus.state,INP);
    bitClear(bus.state,IOUT);
    bitClear(bus.state,WO);
    bitClear(bus.state,MEMR);

    unsigned int seek;
    unsigned char ret_val;
    switch (port) {
      case 0x00:  // 88-SIO Status -> Serial2 (handled by serial module)
       return serial_port00_in();
      case 0x01: // 88-SIO Data -> Serial2 (handled by serial module)
        return serial_port01_in();
      case 0x02:  //printer (OkiData, C700 or generic)
        return 0xFF;
      case 0x03:  //printer (OkiData, C700 or generic)
        return 0xFF;
      case 0x04: //Protec VDM1 Keyboard??
        return 0xFF;
      case 0x05: //Protect VDM1 Keyboard??
        return 0xFF;
      case 0x06: // 88-ACR Board Cassette
        Serial.print("Cassette Port In 0x06, Data:");
        Serial.print(i8080_regs_a(),HEX);
        Serial.println();
        return 0xFF;
      case 0x07: // 88-ACR Board Cassette
        Serial.print("Cassette Port In 0x07, Data:");
        Serial.print(i8080_regs_a(),HEX);
        Serial.println();
        return 0xFF;
      case 0x08: // 88-DCDD Disk Ports
      case 0x09:
      case 0x0A:
        return disk_in(port);
      case 0x10: // 88-2SIO Status -> USBSerial (now handled by serial module)
       return serial_port10_in();
      case 0x11: // 88-2SIO Data -> USBSerial (now handled by serial module)
        return serial_port11_in();
      case 0x12: // 88-2SIO or 88-ACR port 1, Status
        return 0xFF;
      case 0x13: // 88-2SIO or 88-ACR port 1, read/RX
        return 0xFF;
      case 0x18: // 88-ACR Board I/O 2ndary
        Serial.print("Cassette Port In 0x18, Data:");
        Serial.print(i8080_regs_a(),HEX);
        Serial.println();
        return 0xFF;
      case 0x19: // 88-ACR Board I/O 2ndary
        Serial.print("Cassette Port In 0x19, Data:");
        Serial.print(i8080_regs_a(),HEX);
        Serial.println();
        return 0xFF;
      case 0x20: // 88-2SIO or 88-ACR Alternate port 0, Status --> WiFi Serial
        Serial.print("Port 20 In");
        return 0xFF;
      case 0x21: // 88-2SIO or 88-ACR Alternate port 0, read/RX --> WiFi Serial
        return 0xFF;
      case 0x22: // 88-2SIO or 88-ACR Alternate port 1, Status --> WiFi Serial
        Serial.print("Port 22 In");
        return 0xFF;
      case 0x23: // 88-2SIO or 88-ACR Alternate port 1, read/RX --> WiFi Serial
        return 0xFF;
      case 0x24:  // Altair Disk Basic Hit's this port
        return 0xFF;
      case 0x25:  // Altair Disk Basic Hit's this port
        return 0xFF;
      case 0x26:  // Altair Disk Basic Hit's this port
        return 0xFF;
      case 0x27:  // Altair Disk Basic Hit's this port
        return 0xFF;
      case 0xfe: // Get realtime from time.gov
        Serial.print("88-VI/RTC In 0xFE, Data:");
        Serial.print(i8080_regs_a(),HEX);
        Serial.println();
        return 0xFF;
      case 0xff: // Front panel switches
        //bitSet(bus.state,USER0);
        //Serial.print("Address PI: ");
        readSwitches();
        Serial.print(switches.address >> 8,HEX);
        Serial.println();
        return (switches.address >> 8);
      default:
        // --- CRITICAL TRAP FIX: GUARANTEED OUTPUT ---
        // This is the default handler for all unhandled IN ports (including 0xF0)
        Serial.print("!!!TRAP HIT: ");
        Serial.print("Port:");
        Serial.print(port, HEX);
        Serial.print(" Data:");
        Serial.println(i8080_regs_a(), HEX);
        CPUStatus();
        Serial.flush(); // FORCE output buffer clear
        return 0xFF;
    }
  }

  // From i8080_hal 
  void i8080_hal_io_output(unsigned char port, unsigned char vale) {
    //This is the Output handler
    bitClear(bus.state,INP);
    bitSet(bus.state,IOUT);
    bitSet(bus.state,WO);
    bitClear(bus.state,MEMR);

    #ifdef DEBUGIO
    Serial.print("Out Port:");
    Serial.print(port,HEX);
    Serial.print(" Data:");
    Serial.println(i8080_regs_a(),HEX);
    #endif

    //Put the data on the bus
    bus.data = vale;
    unsigned int seek;
    unsigned char ret_val;

    switch (port) {
    case 0x00: // 88-SIO Control (handled by serial module)
      serial_port00_out(vale);
      return;
    case 0x01: // 88-SIO Data (handled by serial module)
      serial_port01_out(vale);
      return;
    case 0x02:  //printer (OkiData, C700 or generic)
      return;
    case 0x03:  //printer (OkiData, C700 or generic)
      return;
    case 0x05:
      return;  // Ignore writes to the VDM1 keyboard port
    case 0x06: // 88-ACR Board Cassette
      Serial.print("Cassette Port Out 0x06, Data:");
      Serial.print(i8080_regs_a(),HEX);
      Serial.println();
    case 0x07: // 88-ACR Board Cassette
      Serial.print("Cassette Port Out 0x07, Data:");
      Serial.print(i8080_regs_a(),HEX);
      Serial.println();
    case 0x08: // 88-DCDD Disk Ports
    case 0x09:
    case 0x0A:
      return disk_out(port,vale);
    case 0x10: // 88-2SIO Control -> USBSerial (now handled by serial module)
      serial_port10_out(vale);
      return;
    case 0x11: // 88-2SIO Data -> USBSerial (now handled by serial module)
      serial_port11_out(vale);
      return;
    case 0x12: // 88-2SIO or 88-ACR port 1, Control
      return;
    case 0x13: // 88-2SIO or 88-ACR port 1, Write/TX
      return;
    case 0x18: // 88-ACR Board I/O 2ndary
    case 0x19: // 88-ACR Board I/O 2ndary
      return;
    case 0x20: // 88-2SIO or 88-ACR Alternate port 0, Control
      return;
    case 0x21: // 88-2SIO or 88-ACR Alternate port 0, Write/TX
      return;
    case 0x22: // 88-2SIO or 88-ACR Alternate port 1, Control
      return;
    case 0x23: // 88-2SIO or 88-ACR Alternate port 1, Write/TX
      return;
    case 0x24:  //Altair Disk Basic hits this port
      return;
    case 0x25:  //Altair Disk Basic hits this port
      return;
    case 0x26:  //Altair Disk Basic hits this port
      return;
    case 0x27:  //Altair Disk Basic hits this port
      return;
    case 0xFE:
      Serial.print("88-VI/RTC Out, Data:");
      Serial.print(i8080_regs_a(),HEX);
      Serial.println();
      return;
    case 0xFF: // panel LEDs
      // The high bits (USER3, USER4 positions) are handled by booleans and masked out 
      // in writeLEDs, so we only update the lower 12 status bits here.
      bus.state = (bus.state & 0x0FFF) + (vale << 12);      
      return;
    default:
      // --- CRITICAL TRAP FIX: GUARANTEED OUTPUT ---
      // This is the default handler for all unhandled OUT ports (including 0xF0)
      Serial.print("!!!TRAP HIT: ");
      Serial.print("Port:");
      Serial.print(port, HEX);
      Serial.print(" Data:");
      Serial.println(i8080_regs_a(), HEX);
      CPUStatus();
      Serial.flush(); // FORCE output buffer clear
      return;
    }
  }

  //From i8080_hal
  void i8080_hal_halt() {
    // A HALT instruction MUST set the WAIT flag to stop the main loop from
    // calling i8080_instruction() repeatedly.
    bitSet(bus.state,HLTA);
    bitSet(bus.state,WAIT); 
    bitClear(bus.state,HLDA);
    return;
  }

  // From i8080_hal 
  //extern void i8080_hal_iff(int on);
  void i8080_hal_iff(unsigned char on) {
    //This routine processes the EI and DI instruction
    //Each peripheral needs to do the interupt in the main loop
    //When doing an interrtupt: Push the PC onto the stack SP -= 2;
    // WR_WORD(SP, (reg));
    //xC7 RST(0) - 0x0000
    //xCF RST(1) - 0x0008
    //xD7 RST(2) - 0x0010
    //xDF RST(3) - 0x0018
    //xE7 RST(4) - 0x0020
    //xEF RST(5) - 0x0028
    //xF7 RST(6) - 0x0030
    //xFF RST(7) - 0x0038
    if (on) {
      //Enable interrupts
      bitSet(bus.state,INTE);
    } else{
      bitClear(bus.state,INTE);
    }
    //Interrupt Vectors
    return;
  }
  // From i8080_hal 
  extern unsigned char* i8080_hal_memory(void) {
    //Not really sure what this is for, set M1 just in case
    bitSet(bus.state,M1);
    return 0;
  }
}

void examine(uint16_t tmp16addr) {
     i8080_jump(tmp16addr); //set program counter
     
     // FIX: Update bus state to reflect memory being examined (manual read)
     bus.state = 0x00;
     bitSet(bus.state, WAIT); // Keep CPU halted
     bitSet(bus.state, MEMR); // Show memory read is active

     // Since the USER flags are now retired, we use the global booleans here to restore
     if (continuousLedUpdates) bitSet(bus.state, USER3);
     // FIX: Update the bus registers to reflect the examined address/data
     bus.address = i8080_pc();
     bus.data = CPUMemory[tmp16addr];
     writeLEDs();
}

void deposit(uint16_t tmp16addr, uint8_t vale) {
     i8080_jump(tmp16addr); //set program counter
     CPUMemory[tmp16addr] = vale;
     // FIX: Update bus state to reflect memory being deposited (manual write)
     bus.state = 0x00;
     bitSet(bus.state, WAIT); // Keep CPU halted
     bitSet(bus.state, MEMR); // Show memory access
     bitSet(bus.state, WO);   // Show memory write is active

     // Since the USER flags are now retired, we use the global booleans here to restore
     if (continuousLedUpdates) bitSet(bus.state, USER3);
     // FIX: Update the bus registers to reflect the deposited address/data
     bus.address = i8080_pc();
     bus.data = CPUMemory[tmp16addr];
     writeLEDs();
}

uint8_t hexcToInt(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  } else if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  } else if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  } else {
    return 0;
  }
}

int loadFile(const char filename[], int offset) {
  File file = SD.open(filename);
  if (!file) {
    Serial.println("ERR:Load");
    return -2; 
  }
  while (file.available()) {
    i8080_hal_memory_write_byte(offset++, file.read());
    //examine(offset);
    //writeLEDs();
  }
  file.close();
  return 0;
}

void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void setup() {
  // --- PHASE 1: CORE HARDWARE INITIALIZATION ---
  // Start the primary USB serial port for debugging and the 88-2SIO
  Serial.begin(115200);
  // Initialize all GPIO pins for LEDs and switches
  pinMode(LlatchPin, OUTPUT);
  pinMode(LclockPin, OUTPUT);
  pinMode(LdataPin, OUTPUT);
  pinMode(SlatchPin, OUTPUT);
  pinMode(SclockPin, OUTPUT);
  pinMode(SdataPin, INPUT);
  pinMode(SD_CS_PIN,OUTPUT);

  // Initialize the NeoPixel
  pixel.begin();
  pixel.setBrightness(128);
  // Start the secondary hardware serial port for the 88-SIO
  Serial2.begin(38400, SERIAL_8N1, Serial2RX, Serial2TX);
  // --- PHASE 2: PERIPHERAL & NETWORK INITIALIZATION ---
  // Connect to Wi-Fi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.print("\n\rIP Address: ");
  Serial.println(WiFi.localIP());
  // Initialize the SPI bus first
  SPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN);
  // Now initialize the SD card, which depends on SPI
  rgbLedWrite(BUILTIN_LED, 0, 64, 0); // Blue light for SD init
  if (!SD.begin(SD_CS_PIN,SPI,4000000)) {
    Serial.println("SD-Card init FAILED!");
    // You might want to halt here in a real scenario
  } else {
    Serial.println("SD-Card init OK!");
  }
  
  // --- PHASE 3: EMULATOR MODULE & CPU INITIALIZATION ---
  // Now that hardware is up, initialize our software modules
  serial_init();
  disk_init();

  // Set initial bus state and flags
  bus.state = 0x00;
  bitSet(bus.state,WAIT);
  continuousLedUpdates = true;
  continuousDebugOutput = false;
  // Load monitor ROMs into emulated memory
  Serial.println("Loading UBMON/DBLROM...");
  for (int i=0; i < 256; i++) {
    CPUMemory[i+0xFD00] = UBMON[i];
    CPUMemory[i+0xFF00] = DBLROM[i];
  }

  // Initialize the 8080 CPU core itself
  i8080_init(); // This sets the PC to 0xF800

  // --- EXAMPLE BREAKPOINT SETUP ---
  // Let's trap when the CPU is about to select disk drive 1 (value 0x01 in register A)
  // before an OUT 0x08 instruction.
  //Serial.println(">>> Breakpoint configured: Trap when A == 0x01 <<<");
  //bp_target_register = BP_REG_A;
  //bp_target_value = 0x01;

  rgbLedWrite(BUILTIN_LED, 64, 0, 0); // Red light, ready for RUN
}

void loop() {

  // --- UNIVERSAL TASKS (ALWAYS RUN) ---
  // 1. Poll hardware switches on every pass.
  readSwitches();

  // 2. Check for universal override switches (STOP, RESET, LED Toggle).
  if (onRelease(STOP)) {
    Serial.println("STOP");
    Serial.flush();
    bitSet(bus.state, WAIT);
  }
  if (onRelease(RESET)) {
      if (!reset_lock) {
          Serial.println("RESET");
          Serial.flush();
          bus.address = 0;
          bus.data = 0;
          i8080_init();
          examine(0); // This also sets the WAIT state
          reset_lock = true;
      }
  }
  if (onRelease(SINGLE_STEP_)) {
      debugStepModeActive = !debugStepModeActive;
      Serial.println(debugStepModeActive ? "--- Debug Step Mode ON ---" : "--- Debug Step Mode OFF ---");
      Serial.flush();
  }
  if (onRelease(AUX1_UP)) {
      if (!aux1_up_lock) {
          continuousLedUpdates = !continuousLedUpdates;
          Serial.println(continuousLedUpdates ? "LEDs Update On" : "LED Update Off");
          Serial.flush();
          if (!continuousLedUpdates) writeLEDs(); // Clear LEDs one last time if turning off
          aux1_up_lock = true;
      }
  }
  
  // 3. Poll for serial characters and handle CPU interrupts if needed.
  if (serial_update() && bitRead(bus.state, INTE)) {
    i8080_interrupt(0xFF);
  }
  // --- MAIN EXECUTION GATE ---
  if (!bitRead(bus.state, WAIT)) {
    // --- CPU IS RUNNING ---
    const long TARGET_HZ = 2000000;
    const int BATCH_SIZE = 1000;

    static unsigned long last_micros = 0;
    static long cycle_count = 0;
    for (int i = 0; i < BATCH_SIZE; i++) {
        // 1. Execute one instruction
        cycle_count += i8080_instruction();
        // 2. IMMEDIATE INTERRUPT CHECK (For responsive I/O)
        if (serial_update() && bitRead(bus.state, INTE)) {
            i8080_interrupt(0xFF);
        }
        // 3. CONDITIONAL BREAKPOINT CHECK
        if (debugStepModeActive) {
            CPUStatus();
            delay(100);
            //uint8_t current_val;
            //switch (bp_target_register) {
            //    case BP_REG_A: current_val = i8080_regs_a(); break;
            //    case BP_REG_B: current_val = i8080_regs_bc() >> 8; break;
            //    case BP_REG_C: current_val = i8080_regs_bc(); break;
            //    case BP_REG_D: current_val = i8080_regs_de() >> 8; break;
            //    case BP_REG_E: current_val = i8080_regs_de(); break;
            //    case BP_REG_H: current_val = i8080_regs_hl() >> 8; break;
            //    case BP_REG_L: current_val = i8080_regs_hl(); break;
            //}
            //if (current_val == bp_target_value) {
            //    Serial.println("--- CONDITIONAL BREAKPOINT HIT! ---");
            //    CPUStatus();
            //    bitSet(bus.state, WAIT);      // Halt the CPU
            //    debugStepModeActive = false;  // Disarm trap to prevent re-triggering
            //}
        }
        // 4. IMMEDIATE HALT CHECK: Exit batch early if WAIT is set by any means
        if (bitRead(bus.state, WAIT)) {
            break; // Exit the for-loop immediately
        }
    }
    
    // --- Timing throttle and LED update logic remains the same ---
    long expected_micros = (cycle_count * 1000000L) / TARGET_HZ;
    unsigned long current_micros = micros();
    unsigned long elapsed_micros = current_micros - last_micros;
    if (elapsed_micros < expected_micros) {
        delayMicroseconds(expected_micros - elapsed_micros);
    }
    
    if (elapsed_micros >= 2000000) {
        last_micros = current_micros;
        cycle_count = 0;
    }
    
    yield();
    
    bus.address = i8080_pc();
    bus.data = CPUMemory[bus.address];
    if (i8080_regs_sp() == bus.address) {
      bitSet(bus.state, STACK);
    } else {
      bitClear(bus.state, STACK);
    }
    if (continuousLedUpdates) {
      writeLEDs();
    }

  } else {
    // --- CPU IS HALTED ---
    // Handle all other, non-critical front panel switches.
    // SINGLE-SHOT LOCK CLEARING
    if (isDown(EXAMINE)) examine_lock = false;
    if (isDown(EXAMINE_NEXT)) examine_next_lock = false;
    if (isDown(DEPOSIT)) deposit_lock = false;
    if (isDown(DEPOSIT_NEXT)) deposit_next_lock = false;
    if (isDown(CLR)) clr_lock = false;
    if (isDown(SINGLE_STEP)) single_step_lock = false;
    if (isDown(AUX1_DOWN)) aux1_down_lock = false;
    if (isDown(AUX2_UP)) aux2_up_lock = false;
    if (isDown(AUX2_DOWN)) aux2_down_lock = false;
    // SWITCH RELEASE LOGIC (excluding universally handled switches)
    if (onRelease(RUN)) {
        Serial.println("Run");
        Serial.flush();
        bitClear(bus.state, WAIT);
    }
    if (onRelease(EXAMINE)) {
        if (!examine_lock) {
            examine(switches.address);
            Serial.printf("EXAMINE: 0x%04X:%02X\r\n", i8080_pc(), CPUMemory[i8080_pc()]);
            Serial.flush();
            examine_lock = true;
        }
        delay(KeyDelay);
    }
    if (onRelease(EXAMINE_NEXT)) {
        if (!examine_next_lock) {
            examine(i8080_pc() + 1);
            Serial.printf("EXAMINE NEXT: 0x%04X:%02X\r\n", i8080_pc(), CPUMemory[i8080_pc()]);
            Serial.flush();
            examine_next_lock = true;
        }
        delay(KeyDelay);
    }
    if (onRelease(DEPOSIT)) {
        if (!deposit_lock) {
            uint16_t current_pc = i8080_pc();
            uint8_t tmpvale = 0x00FF & switches.address;
            deposit(current_pc, tmpvale);
            Serial.printf("DEPOSIT: 0x%04X:%02X\r\n", current_pc, CPUMemory[current_pc]);
            Serial.flush();
            deposit_lock = true;
        }
        delay(KeyDelay);
    }
    if (onRelease(DEPOSIT_NEXT)) {
        if (!deposit_next_lock) {
            deposit(i8080_pc() + 1, switches.address);
            Serial.printf("DEPOSIT NEXT: 0x%04X:%02X\r\n", i8080_pc(), CPUMemory[i8080_pc()]);
            Serial.flush();
            deposit_next_lock = true;
        }
        delay(KeyDelay);
    }
    if (onRelease(CLR)) {
        if (!clr_lock) {
            Serial.println("CLR");
            Serial.flush();
            bitClear(bus.state, PROT);
            bitClear(bus.state, INTE);
            bitClear(bus.state, HLDA);
            bitSet(bus.state, WAIT);
            clr_lock = true;
        }
        delay(KeyDelay);
    }
    if (onRelease(SINGLE_STEP)) {
        if (!single_step_lock) {
            Serial.print("SSTEP. ");
            bitClear(bus.state, WAIT);
            i8080_instruction();
            bitSet(bus.state, WAIT);
            writeLEDs();
            CPUStatus();
            Serial.flush();
            single_step_lock = true;
        }
        delay(KeyDelay);
    }
    if (onRelease(PROTECT)) {
        Serial.println("PROTECT: Loading 88-2SIO Int Echo");
        Serial.flush();
        for (int i=0; i < 64; i++) CPUMemory[i] = Echo2SIOInt[i];
        examine(0x0000);
        bitClear(bus.state, WAIT);
        delay(KeyDelay);
    }
    if (onRelease(UNPROTECT)) {
        Serial.printf("UNPROTECT: MEMORY DUMP AT 0x%04X ---\r\n", switches.address);
        readPage(switches.address);
        delay(KeyDelay);
    }
    if (onRelease(AUX1_DOWN)) {  
        Serial.println("AUX1_DOWN: Loading 8K Basic...");
        Serial.flush();
        loadFile("/8KBAS_E0.BIN", 0xE000);
        loadFile("/8KBAS_E8.BIN", 0xE800);
        loadFile("/8KBAS_F0.BIN", 0xF000);
        loadFile("/8KBAS_F8.BIN", 0xF800);
        examine(0xE000);
        bitClear(bus.state, WAIT);
    }
    if (onRelease(AUX2_UP)) {
        Serial.println("AUX2_UP: Loading Microsoft Basic");
        Serial.flush();
        //if (disk_open_files("/EXTBAS5.DSK", "/BDSC.DSK")) {
        if (disk_open_files("/MBASIC.DSK", "/BDSC.DSK")) { 
            examine(0xff00);
            bitClear(bus.state, WAIT);
        }
    }
    if (onRelease(AUX2_DOWN)) {
        Serial.println("AUX2_DOWN: Loading CP/M + ZORK...");
        Serial.flush();
        //if (disk_open_files("/CPM22B23-56K.DSK", "/WORDSTAR.DSK")) { 
        if (disk_open_files("/CPM22B23-56K.DSK", "/ZORK.DSK")) { 
            examine(0xff00);
            bitClear(bus.state, WAIT);
        }
    }

    // Update bus state and physical LEDs only when halted.
    bus.address = i8080_pc();
    bus.data = CPUMemory[bus.address];
    if (i8080_regs_sp() == bus.address) {
      bitSet(bus.state, STACK);
    } else {
      bitClear(bus.state, STACK);
    }
    if (continuousLedUpdates) {
      writeLEDs();
    }
  } // End of HALTED block
}