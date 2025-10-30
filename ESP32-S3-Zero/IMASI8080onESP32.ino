// This file assumes a microcontroller environment (like ESP32/Arduino)
// where 'byte', 'boolean', 'digitalWrite', 'digitalRead', and 'delayMicroseconds' are available.
// This is specifically for a Waveshare ESP32-S3-Mini
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

// --- Includes for RTC Emulation ---
#include <time.h>       // For time_t, struct tm, localtime, mktime
#include <sys/time.h>   // For gettimeofday, settimeofday

#include "i8080.h"
#include "config.h"
#include "serial.h"
#include "disk.h"
#include "network_task.h"
#include "hdsk_psram.h"
#include "diagnostics.h"

// --- Inter-Core Queue Definitions ---
QueueHandle_t telnet_to_emu_queue;
QueueHandle_t emu_to_telnet_queue;

// Define queue size
#define QUEUE_SERIAL_BUFFER_SIZE 64

Adafruit_NeoPixel pixel(LED_COUNT, BUILTIN_LED, NEO_GRB + NEO_KHZ800);
Preferences preferences;

// --- Global NVS-Loaded Configuration ---
String global_ssid = "";
String global_pass = "";
uint8_t global_aux1_up_action = ACTION_ENTER_MENU; // Default AUX1_UP to menu
uint8_t global_aux1_down_action = ACTION_LOAD_8K_BASIC; // Default to 8K Basic
uint8_t global_aux2_up_action = ACTION_LOAD_MBASIC_DISK; // Default to MBasic
uint8_t global_aux2_down_action = ACTION_LOAD_CPM_DISK; // Default to CP/M
bool global_autoboot_enabled = false;
uint8_t global_autoboot_action = ACTION_NONE;
String global_mbasic_dsk0 = DEFAULT_MBASIC_DSK0;
String global_mbasic_dsk1 = DEFAULT_MBASIC_DSK1;
String global_cpm_hdsk = DEFAULT_CPM_HDSK;
String global_cpm_dsk0 = DEFAULT_CPM_DSK0;
String global_cpm_dsk1 = DEFAULT_CPM_DSK1;

// --- NVS Key Definitions (declared extern in config.h) ---
const char* NVS_WIFI_SSID = "wifi_ssid";
const char* NVS_WIFI_PASS = "wifi_pass";
const char* NVS_AUX1_UP_ACTION = "aux1_up";
const char* NVS_AUX1_DOWN_ACTION = "aux1_down";
const char* NVS_AUX2_UP_ACTION = "aux2_up";
const char* NVS_AUX2_DOWN_ACTION = "aux2_down";
const char* NVS_AUTOBOOT_ENABLED = "auto_en";
const char* NVS_AUTOBOOT_ACTION = "auto_act";
const char* NVS_MBASIC_DSK0 = "mbasic_d0";
const char* NVS_MBASIC_DSK1 = "mbasic_d1";
const char* NVS_CPM_HDSK = "cpm_hdsk";
const char* NVS_CPM_DSK0 = "cpm_d0";
const char* NVS_CPM_DSK1 = "cpm_d1";


// Funciton locks
// Global single-shot flags for momentary switch debounce/logic protection
bool examine_lock = false;
bool examine_next_lock = false;
bool deposit_lock = false;
bool deposit_next_lock = false;
bool reset_lock = false;
bool clr_lock = false;
bool single_step_lock = false;
bool protect_lock = false;
bool unprotect_lock = false;
bool aux1_up_lock = false;
bool aux1_down_lock = false;
bool aux2_up_lock = false;
bool aux2_down_lock = false;

// --- DEBUG & BREAKPOINT CONFIGURATION ---
bool cpu_halted_by_breakpoint = false;
// ... (existing breakpoint vars) ...

// --- 88-VI/RTC VECTORED INTERRUPT EMULATION ---
bool rtc_periodic_interrupt_enabled = false;
uint8_t rtc_interrupt_vector = 0xF7; // Default: RST 6
unsigned long rtc_interrupt_period_ms = 1000; // Default: 1Hz

// CRITICAL FIX: These new internal flags replace the USER3 and USER4 status bits.
bool continuousLedUpdates = true; // Replaces USER3 (starts ON)
bool continuousDebugOutput = false; // Replaces USER4 (starts OFF)

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
  // Use Serial.printf for precise formatting
  Serial.printf(
    "PC:%04X BUS:%04X  SP:%04X Bus Data:%02X Memory Data:%02X AF:%04X BC:%04X  DE:%04X  HL:%04X\r\n",
    i8080_pc(),       // PC (4 digits)
    bus.address,      // BUS (4 digits)
    i8080_regs_sp(),  // SP (4 digits)
    bus.data,         // Bus Data (2 digits)
    CPUMemory[i8080_pc()], // Memory Data (2 digits)
    i8080_regs_af(),  // AF (4 digits)
    i8080_regs_bc(),  // BC (4 digits)
    i8080_regs_de(),  // DE (4 digits)
    i8080_regs_hl()   // HL (4 digits)
  );
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
/**
 * @brief Generic helper to load a PROGMEM ROM into 8080 memory.
 * Clears CPU memory, copies the ROM, and halts the CPU at the load address.
 * @param rom Pointer to the PROGMEM array.
 * @param size Size of the array (use sizeof()).
 * @param load_address The 16-bit address in 8080 memory to load to.
 */
void load_diag_rom(const byte* rom, size_t size, uint16_t load_address) {
  if (load_address + size > 65535) {
    Serial.println("Error: ROM exceeds 64KB memory boundary.");
    return;
  }

  Serial.println("Clearing 8080 memory...");
  memset(CPUMemory, 0, sizeof(CPUMemory)); 

  Serial.printf("Loading %d bytes to 0x%04X...\n", size, load_address);
  memcpy_P(&CPUMemory[load_address], rom, size);

  i8080_init(); // Reset CPU state
  hdsk_psram_reset(); //Reset the Hard Disk
  examine(load_address); // Set PC and halt

  Serial.printf("Load complete. CPU Halted at 0x%04X.\n", load_address);
  Serial.println("Exit menu and toggle RUN switch to execute.");
}
/**
 * @brief Reads a line of text from the Serial console.
 * * Blocks execution until user presses Enter (\n or \r).
 * * Handles backspace for editing.
 * @return String The text entered by the user.
 */
String get_serial_string() {
  String input = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();

      if (c == '\r' || c == '\n') {
        // User pressed Enter
        Serial.println(); // Echo the newline
        break;
      } else if (c == '\b' || c == 127) {
        // User pressed Backspace
        if (input.length() > 0) {
          input.remove(input.length() - 1);
          Serial.print("\b \b"); // Erase character on terminal
        }
      } else if (isPrintable(c)) {
        // Regular printable character
        input += c;
        Serial.print(c); // Echo character
      }
    }
    // Use a small delay/yield to prevent starving other tasks
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  return input;
}

/**
 * @brief Helper to wait for and get a single char, with echo.
 * @return char The character read (uppercased).
 */
char get_serial_char_blocking() {
  while (Serial.available() == 0) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  char c = Serial.read();
  Serial.print(c); // Echo character
  Serial.println(); // Echo newline
  return toupper(c);
}

// Helper for menu_list_disks
void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print(F("  ")); // Print indent
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println(F("/"));
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print(F("\t\t"));
      // Align columns
      if (strlen(entry.name()) < 8) Serial.print(F("\t"));
      if (strlen(entry.name()) < 16) Serial.print(F("\t"));

      Serial.printf(F("%lu bytes\r\n"), entry.size());
    }
    entry.close();
  }
}

/**
 * @brief Displays and handles the main configuration menu.
 * This function is called from loop() when the CPU is HALTED and AUX1_UP is pressed.
 * It takes over the serial console until the user exits.
 */
void run_config_menu() {
  Serial.println(F("\n\n*** IMSAI 8080 (ESP32) Main Menu ***"));
  bool in_menu = true;

  while (in_menu) {
    Serial.println(F("\n[1] Disk Management"));
    Serial.println(F("[2] System Configuration"));
    Serial.println(F("[3] Diagnostics & Utilities"));
    Serial.println(F("\n[R] Resume Emulation"));
    Serial.println(F("[X] Reboot ESP32"));
    Serial.print(F("Select: "));

    while (Serial.available() == 0) {
        vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for user input
    }
    char c = Serial.read();
    Serial.println(c);
    c = toupper(c);

    switch (c) {
      case '1':
        show_disk_menu();
        Serial.println(F("\n\n*** IMSAI 8080 (ESP32) Main Menu ***")); // Re-print main menu
        break;
      case '2':
        show_config_menu();
        Serial.println(F("\n\n*** IMSAI 8080 (ESP32) Main Menu ***")); // Re-print main menu
        break;
      case '3':
        show_diagnostics_menu();
        Serial.println(F("\n\n*** IMSAI 8080 (ESP32) Main Menu ***")); // Re-print main menu
        break;
      case 'R':
        Serial.println(F("Resuming emulation..."));
        in_menu = false; // Exit the while loop
        break;
      case 'X':
        Serial.println(F("Rebooting..."));
        delay(100);
        ESP.restart();
        break;
      default:
        Serial.println(F("Unknown option."));
        break;
    }
  }
}
/**
 * @brief Displays the Disk Management sub-menu
 */
void show_disk_menu() {
    bool in_menu = true;
while (in_menu) {
        Serial.println(F("\n\n*** Disk Management ***"));
        Serial.printf(F("Drive 0 (A:): %s\r\n"), disk_get_filename(0));
        Serial.printf(F("Drive 1 (B:): %s\r\n"), disk_get_filename(1));
        Serial.println(F("---------------------------------"));
        Serial.println(F("[L] Load Disk from SDCard..."));
Serial.println(F("[M] Mount Blank Disk..."));
        Serial.println(F("[S] Save Virtual Disk..."));
        Serial.println(F("[A] Save All Changed Disks"));
        Serial.println(F("[U] Unmount Disk..."));
Serial.println(F("[D] List Disks on SDCard"));
        Serial.println(F("[P] Set Program Load Paths"));
        Serial.println(F("\n[B] Back to Main Menu"));
        Serial.print(F("Select: "));
while (Serial.available() == 0) { vTaskDelay(10 / portTICK_PERIOD_MS); }
        char c = Serial.read();
Serial.println(c);
        c = toupper(c);

        switch (c) {
            case 'L': menu_load_disk();
break;
            case 'M': menu_mount_blank(); break;
            case 'S': menu_save_virtual_disk(); break;
            case 'A': menu_save_all_disks(); break;
            case 'U': menu_unmount_disk(); break;
            case 'D': menu_list_disks();
break;
            case 'P': menu_set_program_paths(); break;
            case 'B':
            case 'R':
                in_menu = false;
break;
        }
    }
}

/**
 * @brief Displays the System Configuration sub-menu
 */
void show_config_menu() {
    bool in_menu = true;
    while (in_menu) {
        Serial.println(F("\n\n*** System Configuration ***"));
        Serial.println(F("[W] Set Wi-Fi Credentials (SSID/Pass)"));
        Serial.println(F("[K] Set AUX Key Mappings"));
        Serial.println(F("[A] Set Auto-Boot Options"));
        Serial.println(F("---------------------------------"));
        Serial.println(F("[S] Save All Settings to NVS (EEPROM)"));
        Serial.println(F("\n[B] Back to Main Menu"));
        Serial.print(F("Select: "));

        while (Serial.available() == 0) { vTaskDelay(10 / portTICK_PERIOD_MS); }
        char c = Serial.read();
        Serial.println(c);
        c = toupper(c);

        switch (c) {
            case 'W': menu_set_wifi(); break;
            case 'K': menu_set_aux_keys(); break;
            case 'A': menu_set_autoboot(); break;
            case 'S': menu_save_all_settings(); break;
            case 'B':
            case 'R':
                in_menu = false;
                break;
        }
    }
}

/**
 * @brief Displays the Diagnostics & Utilities sub-menu
 */
void show_diagnostics_menu() {
    bool in_menu = true;
    while (in_menu) {
        Serial.println(F("\n\n*** Diagnostics & Utilities ***"));
        Serial.println(F("[T] Run Telnet Echo Test (Port 0x12)"));
        Serial.println(F("[D] Dump Memory at Address (from switches)"));
        Serial.println(F("--- 8080 CPU/Memory Tests ---"));
        Serial.println(F("[1] Load SIO Polling Echo (Port 0x00)"));
        Serial.println(F("[2] Load SIO Interrupt Echo (Port 0x00)"));
        Serial.println(F("[3] Load CPU Benchmark"));
        Serial.println(F("[4] Load Altair Memory Test"));
        Serial.println(F("--- Monitor/Boot ROMs ---"));
        Serial.println(F("[M] Load UBMON (at 0xFD00)"));
        Serial.println(F("[K] Load TURNMON (at 0xFD00)"));
        Serial.println(F("[H] Load HDBLROM (at 0xFC00)"));
        Serial.println(F("\n[B] Back to Main Menu"));
        Serial.print(F("Select: "));

        while (Serial.available() == 0) { vTaskDelay(10 / portTICK_PERIOD_MS); }
        char c = Serial.read();
        Serial.println(c);
        c = toupper(c);

        switch (c) {
            // Existing functions
            case 'T': 
                menu_telnet_test(); 
                in_menu = false; // Test auto-runs, must exit menu
                break;
            case 'D': 
                menu_dump_memory(); // Stays in menu
                break;

            // New Diagnostic Loaders
            case '1': 
                menu_diag_sio_echo(); 
                in_menu = false; // Must exit menu to run
                break;
            case '2': 
                menu_diag_sio_int_echo(); 
                in_menu = false; // Must exit menu to run
                break;
            case '3': 
                menu_diag_benchmark(); 
                in_menu = false; // Must exit menu to run
                break;
            case '4': 
                menu_diag_memtest(); 
                in_menu = false; // Must exit menu to run
                break;
            case 'M': 
                menu_load_ubmon(); 
                in_menu = false; // Must exit menu to run
                break;
            case 'K': 
                menu_load_turnmon(); 
                in_menu = false; // Must exit menu to run
                break;
            case 'H': 
                menu_load_hdblrom(); 
                in_menu = false; // Must exit menu to run
                break;

            // Exit
            case 'B':
            case 'R':
                in_menu = false;
                break;
        }
    }
}

// --- Disk Menu Implementations ---

void menu_load_disk() { 
  Serial.println(F("\nLoad to which drive? (0=A, 1=B)"));
  char c = get_serial_char_blocking();
  int drive = c - '0';

  if (drive < 0 || drive > 1) {
    Serial.println(F("Invalid drive."));
    return;
  }
  
  Serial.print(F("Enter full path for disk image (e.g. /MYDISK.DSK): "));
  String new_path = get_serial_string();
  
  if (new_path.length() == 0) {
    Serial.println(F("Aborted."));
    return;
  }

  Serial.printf("Loading %s into Drive %d...\n", new_path.c_str(), drive);

  bool success;
  if (drive == 0) {
    success = disk_open_files(new_path.c_str(), disk_get_filename(1));
  } else {
    success = disk_open_files(disk_get_filename(0), new_path.c_str());
  }
  
  if (success) {
    Serial.println(F("Disk load successful."));
  } else {
    Serial.println(F("!!! Disk load FAILED. See log for details. !!!"));
  }
}

void menu_mount_blank() { 
  Serial.println(F("\nMount blank disk to which drive? (0=A, 1=B)"));
  char c = get_serial_char_blocking();
  int drive = c - '0';

  if (drive < 0 || drive > 1) {
    Serial.println(F("Invalid drive."));
    return;
  }

  // This function is provided by disk.cpp
  disk_mount_blank(drive);
  Serial.println(F("Blank disk mounted."));
}

void menu_save_virtual_disk() { 
  Serial.println(F("\nSave which drive to SD Card? (0=A, 1=B)"));
  char c = get_serial_char_blocking();
  int drive = c - '0';

  if (drive < 0 || drive > 1) {
    Serial.println(F("Invalid drive."));
    return;
  }

  // This function is provided by disk.cpp
  // It handles checking the dirty flag and writing to the SD.
  disk_write_back(drive);
}

void menu_save_all_disks() { 
  Serial.println(F("\nSaving all changed virtual disks to SD Card..."));
  // These calls are provided by disk.cpp
  // They will internally check if a save is needed (dirty flag).
  disk_write_back(0); // Save Drive 0
  disk_write_back(1); // Save Drive 1
  Serial.println(F("Save complete."));
}

void menu_unmount_disk() { 
  Serial.println(F("\nWhich drive to unmount? (0=A, 1=B)"));
  char c = get_serial_char_blocking();
  int drive = c - '0';

  if (drive == 0) {
    Serial.println(F("Unmounting Drive 0 (A:)..."));
    // Reload Drive 1, pass nullptr for Drive 0
    // disk_open_files handles failure by blanking the PSRAM
    disk_open_files(nullptr, disk_get_filename(1));
  } else if (drive == 1) {
    Serial.println(F("Unmounting Drive 1 (B:)..."));
    // Reload Drive 0, pass nullptr for Drive 1
    disk_open_files(disk_get_filename(0), nullptr);
  } else {
    Serial.println(F("Invalid drive."));
    return;
  }
  Serial.println(F("Unmount complete."));
}

void menu_list_disks() { 
  Serial.println(F("\n--- Files on SD Card (Root) ---"));
  File root = SD.open("/");
  if(!root){
      Serial.println(F("Failed to open root directory"));
      return;
  }
  if (!root.isDirectory()) {
      Serial.println(F("Error: Root is not a directory."));
      root.close();
      return;
  }
  printDirectory(root, 0);
  root.close();
  Serial.println(F("---------------------------------"));
}

// --- Config Menu Stubs ---
/**
 * @brief Menu function to set Wi-Fi credentials.
 * * Prompts user for SSID and Password, then saves them to NVS.
 * * Also updates the global config variables immediately.
 */
void menu_set_wifi() {
  Serial.println("\n*** Set Wi-Fi Credentials ***");
  
  Serial.print("Enter Wi-Fi SSID: ");
  String ssid = get_serial_string();
  
  Serial.print("Enter Wi-Fi Password: ");
  String pass = get_serial_string();

  // Save to NVS
  // We use "imsai8080" as the NVS "namespace"
  preferences.begin("imsai8080", false); // false = read/write mode
  preferences.putString(NVS_WIFI_SSID, ssid);
  preferences.putString(NVS_WIFI_PASS, pass);
  preferences.end();

  Serial.println("\n\nWi-Fi credentials saved to NVS.");
  
  // Update the running config in the global variables
  global_ssid = ssid;
  global_pass = pass;
  
  Serial.println("Press [Enter] to return to the menu.");
  while (Serial.available() == 0) vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for keypress
  while (Serial.available() > 0) Serial.read(); // Clear buffer
}

void menu_set_aux_keys() {
  bool in_menu = true;
  while (in_menu) {
    Serial.println(F("\n\n*** Set AUX Key Mappings ***"));
    Serial.printf(F("[1] AUX1 UP   -> %s\r\n"), get_action_name(global_aux1_up_action));
    Serial.printf(F("[2] AUX1 DOWN -> %s\r\n"), get_action_name(global_aux1_down_action));
    Serial.printf(F("[3] AUX2 UP   -> %s\r\n"), get_action_name(global_aux2_up_action));
    Serial.printf(F("[4] AUX2 DOWN -> %s\r\n"), get_action_name(global_aux2_down_action));
    Serial.println(F("\n[S] Save All AUX Mappings to NVS"));
    Serial.println(F("[B] Back to Config Menu"));
    Serial.print(F("Select key to change (1-4) or action (S, B): "));

    while (Serial.available() == 0) { vTaskDelay(10 / portTICK_PERIOD_MS); }
    char c = Serial.read();
    Serial.println(c);
    c = toupper(c);

    int action_choice = -1;
    const char* nvs_key = NULL;
    uint8_t* global_var = NULL;

    switch (c) {
      case '1':
        nvs_key = NVS_AUX1_UP_ACTION;
        global_var = &global_aux1_up_action;
        break;
      case '2':
        nvs_key = NVS_AUX1_DOWN_ACTION;
        global_var = &global_aux1_down_action;
        break;
      case '3':
        nvs_key = NVS_AUX2_UP_ACTION;
        global_var = &global_aux2_up_action;
        break;
      case '4':
        nvs_key = NVS_AUX2_DOWN_ACTION;
        global_var = &global_aux2_down_action;
        break;
      case 'S':
        preferences.begin("imsai8080", false);
        preferences.putUInt(NVS_AUX1_UP_ACTION, global_aux1_up_action);
        preferences.putUInt(NVS_AUX1_DOWN_ACTION, global_aux1_down_action);
        preferences.putUInt(NVS_AUX2_UP_ACTION, global_aux2_up_action);
        preferences.putUInt(NVS_AUX2_DOWN_ACTION, global_aux2_down_action);
        preferences.end();
        Serial.println(F("AUX key mappings saved to NVS."));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        break;
      case 'B':
        in_menu = false;
        break;
    }

    // If a key (1-4) was selected
    if (global_var != NULL) {
      action_choice = select_mappable_action();
      if (action_choice != -1) {
        *global_var = (uint8_t)action_choice;
        Serial.println(F("Mapping set. Press [S] to save permanently."));
      }
    }
  }
}
void menu_set_autoboot() {
  bool in_menu = true;
  while (in_menu) {
    Serial.println(F("\n\n*** Set Auto-Boot Options ***"));
    Serial.printf(F("Status:    %s\n"), global_autoboot_enabled ? "ENABLED" : "DISABLED");
    Serial.printf(F("Action:    %s\n"), get_action_name(global_autoboot_action));
    Serial.println(F("---------------------------------"));
    Serial.println(F("[1] Toggle Auto-Boot (ON/OFF)"));
    Serial.println(F("[2] Set Auto-Boot Action"));
    Serial.println(F("\n[B] Back to Config Menu"));
    Serial.print(F("Select: "));

    while (Serial.available() == 0) { vTaskDelay(10 / portTICK_PERIOD_MS); }
    char c = Serial.read();
    Serial.println(c);
    c = toupper(c);

    switch (c) {
      case '1':
        global_autoboot_enabled = !global_autoboot_enabled;
        // Save change immediately
        preferences.begin("imsai8080", false);
        preferences.putBool(NVS_AUTOBOOT_ENABLED, global_autoboot_enabled);
        preferences.end();
        Serial.printf(F("Auto-boot is now %s.\n"), global_autoboot_enabled ? "ENABLED" : "DISABLED");
        break;
      
      case '2': {
        int action_choice = select_mappable_action();
        if (action_choice != -1) {
          global_autoboot_action = (uint8_t)action_choice;
          // Save change immediately
          preferences.begin("imsai8080", false);
          preferences.putUInt(NVS_AUTOBOOT_ACTION, global_autoboot_action);
          preferences.end();
          Serial.println(F("Auto-boot action set."));
        }
        break;
      }
        
      case 'B':
        in_menu = false;
        break;
    }
  }
}

/**
 * @brief Helper function for menu_set_program_paths to reduce repetition
 */
void get_and_save_path(String& global_var, const char* nvs_key) {
  String new_path = get_serial_string();
  if (new_path.length() > 0) {
    global_var = new_path;
    preferences.begin("imsai8080", false);
    preferences.putString(nvs_key, global_var);
    preferences.end();
    Serial.println("Path saved to NVS.");
  } else {
    Serial.println("No path entered. Keeping original.");
  }
}

/**
 * @brief Displays the menu for setting program load paths
 */
void menu_set_program_paths() {
    bool in_menu = true;
    while (in_menu) {
        Serial.println(F("\n\n*** Set Program Load Paths ***"));
        Serial.println(F("--- MBasic (Disk) ---"));
        Serial.printf(F("[1] Drive 0: %s\r\n"), global_mbasic_dsk0.c_str());
        Serial.printf(F("[2] Drive 1: %s\r\n"), global_mbasic_dsk1.c_str());
        Serial.println(F("--- CP/M ---"));
        Serial.printf(F("[3] HD Img:  %s\r\n"), global_cpm_hdsk.c_str());
        Serial.printf(F("[4] Drive 0: %s\r\n"), global_cpm_dsk0.c_str());
        Serial.printf(F("[5] Drive 1: %s\r\n"), global_cpm_dsk1.c_str());
        
        Serial.println(F("\r\n[B] Back to Disk Menu"));
        Serial.print(F("Select path to change (1-5) or [B]: "));

        while (Serial.available() == 0) { vTaskDelay(10 / portTICK_PERIOD_MS); }
        char c = Serial.read();
        Serial.println(c);
        c = toupper(c);

        switch (c) {
            case '1':
                Serial.print(F("Enter path for MBasic Drive 0: "));
                get_and_save_path(global_mbasic_dsk0, NVS_MBASIC_DSK0);
                break;
            case '2':
                Serial.print(F("Enter path for MBasic Drive 1: "));
                get_and_save_path(global_mbasic_dsk1, NVS_MBASIC_DSK1);
                break;
            case '3':
                Serial.print(F("Enter path for CP/M Hard Disk: "));
                get_and_save_path(global_cpm_hdsk, NVS_CPM_HDSK);
                break;
            case '4':
                Serial.print(F("Enter path for CP/M Drive 0: "));
                get_and_save_path(global_cpm_dsk0, NVS_CPM_DSK0);
                break;
            case '5':
                Serial.print(F("Enter path for CP/M Drive 1: "));
                get_and_save_path(global_cpm_dsk1, NVS_CPM_DSK1);
                break;
            case 'B':
            case 'R':
                in_menu = false;
                break;
        }
    }
}


void menu_save_all_settings() { 
  Serial.println("\n[STUB] Saving all settings to NVS (EEPROM)..."); 
  preferences.putBool("config_saved", true);
  Serial.println("Settings saved.");
}

// --- NVS Load Function ---
/**
 * @brief Loads configuration from NVS into global variables.
 * * This is called once in setup() to retrieve persistent settings.
 * * It will use defaults from config.h if NVS keys are not found.
 */
void load_config_from_nvs() {
  Serial.println("Loading configuration from NVS...");
  
  // Open NVS in read-only mode
  preferences.begin("imsai8080", true); 

  // Read Wi-Fi settings. If key doesn't exist, use the default from config.h.
  global_ssid = preferences.getString(NVS_WIFI_SSID, DEFAULT_WIFI_SSID);
  global_pass = preferences.getString(NVS_WIFI_PASS, DEFAULT_WIFI_PASS);

  // --- Load AUX Key Mappings ---
  // If a key doesn't exist, getUInt will return the default value we passed in.
  global_aux1_up_action = preferences.getUInt(NVS_AUX1_UP_ACTION, ACTION_ENTER_MENU);
  global_aux1_down_action = preferences.getUInt(NVS_AUX1_DOWN_ACTION, ACTION_LOAD_8K_BASIC);
  global_aux2_up_action = preferences.getUInt(NVS_AUX2_UP_ACTION, ACTION_LOAD_MBASIC_DISK);
  global_aux2_down_action = preferences.getUInt(NVS_AUX2_DOWN_ACTION, ACTION_LOAD_CPM_DISK);

  // --- Load Auto-Boot Settings ---
  global_autoboot_enabled = preferences.getBool(NVS_AUTOBOOT_ENABLED, false);
  global_autoboot_action = preferences.getUInt(NVS_AUTOBOOT_ACTION, ACTION_NONE);

  // --- Load Program Load Paths ---
  global_mbasic_dsk0 = preferences.getString(NVS_MBASIC_DSK0, DEFAULT_MBASIC_DSK0);
  global_mbasic_dsk1 = preferences.getString(NVS_MBASIC_DSK1, DEFAULT_MBASIC_DSK1);
  global_cpm_hdsk = preferences.getString(NVS_CPM_HDSK, DEFAULT_CPM_HDSK);
  global_cpm_dsk0 = preferences.getString(NVS_CPM_DSK0, DEFAULT_CPM_DSK0);
  global_cpm_dsk1 = preferences.getString(NVS_CPM_DSK1, DEFAULT_CPM_DSK1);

  Serial.println("NVS load complete.");

  preferences.end(); // Close NVS

  // Report status
  if (global_ssid == DEFAULT_WIFI_SSID) {
    Serial.println("Loaded default Wi-Fi SSID from config.h: " + global_ssid);
    if (global_ssid.length() == 0) {
      Serial.println("WARNING: No default SSID set. Use menu to configure Wi-Fi.");
    }
  } else {
    Serial.println("Loaded custom Wi-Fi SSID from NVS: " + global_ssid);
  }
}
/**
 * @brief Executes a pre-defined mappable action (e.g., Load CP/M).
 * @param action The MappableAction enum value to execute.
 * @param auto_run If true, clears the WAIT flag to run the CPU (for auto-boot).
 * If false, halts the CPU at the examine() address (for AUX keys).
 */
void execute_mapped_action(uint8_t action, bool auto_run) {
  switch (action) {
    case ACTION_NONE:
      break; // Do nothing
      
    case ACTION_ENTER_MENU:
      Serial.println(F("\n--- Configuration Menu ---"));
      run_config_menu();
      Serial.println(F("--- Exiting Menu ---"));
      CPUStatus();
      break;

    case ACTION_SAVE_ALL_DISKS:
      Serial.println("Executing: Save All Disks...");
      menu_save_all_disks(); // Call the existing (stubbed) function
      break;

    case ACTION_LOAD_8K_BASIC:
      Serial.println("Executing: Loading 8K Basic...");
      Serial.flush();
      loadFile("/8KBAS_E0.BIN", 0xE000);
      loadFile("/8KBAS_E8.BIN", 0xE800);
      loadFile("/8KBAS_F0.BIN", 0xF000);
      loadFile("/8KBAS_F8.BIN", 0xF800);
      examine(0xE000);
      break;

    case ACTION_LOAD_MBASIC_DISK:
      Serial.println("Executing: Loading Basic (Disk)");
      Serial.flush();
      // Use global strings instead of hardcoded paths
      if (disk_open_files(global_mbasic_dsk0.c_str(), global_mbasic_dsk1.c_str())) {
          examine(0xff00);
      } else {
           Serial.println("!!! FAILED to open disk images for MBasic !!!");
      }
      break;

    case ACTION_LOAD_CPM_DISK:
      Serial.println("Executing: Loading CP/M");
      Serial.flush();
      // Use global strings instead of hardcoded paths
      hdsk_psram_load_image(global_cpm_hdsk.c_str());
      if (disk_open_files(global_cpm_dsk0.c_str(), global_cpm_dsk1.c_str())) {            
          examine(0xFC00);
      } else {
           Serial.println("!!! FAILED to open disk images for CP/M !!!");
      }
      break;      
    default:
      Serial.println("Unknown action.");
      break;
  }
  // If auto_run is true, clear the WAIT state to start execution
  if (auto_run && (action == ACTION_LOAD_8K_BASIC || 
                   action == ACTION_LOAD_MBASIC_DISK || 
                   action == ACTION_LOAD_CPM_DISK)) {
    Serial.println("Auto-running...");
    bitClear(bus.state, WAIT);
  }
}
/**
 * @brief Returns a string name for a MappableAction.
 */
const char* get_action_name(uint8_t action) {
  switch (action) {
    case ACTION_NONE: return "None";
    case ACTION_ENTER_MENU: return "Enter Config Menu";
    case ACTION_SAVE_ALL_DISKS: return "Save All Disks";
    case ACTION_LOAD_8K_BASIC: return "Load 8K BASIC (RAM)";
    case ACTION_LOAD_MBASIC_DISK: return "Load MBasic (Disk)";
    case ACTION_LOAD_CPM_DISK: return "Load CP/M (Disk)";
    default: return "Unknown";
  }
}

/**
 * @brief Displays a menu to select a MappableAction and returns the choice.
 * @return MappableAction enum value, or -1 if user cancels.
 */
int select_mappable_action() {
  Serial.println(F("\nSelect Action:"));
  Serial.println(F("[0] None"));
  Serial.println(F("[1] Enter Config Menu"));
  Serial.println(F("[2] Save All Disks"));
  Serial.println(F("[3] Load 8K BASIC (RAM)"));
  Serial.println(F("[4] Load MBasic (Disk)"));
  Serial.println(F("[5] Load CP/M (Disk)"));
  Serial.println(F("\n[B] Back/Cancel"));
  Serial.print(F("Select: "));

  while (true) {
    while (Serial.available() == 0) { vTaskDelay(10 / portTICK_PERIOD_MS); }
    char c = Serial.read();
    Serial.println(c);
    c = toupper(c);

    if (c >= '0' && c <= '5') {
      return c - '0'; // Returns the integer value 0-5
    } else if (c == 'B') {
      return -1; // Cancel
    } else {
      Serial.print(F("Invalid. Select 0-5 or B: "));
    }
  }
}
// --- Diagnostics Menu Functions ---
void menu_diag_sio_echo() {
  load_diag_rom(EchoSIO, sizeof(EchoSIO), 0x0000);
}
void menu_diag_sio_int_echo() {
  load_diag_rom(EchoSIOInt, sizeof(EchoSIOInt), 0x0000);
}
void menu_diag_benchmark() {
  load_diag_rom(BenchMark, sizeof(BenchMark), 0x0000);
}
void menu_diag_memtest() {
  load_diag_rom(AltairMemTest, sizeof(AltairMemTest), 0x0000);
}
void menu_load_ubmon() {
  load_diag_rom(UBMON, sizeof(UBMON), 0xFD00);
}
void menu_load_turnmon() {
  load_diag_rom(TURNMON, sizeof(TURNMON), 0xFD00);
}
void menu_load_hdblrom() {
  load_diag_rom(HDBLROM, sizeof(HDBLROM), 0xFC00);
}

/**
 * @brief Menu Function: Loads and runs the Telnet Echo diagnostic.
 * (Logic moved from old PROTECT switch)
 */
void menu_telnet_test() {
  Serial.println(F("Loading 88-2SIOB Polling Echo Test..."));
  for (int i=0; i < 22; i++) CPUMemory[i] = pgm_read_byte(&EchoTelnet[i]);
  examine(0x0000);
  bitClear(bus.state, WAIT); // Automatically RUN the test
  Serial.println(F("Test is now running. Press STOP to halt."));
  // We must exit the menu to let the CPU run
}

/**
 * @brief Menu Function: Dumps 512 bytes of memory from the address switches.
 * (Logic moved from old UNPROTECT switch)
 */
void menu_dump_memory() {
  readSwitches(); // Get the latest address switch values
  Serial.printf("Dumping 512 bytes from 0x%04X:\r\n", switches.address);
  readPage(switches.address);
  Serial.println(F("Dump complete."));
}


/**
 * @brief Converts a decimal value (0-99) to a BCD byte.
 * @param val The decimal value.
 * @return The BCD-encoded byte.
 */
static uint8_t dec_to_bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

/**
 * @brief Converts a BCD byte to its decimal value.
 * @param val The BCD-encoded byte.
 * @return The decimal value.
 */
static uint8_t bcd_to_dec(uint8_t val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}

/**
 * @brief Defines the 8 registers of the emulated RTC.
 * The 8080 will read/write these as BCD values.
 * This structure is based on common RTC chips like the MM58167.
 */
typedef struct {
    uint8_t hundredths_sec; // Register 0
    uint8_t seconds;        // Register 1
    uint8_t minutes;        // Register 2
    uint8_t hours;          // Register 3 (24-hour format)
    uint8_t day_of_week;    // Register 4 (1-7)
    uint8_t day_of_month;   // Register 5 (1-31)
    uint8_t month;          // Register 6 (1-12)
    uint8_t year;           // Register 7 (00-99)
} rtc_registers_t;

/**
 * @brief Internal register bank for *reading* time.
 * This struct is "latched" with the current ESP32 time whenever the
 * 8080 sets the register pointer via port 0xF8.
 */
static rtc_registers_t rtc_read_regs;

/**
 * @brief Internal register bank for *writing* time.
 * This struct is filled by the 8080 via OUT 0xF9 and committed
 * to the ESP32's system clock when the 'year' (reg 7) is written.
 */
static rtc_registers_t rtc_write_regs;

/**
 * @brief The internal pointer that selects which register (0-7) is
 * accessed by port 0xF9. This is set by OUT 0xF8.
 */
static uint8_t rtc_register_pointer = 0;

/**
 * @brief Latches the current ESP32 system time into the BCD `rtc_read_regs`.
 * This is the core of the "Read Time" operation. It's called when the
 * 8080 writes to the control port (0xF8) to ensure that all subsequent
 * reads from 0xF9 are from a single, consistent time snapshot.
 */
static void latch_esp_time_to_rtc_regs() {
    struct timeval tv;
    gettimeofday(&tv, NULL); // Get current time with microsecond precision
    struct tm* timeinfo = localtime(&tv.tv_sec); // Convert to local time structure

    if (timeinfo == NULL) {
        // Time is not set, fill registers with zeros
        memset(&rtc_read_regs, 0, sizeof(rtc_registers_t));
        return;
    }

    // Convert timeinfo (decimal) to BCD and store in our read registers
    
    // Convert microseconds to hundredths of a second
    rtc_read_regs.hundredths_sec = dec_to_bcd((tv.tv_usec / 10000) % 100);
    
    // struct tm members are 0-based for mon, 0-6 for wday
    rtc_read_regs.seconds       = dec_to_bcd(timeinfo->tm_sec);
    rtc_read_regs.minutes       = dec_to_bcd(timeinfo->tm_min);
    rtc_read_regs.hours         = dec_to_bcd(timeinfo->tm_hour);
    rtc_read_regs.day_of_month  = dec_to_bcd(timeinfo->tm_mday);
    rtc_read_regs.month         = dec_to_bcd(timeinfo->tm_mon + 1); // tm_mon is 0-11, RTC wants 1-12
    rtc_read_regs.year          = dec_to_bcd(timeinfo->tm_year % 100); // tm_year is since 1900, RTC wants 00-99
    rtc_read_regs.day_of_week   = dec_to_bcd(timeinfo->tm_wday + 1); // tm_wday is 0-6 (Sun=0), RTC wants 1-7
}

/**
 * @brief Commits the BCD time from `rtc_write_regs` to the ESP32 system clock.
 * This is the core of the "Set Time" operation. It's triggered when the
 * 8080 writes to the final register (year), assuming a full time-set sequence.
 */
static void commit_rtc_regs_to_esp_time() {
    struct tm timeinfo = {0};
    
    // Convert BCD from write registers back to decimal for struct tm
    timeinfo.tm_sec   = bcd_to_dec(rtc_write_regs.seconds);
    timeinfo.tm_min   = bcd_to_dec(rtc_write_regs.minutes);
    timeinfo.tm_hour  = bcd_to_dec(rtc_write_regs.hours);
    timeinfo.tm_mday  = bcd_to_dec(rtc_write_regs.day_of_month);
    timeinfo.tm_mon   = bcd_to_dec(rtc_write_regs.month) - 1; // RTC is 1-12, tm_mon is 0-11

    // Handle 2-digit year. Assume 20xx.
    // BCD '25' -> DEC 25. tm_year needs "years since 1900".
    // So 2025 is 125.
    int dec_year = bcd_to_dec(rtc_write_regs.year);
    timeinfo.tm_year = dec_year + 100; // Assumes 2000-2099
    
    timeinfo.tm_isdst = -1; // Let mktime determine DST

    // Convert struct tm to time_t (seconds since epoch)
    time_t t = mktime(&timeinfo);

    // Set the ESP32 system clock
    if (t != -1) {
        struct timeval tv;
        tv.tv_sec = t;
        // We can't set sub-second precision from this RTC interface
        // Set based on the BCD hundredths we received
        tv.tv_usec = (long)bcd_to_dec(rtc_write_regs.hundredths_sec) * 10000;
        settimeofday(&tv, NULL);
    }
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

    //unsigned int seek; // No longer needed
    unsigned char ret_val;
    switch (port) {
      case 0x00:  // 88-SIO Status -> Serial
       return serial_port00_in();
      case 0x01: // 88-SIO Data -> Serial
        return serial_port01_in();
      case 0x02:  //printer (OkiData, C700 or generic)
        return 0xFF;
      case 0x03:  //printer (OkiData, C700 or generic)
        return 0xFF;
      case 0x04: //Protec VDM1 Keyboard
        return 0xFF;
      case 0x05: //Protect VDM1 Keyboard
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
      case 0x08: // 88-DCDD Disk Port
      case 0x09: // 88-DCDD Disk Port
      case 0x0A: // 88-DCDD Disk Ports
        return disk_in(port);
      case 0x10: // 88-2SIOA Status -> Serial
       return serial_port10_in();
      case 0x11: // 88-2SIOA Data -> Serial
        return serial_port11_in();
      case 0x12: // 88-2SIOB Status -> Telnet for now
      {
      uint8_t status = 0;
        // Check if Core 0 (Telnet) has sent data for us to read
        if (uxQueueMessagesWaiting(telnet_to_emu_queue) > 0) {
          status |= 0x01; // Bit 0: RX Ready
        }
        // Check if Core 0 (Telnet) has space to receive data from us
        if (uxQueueSpacesAvailable(emu_to_telnet_queue) > 0) {
           status |= 0x02; // Bit 1: TX Ready
        }
        #ifdef NETWORK_TRACE
        Serial.printf("[IO IN 0x12: Status=0x%02X]\n", status);
        #endif
        return status;
      }
      case 0x13: // 88-2SIOB Status -> Telnet for now 
      {
        uint8_t data_byte = 0;
        // Try to receive a byte from Core 0 (Telnet) without blocking
        if (xQueueReceive(telnet_to_emu_queue, &data_byte, (TickType_t)0) == pdTRUE) {
           #ifdef NETWORK_TRACE
           Serial.printf("[IO IN 0x13: RX Data=0x%02X ('%c')]\n", data_byte, (data_byte < 32 ? '.' : data_byte));
           #endif
           return data_byte;
        } else {
           #ifdef NETWORK_TRACE
           // Only print if debugging is enabled, as this can be noisy
           Serial.println("[IO IN 0x21: RX Queue Empty]");
           #endif
           return 0x00; // Return 0 if no data is available
        }
      }
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
      case 0x24:  // Altair Disk Basic Hit's this port
        return 0xFF;
      case 0x25:  // Altair Disk Basic Hit's this port
        return 0xFF;
      case 0x26:  // Altair Disk Basic Hit's this port
        return 0xFF;
      case 0x27:  // Altair Disk Basic Hit's this port
        return 0xFF;
      case 0xA0: // --- MITS 4PIO Hard Disk Interface ---
      case 0xA1:
      case 0xA2:
      case 0xA3:
      case 0xA4:
      case 0xA5:
      case 0xA6:
      case 0xA7:
          return hdsk_psram_in(port);
      case 0xF8: //Emulates the 88-VI/RTC board, Reading from the control port returns status. 0 = OK.
        return 0x00;
      case 0xF9: //Emulates the 88-VI/RTC board, Returns the BCD byte from the latched snapshot
        switch (rtc_register_pointer) {
            case 0: return rtc_read_regs.hundredths_sec;
            case 1: return rtc_read_regs.seconds;
            case 2: return rtc_read_regs.minutes;
            case 3: return rtc_read_regs.hours;
            case 4: return rtc_read_regs.day_of_week;
            case 5: return rtc_read_regs.day_of_month;
            case 6: return rtc_read_regs.month;
            case 7: return rtc_read_regs.year;
            default: return 0xFF; // Should not happen
        }
        break; // Unreachable, but good practice
      case 0xFE: // 88-VI IRQ controller port
        // Software can read this port to check status or clear an IRQ.
        // For our emulation, we'll just return 0x00.
        #ifdef DEBUGIO
        Serial.println("IN 0xFE (88-VI IRQ)");
        #endif
        return 0x00;
      case 0xFF: // Front panel switches
        readSwitches();
        // Serial.print("Address PI: "); // Reduced verbosity
        // Serial.print(switches.address >> 8,HEX);
        // Serial.println();
        return (switches.address >> 8);
      default:
        // --- CRITICAL TRAP FIX: GUARANTEED OUTPUT ---
        // This is the default handler for all unhandled IN ports (including 0xF0)
        Serial.print("!!!TRAP IN: "); // Differentiate IN traps
        Serial.print("Port:");
        Serial.print(port, HEX);
        Serial.print(" Data(A):");
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
    Serial.println(vale,HEX); // Use 'vale' here, as A might have changed
    #endif

    //Put the data on the bus (Correct place for this)
    bus.data = vale;
    // unsigned int seek; // Not needed
    // unsigned char ret_val; // Not needed

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
      Serial.print(vale,HEX);
      Serial.println();
      return; // Added return
    case 0x07: // 88-ACR Board Cassette
      Serial.print("Cassette Port Out 0x07, Data:");
      Serial.print(vale,HEX);
      Serial.println();
      return; // Added return
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
      // Often unused for basic serial I/O, we can ignore it for now.
      #ifdef NETWORK_TRACE
      Serial.printf("[IO OUT 0x20: Control=0x%02X (Ignoring)]\n", vale);
      #endif
      return;
    case 0x13: // 88-2SIO or 88-ACR port 1, Write/TX
    {
      #ifdef NETWORK_TRACE
      Serial.printf("[IO OUT 0x13: TX Data=0x%02X ('%c')]\n", vale, (vale < 32 ? '.' : vale));
      #endif
      // Send the byte to Core 0 (Telnet) via the queue
      // Use a small timeout (e.g., 10ms)
      if (xQueueSend(emu_to_telnet_queue, &vale, (TickType_t)(10 / portTICK_PERIOD_MS)) != pdTRUE) {
        // Log an error if the queue is full and data is lost
        Serial.println("!!! [Core 1] emu_to_telnet_queue FULL - TX Data Lost !!!");
      }
      return;
    }
    case 0x18: // 88-ACR Board I/O 2ndary
    case 0x19: // 88-ACR Board I/O 2ndary
      return;

    // --- NEW: Telnet I/O Ports ---
    case 0x20: // Telnet Control (Output from Core 1 perspective)
      return;
    case 0x21: // Telnet Data (Output from Core 1 perspective)
      return;
    case 0x22: // 88-2SIO or 88-ACR Alternate port 1, Control --> (Currently unused)
      return;
    case 0x23: // 88-2SIO or 88-ACR Alternate port 1, Write/TX --> (Currently unused)
      return;
    case 0x24:  //Altair Disk Basic hits this port
      return;
    case 0x25:  //Altair Disk Basic hits this port
      return;
    case 0x26:  //Altair Disk Basic hits this port
      return;
    case 0x27:  //Altair Disk Basic hits this port
      return;
    case 0xA0:  // --- MITS 4PIO Hard Disk Interface ---
    case 0xA1:
    case 0xA2:
    case 0xA3:
    case 0xA4:
    case 0xA5:
    case 0xA6:
    case 0xA7:
        hdsk_psram_out(port, vale);
        return;
    case 0xF8:  //Emulates the RTC for 88-VI/RTC, latching the current ESP32 time
        rtc_register_pointer = vale & 0x07; // Mask to 8 registers (0-7)
        latch_esp_time_to_rtc_regs(); // Latch time on *every* pointer set
        return;
    case 0xF9:   //Emulates the RTC for 88-VI/RTC, Writes a BCD byte to the selected register.
                  // Commits time to ESP32 system clock when reg 7 (year) is written.
        switch (rtc_register_pointer) {
            case 0: rtc_write_regs.hundredths_sec = vale; break;
            case 1: rtc_write_regs.seconds      = vale; break;
            case 2: rtc_write_regs.minutes      = vale; break;
            case 3: rtc_write_regs.hours        = vale; break;
            case 4: rtc_write_regs.day_of_week  = vale; break;
            case 5: rtc_write_regs.day_of_month = vale; break;
            case 6: rtc_write_regs.month        = vale; break;
            case 7: 
                rtc_write_regs.year = vale;
                // Writing to the last register triggers the system time set
                commit_rtc_regs_to_esp_time();
                break;
        }
        return;
    case 0xFE: // 88-VI/RTC Interrupt Controller Port
      // Emulate the interrupt controller.
      // We'll define a simple control byte:
      // Bit 0: 1 = Enable, 0 = Disable
      // Bits 3-1: Rate Select
      //   000 = 1 Hz (1000ms)
      //   001 = 10 Hz (100ms)
      //   010 = 100 Hz (10ms)
      //   011 = 1000 Hz (1ms)
      // Bits 6-4: Vector Select (0=RST0, 1=RST1, ..., 7=RST7)
      
      if (vale & 0x01) {
          rtc_periodic_interrupt_enabled = true;
          
          // Set the interrupt rate
          uint8_t rate_select = (vale >> 1) & 0x07;
          switch (rate_select) {
              case 0: rtc_interrupt_period_ms = 1000; break;
              case 1: rtc_interrupt_period_ms = 100; break;
              case 2: rtc_interrupt_period_ms = 10; break;
              case 3: rtc_interrupt_period_ms = 1; break; // 1000Hz
              default: rtc_interrupt_period_ms = 1000; // Default 1Hz
          }

          // Set the interrupt vector (0xC7 = RST 0, 0xCF = RST 1, etc.)
          uint8_t vector_select = (vale >> 4) & 0x07;
          rtc_interrupt_vector = 0xC7 | (vector_select << 3);

          #ifdef DEBUGIO
          Serial.printf("[IO OUT 0xFE]: RTC IRQ Enabled. Rate=%lums, Vector=RST %d (0x%02X)\n", 
                          rtc_interrupt_period_ms, vector_select, rtc_interrupt_vector);
          #endif
      } else {
          rtc_periodic_interrupt_enabled = false;
          #ifdef DEBUGIO
          Serial.println("[IO OUT 0xFE]: RTC IRQ Disabled.");
          #endif
      }
      return;

    case 0xFF: // panel LEDs
      // The high bits (USER3, USER4 positions) are handled by booleans and masked out
      // in writeLEDs, so we only update the lower 12 status bits here.
      bus.state = (bus.state & 0x0FFF) | (vale << 12); // Use bitwise OR here
      return;
    default:
      // --- CRITICAL TRAP FIX: GUARANTEED OUTPUT ---
      // This is the default handler for all unhandled OUT ports (including 0xF0)
      Serial.print("!!!TRAP OUT: "); // Differentiate OUT traps
      Serial.print("Port:");
      Serial.print(port, HEX);
      Serial.print(" Data:");
      Serial.println(vale, HEX);
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
} // <-- THIS BRACE PROPERLY CLOSES the extern "C" block

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

void setup() {
  // --- PHASE 1: CORE HARDWARE INITIALIZATION ---
  // Start the primary USB serial port for debugging and the 88-2SIO
  Serial.begin(115200);

  // Wait a moment for Serial to initialize
  delay(1000);
  Serial.println("\n\nIMSAI 8080 Emulator Booting on Core 1...");

  load_config_from_nvs();

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

  // !!! --- Create Queues --- !!!
  // Must be created BEFORE network_task_init() is called!
  Serial.println("Creating inter-core queues...");

  // Queue for Core 0 (Net) to send data TO Core 1 (Emu)
  telnet_to_emu_queue = xQueueCreate(QUEUE_SERIAL_BUFFER_SIZE, sizeof(uint8_t));
  if (telnet_to_emu_queue == NULL) {
    Serial.println("!!! FATAL: Failed to create telnet_to_emu_queue !!!");
    while(1) { rgbLedWrite(BUILTIN_LED, 255, 0, 0); delay(100); } // Halt
  }

  // Queue for Core 1 (Emu) to send data TO Core 0 (Net)
  emu_to_telnet_queue = xQueueCreate(QUEUE_SERIAL_BUFFER_SIZE, sizeof(uint8_t));
  if (emu_to_telnet_queue == NULL) {
    Serial.println("!!! FATAL: Failed to create emu_to_telnet_queue !!!");
    while(1) { rgbLedWrite(BUILTIN_LED, 255, 0, 0); delay(100); } // Halt
  }
  Serial.println("Queues created successfully.");

  // Connect to Wi-Fi (this will spawn Core 0 task)
  network_task_init();

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
  serial_init(); // Initializes standard serial ports 00/01 and 10/11
  disk_init();
  hdsk_psram_init();

  // Set initial bus state and flags
  bus.state = 0x00;
  bitSet(bus.state,WAIT);
  continuousLedUpdates = true;
  continuousDebugOutput = false;
  // Load bootloader into emulated memory
  Serial.println("Loading DBLROM...");
  for (int i=0; i < 256; i++) {
    CPUMemory[i+0xFF00] = pgm_read_byte(&DBLROM[i]); // Correct way to read from PROGMEM
  }

  // Initialize the 8080 CPU core itself
  i8080_init(); // This usually sets the PC based on reset vector

  // --- EXAMPLE BREAKPOINT SETUP ---
  // Let's trap when the CPU is about to select disk drive 1 (value 0x01 in register A)
  // before an OUT 0x08 instruction.
  //Serial.println(">>> Breakpoint configured: Trap when A == 0x01 <<<");
  //bp_target_register = BP_REG_A;
  //bp_target_value = 0x01;

  rgbLedWrite(BUILTIN_LED, 64, 0, 0); // Red light, ready for RUN
  #ifdef NETWORK_TRACE
  Serial.println("--- Setup complete. Core 1 entering loop(). ---");
  #endif
  // --- NEW: AUTO-BOOT LOGIC ---
  if (global_autoboot_enabled) {
    Serial.printf("\n*** AUTO-BOOT ENABLED ***\n");
    Serial.printf("Executing: %s\n", get_action_name(global_autoboot_action));
    // Call the helper, passing 'true' to auto-run the CPU
    execute_mapped_action(global_autoboot_action, true);
  } else {
    Serial.println("--- Setup complete. CPU Halted. ---");
  }
}

void loop() {
  static long cycle_count = 0;
  static unsigned long last_micros = 0;
  static unsigned long last_rtc_interrupt_millis = 0;

  // --- REMOVED QUEUE TEST CODE ---

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
          hdsk_psram_reset(); // Reset the hard disk controller state
          examine(0); // This also sets the WAIT state
          reset_lock = true;
      }
  }
  if (onRelease(SINGLE_STEP_)) {
      debugStepModeActive = !debugStepModeActive;
      Serial.println(debugStepModeActive ? "--- Debug Step Mode ON ---" : "--- Debug Step Mode OFF ---");
      Serial.flush();
  }
  if (onRelease(PROTECT)) {
      if(!protect_lock) {
        continuousLedUpdates = !continuousLedUpdates;
        Serial.printf("PROTECT: Continuous LED Updates: %s\n", continuousLedUpdates ? "ON" : "OFF");
      }
      protect_lock = true;
  }

  // 3. Poll for *standard* serial characters and handle CPU interrupts if needed.
  // Note: Telnet data is handled via IN instructions now, not interrupts here.
  if (serial_update() && bitRead(bus.state, INTE)) {
    i8080_interrupt(0xFF); // Or appropriate vector if needed
  }

  // 4. NEW: Poll for RTC periodic interrupt
  if (rtc_periodic_interrupt_enabled) {
      unsigned long current_millis = millis();
      if ((current_millis - last_rtc_interrupt_millis) >= rtc_interrupt_period_ms) {
          last_rtc_interrupt_millis = current_millis;
          
          // Check if 8080 interrupts are globally enabled (EI)
          if (bitRead(bus.state, INTE)) {
              #ifdef DEBUGIO
              Serial.printf("--- RTC Interrupt Trigger (Vector 0x%02X) ---\n", rtc_interrupt_vector);
              #endif
              i8080_interrupt(rtc_interrupt_vector);
              // The i8080_interrupt() call should also clear the WAIT flag
              // if the CPU was HALTed, allowing execution to resume.
          }
      }
  }

  // --- MAIN EXECUTION GATE ---
  if (!bitRead(bus.state, WAIT)) {
    // --- CPU IS RUNNING ---
    const long TARGET_HZ = 2000000;
    const int BATCH_SIZE = 1000; // Adjust for balance between speed and responsiveness

    for (int i = 0; i < BATCH_SIZE; i++) {
        // 1. Execute one instruction
        cycle_count += i8080_instruction();

        // 2. IMMEDIATE INTERRUPT CHECK (For standard serial ports)
        if (serial_update() && bitRead(bus.state, INTE)) {
            i8080_interrupt(0xFF); // Or appropriate vector
        }

        // 3. CONDITIONAL BREAKPOINT / DEBUG STEP CHECK
        if (debugStepModeActive) {
            CPUStatus(); // Print status after each instruction in debug mode
            delay(100);  // Slow down for observation
            // Add conditional breakpoint logic here if needed, similar to before
        }

        // 4. IMMEDIATE HALT CHECK: Exit batch early if WAIT is set (e.g., by HALT instruction)
        if (bitRead(bus.state, WAIT)) {
            break; // Exit the for-loop immediately
        }
    }

    // --- Timing throttle ---
    long expected_micros = (cycle_count * 1000000L) / TARGET_HZ;
    unsigned long current_micros = micros();
    unsigned long elapsed_micros = current_micros - last_micros;
    if (elapsed_micros < expected_micros) {
        delayMicroseconds(expected_micros - elapsed_micros);
    }
    // Reset cycle count periodically to prevent overflow and maintain accuracy
    if (elapsed_micros >= 100000) { // Reset roughly every 100ms
        last_micros = current_micros; // Use current_micros for better sync
        cycle_count = 0;
    }

    yield(); // Give FreeRTOS scheduler time on Core 1 if needed

    // Update bus address/data for LEDs
    bus.address = i8080_pc();
    bus.data = CPUMemory[bus.address];
    if (i8080_regs_sp() == bus.address) {
      bitSet(bus.state, STACK);
    } else {
      bitClear(bus.state, STACK);
    }
    // Update physical LEDs if enabled
    if (continuousLedUpdates) {
      writeLEDs();
    }

  } else {
    // --- CPU IS HALTED ---
    // Handle all other, non-critical front panel switches.
    // SINGLE-SHOT LOCK CLEARING
    if (!isDown(EXAMINE)) examine_lock = false;
    if (!isDown(EXAMINE_NEXT)) examine_next_lock = false;
    if (!isDown(DEPOSIT)) deposit_lock = false;
    if (!isDown(DEPOSIT_NEXT)) deposit_next_lock = false;
    if (!isDown(RESET)) reset_lock = false;
    if (!isDown(CLR)) clr_lock = false;
    if (!isDown(SINGLE_STEP)) single_step_lock = false;
    if (!isDown(PROTECT)) protect_lock = false;  
    if (!isDown(UNPROTECT)) unprotect_lock = false; 
    if (!isDown(AUX1_UP)) aux1_up_lock = false;
    if (!isDown(AUX1_DOWN)) aux1_down_lock = false;
    if (!isDown(AUX2_UP)) aux2_up_lock = false;
    if (!isDown(AUX2_DOWN)) aux2_down_lock = false;

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
            uint8_t tmpvale = 0x00FF & switches.address; // Data comes from lower 8 bits of Address switches
            deposit(current_pc, tmpvale);
            Serial.printf("DEPOSIT: 0x%04X:%02X\r\n", current_pc, CPUMemory[current_pc]);
            Serial.flush();
            deposit_lock = true;
        }
        delay(KeyDelay);
    }
    if (onRelease(DEPOSIT_NEXT)) {
        if (!deposit_next_lock) {
            uint8_t tmpvale = 0x00FF & switches.address; // Data comes from lower 8 bits of Address switches
            deposit(i8080_pc() + 1, tmpvale); // Deposit at PC+1
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
            // CLR should NOT change the WAIT state directly
            clr_lock = true;
        }
        delay(KeyDelay);
    }
    if (onRelease(SINGLE_STEP)) {
        if (!single_step_lock) {
            Serial.print("SSTEP. ");
            bitClear(bus.state, WAIT); // Allow one instruction
            cycle_count = i8080_instruction(); // Execute and get cycle count
            bitSet(bus.state, WAIT); // Halt again immediately
            bus.address = i8080_pc(); // Update bus for display
            bus.data = CPUMemory[bus.address];
            writeLEDs(); // Update LEDs *after* instruction
            CPUStatus(); // Print status *after* instruction
            Serial.flush();
            single_step_lock = true;
        }
        delay(KeyDelay);
    }
    if (onRelease(UNPROTECT)) {
        if (!unprotect_lock) {
            Serial.println(F("\n--- Configuration Menu ---"));
            // Call the menu handler function
            run_config_menu();
            // After menu exits, re-display the CPU status
            Serial.println(F("--- Exiting Menu ---"));
            CPUStatus(); // Re-print status to clear menu text
            unprotect_lock = true;
        }
        delay(KeyDelay);
    }
    if (onRelease(AUX1_UP)) {
        if(!aux1_up_lock) {
          // Call the helper with the mapped action
          execute_mapped_action(global_aux1_up_action, false);
          aux1_up_lock = true;
        }
    }
    if (onRelease(AUX1_DOWN)) {
      if (!aux1_down_lock) {
          // Call the helper with the mapped action
          execute_mapped_action(global_aux1_down_action, false);
          aux1_down_lock = true;
     }
    }
    if (onRelease(AUX2_UP)) {
       if (!aux2_up_lock) {
            // Call the helper with the mapped action
            execute_mapped_action(global_aux2_up_action, false);
            aux2_up_lock = true;
       }
    }
    if (onRelease(AUX2_DOWN)) {
        if(!aux2_down_lock) {
            // Call the helper with the mapped action
            execute_mapped_action(global_aux2_down_action, false);
            aux2_down_lock = true;
        }
    }
    // ... (rest of halted loop) ...
    // Update bus state and physical LEDs only when halted.
    // Ensure Address/Data reflect the *current* PC after examine/deposit/step
    bus.address = i8080_pc();
    bus.data = CPUMemory[bus.address];
    if (i8080_regs_sp() == bus.address) {
      bitSet(bus.state, STACK);
    } else {
      bitClear(bus.state, STACK);
    }
    // Update LEDs if continuous updates are on, OR if they were just turned off
    // OR if a single step just happened (writeLEDs is called inside SINGLE_STEP logic too)
    if (continuousLedUpdates) {
      writeLEDs();
    }
     delay(10); // Small delay when halted to prevent busy-waiting too fast

  } // End of HALTED block
} 
