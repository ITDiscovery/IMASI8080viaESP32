#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

//IMASI 8080
enum Control { EXAMINE, EXAMINE_NEXT, DEPOSIT, DEPOSIT_NEXT,
    RESET, CLR, RUN, STOP, SINGLE_STEP, SINGLE_STEP_,
    PROTECT, UNPROTECT, AUX1_UP, AUX1_DOWN, AUX2_UP, AUX2_DOWN };
/**
 * @brief Defines the bit positions for the 16-bit front panel control switch register.
 * * This enum maps each physical switch to its corresponding bit (0-15) in the 
 * 'switches.control' variable, which is read from the 74HC165 shift registers.
 * * The logic in loop() primarily uses onRelease() checks when the CPU is HALTED
 * to trigger these functions.
   * @brief EXAMINE: Sets PC to the address switches.
   * Halts the CPU, reads the 16-bit value from 'switches.address' (A0-A15),
   * and sets the 8080's Program Counter to that address.
   * @brief EXAMINE_NEXT: Increments PC.
   * Halts the CPU and increments the 8080's Program Counter by one.
   * @brief DEPOSIT: Writes data from A0-A7 to memory at PC.
   * Halts the CPU, reads the 8-bit value from the lower address switches (A0-A7),
   * and writes that byte to the memory location pointed to by the current PC.
   * @brief DEPOSIT_NEXT: Increments PC, then writes data.
   * Halts the CPU, increments the PC by one, and *then* writes the 8-bit value 
   * from switches A0-A7 to the new memory location.
   * @brief RESET: Resets the entire system.
   * Calls i8080_init() (resets CPU, sets PC to 0), resets the hard disk
   * controller, and halts the CPU.
   * This is a "universal" switch, checked even when the CPU is running.
   * @brief CLR: Clears status flags.
   * Clears the PROT (Protect), INTE (Interrupt Enable), and HLDA (Hold Acknowledge)
   * flags in the 'bus.state' variable.
   * @brief RUN: Resumes CPU execution.
   * Clears the WAIT flag, allowing the main loop() to resume i8080_instruction() calls.
   * @brief STOP: Halts CPU execution.
   * Sets the WAIT flag, pausing the main loop()'s execution of i8080_instruction().
   * This is a "universal" switch, checked even when the CPU is running.
   * @brief SINGLE_STEP: Executes one instruction.
   * While HALTED, this clears WAIT, calls i8080_instruction() once, and immediately
   * sets WAIT again. It also forces an update of the LEDs and prints CPU status.
   * @brief SINGLE_STEP_ (Note: Trailing underscore): Toggles Debug Mode.
   * This is a *separate switch* from SINGLE_STEP. It toggles the 'debugStepModeActive'
   * boolean, which causes the CPU to run very slowly and print status
   * after every instruction.
   * This is a "universal" switch, checked even when the CPU is running.
*/
/*Original
enum Control { STOP, SINGLE_STEP, EXAMINE, DEPOSIT, 
     RUN, SINGLE_STEP_, EXAMINE_NEXT, DEPOSIT_NEXT, 
     AUX2_UP, AUX1_UP, PROTECT, RESET, AUX2_DOWN, 
     AUX1_DOWN, UNPROTECT, CLR };
*/
/*ALTAIR 8800
enum Control { STOP, RUN, SINGLE_STEP, SINGLE_STEP_, 
     EXAMINE, EXAMINE_NEXT, DEPOSIT, DEPOSIT_NEXT, 
     RESET, CLR, PROTECT, UNPROTECT, AUX1_UP, AUX1_DOWN,\
     AUX2_UP, AUX2_DOWN };
*/

//enum State {
//  INT, WO, STACK, HLTA, IOUT, M1, INP, MEMR, INTE, PROT,
//  WAIT, HLDA, USER4, USER3, USER2, USER1 };
//MEMR--The memory bus will be used for memory read data.
//INP--The address bus contains the address of an input device
//M1--The CPU is processing the first machine cycle of an instruction.
//OUT--The address contains the address of an output device.
//HLTA--A HALT instruction has been executed and acknowledged.
//STACK--The address bus holds the Stack Pointer's pushdown stack address
//WO--Operation in the current machine cycle will be a WRITE memory or OUTPUT function
//.   otherwise it's a READ memory or INPUT
//INT--An interrupt request has been acknowledged.
//INTE--An interrupt has been enabled.
//PROT—The memory is protected.
//WAIT--The CPU is in a WAIT state.
//HLDA--A HOLD has been acknowledged.

// USER3 and USER4 are kept in the enum for reference, but their bits in bus.state are now ignored
// in favor of the dedicated boolean flags below to prevent conflicts with 8080 code.

// --- Add this enum near the top of config.h ---
/**
 * @brief Defines the actions that can be mapped to AUX keys or Auto-Boot.
 */
enum MappableAction {
  ACTION_NONE,
  ACTION_ENTER_MENU,
  ACTION_SAVE_ALL_DISKS,
  ACTION_LOAD_8K_BASIC,
  ACTION_LOAD_MBASIC_DISK,
  ACTION_LOAD_CPM_DISK
};

// --- Add these to the NVS Key Definitions ---
extern const char* NVS_AUX1_UP_ACTION;
extern const char* NVS_AUX1_DOWN_ACTION;
extern const char* NVS_AUX2_UP_ACTION;
extern const char* NVS_AUX2_DOWN_ACTION;
extern const char* NVS_AUTOBOOT_ENABLED;
extern const char* NVS_AUTOBOOT_ACTION;

// --- Add these extern declarations for new global variables ---
// (These will be defined in the .ino file)
extern uint8_t global_aux1_up_action;
extern uint8_t global_aux1_down_action;
extern uint8_t global_aux2_up_action;
extern uint8_t global_aux2_down_action;
extern bool global_autoboot_enabled;
extern uint8_t global_autoboot_action;

// Common Ports  (ports with "*" are emulated)
// 00-01  * MITS 88-SIO. (Serial2)
// 02       printer (OkiData, C700 or generic)
// 03       printer (OkiData, C700 or generic, output only)
// 04       Cromemco disk controller
// 04-05    ProTec VDM1 keyboard (input only)
// 06-07    MITS 88-ACR
// 08-0A  * MITS 88-DCDD disk controller
// 0E       Cromemco Dazzler
// 0F       Cromemco Dazzler (output only)
// 10-11    MITS 88-2SIO port A (USB Serial)
// 12-13    MITS 88-2SIO port B (Telnet Server)
// 14-15    second MITS 88-2SIO port A
// 16-17    second MITS 88-2SIO port B
// 18       Cromemco D+7A board (Dazzler support, input only)
// 19-1C    Cromemco D+7A board (Dazzler support)
// 1D-1F    Cromemco D+7A board (Dazzler support, output only)
// 30-34    Cromemco disk controller
// 40       Cromemco disk controller (output only)
// A0-A8    MITS hard disk controller (4PIO interface board)
// C8       ProTec VDM-1 (output only)
// F0       Cromemco disk controller (input only)
// F8-FD    Tarbell disk controller
// F8-F9  * MITS 88-VI Real-Time Clock (RTC). F8=Control, F9=Data
// FE     * MITS 88-VI interrupt controller (output only)
// FF     * front-panel sense switches (input only)

// --- Serial Console Configuration ---
// 0 = 88-SIO (Port 0x00) is Primary Console on USB Serial
// 1 = 88-2S-IO (Port 0x10) is Primary Console on USB Serial
#define CONSOLE_IS_2SIO 1

// --- Debugging Flags ---
// Comment out to disable specific debug logs
//#define DEBUG       // Enables general debug functions
//#define DEBUGIO     // Enables high-volume I/O port tracing
//#define DISK_TRACE  // Enables disk operation tracing in disk.cpp
//#define NETWORK_TRACE //Enables network operation tracing in network_task.cpp
//#define INTER_TIME   //Enables time and interrupt tracing
//#define HDSK_PSRAM_TRACE //Enables ZDisk/Hard disk tracing

// =======================================================
// === WiFi / Network Credentials ===
// =======================================================
// NVS Keys for Wi-Fi
#define DEFAULT_WIFI_SSID "XXXXX"
#define DEFAULT_WIFI_PASS "XXXXXXXXXXXXXXXX"
extern const char* NVS_WIFI_SSID ;
extern const char* NVS_WIFI_PASS ;

// =======================================================
// === Default Program Load Paths ===
// =======================================================
#define DEFAULT_MBASIC_DSK0 "/MBASIC.DSK"  //Altair Disk Basic 4.1
//#define DEFAULT_MBASIC_DSK0 "/ALTAIRDOS.DSK"
#define DEFAULT_MBASIC_DSK1 "/GAMES35F.DSK"
#define DEFAULT_CPM_HDSK "/ADAHD.DSK"
#define DEFAULT_CPM_DSK0 "/CPM22B23-56K.DSK"
#define DEFAULT_CPM_DSK1 "/ZORK.DSK"

// --- Add these to your NVS Key Definitions ---
extern const char* NVS_MBASIC_DSK0;
extern const char* NVS_MBASIC_DSK1;
extern const char* NVS_CPM_HDSK;
extern const char* NVS_CPM_DSK0;
extern const char* NVS_CPM_DSK1;

// --- Add these extern declarations for new global variables ---
// (These will be defined in the .ino file)
extern String global_mbasic_dsk0;
extern String global_mbasic_dsk1;
extern String global_cpm_hdsk;
extern String global_cpm_dsk0;
extern String global_cpm_dsk1;

// =======================================================
// === NTP & Time Zone Configuration ===
// =======================================================
#define NTP_SERVER "pool.ntp.org"
// This is the offset for your *Standard* Time (e.g., CST = UTC-6)
// The daylight offset will be added to this automatically.
// USA Central Time (CST) = -6
#define NTP_GMT_OFFSET_SEC (-6 * 3600)
// This is the Daylight Saving offset (e.g., +1 hour)
#define NTP_DAYLIGHT_OFFSET_SEC 3600

// --- Pin Definitions ---
// 74HC595 (LED Output)
#define LlatchPin 2
#define LclockPin 3
#define LdataPin 1

// 74HCT165 (Switch Input)
#define SlatchPin 6
#define SclockPin 5
#define SdataPin 4

// Hardware Serial (Serial2)
#define Serial2RX 14
#define Serial2TX 15

// Misc
#define AnalogOut 7
#define LED_COUNT 1

// SD Card (SPI)
#define SD_CS_PIN 10
#define SD_CLK_PIN 12
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 13

// --- Timing Delays ---
#define Cdelay 4   // Switch Clock Delay
#define Ldelay 1   // Switch Latch Delay
#define KeyDelay 20  // Switch Debounce Delay

#endif // CONFIG_H
