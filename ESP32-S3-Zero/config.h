#ifndef CONFIG_H
#define CONFIG_H

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

// =======================================================
// === WiFi / Network Credentials ===
// =======================================================
#define WIFI_SSID "XXXXXX"
#define WIFI_PASSWORD "XXXXXXXXX"

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
