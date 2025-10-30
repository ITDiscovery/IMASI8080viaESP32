#ifndef DISK_H
#define DISK_H

#include <cstdint>
#include <SD.h>
#include <Arduino.h>

// --- Shared Global Definitions ---
enum State {
  INT, WO, STACK, HLTA, IOUT, M1, INP, MEMR, INTE, PROT,
  WAIT, HLDA, USER4, USER3, USER2, USER1
};

struct Bus {
  uint16_t address;
  uint8_t data;
  uint16_t state; // This holds the 16 status bits for the front panel LEDs
};

// --- Shared Global Variables ---
extern Bus bus; // Declare that 'bus' exists elsewhere (in the .ino file)
extern void CPUStatus(); // Declare CPUStatus so other modules can call it

#ifdef __cplusplus
extern "C" {
#endif

void disk_init(void);
bool disk_open_files(const char* file1, const char* file2);
uint8_t disk_in(uint8_t port);
void disk_out(uint8_t port, uint8_t vale);

void disk_write_back(int drive);
const char* disk_get_filename(int drive);
void disk_mount_blank(int drive);

#ifdef __cplusplus
}
#endif

// --- Disk Module Specifics ---
/* read drive status
+---+---+---+---+---+---+---+---+
| R | Z | I | X | C | H | M | W |
+---+---+---+---+---+---+---+---+
W - When 0, write circuit ready to write another byte.
M - When 0, head movement is allowed
H - When 0, indicates head is loaded for read/write
C - When 0, indicates the controller is ready
I - When 0, indicates interrupts enabled
Z - When 0, indicates head is on track 0
R - When 0, indicates that read circuit has new byte to read
*/
#define STATUS_ENWD         1 // Write Enable/Data Ready (Bit 0)
#define STATUS_MOVE_HEAD    2 // Head Move Allowed (Bit 1)
#define STATUS_HEAD         4 // Head Loaded (Bit 2) - 0=Loaded, 1=Unloaded
#define STATUS_CR           8 // Controller Ready (Bit 3) - 0=Ready
#define STATUS_IE           32 // Interrupt Enable (Bit 5) - 0=Enabled
#define STATUS_TRACK_0      64 // Head at Track 0 (Bit 6)
#define STATUS_NRDA         128 // Not Ready Data Available (Bit 7)

#define CONTROL_STEP_IN     1  // Step head towards spindle
#define CONTROL_STEP_OUT    2  // Step head away from spindle
#define CONTROL_HEAD_LOAD   4  // Load head onto disk surface
#define CONTROL_HEAD_UNLOAD 8  // Unload head from disk surface
#define CONTROL_IE          16 // Enable Interrupts - Unused
#define CONTROL_ID          32 // Disable Interrupts - Unused
#define CONTROL_HCS         64 // Head Current Switch? - Unused
#define CONTROL_WE          128 // Write Enable command

// Disk Geometry
#define SECTOR 137UL
#define TRACK (32UL * SECTOR) // 4384
#define NUM_TRACKS 77
#define NUM_SECTORS 32
#define DISK_IMAGE_SIZE (NUM_TRACKS * TRACK) // Correct size = 337568

typedef struct {
    uint8_t* image_data;
    uint8_t track;
    uint8_t sector;
    uint8_t status;         // MITS 88-DCDD status byte
    uint16_t buffer_pos;
    bool buffer_dirty;      // Only relevant for future save-to-SD
    uint8_t write_track;
    uint8_t write_sector;
    bool write_active; // ADD: Hansel's internal flag
} disk_t;

typedef struct {
    disk_t disk1;
    disk_t disk2;
    disk_t nodisk;
    disk_t *current;
} disks_t;

// --- Public Function Declarations ---
#ifdef __cplusplus
extern "C" {
#endif

void disk_init(void);
bool disk_open_files(const char* file1, const char* file2);
uint8_t disk_in(uint8_t port);
void disk_out(uint8_t port, uint8_t vale);

#ifdef __cplusplus
}
#endif

#endif // DISK_H