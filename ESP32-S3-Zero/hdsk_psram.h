#ifndef HDSK_PSRAM_H
#define HDSK_PSRAM_H

#include <Arduino.h>

// --- Constants ---
#define HDSK_PSRAM_IMAGE_SIZE 1179648 // 144 tracks * 32 sectors/track * 256 bytes/sector
#define HDSK_PSRAM_NUM_TRACKS 144
#define HDSK_PSRAM_NUM_SECTORS 32
#define HDSK_PSRAM_SECTOR_SIZE 256

// --- Public API Functions ---

/**
 * @brief Initializes the PSRAM buffer for the hard disk image.
 * Must be called once in setup().
 */
void hdsk_psram_init(void);

/**
 * @brief Resets the emulated hard disk state (track, sector, status).
 * Call this on system reset.
 */
void hdsk_psram_reset(void);

/**
 * @brief Loads a hard disk image file from SD card into the PSRAM buffer.
 * @param filename The path to the 1.1MB DSK image file (e.g., "/ADAHD.DSK").
 * @return True if successful, false otherwise.
 */
bool hdsk_psram_load_image(const char* filename);

// --- I/O Handlers (Called by i8080_hal) ---

/**
 * @brief Handles IN operations for the PSRAM hard disk ports (0xA0-0xA7).
 * @param port The I/O port number.
 * @return The data byte read from the specified port.
 */
uint8_t hdsk_psram_in(uint8_t port);

/**
 * @brief Handles OUT operations for the PSRAM hard disk ports (0xA0-0xA7).
 * @param port The I/O port number.
 * @param vale The data byte written to the port.
 */
void hdsk_psram_out(uint8_t port, uint8_t vale);

#endif // HDSK_PSRAM_H