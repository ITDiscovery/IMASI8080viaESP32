#include "disk.h"
#include "i8080.h"
#include <esp_psram.h> // Include PSRAM functions

//#define DISK_TRACE // Enables high-level I/O tracing

// --- Private Module Variables ---
static disks_t disk_drive;
static uint8_t* psram_disk1 = nullptr; // Pointer for Disk 1 PSRAM buffer
static uint8_t* psram_disk2 = nullptr; // Pointer for Disk 2 PSRAM buffer

// --- Forward Declarations ---
static void disk_flush(disk_t *d);

// --- Private Helper Functions ---

// restore_disk_image prepares the file on SD card (No longer used directly for loading)
static bool restore_disk_image(const char* dsk_filename) {
    const char* dot = strrchr(dsk_filename, '.');
    if (!dot) {
        Serial.printf("ERR: No extension on filename %s\r\n", dsk_filename);
        return false;
    }
    int base_len = dot - dsk_filename;
    char bak_filename[base_len + 5];
    strncpy(bak_filename, dsk_filename, base_len);
    bak_filename[base_len] = '\0';
    strcat(bak_filename, ".BAK");

    Serial.printf("Restoring %s from master %s...\r\n", dsk_filename, bak_filename);

    if (!SD.exists(bak_filename)) {
        Serial.printf("ERR: Master image %s not found!\r\n", bak_filename);
        return false;
    }
    if (SD.exists(dsk_filename)) {
        SD.remove(dsk_filename);
    }

    File bak_file = SD.open(bak_filename, FILE_READ);
    if (!bak_file) {
        Serial.printf("ERR: Could not open master file %s\r\n", bak_filename);
        return false;
    }
    File dsk_file = SD.open(dsk_filename, FILE_WRITE);
    if (!dsk_file) {
        Serial.printf("ERR: Could not create new disk file %s\r\n", dsk_filename);
        bak_file.close();
        return false;
    }

    const size_t buffer_size = 512;
    uint8_t buffer[buffer_size];
    size_t bytes_read;
    while ((bytes_read = bak_file.read(buffer, buffer_size)) > 0) {
        dsk_file.write(buffer, bytes_read);
    }

    dsk_file.flush();
    dsk_file.close();
    bak_file.close();
    Serial.print("Restore complete.\r\n");
    return true;
}


/**
 * @brief (PSRAM Version) Clears the internal write_active flag.
 */
static void disk_flush(disk_t *d) {
#ifdef DISK_TRACE
    if (d->write_active) { // Log only if we are clearing an active write state
      Serial.printf("[TRACE] Flushing RAM Write State T%d:S%d (Clearing write_active)\r\n", d->write_track, d->write_sector);
    }
#endif
    d->write_active = false; // Clear the internal flag
    // buffer_dirty remains true if set by disk_write - needed for future save-to-SD
}


static void disk_select(uint8_t vale) {
    // Flushing here ensures status bit is correct before switching.
    disk_flush(disk_drive.current);

    uint8_t select = vale & 0xf;
    if (select == 0) {
        disk_drive.current = &disk_drive.disk1;
    } else if (select == 1) {
        disk_drive.current = &disk_drive.disk2;
    } else {
        disk_drive.current = &disk_drive.nodisk;
    }
#ifdef DISK_TRACE
    Serial.printf("[TRACE] Disk Select: Drive %d\r\n", select);
#endif
}

/**
 * @brief (PSRAM Version) Handles disk function commands, sets internal write_active flag.
 */
static void disk_function(uint8_t vale) {
    // Flush does not depend on write_active, just buffer_dirty (for future save)
    // But conceptually, a head move ends any active write sequence
    if ((vale & (CONTROL_STEP_IN | CONTROL_STEP_OUT | CONTROL_HEAD_UNLOAD | CONTROL_HEAD_LOAD))) { // <-- ADD THIS
        if (disk_drive.current->write_active) {
          // If a write was active but maybe not finished, force flush status update
          disk_flush(disk_drive.current); // This will clear write_active
        }
    }

#ifdef DISK_TRACE
    Serial.printf("[TRACE] Disk Function: 0x%02X\r\n", vale);
#endif

    if (vale & CONTROL_STEP_IN) {
        if (disk_drive.current->track < (NUM_TRACKS - 1)) disk_drive.current->track++;
    }
    if (vale & CONTROL_STEP_OUT) {
        if (disk_drive.current->track > 0) disk_drive.current->track--;
    }

    // Status bits related to track/head load (These are positive logic internally)
    uint8_t current_status = disk_drive.current->status; // Keep existing status bits (like IE etc if added)
    
    // Update TRACK_0 bit based on current track
    if (disk_drive.current->track == 0) {
        current_status |= STATUS_TRACK_0; // Set bit 6 if at track 0
    } else {
        current_status &= ~STATUS_TRACK_0; // Clear bit 6 otherwise
    }

    // Update HEAD and NRDA based on load/unload commands
    if (vale & CONTROL_HEAD_LOAD) {
        current_status &= ~(STATUS_HEAD | STATUS_NRDA); // Clear bits 2 and 7
        current_status |= STATUS_IE;                    // SET Bit 5 (32) - CP/M expects this!
    }
    if (vale & CONTROL_HEAD_UNLOAD) {
        current_status |= STATUS_HEAD; // Set bit 2 (Head Unloaded)
        current_status |= STATUS_NRDA; // Set bit 7 (Not Ready/No Data - debatable, but safe)
    }

    // Handle Interrupt Enable / Disable commands
    // (This still handles the separate IE/ID commands, which is good practice)
    if (vale & CONTROL_IE) { // CONTROL_IE is bit 4 (16)
        current_status |= STATUS_IE; // Set internal status bit 5 (32)
    }
    if (vale & CONTROL_ID) { // CONTROL_ID is bit 5 (32)
        current_status &= ~STATUS_IE; // Clear internal status bit 5 (32)
    }

    disk_drive.current->status = current_status; // Update the stored status
    disk_drive.current->status = current_status; // Update the stored status

    // Handle Write Enable command - Set internal flag
    if (vale & CONTROL_WE) {
        disk_drive.current->write_active = true; // Set internal flag
        disk_drive.current->buffer_pos = 0;
        disk_drive.current->buffer_dirty = false; // Mark RAM clean initially for this sector
        // Do NOT modify disk_drive.current->status here for ENWD bit
    }
}

static uint8_t disk_sector() {
    // Flush ensures previous write status is cleared before starting new sector operation
    disk_flush(disk_drive.current);

    if (disk_drive.current->sector >= NUM_SECTORS) {
        disk_drive.current->sector = 0;
    }

    // Set the target track/sector for subsequent read/write operations
    disk_drive.current->write_track = disk_drive.current->track;
    disk_drive.current->write_sector = disk_drive.current->sector;

    // Reset the byte position within the sector for reads/writes
    disk_drive.current->buffer_pos = 0;

    uint8_t ret_val = disk_drive.current->sector << 1; // Return sector number (shifted)

#ifdef DISK_TRACE
    Serial.printf("[TRACE] Set Sector T%d:S%d. Returning: 0x%02X\r\n", disk_drive.current->track, disk_drive.current->sector, ret_val);
#endif

    disk_drive.current->sector++; // Increment for the next operation
    return ret_val;
}


static uint8_t disk_read() {
    if (disk_drive.current == &disk_drive.nodisk || disk_drive.current->image_data == nullptr) {
        return 0xFF; // No disk or RAM allocated
    }

    if (disk_drive.current->buffer_pos < SECTOR) {
        // Calculate offset in the full PSRAM image
        uint32_t offset = disk_drive.current->write_track * TRACK + disk_drive.current->write_sector * SECTOR + disk_drive.current->buffer_pos;
        if (offset >= DISK_IMAGE_SIZE) return 0xE5; // Bounds check

        uint8_t data = disk_drive.current->image_data[offset];
        disk_drive.current->buffer_pos++;
        return data;
    } else {
        return 0xE5; // Read past end of sector buffer
    }
}

/**
 * @brief (PSRAM Version) Writes a byte to the PSRAM buffer if write_active is set.
 */
static void disk_write(uint8_t vale) {
     if (disk_drive.current == &disk_drive.nodisk || disk_drive.current->image_data == nullptr) {
        return;
    }

    // Only write if the internal write sequence flag is active
    if (disk_drive.current->write_active) {
        if (disk_drive.current->buffer_pos < SECTOR) {
            uint32_t offset = disk_drive.current->write_track * TRACK + disk_drive.current->write_sector * SECTOR + disk_drive.current->buffer_pos;
            if (offset >= DISK_IMAGE_SIZE) return; // Bounds check

            disk_drive.current->image_data[offset] = vale;
            disk_drive.current->buffer_pos++;
            disk_drive.current->buffer_dirty = true; // Mark RAM changed

            // If buffer is now full, the emulated sector write is complete. Flush state.
            if (disk_drive.current->buffer_pos >= SECTOR) {
                disk_flush(disk_drive.current); // Clears write_active
            }
        } else {
             // If buffer already full but still getting writes, ensure state is flushed
             disk_flush(disk_drive.current);
        }
    }
}

// --- Public API Functions ---

void disk_init(void) {
    Serial.print("Initializing PSRAM disk buffers...\r\n");
    if (psramFound() && psramInit()) {
        psram_disk1 = (uint8_t*)ps_malloc(DISK_IMAGE_SIZE);
        psram_disk2 = (uint8_t*)ps_malloc(DISK_IMAGE_SIZE);

        if (psram_disk1 != nullptr && psram_disk2 != nullptr) {
            Serial.printf("PSRAM allocation successful (%lu bytes each).\r\n", DISK_IMAGE_SIZE);
        } else {
            Serial.print("!!! PSRAM allocation FAILED !!!\r\n");
            if (psram_disk1) heap_caps_free(psram_disk1);
            if (psram_disk2) heap_caps_free(psram_disk2);
            psram_disk1 = nullptr;
            psram_disk2 = nullptr;
        }
    } else {
        Serial.print("!!! PSRAM not found or failed to initialize !!!\r\n");
    }

    disk_drive.disk1.image_data = psram_disk1;
    disk_drive.disk2.image_data = psram_disk2;
    disk_drive.nodisk.image_data = nullptr;
    disk_drive.nodisk.status = 0xFF;
    disk_drive.current = &disk_drive.nodisk;
}

/**
 * @brief (PSRAM Version - No Restore) Loads specified DSK files directly into PSRAM.
 */
bool disk_open_files(const char* file1, const char* file2) {
    // Reset state before loading new images
    disk_drive.current = &disk_drive.nodisk;
    disk_drive.disk1.write_active = false; // Ensure flags are reset
    disk_drive.disk2.write_active = false;

    // Lambda function to load a single disk image into its PSRAM buffer
    auto load_disk = [](const char* filename, uint8_t* target_buffer) -> bool {
        if (target_buffer == nullptr) {
            Serial.printf("ERR: PSRAM buffer for %s is NULL (not allocated)!\r\n", filename);
            return false;
        }

        Serial.printf("Attempting to load %s directly into PSRAM...\r\n", filename);
        File dsk_file = SD.open(filename, FILE_READ);
        if (!dsk_file) {
            Serial.printf("ERR: Failed to open %s for reading.\r\n", filename);
            return false;
        }

        size_t file_size = dsk_file.size();
        if (file_size == 0) {
             Serial.printf("ERR: File %s is empty!\r\n", filename);
             dsk_file.close();
             return false;
        }

        // Determine how many bytes to actually read (up to buffer size)
        size_t bytes_to_read = file_size;
        if (file_size > DISK_IMAGE_SIZE) {
            Serial.printf("WARN: %s size (%d) > buffer size (%d). Reading only first %d bytes.\r\n",
                          filename, file_size, DISK_IMAGE_SIZE, DISK_IMAGE_SIZE);
            bytes_to_read = DISK_IMAGE_SIZE; // Limit read to buffer capacity
        } else if (file_size < DISK_IMAGE_SIZE) {
             Serial.printf("WARN: %s size (%d) < buffer size (%d). Padding will occur.\r\n",
                           filename, file_size, DISK_IMAGE_SIZE);
        }

        Serial.printf("Loading %s (%d bytes) into PSRAM...\r\n", filename, bytes_to_read);
        size_t bytes_read = dsk_file.read(target_buffer, bytes_to_read);
        dsk_file.close();

        if (bytes_read != bytes_to_read) {
            Serial.printf("!!! ERR: Read only %d bytes from %s (expected %d) !!!\r\n", bytes_read, filename, bytes_to_read);
            memset(target_buffer, 0xE5, DISK_IMAGE_SIZE); // Fill buffer on error
            return false;
        }

        // Pad if the file was smaller than the buffer
        if (bytes_read < DISK_IMAGE_SIZE) {
            #ifdef DISK_TRACE
            Serial.printf("[TRACE] Padding PSRAM buffer for %s from offset %d.\r\n", filename, bytes_read);
            #endif
            memset(target_buffer + bytes_read, 0xE5, DISK_IMAGE_SIZE - bytes_read);
        }

        Serial.printf("Load successful for %s.\r\n", filename);

        // --- PSRAM VERIFICATION ---
        Serial.print("[VERIFY] First 16 bytes read back from PSRAM: ");
        for(int i=0; i<16; i++) {
            if(target_buffer[i] < 0x10) Serial.print("0");
            Serial.print(target_buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.print("\r\n");
        // --- END VERIFICATION ---

        return true;
    };

    // Load both disks
    bool success1 = load_disk(file1, psram_disk1);
    bool success2 = load_disk(file2, psram_disk2);

    if (!success1 || !success2) {
         Serial.print("!!! ERR: Failed to load one or both disk images into PSRAM !!!\r\n");
         if (!success1 && psram_disk1) memset(psram_disk1, 0xE5, DISK_IMAGE_SIZE);
         if (!success2 && psram_disk2) memset(psram_disk2, 0xE5, DISK_IMAGE_SIZE);
         return false;
    }

    // Initialize disk struct states (track, sector, status etc.)
    auto init_disk_struct = [](disk_t& disk) {
        disk.track = 0;
        disk.sector = 0;
        // Base status bits (positive logic: Head Unloaded, Not Ready, Assume Not Track 0 initially)
        disk.status = (STATUS_HEAD | STATUS_NRDA | STATUS_CR);
         if (disk.track == 0) { // Set TRACK_0 bit correctly
             disk.status |= STATUS_TRACK_0;
         } else {
             disk.status &= ~STATUS_TRACK_0; // Ensure it's clear if not track 0
         }
        disk.buffer_pos = 0;
        disk.buffer_dirty = false;
        disk.write_active = false; // Initialize write_active flag
    };

    init_disk_struct(disk_drive.disk1);
    init_disk_struct(disk_drive.disk2);

    // Default to drive 1 (A:) selected after loading
    disk_drive.current = &disk_drive.disk1;
    Serial.print("Disk images loaded into PSRAM successfully.\r\n");
    return true;
}

/**
 * @brief (PSRAM Version) Handles IN commands, applying Hansel's negative logic for status.
 */
uint8_t disk_in(uint8_t port) {
    if (disk_drive.current == &disk_drive.nodisk) return 0xFF;

    // NOTE: The "Strict Flush Timing" block that was here has been removed,
    // as it prematurely cleared the write_active flag during status polling.

    switch (port) {
        case 0x08: {
            // Build the byte representing bits that should be OFF in the final output
            uint8_t temp_bits_to_clear = 0;

            // Bit 0: Write Ready Status (0=Ready)
            // This is the bit CP/M is polling for at F84F (ANA D).
            if (disk_drive.current->write_active) {
                 temp_bits_to_clear |= STATUS_ENWD; // Mark bit 0 (1) to be cleared
             }

            // Bit 1: Move Head Allowed Status (0=Allowed)
            // This is required to pass the 'ANI 02' loop at F92F.
            temp_bits_to_clear |= STATUS_MOVE_HEAD; // Mark bit 1 (2) to be cleared

            // Bit 2: Head Load Status (0=Loaded)
            if (!(disk_drive.current->status & STATUS_HEAD)) {
               temp_bits_to_clear |= STATUS_HEAD; // Mark bit 2 (4) to be cleared
            }

            // Bit 3: Controller Ready (0=Ready)
            // DBLROM (boot ROM) checks this.
            if (disk_drive.current->status & STATUS_CR) { // Check internal CR flag (8)
               temp_bits_to_clear |= STATUS_CR; // Mark output bit 3 (8) to be cleared
            }

            // Bit 5: Interrupt Enable (0=Enabled)
            // CP/M (Burcon) checks this (with ANI A2... which we now know is wrong,
            // but the logic is still correct from disk_function).
            if (disk_drive.current->status & STATUS_IE) { // Check internal IE flag (32)
               temp_bits_to_clear |= STATUS_IE; // Mark output bit 5 (32) to be cleared
            }

            // Bit 6: Track 0 Status (0=At Track 0)
            if (disk_drive.current->status & STATUS_TRACK_0) {
                temp_bits_to_clear |= STATUS_TRACK_0; // Mark bit 6 (64) to be cleared
            }

            // Bit 7: Read Data Available Status (0=Available)
             if (!(disk_drive.current->status & STATUS_HEAD) && !disk_drive.current->write_active) {
                 temp_bits_to_clear |= STATUS_NRDA; // Mark bit 7 (128) to be cleared
             }

            uint8_t final_status = ~temp_bits_to_clear;

            // Force this trace line to be active
            #ifdef DISK_TRACE
            Serial.printf("[TRACE] Disk In: Port 0x08, Internal Status=0x%02X, write_active=%d -> Calculated bits_to_clear=0x%02X -> Returning 0x%02X\r\n",
                          disk_drive.current->status, disk_drive.current->write_active, temp_bits_to_clear, final_status);
            #endif
            return final_status;
        }
        case 0x09: return disk_sector();
        case 0x0A: return disk_read();
        default:   return 0xFF;
    }
}

void disk_out(uint8_t port, uint8_t vale) {
     if (disk_drive.current == &disk_drive.nodisk && port != 0x08) return; // Ignore if no disk selected, unless it's select cmd

    switch (port) {
        case 0x08: disk_select(vale); break;
        case 0x09: disk_function(vale); break;
        case 0x0A: disk_write(vale); break;
    }
}