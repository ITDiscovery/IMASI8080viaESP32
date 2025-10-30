#include "config.h"
#include "disk.h"
#include "i8080.h"
#include <esp_psram.h> // Include PSRAM functions

// --- Private Module Variables ---
static disks_t disk_drive;
static uint8_t* psram_disk1 = nullptr; // Pointer for Disk 1 PSRAM buffer
static uint8_t* psram_disk2 = nullptr; // Pointer for Disk 2 PSRAM buffer
static const char* disk1_filename = nullptr;
static const char* disk2_filename = nullptr;
static bool drive_sector_true = false;

// --- Forward Declarations ---
static void disk_flush(disk_t *d);

// --- Private Helper Functions ---
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
    d->buffer_pos = 0;
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
        disk_drive.current->buffer_pos = 0; // Resets for read
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
    // This function now emulates the polling-based "disk spin" 
    // (case 0011) for non-interrupt mode.

    uint8_t data = 0;
    
    // We simulate the disk spinning by flipping the "sector_true"
    // bit every time this port is polled.
    
    if (drive_sector_true) {
        // State 1: Was true, now flip to false.
        drive_sector_true = false;
        // When sector_true is false, the "T" bit (Bit 0) is 1 (Ready).
        data |= 0x01; 
    } else {
        // State 2: Was false, now flip to true.
        drive_sector_true = true;
        // When sector_true is true, the "T" bit (Bit 0) is 0 (Not Ready).
        
        // We also advance the sector *in this state*, simulating
        // the head moving to the next sector.
        disk_drive.current->sector++;
        if (disk_drive.current->sector >= NUM_SECTORS) {
            disk_drive.current->sector = 0;
        }

        // When we advance to a new sector, any pending read/write burst is over.
        // This resets buffer_pos for the *next* disk_read/disk_write.
        disk_drive.current->buffer_pos = 0;
        
        // If a write was active, it's now "done" (or timed out)
        if (disk_drive.current->write_active) {
            disk_flush(disk_drive.current); // Clears write_active and buffer_pos
        }
    }

    // Now, build the final return value
    // data variable already contains the "T" bit (0 or 1)
    
    // Add the current sector number (shifted left by 1)
    data |= (disk_drive.current->sector << 1);

    // Add unused/reserved bits (matching drive.cpp)
    data |= 0xC0;

    #ifdef DISK_TRACE
    Serial.printf("[TRACE] Disk In: Port 0x09. T%d:S%d. sector_true=%d (T-Bit=%d). Returning: 0x%02X\r\n", 
                  disk_drive.current->track, disk_drive.current->sector, !drive_sector_true, (data & 0x01), data);
    #endif

    return data;
}

static uint8_t disk_read() {
    if (disk_drive.current == &disk_drive.nodisk || disk_drive.current->image_data == nullptr) {
        return 0xFF; // No disk or RAM allocated
    }
    // Set target sector on first read byte
    if (disk_drive.current->buffer_pos == 0) {
        disk_drive.current->write_track = disk_drive.current->track;
        disk_drive.current->write_sector = disk_drive.current->sector;
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
        // Set target sector on first write byte
        if (disk_drive.current->buffer_pos == 0) {
            disk_drive.current->write_track = disk_drive.current->track;
            disk_drive.current->write_sector = disk_drive.current->sector;
            disk_drive.current->buffer_dirty = true; // Mark RAM changed
        }
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
    disk1_filename = file1;
    disk2_filename = file2;
    disk_drive.current = &disk_drive.nodisk;
    disk_drive.disk1.write_active = false; // Ensure flags are reset
    disk_drive.disk2.write_active = false;

    // Lambda function to load a single disk image into its PSRAM buffer
    auto load_disk = [](const char* filename, uint8_t* target_buffer) -> bool {
        if (target_buffer == nullptr) {
            Serial.printf("ERR: PSRAM buffer for %s is NULL (not allocated)!\r\n", filename);
            return false;
        }

        #ifdef DISK_TRACE
        Serial.printf("Attempting to load %s directly into PSRAM...\r\n", filename);
        #endif
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

        #ifdef DISK_TRACE
        Serial.printf("Loading %s (%d bytes) into PSRAM...\r\n", filename, bytes_to_read);
        #endif
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

        #ifdef DISK_TRACE
        Serial.printf("Load successful for %s.\r\n", filename);
        #endif

        #ifdef DISK_TRACE
        // --- PSRAM VERIFICATION ---
        Serial.print("[VERIFY] First 16 bytes read back from PSRAM: ");
        for(int i=0; i<16; i++) {
            if(target_buffer[i] < 0x10) Serial.print("0");
            Serial.print(target_buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.print("\r\n");
        // --- END VERIFICATION ---
        #endif

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

    switch (port) {
        case 0x08: {
            // Build the byte representing bits that should be OFF in the final output
            uint8_t temp_bits_to_clear = 0;

            // Bit 0: Write Ready Status (0=Ready)
            // AFORMAT and PIP both expect this to be 0 (Ready)
            // to proceed. We'll set it as always ready.
            temp_bits_to_clear |= STATUS_ENWD;

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
/**
 * @brief (PSRAM Version) Writes the entire PSRAM buffer for a drive back to its file on the SD card.
 * This is the "Save Disk" function called from the menu.
 * @param drive The drive number (0 or 1) to save.
 */
void disk_write_back(int drive) {
    disk_t* d = nullptr;
    const char* filename = nullptr;

    if (drive == 0) {
        d = &disk_drive.disk1;
        filename = disk1_filename;
    } else if (drive == 1) {
        d = &disk_drive.disk2;
        filename = disk2_filename;
    }

    if (d == nullptr || d->image_data == nullptr || filename == nullptr) {
        Serial.printf("ERR: Save D%d - No disk or filename loaded.\r\n", drive);
        return;
    }

    // Only save if the disk is "dirty" (has been written to)
    if (!d->buffer_dirty) {
        Serial.printf("Save D%d: Buffer not modified, skipping.\r\n", drive);
        return;
    }

    Serial.printf("Saving D%d (%s) to SDCard...", drive, filename);
    
    // Open the file for writing. This will create it or overwrite it.
    File dsk_file = SD.open(filename, FILE_WRITE);
    if (!dsk_file) {
        Serial.printf("\r\n!!! FAILED to open %s for writing !!!\r\n", filename);
        return;
    }

    // Write the entire PSRAM buffer to the file
    size_t bytes_written = dsk_file.write(d->image_data, DISK_IMAGE_SIZE);
    dsk_file.close();

    if (bytes_written == DISK_IMAGE_SIZE) {
        Serial.println(" OK.");
        d->buffer_dirty = false; // Clear the dirty flag now that it's saved
    } else {
        Serial.printf("\r\n!!! FAILED to write %d bytes to %s (wrote %d) !!!\r\n", DISK_IMAGE_SIZE, filename, bytes_written);
    }
}

/**
 * @brief Gets the filename for the specified drive.
 * @param drive The drive number (0 or 1).
 * @return A const char pointer to the filename, or "N/A".
 */
const char* disk_get_filename(int drive) {
    if (drive == 0) {
        return (disk1_filename != nullptr) ? disk1_filename : "N/A";
    } else if (drive == 1) {
        return (disk2_filename != nullptr) ? disk2_filename : "N/A";
    }
    return "N/A";
}
/**
 * @brief (Hardcoded) Mounts a "blank" disk (all 0xE5) into the specified drive's PSRAM buffer.
 * Sets the filename to /BLANK_A.DSK or /BLANK_B.DSK to allow saving.
 * @param drive The drive number (0 or 1) to blank.
 */
void disk_mount_blank(int drive) {
    disk_t* d = nullptr;
    uint8_t* target_buffer = nullptr;
    const char* new_filename = nullptr;

    if (drive == 0) {
        d = &disk_drive.disk1;
        target_buffer = psram_disk1;
        disk1_filename = "/BLANK_A.DSK"; // Hardcode new filename
        new_filename = disk1_filename;
    } else if (drive == 1) {
        d = &disk_drive.disk2;
        target_buffer = psram_disk2;
        disk2_filename = "/BLANK_B.DSK"; // Hardcode new filename
        new_filename = disk2_filename;
    } else {
        return; // Invalid drive
    }

    if (target_buffer == nullptr) {
        Serial.printf("ERR: Cannot mount blank, PSRAM buffer for D%d is NULL.\r\n", drive);
        return;
    }

    Serial.printf("Mounting blank (0xE5) image in Drive %d (as %s)...\r\n", drive, new_filename);
    
    // Fill the entire PSRAM buffer with 0xE5
    memset(target_buffer, 0xE5, DISK_IMAGE_SIZE);

    // Reset the disk structure's state
    d->track = 0;
    d->sector = 0;
    d->status = (STATUS_HEAD | STATUS_NRDA | STATUS_CR | STATUS_TRACK_0); // At track 0
    d->buffer_pos = 0;
    d->buffer_dirty = true; // Mark it as dirty so it *will* be saved!
    d->write_active = false;

    // If this was the current drive, flush its state to be safe
    if (disk_drive.current == d) {
        disk_flush(d);
    }
    
    Serial.println("Blank mount complete.");
}
