#include "hdsk_psram.h"
#include "config.h"
#include <SD.h>
#include <esp_psram.h>

// --- Debugging ---
//#define HDSK_PSRAM_TRACE // Uncomment for detailed logging

// --- 88-4PIO (6820 PIA) Emulation State ---
// These static variables track the internal state of the emulated 4PIO card.
static uint8_t pio_a_control = 0; // Port 161 (0xA1) Control Register
static uint8_t pio_b_control = 0; // Port 163 (0xA3) Control Register
static uint8_t pio_a_ddr = 0;     // Port 160 (0xA0) Data Direction Register (0=In, 255=Out)
static uint8_t pio_b_ddr = 0;     // Port 162 (0xA2) Data Direction Register (0=In, 255=Out)

// --- 88-HDSK Hardware Status Bits (for Port 162) ---
#define HDSK_STAT_BUSY 0x80 // Bit 7: Controller is Busy
#define HDSK_STAT_DRQ  0x02 // Bit 1: Data Request (Ready for data byte in/out)
#define HDSK_STAT_ERR  0x01 // Bit 0: Error

// --- 88-HDSK Hardware Commands (for Port 160) ---
#define HDSK_CMD_READ  0x01 // Read Sector
#define HDSK_CMD_WRITE 0x02 // Write Sector
#define HDSK_CMD_SEEK  0x03 // Seek to Track

// --- 88-HDSK Command Parser State ---
// This enum tracks the multi-byte command sequences
typedef enum {
    HDSK_STATE_IDLE,    // Waiting for a new command
    HDSK_STATE_CMD,     // Command byte just received
    HDSK_STATE_W_TLO,   // Waiting for Track Low byte
    HDSK_STATE_W_THI,   // Waiting for Track High byte
    HDSK_STATE_W_SEC,   // Waiting for Sector byte (and then execute)
    HDSK_STATE_DATA_IN, // Ready for CPU to read data from buffer
    HDSK_STATE_DATA_OUT // Ready for CPU to write data to buffer
} hdsk_cmd_state_t;

static hdsk_cmd_state_t hdsk_cmd_state = HDSK_STATE_IDLE;
static uint8_t hdsk_cmd_byte = 0;    // Stores the active command (READ, WRITE, SEEK)
static uint8_t hdsk_temp_tlo = 0;    // Stores the track low byte
static uint8_t hdsk_temp_thi = 0;    // Stores the track high byte

// Structure to hold the emulated hard disk state
typedef struct {
    uint8_t* image_data;     // Pointer to the PSRAM buffer
    uint16_t track;          // Current track (0-143)
    uint8_t sector;         // Current sector (0-31)
    uint16_t buffer_pos;     // Current byte position within the sector buffer (0-255)
    uint8_t status;         // Status flags (BUSY, DRQ, ERR)
    bool image_loaded;     // Flag indicating if an image is loaded
    bool write_protect;    // Write protect flag
    uint8_t sector_buffer[256]; // --- NEW --- SRAM buffer for one sector
} hdsk_psram_drive_t;

static hdsk_psram_drive_t hdsk_drive;
static uint8_t* psram_hdsk_buffer = nullptr;

// --- Private Helper Functions ---

/**
 * @brief Sets one or more status bits.
 */
static void hdsk_set_status(uint8_t bits) {
    hdsk_drive.status |= bits;
}

/**
 * @brief Clears one or more status bits.
 */
static void hdsk_clear_status(uint8_t bits) {
    hdsk_drive.status &= ~bits;
}

/**
 * @brief Reads a full 256-byte sector from PSRAM into the SRAM sector buffer.
 * @return True on success, false on error.
 */
static bool hdsk_read_sector_to_buffer() {
    if (!hdsk_drive.image_loaded || hdsk_drive.image_data == nullptr) {
        hdsk_set_status(HDSK_STAT_ERR);
        return false;
    }
    
    uint32_t offset = (uint32_t)hdsk_drive.track * HDSK_PSRAM_NUM_SECTORS * HDSK_PSRAM_SECTOR_SIZE +
                      (uint32_t)hdsk_drive.sector * HDSK_PSRAM_SECTOR_SIZE;

    if (offset + HDSK_PSRAM_SECTOR_SIZE > HDSK_PSRAM_IMAGE_SIZE) {
        #ifdef HDSK_PSRAM_TRACE
        Serial.printf("[HDSK ERR] Read bounds error: T%d:S%d (Offset %lu)\n", hdsk_drive.track, hdsk_drive.sector, offset);
        #endif
        hdsk_set_status(HDSK_STAT_ERR);
        return false;
    }

    // Copy 256 bytes from PSRAM to our internal SRAM buffer
    memcpy(hdsk_drive.sector_buffer, hdsk_drive.image_data + offset, HDSK_PSRAM_SECTOR_SIZE);
    hdsk_drive.buffer_pos = 0; // Reset buffer pointer

    #ifdef HDSK_PSRAM_TRACE
    Serial.printf("[HDSK TRACE] Sector Read: T%d:S%d -> SRAM Buffer\n", hdsk_drive.track, hdsk_drive.sector);
    #endif
    return true;
}

/**
 * @brief Writes the full 256-byte SRAM sector buffer to PSRAM.
 * @return True on success, false on error.
 */
static bool hdsk_write_sector_from_buffer() {
    if (hdsk_drive.write_protect) {
        hdsk_set_status(HDSK_STAT_ERR);
        return false;
    }
    if (!hdsk_drive.image_loaded || hdsk_drive.image_data == nullptr) {
        hdsk_set_status(HDSK_STAT_ERR);
        return false;
    }

    uint32_t offset = (uint32_t)hdsk_drive.track * HDSK_PSRAM_NUM_SECTORS * HDSK_PSRAM_SECTOR_SIZE +
                      (uint32_t)hdsk_drive.sector * HDSK_PSRAM_SECTOR_SIZE;

    if (offset + HDSK_PSRAM_SECTOR_SIZE > HDSK_PSRAM_IMAGE_SIZE) {
        #ifdef HDSK_PSRAM_TRACE
        Serial.printf("[HDSK ERR] Write bounds error: T%d:S%d (Offset %lu)\n", hdsk_drive.track, hdsk_drive.sector, offset);
        #endif
        hdsk_set_status(HDSK_STAT_ERR);
        return false;
    }

    // Copy 256 bytes from our internal SRAM buffer to PSRAM
    memcpy(hdsk_drive.image_data + offset, hdsk_drive.sector_buffer, HDSK_PSRAM_SECTOR_SIZE);
    hdsk_drive.buffer_pos = 0; // Reset buffer pointer

    #ifdef HDSK_PSRAM_TRACE
    Serial.printf("[HDSK TRACE] Sector Write: SRAM Buffer -> T%d:S%d\n", hdsk_drive.track, hdsk_drive.sector);
    #endif
    return true;
}

/**
 * @brief --- NEW --- Handles a byte read from the Data Port (160).
 */
static uint8_t hdsk_handle_data_read() {
    uint8_t data = 0xE5; // Default error data

    if (hdsk_cmd_state != HDSK_STATE_DATA_IN) {
        #ifdef HDSK_PSRAM_TRACE
        Serial.printf("[HDSK ERR] INP(160) when not in DATA_IN state!\n");
        #endif
        hdsk_set_status(HDSK_STAT_ERR);
        return data;
    }

    if (hdsk_drive.buffer_pos < HDSK_PSRAM_SECTOR_SIZE) {
        data = hdsk_drive.sector_buffer[hdsk_drive.buffer_pos];
        hdsk_drive.buffer_pos++;

        // If that was the last byte, clear DRQ and go IDLE
        if (hdsk_drive.buffer_pos == HDSK_PSRAM_SECTOR_SIZE) {
            hdsk_clear_status(HDSK_STAT_DRQ);
            hdsk_cmd_state = HDSK_STATE_IDLE;
            #ifdef HDSK_PSRAM_TRACE
            Serial.printf("[HDSK TRACE] Sector read complete.\n");
            #endif
        }
    }
    return data;
}

/**
 * @brief --- NEW --- Handles a byte written to the Data Port (160).
 */
static void hdsk_handle_data_write(uint8_t vale) {
    if (hdsk_cmd_state != HDSK_STATE_DATA_OUT) {
        #ifdef HDSK_PSRAM_TRACE
        Serial.printf("[HDSK ERR] OUT(160) when not in DATA_OUT state!\n");
        #endif
        hdsk_set_status(HDSK_STAT_ERR);
        return;
    }

    if (hdsk_drive.buffer_pos < HDSK_PSRAM_SECTOR_SIZE) {
        hdsk_drive.sector_buffer[hdsk_drive.buffer_pos] = vale;
        hdsk_drive.buffer_pos++;

        // If that was the last byte, clear DRQ, write buffer to PSRAM, go IDLE
        if (hdsk_drive.buffer_pos == HDSK_PSRAM_SECTOR_SIZE) {
            hdsk_clear_status(HDSK_STAT_DRQ);
            hdsk_set_status(HDSK_STAT_BUSY); // Go busy during the "write"
            
            if (!hdsk_write_sector_from_buffer()) {
                hdsk_set_status(HDSK_STAT_ERR);
            }

            hdsk_clear_status(HDSK_STAT_BUSY); // Write is done (instant)
            hdsk_cmd_state = HDSK_STATE_IDLE;
            #ifdef HDSK_PSRAM_TRACE
            Serial.printf("[HDSK TRACE] Sector write complete.\n");
            #endif
        }
    }
}

/**
 * @brief --- NEW --- The main command parsing state machine.
 * Called when a byte is written to Port 160 (and 4PIO is in "run mode").
 */
static void hdsk_handle_command(uint8_t vale) {
    // We are only here if the controller is NOT BUSY
    if (hdsk_cmd_state == HDSK_STATE_IDLE) {
        // This is a new command byte
        hdsk_cmd_byte = vale;
        hdsk_clear_status(HDSK_STAT_ERR); // Clear error on new command

        switch (hdsk_cmd_byte) {
            case HDSK_CMD_READ:
            case HDSK_CMD_WRITE:
            case HDSK_CMD_SEEK:
                hdsk_cmd_state = HDSK_STATE_W_TLO;
                hdsk_set_status(HDSK_STAT_DRQ); // Ask for Track Low
                #ifdef HDSK_PSRAM_TRACE
                Serial.printf("[HDSK CMD] Received 0x%02X. Waiting for TLO.\n", hdsk_cmd_byte);
                #endif
                break;
            default:
                #ifdef HDSK_PSRAM_TRACE
                Serial.printf("[HDSK ERR] Unknown command 0x%02X\n", vale);
                #endif
                hdsk_set_status(HDSK_STAT_ERR);
                break;
        }
        return;
    }

    // We are in the middle of a command sequence
    // The DRQ bit must have been set for the CPU to send this
    hdsk_clear_status(HDSK_STAT_DRQ); // We got the byte, so clear DRQ

    switch (hdsk_cmd_state) {
        case HDSK_STATE_W_TLO:
            hdsk_temp_tlo = vale;
            hdsk_cmd_state = HDSK_STATE_W_THI;
            hdsk_set_status(HDSK_STAT_DRQ); // Ask for Track High
            #ifdef HDSK_PSRAM_TRACE
            Serial.printf("[HDSK CMD] Got TLO: 0x%02X. Waiting for THI.\n", vale);
            #endif
            break;
        
        case HDSK_STATE_W_THI:
            hdsk_temp_thi = vale;
            hdsk_drive.track = (hdsk_temp_thi << 8) | hdsk_temp_tlo;
            hdsk_cmd_state = HDSK_STATE_W_SEC;
            hdsk_set_status(HDSK_STAT_DRQ); // Ask for Sector
            #ifdef HDSK_PSRAM_TRACE
            Serial.printf("[HDSK CMD] Got THI: 0x%02X. Track set to %d. Waiting for SEC.\n", vale, hdsk_drive.track);
            #endif
            break;

        case HDSK_STATE_W_SEC:
            hdsk_drive.sector = vale & 0x1F; // Mask to 0-31
            #ifdef HDSK_PSRAM_TRACE
            Serial.printf("[HDSK CMD] Got SEC: %d. Executing...\n", hdsk_drive.sector);
            #endif

            // This is the last parameter, execute the command
            hdsk_set_status(HDSK_STAT_BUSY); // Go busy
            
            switch (hdsk_cmd_byte) {
                case HDSK_CMD_SEEK:
                    // Seek is instant. We already set hdsk_drive.track.
                    hdsk_cmd_state = HDSK_STATE_IDLE;
                    hdsk_clear_status(HDSK_STAT_BUSY);
                    #ifdef HDSK_PSRAM_TRACE
                    Serial.printf("[HDSK CMD] Seek complete.\n");
                    #endif
                    break;
                
                case HDSK_CMD_READ:
                    if (hdsk_read_sector_to_buffer()) {
                        hdsk_cmd_state = HDSK_STATE_DATA_IN;
                        hdsk_set_status(HDSK_STAT_DRQ); // Signal data is ready to be read
                    } else {
                        hdsk_cmd_state = HDSK_STATE_IDLE; // Error occurred
                    }
                    hdsk_clear_status(HDSK_STAT_BUSY); // Read is done
                    break;

                case HDSK_CMD_WRITE:
                    hdsk_drive.buffer_pos = 0; // Reset buffer pos for writing
                    hdsk_cmd_state = HDSK_STATE_DATA_OUT;
                    hdsk_set_status(HDSK_STAT_DRQ); // Signal ready to receive data
                    hdsk_clear_status(HDSK_STAT_BUSY);
                    break;
            }
            break;

        default:
            // Should not happen
            hdsk_cmd_state = HDSK_STATE_IDLE;
            break;
    }
}


// --- Public API Functions ---

void hdsk_psram_init(void) {
    Serial.print("Initializing PSRAM hard disk buffer...");
    if (psramFound() && psramInit()) {
        psram_hdsk_buffer = (uint8_t*)ps_malloc(HDSK_PSRAM_IMAGE_SIZE);

        if (psram_hdsk_buffer != nullptr) {
            Serial.printf("PSRAM allocation successful (%lu bytes).\n", (uint32_t)HDSK_PSRAM_IMAGE_SIZE);
            hdsk_drive.image_data = psram_hdsk_buffer;
        } else {
            Serial.println("!!! PSRAM allocation FAILED for hard disk !!!");
            hdsk_drive.image_data = nullptr;
        }
    } else {
        Serial.println("!!! PSRAM not found or failed to initialize !!!");
        hdsk_drive.image_data = nullptr;
    }
    hdsk_psram_reset();
}

void hdsk_psram_reset(void) {
    hdsk_drive.track = 0;
    hdsk_drive.sector = 0;
    hdsk_drive.buffer_pos = 0;
    hdsk_drive.status = 0; // --- NEW --- Start not-busy, no-drq, no-err
    hdsk_drive.image_loaded = false;
    hdsk_drive.write_protect = false;

    // --- NEW --- Reset 4PIO and HDSK state
    pio_a_control = 0;
    pio_b_control = 0;
    pio_a_ddr = 0;
    pio_b_ddr = 0;
    hdsk_cmd_state = HDSK_STATE_IDLE;
    hdsk_cmd_byte = 0;
    hdsk_temp_tlo = 0;
    hdsk_temp_thi = 0;

    #ifdef HDSK_PSRAM_TRACE
    Serial.println("[HDSK TRACE] Reset complete.");
    #endif
}

/**
 * @brief Loads an image from SD. This function is unchanged.
 */
bool hdsk_psram_load_image(const char* filename) {
    if (psram_hdsk_buffer == nullptr) {
        Serial.printf("ERR: PSRAM buffer for HDSK is NULL (not allocated)!\n");
        return false;
    }
    hdsk_psram_reset(); 

    File dsk_file = SD.open(filename, FILE_READ);
    if (!dsk_file) {
        Serial.printf("ERR: Failed to open %s for reading.\n", filename);
        return false;
    }
    size_t file_size = dsk_file.size();
    if (file_size == 0) { /* ... (rest of function is identical) ... */ }
    
    // Check file size
    if (file_size != HDSK_PSRAM_IMAGE_SIZE) {
        Serial.printf("WARN: %s size (%d) != expected HDSK size (%d).\n",
                      filename, file_size, HDSK_PSRAM_IMAGE_SIZE);
    }
    size_t total_bytes_to_read = min((size_t)HDSK_PSRAM_IMAGE_SIZE, file_size);
    size_t total_bytes_read = 0;
    const size_t CHUNK_SIZE = 256 * 1024; // 256KB chunk size

    while (total_bytes_read < total_bytes_to_read) {
        size_t bytes_remaining = total_bytes_to_read - total_bytes_read;
        size_t current_chunk_size = min(CHUNK_SIZE, bytes_remaining);
        uint8_t* current_buffer_pos = psram_hdsk_buffer + total_bytes_read;
        size_t bytes_read_this_chunk = dsk_file.read(current_buffer_pos, current_chunk_size);
        if (bytes_read_this_chunk != current_chunk_size) {
            Serial.printf("!!! ERR: Read only %d bytes during chunk read (expected %d) !!!\n", bytes_read_this_chunk, current_chunk_size);
            memset(psram_hdsk_buffer, 0xE5, HDSK_PSRAM_IMAGE_SIZE); 
            dsk_file.close();
            return false;
        }
        total_bytes_read += bytes_read_this_chunk;
         yield();
    }
    dsk_file.close();

    // Pad if the file was smaller
    if (total_bytes_read < HDSK_PSRAM_IMAGE_SIZE) {
        memset(psram_hdsk_buffer + total_bytes_read, 0xE5, HDSK_PSRAM_IMAGE_SIZE - total_bytes_read);
    }

    hdsk_drive.image_loaded = true;
    hdsk_drive.write_protect = false; // --- NEW --- Default to R/W
    hdsk_drive.status = 0x00; // Ready, not busy
    hdsk_cmd_state = HDSK_STATE_IDLE; // Ready for commands
    
    #ifdef HDSK_PSRAM_TRACE
    Serial.printf("[HDSK TRACE] Load successful for %s.\n", filename);
    #endif
    return true;
}


// --- I/O Handlers (REWRITTEN FOR 4PIO) ---

/**
 * @brief --- NEW --- Handles IN operations for the 88-HDSK (0xA0-0xA3).
 */
uint8_t hdsk_psram_in(uint8_t port) {
    uint8_t ret_val = 0xFF; // Default for unmapped reads

    switch (port) {
        // Port A Data/DDR (Port 160)
        case 0xA0:
            // Check 4PIO "Run Mode" (Control Bit 2=1) AND DDR is set to Input
            if (((pio_a_control & 0x04) != 0) && ((pio_a_ddr & 0xFF) == 0)) {
                ret_val = hdsk_handle_data_read();
            } else {
                 #ifdef HDSK_PSRAM_TRACE
                 Serial.printf("[HDSK IN] Port 0xA0 (Data) read while not in read mode! (Ctrl=0x%02X, DDR=0x%02X)\n", pio_a_control, pio_a_ddr);
                 #endif
            }
            break;

        // Port A Control (Port 161)
        case 0xA1:
            ret_val = pio_a_control; // Reading control register is valid
            break;

        // Port B Data/DDR (Port 162) - THIS IS THE STATUS PORT
        case 0xA2:
            // Check 4PIO "Run Mode" (Control Bit 2=1) AND DDR is set to Input
            if (((pio_b_control & 0x04) != 0) && ((pio_b_ddr & 0xFF) == 0)) {
                if (!hdsk_drive.image_loaded) {
                    ret_val = HDSK_STAT_BUSY; // If no image, stay busy
                } else {
                    ret_val = hdsk_drive.status;
                }
            } else {
                 #ifdef HDSK_PSRAM_TRACE
                 Serial.printf("[HDSK IN] Port 0xA2 (Status) read while not in run mode! (Ctrl=0x%02X, DDR=0x%02X)\n", pio_b_control, pio_b_ddr);
                 #endif
            }
            break;

        // Port B Control (Port 163)
        case 0xA3:
            ret_val = pio_b_control; // Reading control register is valid
            break;

        default:
            // Ports 0xA4-0xA7 are not used by the 4PIO
            break;
    }
    
    #ifdef HDSK_PSRAM_TRACE
    if(port >= 0xA0 && port <= 0xA3) { // Log only our ports
      Serial.printf("[HDSK IN] Port 0x%02X -> 0x%02X\n", port, ret_val);
    }
    #endif
    return ret_val;
}

/**
 * @brief --- NEW --- Handles OUT operations for the 88-HDSK (0xA0-0xA3).
 */
void hdsk_psram_out(uint8_t port, uint8_t vale) {
    #ifdef HDSK_PSRAM_TRACE
    if(port >= 0xA0 && port <= 0xA3) { // Log only our ports
      Serial.printf("[HDSK OUT] Port 0x%02X <- 0x%02X\n", port, vale);
    }
    #endif

    switch (port) {
        // Port A Data/DDR (Port 160) - THIS IS THE COMMAND/DATA PORT
        case 0xA0:
            if ((pio_a_control & 0x04) == 0) { // Check if Bit 2 is 0 ("Setup Mode")
                pio_a_ddr = vale; // Write to Data Direction Register
                #ifdef HDSK_PSRAM_TRACE
                Serial.printf("[HDSK 4PIO] Port A (160) DDR set to 0x%02X\n", vale);
                #endif
            } else { // "Run Mode"
                if ((hdsk_drive.status & HDSK_STAT_BUSY) == 0) {
                    // MITS software sends data byte (for write)
                    if (hdsk_cmd_state == HDSK_STATE_DATA_OUT) {
                        hdsk_handle_data_write(vale);
                    } else { // It must be a command or parameter byte
                        hdsk_handle_command(vale);
                    }
                } else {
                    #ifdef HDSK_PSRAM_TRACE
                    Serial.printf("[HDSK WARN] OUT to Port 160 while BUSY!\n");
                    #endif
                }
            }
            break;

        // Port A Control (Port 161)
        case 0xA1:
            pio_a_control = vale;
            #ifdef HDSK_PSRAM_TRACE
            Serial.printf("[HDSK 4PIO] Port A (161) Control set to 0x%02X\n", vale);
            #endif
            break;

        // Port B Data/DDR (Port 162) - THIS IS THE STATUS PORT
        case 0xA2:
            if ((pio_b_control & 0x04) == 0) { // "Setup Mode"
                pio_b_ddr = vale; // Write to Data Direction Register
                #ifdef HDSK_PSRAM_TRACE
                Serial.printf("[HDSK 4PIO] Port B (162) DDR set to 0x%02X\n", vale);
                #endif
            } else { // "Run Mode"
                // Software should not be writing to the status port
                #ifdef HDSK_PSRAM_TRACE
                Serial.printf("[HDSK WARN] OUT to Port 162 in Run Mode!\n");
                #endif
            }
            break;

        // Port B Control (Port 163)
        case 0xA3:
            pio_b_control = vale;
            #ifdef HDSK_PSRAM_TRACE
            Serial.printf("[HDSK 4PIO] Port B (163) Control set to 0x%02X\n", vale);
            #endif
            break;
        
        default:
            // Ports 0xA4-0xA7 are not used
            break;
    }
}