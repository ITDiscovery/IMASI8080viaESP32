
# IMASI 8080 on ESP32 Project

## Project Status Summary

The project has successfully completed the core emulation, achieved a stable **CP/M system**, and implemented crucial **Disk Read, Disk Write, WiFi and RTC** functionality. The project state is highly stable, with efforts now shifting to user experience and advanced hardware emulation.

* **Core Emulation is Stable:** The CPU, BASIC, and CP/M 2.2 B23 environments are fully functional, running complex applications like ZORK I and utilities like PIP.COM.
* **Disk Persistence is Solved:** The read/write/write-back loop is now complete.
* **Telnet/Network is Integrated:** Inter-Core Communication is verified, and Telnet is correctly mapped to standard 88-2SIO ports.
* **Next Critical Step:** Focus shifts from core functionality to the **Configuration/Menu System (Phase 3)** to allow users to manage Wi-Fi, disks, and keys easily.
* **Hard Drive Emulation:** Is in place but not tested (it will load Z80Packs).
* **Phase 3 is complete** with the addition of the **Disk, Configuration and Diags menus**.

***

## Phase 3: Menu System Details

### Disk Management Menu

The menu first displays the filenames currently loaded into Drive 0 (**A:**) and Drive 1 (**B:**).

Here are the available commands:

* **[L] Load Disk from SDCard...**
    * Prompts to select Drive 0 or 1.
    * Asks for the full path of the disk image from the SD card (e.g., `/CPM.DSK`).
    * Loads that file into the selected drive's PSRAM buffer.
* **[M] Mount Blank Disk...**
    * Prompts to select Drive 0 or 1.
    * Fills that drive's PSRAM buffer with a "blank" (all `0xE5`) disk image.
    * The disk is assigned a default filename (like `/BLANK_A.DSK`) so it can be saved later.
* **[S] Save Virtual Disk...**
    * Prompts to select Drive 0 or 1.
    * Saves the current contents of that drive's PSRAM buffer back to its original file on the SD card. (This is how you make changes persistent.)
* **[A] Save All Changed Disks**
    * Shortcut that automatically saves both Drive 0 and Drive 1 back to their respective files on the SD card.
    * The disk module is smart enough to skip saving if no changes have been made.
* **[U] Unmount Disk...**
    * Prompts to select Drive 0 or 1.
    * "Ejects" the disk by clearing its PSRAM buffer and filename.
* **[D] List Disks on SDCard**
    * Lists the files and directories found in the root (`/`) of the SD card, allowing you to see available disk images.
* **[P] Set Program Load Paths**
    * Takes you to a sub-menu where you can define the default disk image paths (e.g., `global_mbasic_dsk0`, `global_cpm_hdsk`) that are used by the Auto-Boot or AUX key actions.
    * These paths are saved permanently to NVS (EEPROM).
* **[B] Back to Main Menu**
    * Exits the Disk Management menu and returns to the main configuration menu.

### Configuration Menu

* **[W] Set Wi-Fi Credentials:** Prompts for an SSID and a password, then immediately writes these values to the **EEPROM (or NVS)** so they are persistent.
* **[K] Set AUX Key Mappings:** Allows mapping the physical panel switches (AUX1, AUX2) to specific functions (e.g., "Enter Menu," "Trigger Disk Save").
* **[A] Set Auto-Boot Options:** Allows setting a flag to **"Auto-boot into CP/M"** on power-up, skipping the menu, and specifying default disks to load (e.g., "Load CPMA.DSK into A:").
* **[S] Save All Settings to EEPROM:** Master save for settings from [P], [K], and [A], committing them to persistent storage at once. (Wi-Fi is saved separately).

***

## Phase 4: New Feature Development (Current Focus)

| Task | Detail |
| :--- | :--- |
| **Emulate Hard Disk Controller** | The HDD emulation is complete, but the ESP-32 at 2Mb PSRAM lacks memory. A wedge was created to allow the load and read of a **Z80Pack 1.1Mb file** to patch CP/M on the fly and mount the filesystem as Read-Only. |
| **Minidisk Support** | Auto-detect and support smaller disk image formats by updating `disk_open_files` and adjusting geometry parameters (`NUM_TRACKS`, `NUM_SECTORS`). |
| **Verify and Post Rev D board** | Finalize and verify the new Rev D board, which takes an **ESP-32-S3-Zero**. |

### Phase 4: Future Enhancements (Deep Emulation)

These tasks involve deep hardware emulation.

| Task | Detail |
| :--- | :--- |
| **Emulate Memory Mapping** | Likely necessary for supporting more advanced operating systems or software that utilizes bank switching. |
| **Emulate Z-80 processor** | A major enhancement that would significantly expand the range of compatible software. |

***

## Completed Tasks ✅ (Consolidated)

### Feature Summary: MITS 88-VI Real-Time Clock (RTC) Emulation

This implements a full, read/write emulation of the MITS 88-VI Real-Time Clock card, making the ESP32's current system time accessible to the 8080 CPU and CP/M software.

* **I/O Port Mapping:** Emulated on standard 88-VI ports: `OUT 0xF8` (Control) and `IN/OUT 0xF9` (Data).
* **Time Source:** Leverages the ESP32's main system clock, synchronized with an **NTP server**.
* **Read Operation:** `IN 0xF9` reads a static snapshot of the system time, latched upon `OUT 0xF8`.
* **Write Operation:** Writing to the final register (`Register 7, Year`) triggers a function to decode all BCD registers and update the ESP32's system clock using `settimeofday()`.
* **Configuration:** NTP server, GMT offset, and Daylight Saving offset moved to `config.h`.

### 88-VI/RTC Interrupt Controller Emulation

This activates the second, more advanced feature: the periodic vectored interrupt controller, required for time-slicing in multi-user operating systems.

1.  **I/O Port 0xFE: The Interrupt Controller**
    * `OUT 0xFE` (Control): Configures the controller to **enable/disable** the periodic interrupt, set the **interrupt rate** (e.g., 1Hz, 10Hz, 100Hz), and set the **interrupt vector** (e.g., RST 0 through RST 7).
    * `IN 0xFE` (Status): Returns a status code to signal the OS that the controller is present.
2.  **Main Loop Timer Mechanism**
    * A new non-blocking timer in `loop()` checks if the feature is enabled, the time has elapsed, and the 8080's interrupt enable (**INTE**) flag is set.
    * If all conditions are met, it fires the interrupt using the specific vector (e.g., `0xF7` for RST 6) requested by the OS.

### Disk and I/O Functionality

* **Disk Write Functionality:** Implemented with a **Write Back Mechanism** to save PSRAM disk buffers back to the SD card.
* **Disk Emulation Solved & Clean CP/M Boot:** Fixed initial loops and serial I/O logic; successfully boots multiple CP/M versions to a stable, responsive `A>` prompt.
* **Flexible Serial I/O:** Implemented a compile-time `#define` to easily swap the console between the emulated 88-SIO and 88-2S-IO boards.
* **Application Success:** System can change drives, list directories, and successfully load and run complex, disk-intensive applications like **ZORK I**.

### WiFi Functionality

* **Established Inter-Core Communication:** Successfully implemented two **FreeRTOS queues** (`telnet_to_emu_queue` and `emu_to_telnet_queue`) for data exchange between Core 1 and Core 0.
* **Mapped Telnet to Correct Ports:** Corrected port mapping to the standard 88-2SIO Port
