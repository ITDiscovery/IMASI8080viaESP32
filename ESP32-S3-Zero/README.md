Completed Tasks ✅
Your emulator has achieved several major milestones and is now a stable, interactive system.
Updated and renamed Github Repository to match project effort.
Stable CPU & BASIC: The CPU core is stable, and multiple versions of disk-based Microsoft BASIC boot and run perfectly.
Modular Code: UART and Disk I/O logic have been successfully modularized and are working correctly.
Flexible Serial I/O: Implemented and debugged a compile-time #define to easily swap the console between the emulated 88-SIO and 88-2SIO boards.
CP/M Boot Success: You have fixed the critical serial I/O logic, resolving the Function 10 line-buffering issue. The simulator now successfully boots multiple CP/M versions to a stable, responsive A> prompt.
Application Success: The system can change drives, list directories, and successfully load and run complex, disk-intensive applications like ZORK I.
Performance: The RUN state gate and speed throttling correctly maintain a stable ~2 MHz emulation speed.
Robust Debounce: Implemented a targeted debounce for HALT-mode switches that does not impact RUN mode performance.

Remaining Issues & Next Steps
Here is the revised plan focusing on the most critical next steps to create a fully-featured CP/M machine.
Task
Priority
Note
Implement Disk Write Functionality
CRITICAL
With a stable read-only system, this is the undisputed top priority. Implementing writes will unlock the full power of CP/M, enabling file saving, copying with PIP.COM, and using development tools. This involves adding logic to disk_write() in disk.cpp to modify .DSK files on the SD card. 
Create a Global Config File
HIGH
To prevent future build system issues and improve maintainability, move all configuration switches (like CONSOLE_IS_2SIO) into a single config.h header file. This ensures definitions are visible across all .cpp files.
Finalize C/C++ Build Refactoring
MEDIUM
Applying extern "C" wrappers to all necessary C headers (i8080.h, i8080_hal.h) is a key step for ensuring robust, error-free linking and long-term stability.
Add #ifdefWrappers for Debug Code
MEDIUM
Wrap all remaining diagnostic Serial.print statements in #ifdef DEBUGIO blocks to create a clean, silent final build that can be easily toggled for debugging.
Refactor and Remove Unused Code
LOW
Streamline the codebase by removing non-essential test functions and old debug code to improve readability and reduce final binary size.
Emulate 88-VI/RTC Board
LOW
While a useful feature, this is no longer a critical-path item for basic CP/M functionality. It can be implemented later to improve compatibility with software that requires a real-time clock.
Emulate Hard Disk Controller
FUTURE
Support for hard disk images remains a great long-term goal for expanding the emulator's capabilities beyond floppy disks.
Expand WiFi functionality
FUTURE
Use WiFi to allow additional serial port connections (Telnet), remote file transfers, or other advanced tasks.
Menu System
FUTURE
Provide a way for users to enter WiFI, what AUX keys boot to, etc.
Boot Sense 
FUTURE
Read the panel switches on boot up to allow changes to the AUX swtiches.
Configuration/Menu System

• User Interface: Provide a user interface (perhaps via serial monitor commands) to:
  • Configure Wi-Fi credentials.
  • Customize which disk images are loaded by the AUX switches.
  • Manually trigger the saving of PSRAM disks to the SD card.
• Write Back Mechanism: Implement a method (e.g., a combination of an AUX key and a serial command) to write the contents of the PSRAM disk buffers back to their corresponding `.DSK` files on the SD card, making changes persistent.

Minidisk Support

• Enhanced `disk_open_files`: Detect smaller disk image file sizes (e.g., < 100KB) and automatically adjust the geometry parameters (NUM_TRACKS, NUM_SECTORS) used for offset calculations, similar to Mr. Hansel's `drive.cpp` logic.
• PSRAM Logic Update: Update the PSRAM allocation or loading logic accordingly if needed.
