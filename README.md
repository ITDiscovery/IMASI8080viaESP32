# IMASI8080viaESP32

This project creates an emulator of an **IMSAI 8080** front panel experience using an **ESP32** microcontroller. It leverages TTL shift registers to connect a large number of DIP switches and LED bars via a custom BlinklinLights PCB. While this project's goal is to create a complete representation, the software and hardware can be leveraged for many other uses.

---

## 🌟 Design Goals

1.  Use the **ESP32** and add the required I/O via **74HCT165** (input) and **74LS595** (output) shift registers. (The AVR128 was dropped due to the need to leverage external memory, which made the whole design too slow).
2.  Enable connectivity to a **Raspberry Pi** for enhanced functionality.
3.  Create smaller "Blinkin Boards" to extend the display, such as a 7-Segment module to show address and data bus values.
4.  Allow for emulation of other classic machines, such as a **PDP-8**.

---

## 🛠️ Build and Setup Instructions

This section outlines the steps required to build and flash the firmware for the **RevB** (Current Working) version.

### 1. Prerequisites

You'll need the following installed on your development machine:

* **Arduino IDE** (or PlatformIO).
* **ESP32 Board Support Package** for the Arduino IDE.
* **Git** for cloning the repository.

### 2. Required Libraries

The project relies on these libraries, which must be installed via the Arduino Library Manager:

| Library | Purpose |
| :--- | :--- |
| **`SPI`** | Communication with SPI devices (e.g., SD Card) |
| **`SD`** | Handling the SD card for disk images |
| **`WiFi`** | Network connectivity |
| **`Adafruit_NeoPixel`** | Controls the built in LED on the ESP32|

### 3. Build and Flash Steps

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/ITDiscovery/IMASI8080viaESP32.git](https://github.com/ITDiscovery/IMASI8080viaESP32.git)
    cd IMASI8080viaESP32
    ```
2.  **Open the `.ino` file** in the Arduino IDE.
3.  **Configure Board Settings:** Select the appropriate ESP32 board in the Arduino IDE's **Tools** menu.
    > **⚠️ SD Card Note:** This project uses a custom pin mapping (Pins 10-13) for the SD Card SPI interface. If you are using the **Waveshare ESP32-S3-Zero** variant, you may need to modify the `pins_arduino.h` file.
4.  **Configure Network:** Update the `ssid` and `password` variables in the `.ino` file with your preferred Wi-Fi network credentials (if using the WiFi feature).
5.  **Compile and Upload:** Click the **Upload** button to compile the code and flash it to your ESP32.

---

## 🏗️ Hardware Revisions

The project has undergone several hardware revisions:

* **AVR Hardware (Dropped):** The initial design using an AVR was dropped due to speed issues caused by the need for external memory. Early concepts using the **AVR128DB28 Flash** were abandoned because newly posted specifications showed only a **1K write cycle endurance**. While using an external **23LC512 SRAM** was considered (and would have allowed for an **AVR64DB28**), this approach was too slow and required an additional 4 pins. The decision was made to switch to the ESP32 platform.
* **RevA:** Boards would require too many modifications to function.
* **RevB:** This version requires a mod to go from Pin 1 of U2 to Pin 24 of H3, and it used a smaller footprint AVR128DB28.
* **RevC (Current Design):** Board has been heavily bodged to accomdate an ESP32 and a 5V-to-3.3V transceiver, but worked well during AVR development.
* **RevD (Future Design):** Has the ESP32-S3-Zero and the transciever designed onto the board. The board has not been produced or tested.
---

## 🔌 Pin Definitions and Wiring

The following tables detail the pin connections for the ESP32 and the external 40-pin headers (H1 and H2).

### ESP32 Pin Mapping (To H1/H2 Headers)

| Function | H1/H2 Pin | ESP32-S3-Zero Pin |
| :--- | :--- | :--- |
| **LED Data (SER)** | 36 | 1 (`LdataPin`) |
| **LED Latch (RClk)** | 38 | 2 (`LlatchPin`) |
| **LED Clock (SRCK)** | 40 | 3 (`LclockPin`) |
| **SW Clock (CP)** | 37 | 5 (`SclockPin`) |
| **SW Data (Q7)** | 35 | 4 (`SdataPin`) |
| **SW Latch (PL!)** | 33 | 6 (`SlatchPin`) |
| **Analog Out** | 29 | 7 (`AnalogOut`) |
| **RX (Serial2 RX)** | 8 | 14 (`Serial2RX`) |
| **TX (Serial2 TX)** | 10 | 15 (`Serial2TX`) |
| **SD Card CS** | 24 | 10 (`SD_CS_PIN`) |
| **SD Card MOSI** | 19 | 11 (`SD_MOSI_PIN`) |
| **SD Card SClk** | 23 | 12 (`SD_CLK_PIN`) |
| **SD Card MISO** | 21 | 13 (`SD_MISO_PIN`) |
| **Built-In LED** | N/A | 21 |

> **Note:** As the only input line, **SWData** must be **level converted**. The low output levels (3.3V) of the remaining signals will be respected by the TTL inputs.

### H1 (LED) Connections

| Pin | Signal | Pin | Signal |
| :--- | :--- | :--- | :--- |
| **H1-1** | A15 | **H1-2** | INTE |
| **H1-3** | A14 | **H1-4** | PROT |
| **H1-5** | A13 | **H1-6** | WAIT |
| **H1-7** | A12 | **H1-8** | HLDA |
| **H1-9** | A11 | **H1-10** | D7 |
| **H1-11** | A10 | **H1-12** | D6 |
| **H1-13** | A9 | **H1-14** | D5 |
| **H1-15** | A8 | **H1-16** | D4 |
| **H1-17** | A7 | **H1-18** | D3 |
| **H1-19** | A6 | **H1-20** | D2 |
| **H1-21** | A5 | **H1-22** | D1 |
| **H1-23** | A4 | **H1-24** | D0 |
| **H1-25** | A3 | **H1-26** | INT |
| **H1-27** | A2 | **H1-28** | WO |
| **H1-29** | A1 | **H1-30** | Stack |
| **H1-31** | A0 | **H1-32** | HLTA |
| **H1-33** | User2 | **H1-34** | OUT |
| **H1-35** | User1 | **H1-36** | IN |
| **H1-37** | +5V | **H1-38** | INP |
| **H1-39** | Gnd | **H1-40** | MEMR |

### H2 (Switch) Connections

| Pin | Signal | Pin | Signal |
| :--- | :--- | :--- | :--- |
| **H2-1** | A0 | **H2-2** | A1 |
| **...** | ... | **...** | ... |
| **H2-15** | A14 | **H2-16** | A15 |
| **H2-17** | Examine | **H2-18** | Examine\_Next |
| **H2-19** | Deposit | **H2-20** | Deposit\_Next |
| **H2-21** | Reset | **H2-22** | Clr |
| **H2-23** | Run | **H2-24** | Stop |
| **H2-25** | Single\_Step | **H2-26** | Single\_Step\_ |
| **H2-27** | Protect | **H2-28** | Unprotect |
| **H2-29** | Aux1 Up | **H2-30** | Aux1 Down |
| **H2-31** | Aux2 Up | **H2-32** | Aux2 Down |
| **H2-33** | +5V | **H2-34** | Gnd |
| **...** | ... | **...** | ... |
| **H2-39** | +5V | **H2-40** | Gnd |

> **Note:** The control switch order can be changed by editing the `enum` of `Control` declaration in the source code.

---

## 💻 I/O Implementation

The emulation implements "new" hardware through the `i8080_in` and `i8080_out` functions, which use a `switch/case` based on the port number.

### Currently Implemented Ports

* **Port 0x00 (In):** Sends back `0x00`.
* **Port 0x01 (In/Out):** Handles `term_in`/`term_out` for the primary terminal interface.
* **Port 0x08, 0x09, 0x0A (I/O):** Disk Controller interface.
* **Port 0x10, 0x11 (I/O):** 2SIO Port 1 (needs to be routed to sockets instead of serial).
* **Port 0x06, 0x07 (I/O):** Cassette Port (Not implemented).
* **Port 0xFF (Out):** Writes **4 bits** to control the `User4 - User1` LEDs.

---

## 🚦 Programming Bit Reference

This defines the mapping of the 40 shift register bits to the 8080's address, data, and status signals.

### LED Data (Shift Register Output)

| Bit | Signal | Description |
| :--- | :--- | :--- |
| 0-7 | D0-D7 | **Data Bus** |
| 8 | INT | Interupt request (Not implemented) |
| 9 | WO | **Write to memory or output port.** |
| 10 | STACK | Address bus holds stack pointer (Not implemented) |
| 11 | HLTA | A **HALT** instruction has been acknowledged. |
| 12 | OUT | **Out to port.** |
| 13 | M1 | CPU is performing the 1st cycle of the instruction (Not implemented) |
| 14 | INP | **In from port.** |
| 15 | MEMR | **Memory bus is being used.** |
| 16-31 | A0-A15 | **Address Bus** |
| 32 | PROT | Memory is protected (Not implemented) |
| 33 | INTE | An **interrupt has been generated.** |
| 34 | WAIT | CPU is in a **WAIT** state (Not implemented) |
| 35 | HLDA/RUN | A Hold has been acknowledged/Run Mode. |
| 36-39 | User4-User1 | Send 4 bits OUT to Port 0xFF. |

### Switch Data (Shift Register Input)

| Bit | Signal |
| :--- | :--- |
| 0-15 | A0-A15 (Address Switches) |
| 16 | STOP |
| 17 | RUN |
| 18 | SINGLE STEP |
| 19 | SS DOWN |
| 20 | EXAMINE |
| 21 | EXAMINE NEXT |
| 22 | DEPOSIT |
| 23 | DEPOSIT NEXT |
| 24 | RESET |
| 25 | CLR |
| 26 | PROTECT |
| 27 | UNPROTECT |
| 28 | AUX UP |
| 29 | AUX DOWN |
| 30 | AUX1 UP |
| 31 | AUX1 DOWN |

---

## ⏭️ ToDo List (Current Priorities)

This list focuses on tasks required to achieve a complete, functional IMSAI 8080 replica:

1.  **Floppy Disk Write implementation.** (Current Major Focus)
2.  The code needs **refactoring**, specifically in the **I/O handling routines.**
3.  Look into adding support for the optional **8K BASIC interpreter.**
4.  I can now add **WiFi via SSH** to this.
5.  Needs lots of **speed ups** (I/O refactoring will help with this).
6.  Review **pull request #45** (LED refresh issue).

---

## ✨ Future Features (Secondary Goals)

These are secondary goals that can be addressed once the core IMSAI 8080 emulation is complete and stable:

* Would be nice to have it connectable to a **Raspberry Pi**.
* Make smaller **Blinkin Boards to extend:** Such as a 7-Segment to show address and data bus values.
* Would be nice to allow it to emulate other blinkinlight platforms (e.g., PDP-8).

---

## 🔗 External Resources

* **3D Printer Toggle Paddles, Switch and LED Mounts:**
    * <https://www.thingiverse.com/thing:5627711>
* **Blinkenbone High-Level Interface API to Panels:**
    * <http://retrocmp.com/projects/blinkenbone/172-blinkenbone-high-level-interface-api-to-panels>
* **Blinkenlights EXE (BlinkenBone):**
    * <https://github.com/j-hoppe/BlinkenBone/releases>
* **Altair via shift registers (Reference):**
    * <https://github.com/companje/Altair8800>
* **Raspberry Pi `wiringPi` Install Notes (for GPIO):**
    * ```bash
        git clone [https://github.com/WiringPi/WiringPi.git](https://github.com/WiringPi/WiringPi.git)
        cd WiringPi
        ./build
        ```
