# 📝 Walkthrough: Initial Setup and Wi-Fi Configuration

This guide covers the initial boot sequence, setting up persistent Wi-Fi credentials, and successfully launching a CP/M application.

## Step 1: Initial Boot Log

On first power-up, the emulator will attempt to connect to the default Wi-Fi (if not previously configured) and halt the CPU after timing out:

ESP-ROM:esp32s3-20210327 Build:Mar 27 2021 ... (omitted boot details) ...

IMSAI 8080 Emulator Booting on Core 1...
Loading configuration from NVS... 
NVS load complete. Loaded default Wi-Fi SSID from config.h: XXXXXXXX 
!!! wifiTask: WiFi connection timed out! Check SSID/Password. Halting task. !!!
--- Setup complete. CPU Halted. --- .....

---

## Step 2: Access the Configuration Menu

To update the network name and password, you must enter the Configuration Menu.

* **Action:** Toggle the **UNPROTECT** button on the front panel.
* **Result:** The main configuration menu is displayed in the serial console.

## Step 3: Set Wi-Fi Credentials

Navigate to the System Configuration menu to input your new Wi-Fi credentials, then reboot the ESP32.

1.  **Select System Configuration:** Enter **`2`**
2.  **Select Wi-Fi:** Enter **`W`**

    ```
    Enter Wi-Fi SSID: [Type your Network Name here and press Enter]
    Enter Wi-Fi Password: [Type your Password here and press Enter]
    ```

3.  **Return and Reboot:**
    * Enter **`B`** to go **Back** to the Main Menu.
    * Enter **`X`** to **Reboot** the ESP32.

---

## Step 4: Verification of Wi-Fi Connection

Upon reboot, the system will successfully connect and report the new IP address before halting, confirming persistent Wi-Fi is configured.

---

## Step 5: Boot into CP/M and Launch ZORK I

The final step is to boot the emulated system using the AUX2 key action and launch a sample application.

1.  **Load/Boot CP/M:** Toggle the **AUX2** button **down**. This executes the Auto-Boot action, loading the default disk images.
2.  **Start Emulation:** Toggle the **RUN** switch **up**.
3.  **Launch ZORK I:** At the CP/M prompt, switch to the B: drive and execute the ZORK1 program.

* **Result (Serial Output):**

......WiFi Connected. IP Address: 192.168.1.100 
Executing: Loading CP/M WARN: /ADAHD.DSK size (1113536) != expected HDSK size (1179648). 
WARN: /CPM22B23-56K.DSK size (337664) > buffer size (337568). 
Reading only first 337568 bytes. 
Disk images loaded into PSRAM successfully.

RUN

56K CP/M 2.2b v2.3 For Altair 8" Floppy

A>B:
B>ZORK1
ZORK I: The Great Underground Empire Copyright (c) 1981, 1982, 1983 Infocom, Inc. 
All rights reserved. ZORK is a registered trademark of Infocom, Inc. Revision 88 / Serial number 840726

West of House You are standing in an open field west of a white house, with a boarded front door. 
There is a small mailbox here.
