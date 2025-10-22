# IMASI8080onESP32
I created a small board the also has a mini "mock up" of an Altair 8800 front panel via DIP switches and LED bar graphs.
The board has two 40 pin connectors to wire up to external LEDs and switches.

Design Goals:
1. Use the ESP32 and add the required I/O 74HCT165 (input) and 74LS595 (output) -the AVR128 was dropped due to speed issues.
1. Would be nice to have it connectable to a Raspberry Pi.
1. Make smaller Blinkin Boards to extend? Such as a 7-Segment to show address and data bus values.
1. Would be nice to allow it to emulate a PDP-8 also.

Had to dump the idea of using AVR128DB28 Flash...newly posted specs show only a 1K cycle. RevB and on uses a 23LC512 (much slower, and uses an additional 4 pins from the AVR, but using an AVR64DB28 is now possible).

RevA boards would need too many mods.
RevB boards will need to a mod to go from Pin 1 of U2 to Pin 24 of H3.
RevC boards use the ESP32 and drop the 23LC512

ToDo:
1. I can now add WiFi via SSH to this.
2. Serial needs work
3. Needs lots of speed ups.

3D Printer Toggle Paddles, Switch and LED Mounts:
https://www.thingiverse.com/thing:5627711

Programming Bit Reference:

LED Data (PIN_PD2):

Bit 0 - 15
| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10| 11| 12| 13| 14| 15| 
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| D0| D1| D2| D3| D4| D5| D6| D7|INT| WO|Stack|HLTA|OUT|M1|INP|MEMR|

- INTA: Interupt request has been acknowledged (not implemented)
- WO: Write to memory or output port.
  - Implemented by setting bus_state.WO in i8080_out, i8080_mwrite, i8080_pairwrite
  - Implemented by clearing bus_state.WO in i8080_in, i8080_mread, i8080_pairread
- STACK: The address buss hold's the stack pointer (not implemented)
- HLTA: A HALT instruction has been acknowledged
- OUT: Out to port
  - Implemented by setting bus_state.OUT in intel8080_out
  - Implemented by clearing bus_stat.OUT in intel8080_in, i8080_mwrite, i8080_mread
- M1: The CPU is performing the 1st cycle of the instruction (not implemented)
- INP: In from port.
  - Implemented by setting bus_state.INP in intel8080_in
  - Implemented by clearing bus_stat.INP in intel8080_out, i8080_mwrite, i8080_mread
- MEMR: The memory bus is being used.
  - Implemented by setting bus_state.MEMR in i8080_mwrite, i8080mread
  - Implemented by clearing bus_state.MEMR in i8080_in, i8080_out 

Bit 16 - 32
| 16| 17| 18| 19| 20| 21| 22| 23| 24| 25| 26| 27| 28| 29| 30| 31|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A0| A1| A2| A3| A4| A5| A6| A7| A8| A9|A10|A11|A12|A13|A14|A15|

Bit 32-39
| 32| 33| 34| 35| 36| 37| 38| 39|
|---|---|---|---|---|---|---|---|
|PROT|INTE|WAIT|HLDA|User1|User2|User3|User4|

- PROT: The memory is protected (not implemented)
- INTE: An interupted has been generated
   - Implemented by setting bus_state.INTE on i8080_ei
   - Implemented by clearing bus_state.INTE on i8080_di
- WAIT: CPU is in a WAIT state (not implemented)
- HLDA/RUN: A Hold has been acknowledged.
  - Implemented by clearing the bus_state.HLDA on i8080_HLT
  - Implemented by setting the bus_state.HLDA on Run Mode
- User4 - User1: Send 4 bits OUT to Port 0xFF 

Switch Data (PIN_PD5):

BIT 0-15
| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10| 11| 12| 13| 14| 15| 
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A0| A1| A2| A3| A4| A5| A6| A7| A8| A9|A10|A11|A12|A13|A14|A15|

Bit 16 - 32
| 16| 17| 18| 19| 20| 21| 22| 23| 24| 25| 26| 27| 28| 29| 30| 31|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|STOP|RUN|SINGLE STEP|SS DOWN|EXAMINE|EXAMINE NEXT|DEPOSIT|DEPOSIT NEXT|RESET|CLR|PROTECT|UNPROTECT|AUX UP|AUX DOWN|AUX1 UP|AUX1 DOWN|


H1 (LED) Connections:

| Pin | Name |  | Pin | Name |
|-----|------|---|-----|------|
|H1-1 |A15| |H1-2 | INTE |
|H1-3 |A14| |H1-4 | PROT |
|H1-5 |A13| |H1-6 | WAIT |
|H1-7 |A12| |H1-8 | HLDA |
|H1-9 |A11| |H1-10 | D7 |
|H1-11 |A10| |H1-12 | D6 |
|H1-13 |A9| |H1-14 | D5 |
|H1-15 |A8| |H1-16 | D4 |
|H1-17 |A7| |H1-18 | D3 |
|H1-19 |A6| |H1-20 | D2 |
|H1-21 |A5| |H1-22 | D1 |
|H1-23 |A4| |H1-24 | D0 |
|H1-25 |A3| |H1-26 | INT |
|H1-27 |A2| |H1-28 | WO |
|H1-29 |A1| |H1-30 | Stack |
|H1-31 |A0| |H1-32 | HLTA |
|H1-33 |User2| |H1-34 | OUT |
|H1-35 |User1| |H1-36 | IN |
|H1-37 |+5V| |H1-38 | INP |
|H1-39 |Gnd| |H1-40 | MEMR |

H2 (Switch) Connections:

| Pin | Name |  | Pin | Name |
|-----|------|---|-----|------|
|H2-1 |A0|H2-2 |A1|
|H2-3 |A2|H2-4 |A3|
|H2-5 |A4|H2-6 |A5|
|H2-7 |A6|H2-8 |A7|
|H2-9 |A8|H3-10 |A9|
|H2-11 |A10|H2-12 |A11|
|H2-13 |A12|H2-14 |A13|
|H2-15 |A14|H2-16 |A15|
|H2-17 |Examine|H2-18 |Examine_Next|
|H2-19 |Deposit|H2-20 |Deposit_Next|
|H2-21 |Reset|H2-22 |Clr|
|H2-23 |Run|H2-24 |Stop|
|H2-25 |Single_Step|H2-26 |Single_Step_|
|H2-27 |Protect|H2-28 |Unprotect|
|H2-39 |Aux1 Up|H2-30 |Aux1 Down|
|H2-31 |Aux2 Up|H2-32 |Aux2 Down|
|H2-33 |+5V|H2-34 |Gnd|
|H2-35 |+5V|H2-36 |Gnd|
|H2-37 |+5V|H2-38 |Gnd|
|H2-39 |+5V|H2-40 |Gnd|

The above Control switch order can be changed by editing the enum of Control declaration.

Note: Implementing "new" hardware is via i8080_in and i8080_out which has a switch/case based on the port number.
Currently Implemented:

Port 0x00 in: Sends back 0x00

Port 0x01: term_in/term_out in main.c

Port 0x06 and 0x07: Cassette Port (not implemented)

Port 0x08,0x09,0x0A: Disk Controller

Port 0x10,0x11: 2SIO Port 1 (needs to go to sockets, not serial)

Update: Dumped the TM1638, as it had trouble dealing with more than a few switches on at the same time, especially with 7-Segments
connected. 

ESP32 to Blinkenlight:
| Name | H2 Pin | ESP32-S3-Zero Pin |
|-----|------|----- |
| LEDData |36|1|
| LEDLatch |38|2|
| LEDClock |40|3|
| SWClock |37|5|
| SWData |35|4|
| SWLatch|33|6|
| SDA |3|8|
| SCL |5|9|
| Ain |31|17|
| Aout|29|7|
| MOSI |19|11|
| MISO |21|13|
| SClk |23|12|
| CS |24|10|
| RX |8|14|
| TX |10|15|
| Built In LED| |21|

Note: As the only input line, SWData must be level converted. The low output levels (3.3V) of the remaining signals will "respected" by the TTL inputs.
Note: Using 10-13 for the SPI will require a change to the pins_arduino.h for the Waveshare board, found at: /Users/...../Library/Arduino15/packages/esp32/hardware/esp32/3.0.7/variants/waveshare_esp32_s3_zero


http://retrocmp.com/projects/blinkenbone/172-blinkenbone-high-level-interface-api-to-panels

Blinkinlights EXE:
https://github.com/j-hoppe/BlinkenBone/releases

Altair via shift registers: 
https://github.com/companje/Altair8800

AVR Install Notes:
Needs SdFat library (current is 2.2)

Rasperry Pi Note:
Big problems with a C++ library for GPIO, due to trolls annoying the original author. Here's how to install wiringPi:

```
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build
```
