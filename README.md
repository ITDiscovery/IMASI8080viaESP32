# Altair8800viaAVR
I'll be creating a smaller board the will have a mini "mock up" of an Altair 8800.

Design Goals:
1. Use the AVR12828 and add the required I/O 74HCT165 (input) and 74LS595 (output).
1. Would be nice to have it connectable to a Raspberry Pi.
1. Make smaller Blinkin Boards to extend?
1. Would be nice to allow it to emulate a PDP-8 also.

Programming Bit Reference:

LED Data (PIN_PD2):

Bit 0 - 15
| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10| 11| 12| 13| 14| 15| 
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| D0| D1| D2| D3| D4| D5| D6| D7|INT| WO|Stack|HLTA|OUT|IN|INP|MEMR|

Bit 16 - 32
| 16| 17| 18| 19| 20| 21| 22| 23| 24| 25| 26| 27| 28| 29| 30| 31|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A0| A1| A2| A3| A4| A5| A6| A7| A8| A9|A10|A11|A12|A13|A14|A15|

Bit 32-39
| 32| 33| 34| 35| 36| 37| 38| 39|
|---|---|---|---|---|---|---|---|
|PROT|INTE|WAIT|HLDA|User1|User2|User3|User4|

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
|H2-1 |Gnd|H2-2 |Gnd|
|H2-3 |Gnd|H2-4 |Gnd|
|H2-5 |+5V|H2-6 |+5V|
|H2-7 |+5V|H2-8 |+5V|
|H2-9 |AUX1 DOWN|H3-10 |A15|
|H2-11 |AUX1 UP|H2-12 |A14|
|H2-13 |AUX DOWN|H2-14 |A13|
|H2-15 |AUX UP|H2-16 |A12|
|H2-17 |UNPROTECT|H2-18 |A11|
|H2-19 |PROTECT|H2-20 |A10|
|H2-21 |CLR|H2-22 |A9|
|H2-23 |RESET|H2-24 |A8|
|H2-25 |DEPOSIT NEXT|H2-26 |A7|
|H2-27 |DEPOSIT|H2-28 |A6|
|H2-39 |EXAMINE NEXT|H2-30 |A5|
|H2-31 |EXAMINE|H2-32 |A4|
|H2-33 |SS DOWN|H2-34 |A3|
|H2-35 |SINGLE STEP|H2-36 |A2|
|H2-37 |RUN|H2-38 |A1|
|H2-39 |STOP|H2-40 |A0|

Update: Dumped the TM1638, as it had trouble dealing with more than a few switches on at the same time, especially with 7-Segments
connected. 



Looks like the Blinkenlights is what I am looking for. I'll be looking to create a board that I can get it to read off the
Blinkinlights API. Since the board I am making can run off of Data Clock and Latch (and I'm gonna see if Clock can be shared between
Input and Output, so I can narrow it down to 5 GPIO lines.

http://retrocmp.com/projects/blinkenbone/172-blinkenbone-high-level-interface-api-to-panels

Blinkinlights EXE:
https://github.com/j-hoppe/BlinkenBone/releases
