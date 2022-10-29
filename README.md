# Altair8800viaAVR
I'll be creating a smaller board the will have a mini "mock up" of an Altair 8800.

Design Goals:
1. Use the AVR12828 and add the required I/O 74HCT165 (input) and 74LS595 (output).
1. Would be nice to have it connectable to a Raspberry Pi.
1. Make smaller Blinkin Boards to extend?
1. Would be nice to allow it to emulate a PDP-8 also.

Connections:

| Pin | Name |  | Pin | Name |
|-----|------|---|-----|------|
H1-1 |A15| |H1-2 | INTE |
H1-3 |A14| |H1-4 | PROT |
H1-5 |A13| |H1-6 | WAIT |
H1-7 |A12| |H1-8 | HLDA |
H1-9 |A11| |H1-10 | D7 |
H1-11 |A10| |H1-12 | D6 |
H1-13 |A9| |H1-14 | D5 |
H1-15 |A8| |H1-16 | D4 |
H1-17 |A7| |H1-18 | D3 |
H1-19 |A6| |H1-20 | D2 |
H1-21 |A5| |H1-22 | D1 |
H1-23 |A4| |H1-24 | D0 |
H1-25 |A3| |H1-26 | INT |
H1-27 |A2| |H1-28 | WO |
H1-29 |A1| |H1-30 | Stack |
H1-31 |A0| |H1-32 | HLTA |
H1-33 |User2| |H1-34 | OUT |
H1-35 |User1| |H1-36 | IN |
H1-37 |+5V| |H1-38 | INP |
H1-39 |Gnd| |H1-40 | MEMR |

Update: Dumped the TM1638, as it had trouble dealing with more than a few switches on at the same time, especially with 7-Segments
connected. 



Looks like the Blinkenlights is what I am looking for. I'll be looking to create a board that I can get it to read off the
Blinkinlights API. Since the board I am making can run off of Data Clock and Latch (and I'm gonna see if Clock can be shared between
Input and Output, so I can narrow it down to 5 GPIO lines.

http://retrocmp.com/projects/blinkenbone/172-blinkenbone-high-level-interface-api-to-panels

Blinkinlights EXE:
https://github.com/j-hoppe/BlinkenBone/releases
