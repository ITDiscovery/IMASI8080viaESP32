# Killbits Walkthrough 
Kill the Bit game by Dean McDaniel, May 15, 1975

## Introduction
Object: Kill the rotating bit. If you miss the lit bit, another bit turns on leaving two bits to destroy. Quickly
toggle the switch, don't leave the switch in the up position. Before starting, make sure all the switches are in the down position.

## Assembly

```
Orig: 0x0000

START: LXI H,0x0000  ;Initialize the H Register (Counter)
       MVI H,0x80    ;Set up initial display bit
       LIX B,0x0E    ;Move 0x0E into the B Register (Delay Value, higher = faster)
LOOP:  LDAX D        ;Display the bit pattern on the upper 8 address lights
       LDAX D        ;
       LDAX D        ;
       LDAX D        ;
       DAD B         ;Increment display counter
       JNC LOOP      ;Jump to the start of the loop
       IN 0xFF       ;Input data from sense switches
       XRA D         ;Exclusive Or with A
       RRC           ;Rotate display Right one bit
       MOV D,A       ;Move data to display reg
       JMP LOOP      ;Repeat sequence

```

## Hand Assembly

```
0000
0000 21 00 00   lxi h,0
0003 16 80      mvi h,80
0005 01 0E 00   lxi b,0E
0008 1A         ldax d
0009 1A         ldax d
000A 1A         ldax d
000B 1A         ldax d
000C 09         dad b
000D D2 08 00   jnc 0x0008     
0010 DB FF      in 0xff
0012 AA         xra d
0013 0F         rrc
0014 57         mov d,a
0015 C3 08 00   jmp 0x0008
```

## Entering in via the panel

| Command | Address PI | Address DB |
|---------|------------|------------|
| Examine | 00 | 00 |
| Deposit |  | 21 |
| Deposit-Next |  | 00 |
| Deposit-Next |  | 00 |
| Deposit-Next |  | 16 |
| Deposit-Next |  | 80 |
| Deposit-Next |  | 01 |
| Deposit-Next |  | 0E |
| Deposit-Next |  | 00 |
| Deposit-Next |  | 1A |
| Deposit-Next |  | 1A |
| Deposit-Next |  | 1A |
| Deposit-Next |  | 1A |
| Deposit-Next |  | 09 |
| Deposit-Next |  | D2 |
| Deposit-Next |  | 08 |
| Deposit-Next |  | 00 |
| Deposit-Next |  | DB |
| Deposit-Next |  | FF |
| Deposit-Next |  | AA |
| Deposit-Next |  | 0F |
| Deposit-Next |  | 57 |
| Deposit-Next |  | C3 |
| Deposit-Next |  | 08 |
| Deposit-Next |  | 00 |
| Examine | 00 | 00 |
| Run | | |

## Code to add to a switch
```
byte killbits[] = {
       0x21,0x00,0x00,
       0x16,0x80,
       0x01,0x0E,0x00,
       0x1A,
       0x1A,
       0x1A,
       0x1A,
       0x09,
       0xD2,0x08,0x00,
       0xDB,0xFF,
       0xAA,
       0x0F,
       0x57,
       0xC3,0x08,0x00 };
       loadData(killbits,sizeof(killbits),0);
       examine(0);
```
