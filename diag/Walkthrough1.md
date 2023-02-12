# Walkthrough of 8080 Assembler Entry onto IMASI
This example will walk through entry on the main panel switches a program that counts to 15 and lights up the USR1 to USR4 LEDs

## Assembler

```
.originates at 0x0000


MOV C, 0        ; Initialize C to 0

DELAY:
    DJNZ DELAY    ; Decrement B and jump to DELAY if not zero

START:
    MOV A, C      ; Load counter value into A
    OUT 0xff      ; Output counter value
    INR C         ; Increment C
    CJNE C, 16, START ; Jump to start if C is not equal to 16
    MOV B, 200    ; Load 200 into B for delay
    JMP DELAY     ; Jump to DELAY

END
```

## Manual Assembly

```
0x00: 0E 00 ; MOV C, 0x00
0x02: D2 07 ; DJNZ DELAY
0x04: D3 FF ; OUT 0xff
0x05: 0C  ; INR C
0x06: FE 10  ; CJNE C, 0x10, <offset>
0x08: 06 C8 ; MOV B, 0xC8
0x09: C2 02 00 ; JMP DELAY
```

## Entering in via the panel

| Command | Address PI | Address DB |
|---------|------------|------------|
| Examine | 00 | 00 |
| Deposit |  | 0E |
| Deposit-Next | | 00| 
| Deposit-Next | | D3| 
| Deposit-Next | | FF| 
| Deposit-Next | | 0C| 
| Deposit-Next | | FE| 
| Deposit-Next | | 10| 
| Deposit-Next | | 06| 
| Deposit-Next | | C8| 
| Deposit-Next | | C2| 
| Deposit-Next | | 02| 
| Deposit-Next | | 00|
| Examine | 00 | 00 |
| Run | | |

Thanks to ChatGPT for creating this code.

