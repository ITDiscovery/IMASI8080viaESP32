# Walkthrough of 8080 Assembler Entry onto IMASI
This example will walk through entry on the main panel switches a program that counts to 15 and lights up the USR1 to USR4 LEDs

## Assembler

```
.originates at 0x0000

    MOV C, 0      ; Initialize C to 0
START:
    MOV C, A          ; Load counter value into A
    OUT 0xff      ; Output counter value
    INR C         ; Increment C
    MOV B, 255    ; Load 255 into B for delay

DELAY:
    DEC B         ; Decrement B
    JNZ DELAY     ; Jump to Delay if B is not Zero
    JMP START     ; Jump to next value of C
    JMP 0x000     ; Jump to start of program

END
```

## Manual Assembly

```
0x0000: 0E 00      ; MOV C, 0
0x0002: 4F         ; MOV A, C
0x0003: D3 FF      ; OUT 0xff
0x0005: 0C         ; INR C
0x0006: 06 FF      ; MOV B, 255
0x0008: 05         ; DEC B
0x0009: C2 08 00   ; JNZ 0x0008
0x000C: C3 00 00   ; JMP 0x0000
0x000F:              ; END
```

## Entering in via the panel

| Command | Address PI | Address DB |
|---------|------------|------------|
| Examine | 00 | 00 |
| Deposit |  | 0E |
| Deposit-Next | | 00| 
| Deposit-Next | | 4F| 
| Deposit-Next | | D3| 
| Deposit-Next | | FF| 
| Deposit-Next | | 0C| 
| Deposit-Next | | 06| 
| Deposit-Next | | FF| 
| Deposit-Next | | 05| 
| Deposit-Next | | C2| 
| Deposit-Next | | 08| 
| Deposit-Next | | 00|
| Deposit-Next | | C3| 
| Deposit-Next | | 00| 
| Deposit-Next | | 00|
| Examine | 00 | 00 |
| Run | | |

Thanks to ChatGPT for creating this code.

