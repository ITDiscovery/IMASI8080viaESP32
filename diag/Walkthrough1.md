# Walkthrough of 8080 Assembler Entry onto IMASI
This example will walk through entry on the main panel switches a program that counts to 15 and lights up the USR1 to USR4 LEDs

## Assembler

```
.originates at 0x0000
START:
    MOV C, 0x0F   ; Initialize C to 15
COUNT:
    MOV A, C      ; Load counter value into A
    OUT 0xff      ; Output counter value
    DCR C         ; Decrement C
    JZ START      ; Jump to START if C is Zero
    MOV B, 255    ; Load 255 into B for delay

DELAY:
    DCR B         ; Decrement B
    NOP           ; 
    JNZ DELAY     ; Jump to Delay if B is not Zero
    JMP START     ; Jump to next value of C
    JMP 0x000     ; Jump to start of program
END
```

## Manual Assembly

```
0x0000: 0E 0F      ; MOV C, 0x00
0x0002: 79         ; MOV A, C
0x0003: D3 FF      ; OUT 0xff
0x0005: 0D         ; DCR C
0x0006: CA 00 00   ; JZ 0x0000
0x0009: 06 FF      ; MOV B, 255
0x000B: 05         ; DCR B
0x000C: 00 00 00   ; NOP
0x000F: C2 0B 00   ; JNZ 0x000B
0x000F: C3 02 00   ; JMP 0x0002
                   ; END
```

## Entering in via the panel

| Command | Address PI | Address DB |
|---------|------------|------------|
| Examine | 00 | 00 |
| Deposit |  | 0E |
| Deposit-Next | | 0F| 
| Deposit-Next | | 79| 
| Deposit-Next | | D3| 
| Deposit-Next | | FF| 
| Deposit-Next | | 0D| 
| Deposit-Next | | CA| 
| Deposit-Next | | 00| 
| Deposit-Next | | 00| 
| Deposit-Next | | 06| 
| Deposit-Next | | FF| 
| Deposit-Next | | 05|
| Deposit-Next | | 00| 
| Deposit-Next | | 00| 
| Deposit-Next | | 00|
| Deposit-Next | | C2|
| Deposit-Next | | 0B|
| Deposit-Next | | 00|
| Deposit-Next | | C3|
| Deposit-Next | | 02|
| Deposit-Next | | 00|
| Examine | 00 | 00 |
| Run | | |

Thanks to ChatGPT for creating this code.

