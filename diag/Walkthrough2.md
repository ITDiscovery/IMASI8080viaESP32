# Walkthrough 2: Hello World!

## Introduction
In this walkthrough of the famous "Hello World!" we first load the address of the message string into the H register pair using the LXI instruction. 
Then we use a loop to output each character of the message by loading it into the accumulator with the MOV instruction, outputting it to port 1 with
the OUT instruction, and then incrementing the H register pair to point to the next character in the string.

The loop counter is stored in the B register, which is initialized to 0 with the MVI instruction. We use the INR instruction to increment the loop
counter and the CPI instruction to compare it to the length of the message (including the null terminator). If the loop counter is not equal to the
length of the message, we jump back to the LOOP label using the JNZ instruction. When the loop is finished, we halt the processor with the HLT
instruction.The message is defined as a null-terminated string using the DB (define byte) directive. We include a carriage return (0x0D) at the end
of the message to move the cursor to the beginning of the next line.

## Assembly

```
Orig: 0x0000

START:  LXI H, MESSAGE ; Load the address of the message into the H register pair
        MVI B, 0       ; Initialize the loop counter to 0
LOOP:   MOV A, M       ; Load the character from the memory location pointed to by the H register pair into the accumulator
        OUT 0x01       ; Output the value in the accumulator to port 0x01
        INX H          ; Increment the H register pair to point to the next character
        INR B          ; Increment the loop counter
        CPI 0x0D       ; Compare the loop counter to the length of the message (including the null terminator)
        JNZ LOOP       ; If the loop counter is not equal to the length of the message, jump back to the LOOP label
        HLT            ; Halt the processor
MESSAGE:
        DB "Hello World!", 0x0D  ; Define the message as a null-terminated string
```

## Manual Assembly

```
 0000: 21 20 00     LXI H, MESSAGE  ; Load the address of the message into the H register pair
 0003: 06 00        MVI B, 0       ; Initialize the loop counter to 0
 0005: 7E           LOOP: MOV A, M  ; Load the character from the memory location pointed to by the H register pair into the accumulator
 0006: D3 01        OUT 0x01       ; Output the value in the accumulator to port 1
 0008: 23           INX H          ; Increment the H register pair to point to the next character
 0009: 04           INR B          ; Increment the loop counter
 000A: FE 0D        CPI 0x0D       ; Compare the loop counter to the length of the message (including the null terminator)
 000C: C2 05 00     JNZ LOOP       ; If the loop counter is not equal to the length of the message, jump back to the LOOP label
 000F: 76           HLT            ; Halt the processor
 0020: 48 65 6C 6C 6F 20          MESSAGE:       ; Define the message as a null-terminated string
 0011: 57 6F 72 6C 64 21 0D 00      DB "Hello World!", 0x0D
```

## Entering in via the panel

| Command | Address PI | Address DB |
|---------|------------|------------|
| Examine | 00 | 00 |
| Deposit |  | 21 |
| Deposit-Next |  | 20 |
| Deposit-Next |  | 00 |
| Deposit-Next |  | 06 |
| Deposit-Next |  | 00 |
| Deposit-Next |  | 7E |
| Deposit-Next |  | D3 |
| Deposit-Next |  | 01 |
| Deposit-Next |  | 23 |
| Deposit-Next |  | 04 |
| Deposit-Next |  | FE |
| Deposit-Next |  | 0D |
| Deposit-Next |  | C2 |
| Deposit-Next |  | 05 |
| Deposit-Next |  | 00 |
| Deposit-Next |  | 76 |
| Examine | 00 | 20 |
| Deposit |  | 48 |
| Deposit-Next |  | 65 |
| Deposit-Next |  | 6C |
| Deposit-Next |  | 6C |
| Deposit-Next |  | 6F |
| Deposit-Next |  | 20 |
| Deposit-Next |  | 57 |
| Deposit-Next |  | 6F |
| Deposit-Next |  | 72 |
| Deposit-Next |  | 6C |
| Deposit-Next |  | 64 |
| Deposit-Next |  | 21 |
| Deposit-Next |  | 0D |
| Examine | 00 | 00 |
| Run | | |

Code should stop automatically as an HLT in the emulator changes the mode.
