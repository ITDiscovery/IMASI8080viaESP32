In this walkthrough of the famous "Hello World!" we first load the address of the message string into the H register pair using the LXI instruction. 
Then we use a loop to output each character of the message by loading it into the accumulator with the MOV instruction, outputting it to port 1 with
the OUT instruction, and then incrementing the H register pair to point to the next character in the string.

The loop counter is stored in the B register, which is initialized to 0 with the MVI instruction. We use the INR instruction to increment the loop
counter and the CPI instruction to compare it to the length of the message (including the null terminator). If the loop counter is not equal to the
length of the message, we jump back to the LOOP label using the JNZ instruction. When the loop is finished, we halt the processor with the HLT
instruction.The message is defined as a null-terminated string using the DB (define byte) directive. We include a carriage return (0x0D) at the end
of the message to move the cursor to the beginning of the next line.

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
