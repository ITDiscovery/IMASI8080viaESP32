# Walkthrough of 8080 Assembler Entry onto IMASI
This example will walk through entry on the main panel switches a program that adds two numbers from memory and stores it in a third location.

## Assembler

.orignates at $0000
LDA $0100
MOV A->B
LDA $0101
ADD A+B
STA $0102
JMP $0000

## Manual Assembly
0000: 3A 01 00
0003: 47
0004: 3A 01 01
0007: 80
0008: 32 02 01
000B: C3 00 00

## Entering in via the panel

| Command | Address PI | Address DB |
|---------|------------|------------|
| Examine | 00 | 00 |
| Deposit |  | 3A |
| Deposit-Next | | 00| 
| Deposit-Next | | 01| 
| Deposit-Next | | 47| 
| Deposit-Next | | 3A| 
| Deposit-Next | | 01| 
| Deposit-Next | | 01| 
| Deposit-Next | | 80| 
| Deposit-Next | | 32| 
| Deposit-Next | | 02| 
| Deposit-Next | | 01| 
| Deposit-Next | | C3|
| Deposit-Next | | 00| 
| Deposit-Next | | 00| 
| Examine | 01 | 00 |
| Deposit |  | A0 |
| Deposit-Next |  | 05 |
| Deposit-Next |  | 00 |
| Examine | 00 | 00 |
| Run | | |
| Stop | | | 
| Examine | 01 | 02 |

A5 Should be lit up on the Data Bus.

