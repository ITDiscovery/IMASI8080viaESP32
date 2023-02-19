# Walkthrough 3, Test SIO Port

## Introduction
This walkthrough tests UART Ports where the code should be waiting for a status from the UART.

## Assembly
```
;  2SIO echo routine

...originate 0x0000
	    MVI A,0x03	  ;Put UART reset Value in A
	    OUT	0x10      ;Send value to Port 0x10 (or whichever status port your checking)
      MVI	A,0x15  	;no RI, no XI, RTS Low, 8n1 to UART
      OUT 0x10      ;Send value to Port 0x10
wtRcv	IN	0x10   	  ;See if UART has a character that has been received
      RRC		        ;receive flag in Least Significant Byte
      JNC	wtRcv     ;Jump If No Carry back to wait for UART to get a byte
      IN	0x11      ;Read the character
      OUT 0x11      ;Echo it back out
      JMP	wtRcv     ;Jump back to wait loop
END
```
  
## Hand Assembly
```
0000: 3E 03     ;Put UART reset Value in A
0002: D3 10     ;Send value to Port 0x10
0004: 3E 11     ;no RI, no XI, RTS Low, 8n1 to UART
0006: D3 10     ;Send value to Port 0x10
0008: DB 10     ;See if UART has a character that has been received
000A: 0F        ;receive flag in Least Significant Byte
000B: D2 08 00  ;Jump If No Carry back to wait for UART to get a byte
000E: DB 11     ;Read the character
0010: D3 11     ;Echo it back out
0012: C3 08 00  ;Jump back to wait loop
```


## Entering in via the panel

| Command | Address PI | Address DB |
|---------|------------|------------|
| Examine | 00 | 00 |
| Deposit |  | 3E |
| Deposit-Next |  | 03 |
| Deposit-Next |  | D3 |
| Deposit-Next |  | 10 |
| Deposit-Next |  | 3E |
| Deposit-Next |  | 11 |
| Deposit-Next |  | D3 |
| Deposit-Next |  | 10 |
| Deposit-Next |  | D8 |
| Deposit-Next |  | 10 |
| Deposit-Next |  | 0F |
| Deposit-Next |  | D2 |
| Deposit-Next |  | 08 |
| Deposit-Next |  | 00 |
| Deposit-Next |  | DB |
| Deposit-Next |  | 11 |
| Deposit-Next |  | D3 |
| Deposit-Next |  | 11 |
| Deposit-Next |  | C3 |
| Deposit-Next |  | 08 |
| Deposit-Next |  | 00 |
| Examine | 00 | 00 |
| Run |  |  |
  
