# Monitor mode for Altair

## Assembly
```
; **************************************************************
;
; MITS ALTAIR 8800 ROM MONITOR
;
; **************************************************************
;
; BASED ON THE ORIGINAL ALTAIR TURNKEY SYSTEM MONITOR
;
; MODIFIED BY:  SCOTT LABOMBARD
;               8/9/02
;
; MODIFIED TO ASSEMBLE WITH INTEL 8080 CROSS ASSEMBLER
;               UDO MUNK
;               10/11/08
;
; *************************************************************

STACK   EQU     0FC00H          ;MUST BE VALID RAM, NOT ROM!
CONSTAT EQU     00H            ;IO STATUS PORT
CONDATA EQU     01H            ;IO DATA PORT
        ORG     0FD00H          ;ROM BASED CODE

MON:    MVI     A,3             ;RESET UART
        OUT     CONSTAT
        MVI     A,011H          ;INITIALIZE UART
        OUT     CONSTAT
ENTER:  LXI     SP,STACK        ;INITIALIZE STACK POINTER
        CALL    CRLF            ;PRINT CARRIAGE RET+LINE FEED
        CALL    CRLF            ;PRINT CARRIAGE RET+LINE FEED
        MVI     A,'.'           ;MONITOR PROMPT
        CALL    OUTCHK          ;PRINT CHAR TO CONSOLE
        CALL    INCH            ;GET CHAR FROM CONSOLE
        CPI     'M'
        JZ      MEM             ;DO MEMORY EXAMINE/ALTER
        CPI     'D'
        JZ      DMP             ;DO MEMORY DUMP
        CPI     'J'
        JNZ     ENTER

; **************************************************************
; GET JUMP ADDRESS, LOAD TO PC, AND GO
; **************************************************************

        CALL    OCTL6           ;GET 6 OCTAL DIGITS IN HL

        PCHL




; **************************************************************

; MEMORY FUNCTION - DISPLAY AND/OR ALTER MEMORY

; **************************************************************

MEM:    CALL    OCTL6           ;GET 6 OCTAL DIGITS IN HL

        JMP     CONT1

CONT:   INX     H               ;POINT TO NEXT ADDRESS

CONT1:  CALL    CRLF            ;PRINT CARRIAGE RET+LINE FEED

        MOV     D,H             ;SAVE ADDR TO DE

        MOV     E,L

        CALL    PRT6            ;CVT TO ASCII + PRINT

        LDAX    D               ;LOAD DATA FROM CURRENT MEM LOC

        MOV     H,A

        CALL    PRT3            ;CVT TO ASCII + PRINT

        CALL    OCTL3           ;GET 3 OCTAL DIGITS IN HL

        XCHG                    ;EXCHANGE HL AND DE

        JC      CONT

        MOV     M,A             ;STORE USER SPECIFIED BYTE

        CMP     M               ;VALIDATE DATA BYTE IN MEMORY

        JZ      CONT            ;IF BYTE OKAY, KEEP GOING

ERR:    MVI     A,'?'           ;WE HAVE A PROBLEM

        CALL    OUTCHK          ;PRINT ERROR CHAR TO CONSOLE

        JMP     ENTER




; **************************************************************

; DUMP FUNCTION - DISPLAY DATA BETWEEN TWO SPECIFIED MEM LOCS

; **************************************************************

DMP:    CALL    OCTL6           ;GET 6 OCTAL DIGITS IN HL

        XCHG                    ;SAVE START ADDR TO DE

        CNC     SPACE

        CALL    OCTL6           ;GET 6 OCTAL DIGITS IN HL

        PUSH    H               ;SAVE END ADDR

DCONT:  MOV     H,D             ;MOV ADDR IN DE TO HL FOR PRINT

        MOV     L,E

        CALL    CRLF            ;PRINT CARRIAGE RET+LINE FEED

        CALL    PRT6            ;CVT TO ASCII + PRINT

        CALL    SPACE

        LXI     B,010H          ;PRINT 16 MEM LOCATIONS PER LINE

DO20:   LDAX    D               ;LOAD DATA FROM CURRENT MEM LOC

        MOV     H,A

        PUSH    B               ;SAVE PRINT LOCATION COUNTER

        MVI     A,08H          ;IS HALF THE LINE PRINTED?

        CMP     C

        JNZ     NXTMEM

        MVI     A,'-'           ;MAKES EACH LINE EASIER TO READ

        CALL    OUTCHK

        CALL    SPACE

NXTMEM: CALL    PRT3            ;CVT TO ASCII + PRINT MEM DATA

        POP     B               ;RESTORE PRINT LOCATION COUNTER

        POP     H               ;RESTORE END ADDR

        MOV     A,H             ;COMPARE CURRENT ADDR WITH END

        CMP     D

        JNZ     DAGN

        MOV     A,L

        CMP     E

        JZ      ENTER           ;PROCESSED LAST ADDRESS SO DONE

DAGN:   PUSH    H               ;SAVE END ADDR TO USE AGAIN

        INX     D               ;NEXT MEMORY LOCATION TO PRINT

        DCR     C               ;CURRENT PRINT LOCATION COUNTER

        JNZ     DO20            ;16 LOCATIONS PRINTED YET?              

        JMP     DCONT           ;NEXT LINE IF 16 LOCATIONS DONE




; **************************************************************

; PRINT CARRIAGE RETURN AND LINE FEED

; **************************************************************
CRLF:   MVI     A,0DH
        CALL    OUTCHK          ;PRINT CHAR TO CONSOLE
        MVI     A,0AH
        JMP     OUTCHK          ;PRINT CHAR TO CONSOLE

; **************************************************************
; BUILD 3/6 OCTAL DIGITS IN HL
; **************************************************************

OCTL6:  MVI     B,6             ;SET DIGIT COUNTER
        JMP     OCTL
OCTL3:  MVI     B,3             ;SET DIGIT COUNTER
OCTL:   LXI     H,0             ;CLEAR ALL 16 BITS OF HL REG
AGN:    CALL    INCH            ;GET CHAR FROM CONSOLE
        MOV     C,A
        CPI     ' '             ;CHECK FOR SPACE CHAR
        STC
        RZ                      ;SPACE CHAR ENTERED SO QUIT
        ANI     184            ;CHECK FOR VALID OCTAL DIGIT
        XRI     30H
        JNZ     ERR             ;NOT OCTAL SO LEAVE
        MOV     A,C             ;CONVERT ASCII TO BINARY
        ANI     07H            ;STRIP ASCII
        DAD     H               ;SHIFT HL LEFT 3 BITS
        DAD     H
        DAD     H
        ADD     L
        MOV     L,A             ;PUT OCTAL IN H
        DCR     B               ;MORE DIGITS?
        JNZ     AGN
        RET

; **************************************************************
; PRINT 3 OR 6 OCTAL DIGITS FROM H OR HL
; **************************************************************
PRT6:   MVI     B,6             ;SET DIGIT COUNTER
        XRA     A
        JMP     NEXT1
PRT3:   MVI     B,3             ;SET DIGIT COUNTER
        XRA     A
        JMP     NXT3
NEXT3:  DAD     H               ;SHIFT 1 BIT
NXT3:   RAL
        DAD     H               ;SHIFT 1 BIT
        RAL
NEXT1:  DAD     H               ;SHIFT 1 BIT
        RAL
        ANI     7               ;STRIP OFF OCTAL
        ORI     30H            ;CONVERT TO ASCII
        CALL    OUTCHK          ;PRINT CHAR TO CONSOLE
        DCR     B
        JNZ     NEXT3
SPACE:  MVI     A,' '           ;ASCII SPACE CHARACTER
        JMP     OUTCHK          ;PRINT CHAR TO CONSOLE

; **************************************************************
; INPUT AND ECHO CHARACTER
; **************************************************************
INCH:   IN      CONSTAT
        RRC
        JNC     INCH            ;CHECK READ STATUS
        IN      CONDATA         ;READ CHARACTER
        ANI     7FH            ;STRIP PARITY BIT
OUTCHK: PUSH    PSW             ;SAVE CHARACTER
        ADD     C               ;ADD IN CHECKSUM
        MOV     C,A             ;RESTORE CHECKSUM
LOOP:   IN      CONSTAT
        RRC
        RRC
        JNC     LOOP            ;GET READ STATUS
        POP     PSW
        OUT     CONDATA         ;PRINT USER TYPED CHARACTER
        RET
END
```

## Walkthrough Code

```
0xFD00: 3E 03      LD A, 0x03
0xFD02: D3 00      OUT (0x00), A
0xFD04: 3E 11      LD A, 0x11
0xFD06: D3 00      OUT (0x00), A
0xFD08: 31 00 FC   LD SP, 0xFC00 <-- ENTER
0xFD0B: CD 96 FD   CALL 0xFD96
0xFD0E: CD 96 FD   CALL 0xFD96
0xFD11: 3E 2E      LD A, 0x2E
0xFD13: CD F2 FD   CALL 0xFDF2
0xFD16: CD E8 FD   CALL 0xFDE8
0xFD19: FE 4D      CP 0x4D
0xFD1B: CA 2C FD   JP Z, 0xFD2C
0xFD1E: FE 44      CP 0x44
0xFD20: CA 54 FD   JP Z, 0xFD54
0xFD23: FE 4A      CP 0x4A
0xFD25: C2 08 FD   JP NZ, 0xFD08 (ENTER)
0xFD28: CD A0 FD   CALL 0xFDA0
0xFD2B: E9         JP (HL)
0xFD2C: CD A0 FD   CALL 0xFDA0
0xFD2F: C3 33 FD   JP 0xFD33
0xFD32: 23         INC HL
0xFD33: CD 96 FD   CALL 0xFD96
0xFD36: 54         LD D, H
0xFD37: 5D         LD E, L
0xFD38: CD C6 FD   CALL 0xFDC6
0xFD3B: 1A         LD A, (DE)
0xFD3C: 67         LD H, A
0xFD3D: CD CC FD   CALL 0xFDCC
0xFD40: CD A5 FD   CALL FD A5
0xFD43: EB         EX DE, HL
0xFD44: DA 32 FD   JP C, FD 32
0xFD47: 77         LD (HL), A
0xFD48: BE         CP (HL)
0xFD49: CA 32 FD   JP Z, FD32
0xFD4C: 3E 3F      LD A, 3F
0xFD4E: CD F2 FD   CALL FDF2
0xFD51: C3 08 FD   JP FD08
0xFD54: CD A0 FD   CALL FDA0





0xFDE3: 3E 20       MVI A, 0x20
0xFDE5: C3 F2 FD    JMP 0xFDF2
0xFDE8: DB 00       IN  A, (0x00)
0xFDEA: 0F          RRC
0xFDEB: D2 E8 FD    JNC 0xFDE8
0xFDEE: DB 01       IN  A, (0x01)
0xFDF0: E6 7F       ANI 0x7F
0xFDF2: F5          PUSH PSW
0xFDF3: 81          ADD C
0xFDF4: 4F          MOV C, A
0xFDF5: DB 00       IN  A, (0x00)
0xFDF7: 0F          RRC
0xFDF8: 0F          RRC
0xFDF9: D2 F5 FD    JNC 0xFDF5
0xFDFC: F1          POP PSW
0xFDFD: D3 01       OUT (0x01), A
0xFDFF: C9          RET
```

## TURNMON HEX
```
:10FD00003E03D3003E11D3003100FCCD96FDCD96CD
:10FD1000FD3E2ECDF2FDCDE8FDFE4DCA2CFDFE448C
:10FD2000CA54FDFE4AC208FDCDA0FDE9CDA0FDC329
:10FD300033FD23CD96FD545DCDC6FD1A67CDCCFDB8
:10FD4000CDA5FDEBDA32FD77BECA32FD3E3FCDF2E6
:10FD5000FDC308FDCDA0FDEBD4E3FDCDA0FDE56224
:10FD60006BCD96FDCDC6FDCDE3FD0110001A67C534
:10FD70003E08B9C27EFD3E2DCDF2FDCDE3FDCDCCDA
:10FD8000FDC1E17CBAC28DFD7DBBCA08FDE5130D46
:10FD9000C26DFDC35FFD3E0DCDF2FD3E0AC3F2FD17
:10FDA0000606C3A7FD0603210000CDE8FD4FFE2097
:10FDB00037C8E6B8EE30C24CFD79E6072929298517
:10FDC0006F05C2AAFDC90606AFC3D6FD0603AFC3C1
:10FDD000D3FD291729172917E607F630CDF2FD05BF
:10FDE000C2D2FD3E20C3F2FDDB000FD2E8FDDB01F5
:10FDF000E67FF5814FDB000F0FD2F5FDF1D301C98E
:00000001FF
```
## Memory Dump
```
0xFD00:
3E 03 D3 00 3E 11 D3 00 31 00 FC CD 96 FD CD 96 FD 3E 2E CD F2 FD CD E8 FD FE 4D CA 2C FD FE 44
CA 54 FD FE 4A C2 08 FD CD A0 FD E9 CD A0 FD C3 33 FD 23 CD 96 FD 54 5D CD C6 FD 1A 67 CD CC FD
CD A5 FD EB DA 32 FD 77 BE CA 32 FD 3E 3F CD F2 FD C3 08 FD CD A0 FD EB D4 E3 FD CD A0 FD E5 62
6B CD 96 FD CD C6 FD CD E3 FD 01 10 00 1A 67 C5 3E 08 B9 C2 7E FD 3E 2D CD F2 FD CD E3 FD CD CC
FD C1 E1 7C BA C2 8D FD 7D BB CA 08 FD E5 13 0D C2 6D FD C3 5F FD 3E 0D CD F2 FD 3E 0A C3 F2 FD
06 06 C3 A7 FD 06 03 21 00 00 CD E8 FD 4F FE 20 37 C8 E6 B8 EE 30 C2 4C FD 79 E6 07 29 29 29 85
6F 05 C2 AA FD C9 06 06 AF C3 D6 FD 06 03 AF C3 D3 FD 29 17 29 17 29 17 E6 07 F6 30 CD F2 FD 05
C2 D2 FD 3E 20 C3 F2 FD DB 00 0F D2 E8 FD DB 01 E6 7F F5 81 4F DB 00 0F 0F D2 F5 FD F1 D3 01 C9
```
