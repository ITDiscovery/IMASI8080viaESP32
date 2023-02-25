```
0xFD00 3E 03    MVI A, #0x03
0xFD02 D3 10    OUT #0x10, A
0xFD04 3E 11    MVI A, #0x11
0xFD06 D3 10    OUT #0x10, A
0xFD08 31 00 FC LXI SP, #0xFC00
0xFD0B CD 9D FD CALL 0xFD9D
0xFD0E 3E 2E    MVI A, #0x2E
0xFD10 CD F2 FD CALL 0xFDF2
0xFD13 CD E8 FD CALL 0xFDE8
0xFD16 FE 4D    CPI #0x4D
0xFD18 CA 29 FD JZ 0xFD29
0xFD1B FE 44    CPI #0x44
0xFD1D CC 4F FD CZ 0xFD4F
0xFD20 FE 4A    CPI #0x4A
0xFD22 C2 08 FD JNZ 0xFD08
0xFD25 CD A7 FD CALL 0xFDA7
0xFD28 E9       PCHL
0xFD29 CD A7 FD CALL 0xFDA7
0xFD2C 3E 23    MVI A, #0x23
0xFD2E CD 9D FD CALL 0xFD9D
0xFD31 54       MOV H, A
0xFD32 5D       MOV E, L
0xFD33 CD C9 FD CALL 0xFDC9
0xFD36 1A       LDAX D
0xFD37 67       MOV H, A
0xFD38 CD CF FD CALL 0xFDCF
0xFD3B CD A8 FD CALL 0xFDA8
0xFD3E EB       XCHG
0xFD3F DA 2D FD JC 0xFD2D
0xFD42 77       MOV M, A
0xFD43 BE       CMP M
0xFD44 CA 2D FD JZ 0xFD2D
0xFD47 3E 3F    MVI A, #0x3F
0xFD49 CD F2 FD CALL 0xFDF2
0xFD4C C3 08 FD JMP 0xFD08
0xFD4F CD A7 FD CALL 0xFDA7
0xFD52 EB       XCHG
0xFD53 D4 E3 FD CNC 0xFDE3
0xFD56 CD A7 FD CALL 0xFDA7
0xFD59 3E 0D    MVI A, #0x0D
0xFD5B 06 3C    MVI B, #0x3C
0xFD5D CD F2 FD CALL 0xFDF2
0xFD60: 78        MOV A,B
0xFD61: C2 5B FD  JNZ $FD5B
0xFD64: 7D        MOV A,L
0xFD65: 93        SUB E
0xFD66: 6F        MOV L,A
0xFD67: 7C        MOV A,H
0xFD68: 9A        SBB D
0xFD69: 67        MOV H,A
0xFD6A: 23        INX H
0xFD6B: 05        DCR B
0xFD6C: 7C        MOV A,H
0xFD6D: B7        ORA A
0xFD6E: C2 77 FD  JNZ $FD77
0xFD71: 45        MOV B,L
0xFD72: 3E 3C     MVI A,0x3C
0xFD74: CD F2 FD  CALL $FDF2
0xFD77: 78        MOV A,B
0xFD78: CD F2 FD  CALL $FDF2
0xFD8B: 0E 00       LD C, 0x00
0xFD8D: 7B          LD A, E
0xFD8E: CD F2 FD    CALL 0xFDF2
0xFD91: 7A          LD A, D
0xFD92: CD F2 FD    CALL 0xFDF2
0xFD95: 1A          LD A, (DE)
0xFD96: CD F2 FD    CALL 0xFDF2
0xFD99: 13          INC DE
0xFD9A: 2B          DEC HL
0xFD9B: 05          DEC B
0xFD9C: C2 8A FD    JP NZ, 0xFD8A
0xFD9F: 79          LD A, C
0xFDA0: CD F2 FD    CALL 0xFDF2
0xFDA3: 7C          LD A, H
0xFDA4: B5          OR L
0xFDA5: C2 70 FD    JP NZ, 0xFD70
0xFDA8: 3E 0D       LD A, 0x0D
0xFDAA: CD F2 FD    CALL 0xFDF2        ; Call subroutine at address 0xFDF2
0xFDAD: 3E 0A       LD   A,0x0A         ; Load 0x0A into register A
0xFDAF: C3 F2 FD    JP   0xFDF2         ; Jump to address 0xFDF2
0xFDB2: 06 06       LD   B,0x06         ; Load 0x06 into register B
0xFDB4: 03          INC  BC             ; Increment contents of register pair BC
0xFDB5: 21 00 00    LD   HL,0x0000      ; Load 0x0000 into register pair HL
0xFDB8: CD E8 FD    CALL 0xFDE8        ; Call subroutine at address 0xFDE8
0xFDBB: 4F          LD   C,A            ; Load contents of register A into register C
0xFDBC: FE 20       CP   0x20           ; Compare contents of register A with 0x20
0xFDBE: 37          STC
0xFDBF: C8          RZ
0xFDC0: E6 B8       ANI 0xB8
0xFDC2: EE 30       XRI 0x30
0xFDC4: C2 47 FD    JNZ 0xFD47
0xFDC7: 79          MOV A,C
0xFDC8: E6 07       ANI 0x07
0xFDCA: 29          DAD H
0xFDCB: 29          DAD H             | Add HL to HL
0xFDCC: 29          DAD H             | Add HL to HL
0xFDCD: 85          ADD A, L          | Add L to A
0xFDCE: 6F          LD L, A           | Load A into L
0xFDCF: 05          DEC B             | Decrement B
0xFDD0: C2 AD FD    JNZ $FDA0         | Jump if not zero to address $FDA0
0xFDD3: C9          RET               | Return
0xFDD4: 06 06       LD B, $06         | Load $06 into B
0xFDD6: AF          XOR A             | XOR A with A
0xFDD7: C3 D6 FD    JMP $FDD6         | Jump to address $FDD6
0xFDDA: 06 03       LD B, $03         | Load $03 into B
0xFDDC: E6 29       AND $29           | Logical AND with $29
0xFDDE: 17          RAL               | Rotate A left through carry
0xFDDF: 29          DAD H             | Add HL to HL
0xFDE0: 17          RAL               | Rotate A left through carry
0xFDE1: 29          DAD H             | Add HL to HL
0xFDE2: 17          RAL               | Rotate A left through carry
0xFDE3: E6 07       AND $07           | Logical AND with $07
0xFDE5: F6 30       OR $30            | Logical OR with $30
0xFDE7: CD F2 FD    CALL $FDF2        | Call subroutine at address $FDF2
0xFDEA: 05          DEC B             | Decrement B
0xFDEB: C2 D2 FD    JP NZ, 0xFDCD      ; Jump if Z flag is not set to 0xFDCD
0xFDEE: 3E 20       LD A, 0x20         ; Load value 0x20 into register A
0xFDF0: C3 F2 FD    JP 0xFDF2          ; Jump to 0xFDF2
0xFDF2: DB 10 0F    IN A, (0xF)        ; Read from port 0xF and store the result in A
0xFDF5: D2 E8 FD    JP NC, 0xFDEE      ; Jump if C flag is not set to 0xFDEE
0xFDF8: DB 11       IN A, (C)          ; Read from port specified by register C and store the result in A
0xFDFA: E6 7F       AND 0x7F           ; Perform a bitwise AND operation with value 0x7F and A
0xFDFC: F5          PUSH AF            ; Push register AF onto the stack
0xFDFD: 81          ADD A, C           ; Add the value of register C to A
0xFDFE: 4F          LD C, A            ; Load the value of A into register C
0xFDFF: DB 10 0F    IN A, (0xF)        ; Read from port 0xF and store the result in A
0xFE01: 0F          RRCA               ; Rotate A right
0xFE02: D2 F5 FD    JP NC, 0xFDF3      ; Jump if C flag is not set to 0xFDF3
0xFE05: F1          POP AF             ; Pop the value from the stack into register AF
0xFE06: D3 11       OUT (0x11), A      ; Write the value of A to port 0x11
0xFE08: C9          RET                ; Return from subroutine
```

3E03D3103E11D3103100FCCD96FDCD96
FD3E2ECDF2FDCDE8FDFE4DCA2CFDFE44
CA54FDFE4AC208FDCDA0FDE9CDA0FDC3
33FD23CD96FD545DCDC6FD1A67CDCCFD
CDA5FDEBDA32FD77BECA32FD3E3FCDF2
FDC308FDCDA0FDEBD4E3FDCDA0FDE562
6BCD96FDCDC6FDCDE3FD0110001A67C5
3E08B9C27EFD3E2DCDF2FDCDE3FDCDCC
FDC1E17CBAC28DFD7DBBCA08FDE5130D
C26DFDC35FFD3E0DCDF2FD3E0AC3F2FD17
0606C3A7FD0603210000CDE8FD4FFE2097
37C8E6B8EE30C24CFD79E6072929298517
6F05C2AAFDC90606AFC3D6FD0603AFC3C1
D3FD291729172917E607F630CDF2FD05BF
C2D2FD3E20C3F2FDDB100FD2E8FDDB11D5
E67FF5814FDB100F0FD2F5FDF1D311C96E


