;===============================================================
; MBLe - Enhanced Multi Boot Loader for the Altair 8800
;
; Loads and runs an Altair 'Absolute Binary File' from input
; transfer port specified by the Sense Switch settings.
; Normally run in PROM at address 0FE00h. However, since
; version 3.00 the PROM is position independent and can run
; at most any 256 byte boundary.
;
; Vers.   Date      Author    Comments
; 1.00  01Sep2013  M. Eberhard
;    Disassembled from MITS EPROMS
; 1.01  01Sep2013  M. Eberhard
;    Modified to support e.g. a fast reader on the 2SIO's 2nd
;    port, instead of a MITS HSR
; 1.02  03Sep2013  M. Eberhard
;    initial read from parallel ports to clear latches
;    (fixes the OP-80)
; 1.03  09Sep2013  M. Eberhard
;    fix for no checksum loader (see 7 below.)
; 1.04  12Mar2014  M. Eberhard
;    Search for end of RAM before initializing ports, so the
;    UART has time to finish echoing, upon entry
; 1.05  05Jun2014  M. Eberhard
;    Ignore sense switch A11 (see 9 below)
; 2.00  11Aug2014  M. Eberhard
;    Major rewrite: copy to RAM and run from there, so it works
;    with an 8800b Turnkey Module too
; 2.01 24Aug2014  M. Eberhard
;   Also move I/O port setup to RAM code, so that MBLe will
;   work with older Turnkey modules, with just the 88-SYS-CLG
;   rework (Which may disable PROM on any IN or OUT
;   instruction). Also tidy up comments.
; 3.00 16Jan2016 M. Douglas
;   Make the PROM position independent by making the RAM
;   sizing and relocation routines position independent.
;   Change the address relocation technique to free up the
;   space required for the position independent code.
;   Eliminate the two second start-up delay by sizing RAM with
;   a page-by-page search instead of a byte-by-byte search.
;
; Written to assemble with ASM by Digital Research. 
;
; Thanks to Geoff Harrison for his MBL disassembly, which I
; plagerized freely.
;
;** Differences between MITS MBL and MBLe **
;
; 1) The code starts off by relocating itself to the highest
;    page of RAM that is found, so that it will still work on
;    a Turnkey Module that phantoms the PROMs upon a IN from
;    port FFh (the Sense Switches), or any IN or OUT
;    instruction (i.e. 88-SYS-CLG Turnkey Modules) 
; 2) All HSR support is eliminated, including 88-4PIO Port 1
;    initialization and code for starting the HSR transport.
; 3) The second 88-2SIO port (port 1) is initialized.
; 4) The 88-HSR entry in PTABLE is replaced with an entry for
;    the 8-2SIO port 1. See sense switch table below.
; 5) PTABLE has an 8th entry, which is the same as the 7th
;    (2SIO port 1). Testing for illegal sense switch setting
;    is eliminated.
; 6) An initial read is performed for both the 88-PIO and the
;    88-4PIO Port 0, to clear data handshake latches in
;    external devices such as the OP-80 paper tape reader
; 7) If the leader character is 0, then no checksum loader
;    will be skipped. 
; 8) The end-of-memory hunt to find the end of RAM occurs before
;    the I/O ports get initialized, instead of afterwards. This
;    gives time for the UART to finish transmitting the echo of
;    e.g. the 'T' character, when entering MBLe from UBMON.
; 9) Sense switch A11 is ignored when getting the load device,
;    rather than generating an I error. This allows A11 to be
;    used for something else - e.g. selecting a boot disk.
;===============================================================
; Program Notes
; Since the 8800b Turnkey Module disables PROMS whenever an IN
; instruction accesses port FFh, this code cannot execute from
; PROM - at least not from the point where the Sense Switches
; are read onwards. Additionally, some versions of the Turnkey
; Module are broken, and will disable PROM when *any* IN
; instruction is executed. This means that MITS's MBL, as well
; as versions 1.xx of MBLe, will not work in a Turnkey Module.
;
; MBLe 3.00 Strategy:
; 1) Search the memory space for the highest actual RAM, as
;    MITS's MBL did. This page of memory will be used not only
;    for the stack, but also for the relocated MBLe code.
; 2) Copy code into the high RAM page that was found in step 1.
;    (This is called the RAM Execution Page.) The high byte
;    of addresses are relocated to the RAM execution address
;    as the bytes are copied.
; 3) Jump to the RAM code, and run from there - never to
;    return to PROM.
;
; The RAM page is laid out as follows:
;  * The high portion (From RAMPAG up to FFh) contains the
;    relocated MBLe code
;  * Immediately below this is the stack, initialized to RAMPAG
;    and growing downward. (Note that a PUSH decrements the
;    stack pointer before writing to the stack.)
; The stack therefore has as much space as is occupied by the
; bit of code that executes from PROM: plenty of room.
;
; Although this uses more memory in the highest page of RAM,
; it will behave the same as MITS's MBL, because both programs
; still abort with an "M" error if a Load Record attempts to
; write anywhere into the page that contains the stack.
;
;===============================================================
; An Altair 'Absolute Binary File' has 4 sections, which may be
; separated by any number of nulls. These sections are:
;
; 1) The Leader, which comprises 2 or more identical bytes, the
;    value of which is the length of the Checksum Loader.
;
; 2) The Checksum Loader, which is a program that is normally
;    used to load the subsequent sections
;
; 3) Zero or more Load Records, each structured as follows:
;       Byte 0: Sync Byte = 3Ch (identifies a Load Record)
;       Byte 1: NN = number of data bytes in record
;       Byte 2: LL = Load address low byte
;       Byte 3: HH = Load address high byte
; Bytes 4-NN+3: NN data bytes to store at HHLL, NN>0
;    Byte NN+4: CC = checksum of bytes 2 through NN+3
;
; 4) The Go Record, structured as follows
;       Byte 0: Sync Byte = 78H (identifies the Go Record)
;       Byte 1: LL = low byte of go address
;       Byte 2: HH = high byte of go address
;
; Altair file Leaders and Checksum Loaders are specific to
; both the version of the particular software and the memory
; size. For example, the Checksum Loader for 4K Basic 3.2 is
; different than the Checksum Loader for 8K Basic 3.2. And
; both the Leader and Checksum Loader for 8K Basic 3.2 are
; different than those for 8K Basic 4.0.
;
; MBL and MBLe are able to read any such Altair file by simply
; skipping over the Leader and Checksum Loader, and loading
; the Load and Go Records directly.
;
; MBLe chooses its input port based on the front panel Sense
; Switches <2:0>, using the conventions set up in Basic 4.X,
; more or less.
;
;  Device                   bits 2:0
;  88-2SIO Port A (2 stops)   000b
;  88-2SIO Port A (2 stops)   001b
;  88-SIO                     010b
;  88-ACR                     011b
;  88-4PIO                    100b
;  88-PIO                     101b
;  88-2SIO Port B (2 stops)   110b
;  88-2SIO Port B (2 stops)   111b
;
; Prior to Basic 4.0, MITS used different Sense Switch settings
; to specify the console device. You can load an older tape
; with MBLe by setting the switches according to the above
; table and starting the load. After MBL has skipped over the
; Checksum Loader on the tape and has begun to load the Load
; Records (but before the load completes) change the Sense
; Switch settings as required by the earlier version of Basic
; (or other program) that you are loading.
;--------------------------------------------------------------
; 8080 EQUATES
;--------------------------------------------------------------
JZOP	EQU	0CAH	;JZ OPCODE
JNZOP	EQU	0C2H	;JNZ OPCODE
;--------------------------------------------------------------
; ALTAIR ABSOLUTE BINARY FILE EQUATES
;--------------------------------------------------------------
ALTPLR	EQU	3CH	;PROGRAM LOAD RECORD
ALTEOF	EQU	78H	;EOF/GO ADDRESS RECORD
ALTBNR	EQU	55H	;BEGIN/PROGRAM NAME (NOT SUPPORTED)
ALTBND	EQU	0DH	;END-OF-NAME MARK (NOT SUPPORTED)
;--------------------------------------------------------------
;SENSE SWITCH EQUATES
;--------------------------------------------------------------
SSWTCH	EQU	0FFH	;FRONT PANEL SWITCH REGISTER
LDMASK	EQU	007H	;LOAD DEVICE MASK <-ME (WAS 00FH)
;--------------------------------------------------------------
;88-2SIO EQUATES
;--------------------------------------------------------------
;88-2SIO REGISTERS

S2CTLA	EQU	10H	;ACIA A CONTROL OUTPUT PORT
S2STAA	EQU	10H	;ACIA A STATUS INPUT PORT
S2TXDA	EQU	11H	;ACIA A TX DATA REGISTER
S2RXDA	EQU	11H	;ACIA A RX DATA REGISTER
S2CTLB	EQU	12H	;ACIA B CONTROL OUTPUT PORT
S2STAB	EQU	12H	;ACIA B STATUS INPUT PORT
S2TXDB	EQU	13H	;ACIA B TX DATA REGISTER
S2RXDB	EQU	13H	;ACIA B RX DATA REGISTER

;ACIA CONTROL REGISTER BITS

S2DS1	EQU	00000001B	;COUNTER DIVIDE SEL 1
S2DS2	EQU	00000010B	;COUNTER DIVIDE SEL 2
S2WS1	EQU	00000100B	;WORD SELECT 1
S2WS2	EQU	00001000B	;WORD SELECT 2
S2WS3	EQU	00010000B	;WORD SELECT 3
S2TC1	EQU	00100000B	;TX CONTROL 1
S2TC2	EQU	01000000B	;TX CONTROL 2
S2RIE	EQU	10000000B	;RX INT ENABLE

S2RST	EQU	00000011B	;MASTER RESET

;ACIA STATUS REGISTER BITS

S2RDF	EQU	00000001B	;RX DATA REG FULL
S2TDE	EQU	00000010B	;TX DATA REG EMPTY
S2DCD	EQU	00000100B	;DATA CARRIER DETECT
S2CTS	EQU	00001000B	;CLEAR TO SEND
S2FE	EQU	00010000B	;FRAMING ERROR
S2ORE	EQU	00100000B	;RX OVERRUN ERROR
S2PE	EQU	01000000B	;PARITY ERROR
S2IRQ	EQU	10000000B	;INTERRUPT REQUEST
;--------------------------------------------------------------
;88-SIO EQUATES
;--------------------------------------------------------------
;88-SIO REGISTERS	

SIOCTL	EQU	00		;CONTROL PORT
SIOSTA	EQU	00		;STATUS
SIOTXD	EQU	01		;TRANSMIT DATA
SIORXD	EQU	01		;RECEIVE DATA

;STATUS REGISTER BITS

SIOIDR	EQU	00000001B	;INPUT DEV RDY (RX BUF FULL)
SIOPE	EQU	00000100B	;PARITY ERROR
SIOFE	EQU	00001000B	;FRAMING ERROR
SIODOV	EQU	00010000B	;DATA OVERFLOW
SIOODR	EQU	10000000B	;OUTPUT DEV RDY (TX BUF EMPTY)
;--------------------------------------------------------------
;88-ACR (AUDIO CASSETTE RECORDER) EQUATES
;NOTE: THE ALTAIR 88-ACR IS BUILT AROUND AN ALTAIR 88-SIO
;--------------------------------------------------------------
;88-ACR REGISTERS	

ACRCTL	EQU	06		;CONTROL PORT
ACRSTA	EQU	06		;STATUS
ACRTXD	EQU	07		;TRANSMIT DATA
ACRRXD	EQU	07		;RECEUVE DATA

;STATUS REGISTER BITS

ACRIDR	EQU	00000001B	;INPUT DEV RDY (RX BUF FULL)
ACRPE	EQU	00000100B	;PARITY ERROR
ACRFE	EQU	00001000B	;FRAMING ERROR
ACRDOV	EQU	00010000B	;DATA OVERFLOW
ACRODR	EQU	10000000B	;OUTPUT DEV RDY (TX BUF EMPTY)
;--------------------------------------------------------------
;88-4PIO EQUATES
;NOTE: THE 88-HSR USES PORT 1 OF THE 88-4PIO
;--------------------------------------------------------------
;88-4PIO REGISTERS	

P4CA0	EQU	20H		;PORT 0 SECTION A CTRL/STATUS
P4DA0	EQU	21H		;PORT 0 SECTION A DATA
P4CB0	EQU	22H		;PORT 0 SECTION B CTRL/STATUS
P4DB0	EQU	23H		;PORT 0 SECTION B DATA
P4CA1	EQU	24H		;PORT 1 SECTION A CTRL/STATUS
P4DA1	EQU	25H		;PORT 1 SECTION A DATA
P4CB1	EQU	26H		;PORT 1 SECTION B CTRL/STATUS
P4DB1	EQU	27H		;PORT 1 SECTION B DATA

;CONTROL REGISTER BITS

P4C1C0	EQU	00000001B	;C1 CONTROL BIT 0
P4C1C1	EQU	00000010B	;C1 CONTROL BIT 1
P4DDR	EQU	00000100B	;DATA DIRECTION REGISTER
P4C2C3	EQU	00001000B	;C2 CONTROL BIT 3
P4C2C4	EQU	00010000B	;C2 CONTROL BIT 4
P4C2C5	EQU	00100000B	;C2 CONTROL BIT 5
P4IC2	EQU	01000000B	;C2 INTERRUPT CONTROL BIT
P4IC1	EQU	10000000B	;C1 INTERRUPT CONTROL BIT

;STATUS REGISTER BITS

P4RDF	EQU	10000000B	;RX DATA REG FULL
HSRRDF	EQU	01000000B	;RX DATA REG FULL for HSR
;--------------------------------------------------------------
;88-PIO EQUATES
;--------------------------------------------------------------
;88-PIO REGISTERS	

PIOCTL	EQU	04		;CONTROL PORT
PIOSTA	EQU	04		;STATUS
PIOTXD	EQU	05		;TRANSMIT DATA
PIORXD	EQU	05		;RECEIVE DATA

;STATUS REGISTER BITS

PIORDF	EQU	00000010B	;RX DATA REG FULL
;--------------------------------------------------------------
;SINGLE-BYTE ERROR MESSAGES
;--------------------------------------------------------------
CERMSG	equ	'C'	;checksum error
MERMSG	equ	'M'	;memory error
OERMSG	equ	'O'	;overwrite error

;--------------------------------------------------------------
;RELOCATION EQUATES
;--------------------------------------------------------------

;  ORG statement
;     Run-time relocation of addresses is done by replacing any
;     byte that matches the MSB of the ORG address with the MSB
;     of the destination RAM address. This requires that the value
;     of the ORG MSB never appears in the assembled code other
;     than as the MSB of an address. FD00 works well for this.

	org	0FD00h
ADRMARK	equ	$FD00	;address mark value	*****changed from ($ SHR 8)
STACK0	equ	80h		;page zero stack


;==============================================================
; Start of Code
;==============================================================

	di			;interrupts off during load

;--------------------------------------------------------------
;  Size RAM (position independent). Assumes page 0 exists,
;     then checks byte C9 in each page until a page is found
;     that is not RAM.
;--------------------------------------------------------------
	lxi	sp,STACK0	;setup a stack on page zero
	lxi	h,$C900		;H=0, L=RET instruction *****changed from RET
 	push	h		;RET at STACK0-2
	call	STACK0-2	;puts addr of SZLOOP in stack RAM

; SZLOOP - check a byte in each page until RAM not found

SZLOOP:	dcx	sp		;point SP to SZLOOP address
	dcx	sp		;    in stack memory

	inr	h		;move to next 256 byte page
	mov	a,m		;read from address in hl
	mov	b,a		;save original value in b
	cma			;form and write inverted value
	mov	m,a
	cmp	m		;read and compare
	mov	m,b		;restore original value
	rz	;jz SZLOOP

;--------------------------------------------------------------
;  Move PROM image to high RAM (position independent).
;     On entry, HL is within the 1st page AFTER the end of
;     RAM. At the current stack pointer is the address of
;     SZLOOP from the RAM sizing loop above.
;--------------------------------------------------------------
	pop	b		;BC->SZLOOP in PROM
	mvi	c,$2C	;BC->NOTREL in PROM ***changed from NOTREL AND 0ffH

	dcr	h		;point HL inside last page of RAM
	xchg			;form RAM destination in DE
	mvi	e,$23	;DE->MOVELP in RAM ***changed from MOVELP AND 0ffH

	lxi	h,0E9E1h	;H=PCHL,L=POP H
	push	h		;POP H, PCHL at STACK-4, STACK-3
	call	STACK0-4	;PROM MOVELP in stack mem and HL
	
; Move code from PROM to RAM. If a byte matches the MSB of the 
;     assembled ORG address (i.e., it is an address MSB), then
;     it is replaced with the destination RAM MSB.

MOVELP:	dcx	sp		;point SP to MLOOP address
	dcx	sp		;    in stack memory

	mov	a,m		;Get next EPROM byte

	cpi	$FD		;relocatable address byte? ***changed from ADRMARK
	push	b		;put jump address on stack
	rnz 	;jnz NOTREL
	pop	b		;remove unused jump address
	mov	a,d		;a=MSB of code image in RAM

NOTREL:	stax	d		;move byte to RAM

	inr	e		;bump pointers
	inr	l
	rnz 	;jnz MOVELP  	copy to end of 256 byte page

; code image has been copied to high RAM and addresses relocated. Init
;   the stack pointer just below the code, then jump to the code.

	mov	h,d		;HL and DE in RAM execution page
	mvi	l,$35	;HL->entry address in RAM **changed from RAMCOD AND 0ffh
	sphl			;stack grows down from start
	pchl			;jump to start
	
;==============================================================
; RAM Execution Code
; All of the following code gets copied into the RAM Execution
; Page (which is the highest page of RAM that was discovered
; during initialization).
; On Entry:
;    d = h = RAM Execution Page
;==============================================================

; Delay 1/10s to allow time for a 110 baud character to finish
;    transmission. A character may have been typed in a monitor
;    (e.g., UBMON) just prior to entering MBL. The RAM size and
;    copy loops above provide a max delay of about 10ms.

RAMCOD:	lxi	b,8333		;1/10s, 24 cycle loop @2mhz

DELAY:	dcx	b		;5
	mov	a,b		;5
	ora	c		;4
	jnz	DELAY		;10

;--------------------------------------------------------------
; Reset all known load devices
; Note that a bug in the 88-SYS-CLG rework to older Turnkey
; Modules will cause any IN or OUT instruction to occasionally
; cause the PROMs to become disabled. For this reason, this
; initiaization is done after the code relocates to RAM.
; On Entry:
;    a = 0
;    h = d = RAM Execution Page
; On Exit:
;    h = d = RAM Execution Page
;--------------------------------------------------------------
;Make 4PIO 'A' channels inputs and 'B' channels outputs

	out	P4CA0		;access 4PIO Port 0A DDR
	out	P4DA0		;set 4PIO Port 0A as input

	out	P4CB0		;access 4PIO Port 0B DDR
	cma			;0FFH
	out	P4DB0		;set 4PIO Port 0B as output

;Set up the other 3 4PIO ports all the same

	mvi	a,2cH	;bits 0,1: C1 input active low, int off
			;bit 2: access data reg
			;bits 3-5: C2 output handshake

	out	P4CA0		;4PIO Port 0A control
	out	P4CB0		;4PIO Port 0B control

;Send reset command to both 2SIO ports

	mvi	a,S2RST		;2SIO reset
	out	S2CTLA		;2SIO Port A
	out	S2CTLB		;2SIO Port B

;Set up both 2SIO ports: 8 data bits, 2 stop bits, no parity, 
;   clock divide by 16 

	mvi	a,11H		;8N2, /16
	out	S2CTLA		;2SIO Port 0 control
	out	S2CTLB		;2SIO Port 1 control

;--------------------------------------------------------------
; Patch the GETBYT routine with the correct parameters for the
; load port that is specified by Sense Switches 2:0
; On Entry & Exit:
;    h = d = RAM Execution Page
;--------------------------------------------------------------
	in	SSWTCH			;read sense switches
					;This also disables PROMS...
	ani	LDMASK			;bits specifies load device

	add	a			;2 bytes/entry
	adi	$ED	    ;Look up in PTABLE **changed PTABLE and 0FFh
	mov	e,a		;de=PTABLE((SWITCHS) <2:0>)

	ldax	d
	mvi	l,$E6	;Data port addr place **changed (GBDP+1) and 0FFH
	rar				;move jnz flag int carry
	mov	m,a			;install data port addr

	mvi	l,$DF	;Status port addr place **changed (GBSP+1) and 0FFh
	dcr	a			;stat port = data port-1
	mov	m,a			;install stat port addr

	inr	e			;next table entry is
	ldax	d			;..the status port mask
	mvi	l,$E1	;status mask place **changed (GBMASK+1) and 0FFh
	mov	m,a			;install stat port mask

	jnc	ITSJZ			;test jnz flag
	inr	l			;jnz right after mask
	mvi	m,JNZOP			;install jnz opcode

;------------------------------------------------
; Flush external data latches for e.g. the OP-80
; or flush garbage from UARTs
; On Entry & Exit:
;    d = RAM Execution Page
;-------------------------------------------------
ITSJZ:	call	GETNOW

;--------------------------------------------------------------
; Skip over leader - a sequence of identical bytes, the value
; of which is the length of the checksum loader. If the value
; is  0, then there is no loader to skip, so go get records.
; On Entry:
;    d = RAM Execution Page
; On exit:
;    c = checksum loader length
;    d = RAM Execution Page
;    The 1st byte of the checksum loader has already been read
;--------------------------------------------------------------
	call 	GETBYT		;get 1st byte
	mov	c,a		;number of bytes in loader
	ora	a		;Null leader?
	jz	RCHUNT		;y: skip leader


LDSKIP:	call	GETBYT		;get another byte
	cmp	c
	jz	LDSKIP		;loop until different

;--------------------------------------------------------------
; Skip over checksum loader
;
; On Entry:
;    The 1st byte of the checksum loader has already been read
;    c=checksum loader length
;    d = RAM Execution Page
; On Exit:
;    d = RAM Execution Page
;    The checksum loader has been skipped
;--------------------------------------------------------------
	dcr	c		;since we got a byte already

CLSKIP:	call	GETBYT		;get a loader byte
	dcr	c
	jnz	CLSKIP

;--------------------------------------------------------------
; Main Record-Loading Loop
;
; Hunt for a sync character - either for another Load Record
; or for the Go Record. Ignore all else.
; On Entry:
;    c = 0
;    d = RAM Execution Page
; On jmp tp LDREC:
;    c = 0
;    d = RAM Execution Page
;    RCHUNT address is on the stack
;--------------------------------------------------------------
RCHUNT:	mov	h,d		;restore page address
	mvi	l,$8F  ;**change RCHUNT and 0FFh
	push	h		;create return address

	call	GETBYT		;hunt for sync character
	cpi	ALTPLR		;load record sync byte?
	jz	LDREC		;Y: go load the record

	cpi	ALTEOF		;EOF record sync byte?
	rnz			;N: ignore

; Fall into GO record execution

;--------------------------------------------------------------
; Go Record: Get the Go Address and go there
;
; On Entry:
;    Go-Record sync byte has already been read
;--------------------------------------------------------------
	call	GETWRD		;get a,l=address
	mov	h,a		;high byte
	pchl			;go there

;--------------------------------------------------------------
; Load Record: Read and store data from a Load Record
;
; On Entry:
;    The Load Record sync byte has already been read
;    c = 0
;    d = RAM Execution Page
;    RCHUNT's address is on the stack
; On Return (to RCHUNT):
;    c = 0
;    d = RAM Execution Page
;    A complete Load Record's data has been loaded into RAM
;--------------------------------------------------------------
LDREC:	call	GETBYT		;get record byte count
	mov	b,c		;c=0: initialize checksum
	mov	c,a		;c counts data bytes

	call	GETWRD		;get load address into a,l
	mov	h,a		;hl = record load address
	
;Loop to read c data bytes into memory at hl.
;Make sure data won't overwrite RAM Execution Page.

LRLOOP:	mov	a,d		;d=RAM Execution Page
	cmp	h		;error if same page as load address
	mvi	a,OERMSG	;overwrite error message
	jz	ERDONE		;error exit if overwrite

	call	GETBYT		;get a data byte
	mov	m,a		;store data byte
	cmp	m		;did it store correctly?

;Entry at MERR from end-of-RAM search (while running in ROM)

MERR:	mvi	a,MERMSG	;Memory Error message
	jnz	ERDONE		;error exit if mismatch

	inx	h		;bump dest pointer
	dcr	c		;bump byte count
	jnz	LRLOOP		;loop through all bytes

; Validate checksum, fail if it doesn't match
; c = 0 here

	call	GETBYT		;test record's checksum
	rz			;match: get another record

	mvi	a,CERMSG	;Checksum Error message

; Fall into ERDONE

;---End-------------------------------------------------------
;Error handler:
; Save error code and address at beginning of memory
; Hang writing the error code forever, to all known consoles.
;-------------------------------------------------------------
ERDONE:	sta	00000H
	shld	00001H
	ei

ERHANG:	out 	SIOTXD		;SIO
	out 	S2TXDA		;2SIO
	out 	PIOTXD		;PIO
	out 	P4DB0		;4PIO port 0
	jmp 	ERHANG

;---Subroutine-----------------------
; Get 2-byte word from transfer port
; On Entry:
;   b=checksum so far
; On Exit:
;   l = next byte
;   a = subsequent byte
;   b := b+a+l
;------------------------------------
GETWRD:	call	GETBYT
	mov	l,a

; Fall into GETBYT
	
;---Subroutine-----------------------------------------
; Wait for and get a byte from the transfer port
; This code gets modified once the input port is known
; On Entry:
;   b=checksum so far
; On Exit:
;   a = input character
;   Z set if received byte matched previous checksum
;   b := b+a
;------------------------------------------------------
GETBYT:
GBSP:	in	0	;(Status Port Address)read status
GBMASK:	ani	0	;(Port Mask)

	jz	GETBYT	;(may become jnz) wait for data

; Fall into GETNOW

;---Subroutine-----------------------------------------
; Get a byte from the transfer port
; This code gets modified once the input port is known
; On Entry:
;   b=checksum so far
; On Exit:
;   a = input character
;   Z set if received byte matched previous checksum
;   b := b+a
;------------------------------------------------------
GETNOW:			;call to flush port
GBDP:	in	0	;(Data Port place)get data byte


	cmp	b	;set Z if this byte matched cksum
	push	psw	;temp save
	add	b	;update checksum in B
	mov	b,a
	pop	psw	;recover data byte
	ret		;A=byte, B=checksum

;---Table------------------------------------------------------
;Port Parameters: One 2-byte entry for each input port:
; Byte 1 = Data port address * 2 + JNZ flag
; (The status port is assumed to immediately preceed the data
; port.)
; Byte 2 = ready mask for data input
;--------------------------------------------------------------
PTABLE:	db	S2RXDA*2,S2RDF		;0:2SIO A (2 stop bits)
	db	S2RXDA*2,S2RDF		;1:2SIO A (2 stop bits)
	db	SIORXD*2+1,SIOIDR	;2:SIO
	db	ACRRXD*2+1,ACRIDR	;3:ACR
	db	P4DA0*2,P4RDF		;4:4PIO Port 0
	db	PIORXD*2,PIORDF		;5:PIO
	db	S2RXDB*2,S2RDF		;6:2SIO B (2 stop bits)

; The last table entry is just a copy to fill it out.

	db	S2RXDB*2,S2RDF		;7:2SIO B (2 stop bits)

	end
