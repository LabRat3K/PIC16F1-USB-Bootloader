; vim:noet:sw=8:ts=8:ai:syn=pic
; asmsyntax=pic
;
; File:   Main.c
; Based on:  HID demo from Szymon Roslowski
;
; 2014-10-13 S. Roslowski Created
; 2023-01-24 A. Williams - added Wyse Keyboard Handler & converted to ASM
;
;
; Build flag - re-use CDC code from the bootloader
USE_BOOTLOADER_CDC      equ 0
LOGGING_ENABLED equ 0

        radix dec
        list n=0,st=off
        include "p16f1455.inc"
        nolist
        include "macros.inc"
        include "bdt.inc"
        include "protocol_constants.inc"
        include "import_list.inc"
        list
        errorlevel -302


; -----------------------------------------------------------------------------
; Configuration Word : For standalone mode. Not required for bootloaded build.
; -----------------------------------------------------------------------------
	__config _CONFIG1, _FOSC_INTOSC & _WDTE_SWDTEN & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _BOREN_ON & _IESO_OFF & _FCMEN_OFF
	__config _CONFIG2,  _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_3x & _PLLEN_ENABLED & _STVREN_ON & _BORV_LO & _LVP_OFF

#include "usb.asm"

; ------------------------
; Button & LED definitions
; ------------------------
; Button used to trigger the bootloader reset & code
#define PROG_BUTTON              PORTA,3


; LED definitions
#define LED_NUMLOCK              LATA,LATA4
#define LED_CAPSLOCK             LATC,LATC5
#define LED_SCROLLOCK            LATC,LATC2

; ----------------
; Global Variables
; ----------------

; Use Extra flags in StatusBits : 
#define gDataPending (5)
#define gCapsLock    (6)	; Track CAPSLOCK state (used to turn on/off the CAPSLOCK LED)

 CBLOCK end_bank0_minor_vars
	gWyseTemp        ; 0x78
	gKeyIndex       ; 0x79
	gWyseBuffer     ; 0x7A
	gScanIndex		; 0x7B 0 to 19
	gScanBitIndex	; 0x7C 0 to 159
	gCalcKeyIndex	; 0x7D scratchpad calculated key index
	gTestByte		; 0x7E
	gPollBuffer		; 0x7F
        end_bank0_global_vars:0
 ENDC
 if end_bank0_global_vars > 0x80
        error "Bank 0 Global  Variable Space overrun"
 endif

   ; Wyse Input Buffers
   #define BUFFER_SIZE 20
   #define BUFFER_BITS (BUFFER_SIZE*8)

 CBLOCK 0x120 
	 gWyseReadBuffer:BUFFER_SIZE
	gSPACER:4
	 gWyseChangeBuffer:BUFFER_SIZE
	end_bank3_vars:0
 ENDC
 if end_bank3_vars > 0x170 
	error "Bank 3 GPR Variable Space Overrun"
 endif
; - - - - - - - - - - - - - - 
; API from bootloader
; - - - - - - - - - - - - - - 
  org APP_ENTRY_POINT
;org 0x000
  PAGESEL _app_main
  goto    _app_main

  org APP_CONFIG
  PAGESEL _app_config
  goto    _app_config ; must end with 'retlw' instruction

  org APP_INTERRUPT
;  org	0x004
  PAGESEL _app_interrupt
  call    _app_interrupt
  retfie

_app_config
	retlw	0x0A

_app_interrupt:
	BANKSEL	PIR2
	PAGESEL	ProcessUSBTransactions
	btfsc	PIR2,USBIF
	call	ProcessUSBTransactions
	PAGESEL $
	return

; -------------------------------------------------
; InitializeSystem(void)
;   System setup: Configure defaults for PIC SFRs
; -------------------------------------------------
InitializeSystem:
	BANKSEL	TRISC
	movlw	0x00    ; Set RC3&RC2 as output 
	movwf	TRISC
	movlw	0x28	; A5 & A3 as input
	movwf	TRISA

	BANKSEL LATA
	movlw	0x0C	; Clear Port C Latches
	movwf	LATC
	movlw	0x10	; Clear Port A Latches
	movwf	LATA

	BANKSEL	ANSELA  ; Bank 3
	clrf	ANSELA
	clrf	ANSELC
	
	BANKSEL	OSCTUNE  ; Bank 1
	clrf	OSCTUNE
	movlw	0xFC
	movwf	OSCCON ; 16MHz HFINTOSC with 3x PLL enabled (48MHz operation)
	movlw	0xC3
	movwf	OPTION_REG ; Set prescaler to 256

	BANKSEL ACTCON  ; Bank 7
	movlw	0x90	; Enable active clock tuning with USB
	movwf	ACTCON

	; Initialized Memory
	clrf	StatusBits
	clrf	outPtrL
	clrf	outPtrH
	clrf	inPtrL
	clrf	inPtrH
	clrf	DeviceState
	clrf	wCount
	
	BANKSEL CtrlTransferStage
	clrf	CtrlTransferStage
	clrf	DeviceAddress
	clrf	CurrentConfiguration
	clrf	HidIdleRate
	clrf	HidProtocol
	return

; --------------------------------------------
; EnableInterrupts(void):
;    Enable Interrupt handler for USB events
; --------------------------------------------
EnableInterrupts:

	BANKSEL	UIE ; BANK 29
	movlw	0x4B
	movwf	UIE ; Transaction complete, Start Of Frame, Error, Reset

	bsf	INTCON,PEIE ; Peripheral Interrupt enable
	bsf	INTCON,GIE  ; Global Interrupt Enable

	BANKSEL	PIE2 ; BANK 1
	bsf	PIE2,USBIE  ; Enable Usb Global Interrupt

	return 

; ------- WYSE Keyboard Driver Code --------

; --------------------------------------------
; HID Key Modifiers
;    Modifier Keys (First Byte in Keyboard Message) 
; --------------------------------------------
#define KEY_L_CTRL        (0)
#define KEY_L_SHIFT       (1)
#define KEY_L_ALT         (2)
#define KEY_L_WIN         (3)
#define KEY_R_CTRL        (4)
#define KEY_R_SHIFT       (5)
#define KEY_R_ALT         (6)
#define KEY_R_WIN         (7)

; ---------------------
; Wyse Defines 
; ---------------------
   ; Clock & Data PIN Definitions
   #define KBD_CLOCK LATC,3
   #define KBD_CLOCK_TRIS  TRISC,3

   #define KBD_DATA  PORTA,5
   #define KBD_DATA_TRIS TRISA,5


; ---------------------
; Wyse Global Variables
; ---------------------

; ------------------------------------------------------
; KEYMAP: Map keypresses to windows HID events. 
; Some keys can/should appear in the byte 0 bitmask
;    ALT 0xe6 can be in the BIT mask at start of packet
;    CTL 0xe0 "" ""
;    DO = Right Meta (0x80)
;  L&R Shift
; Other mappings: 
;    PRINT = SYSRQ (0x46)
; ------------------------------------------------------
; 80 byte keymap table
;KEYMAP:
;   dt 0x2b,0x08,0x1a,0x14,0x15,0x17,0x1c,0x00
;   dt 0x34,0x28,0x49,0x07,0x2c,0x16,0x00,0x0c
;   dt 0x25,0x26,0x27,0x2d,0x2e,0x00,0x04,0x2a
;   dt 0x19,0x05,0x11,0x10,0x00,0x1e,0x09,0x18
;   dt 0xe5,0x00,0x29,0x00,0x36,0x1f,0x0a,0x12
;   dt 0xe6,0xe5,0x00,0x48,0x37,0x20,0x0b,0x13
;   dt 0x80,0x00,0xe0,0x00,0x38,0x21,0x0d,0x2f
;   dt 0x00,0x51,0x50,0x39,0x1d,0x22,0x0e,0x30
;   dt 0x52,0x00,0x00,0x31,0x1b,0x23,0x0f,0x35
;   dt 0x4f,0x00,0x00,0x46,0x06,0x24,0x33,0x2a 

KEYMAP2
keymap_page_start equ	high KEYMAP2
	brw	; Add W to PC
   retlw 0x2b		;0
	retlw 0x08
	retlw 0x1a
	retlw 0x14
	retlw 0x15
	retlw 0x17
	retlw 0x1c
	retlw 0x00
   retlw 0x34		;8
	retlw 0x28
	retlw 0x49
	retlw 0x07
	retlw 0x2c
	retlw 0x16
	retlw 0x00
	retlw 0x0c
   retlw 0x25		;16
	retlw 0x26
	retlw 0x27
	retlw 0x2d
	retlw 0x2e
	retlw 0x00
	retlw 0x04
	retlw 0x2a
   retlw 0x19		;24
	retlw 0x05
	retlw 0x11
	retlw 0x10
	retlw 0x00
	retlw 0x1e
	retlw 0x09
	retlw 0x18
   retlw 0xe5		;32
	retlw 0x00
	retlw 0x29
	retlw 0x00
	retlw 0x36
	retlw 0x1f
	retlw 0x0a
	retlw 0x12
   retlw 0xe6		;40
	retlw 0xe5
	retlw 0x00
	retlw 0x48
	retlw 0x37
	retlw 0x20
	retlw 0x0b
	retlw 0x13
   retlw 0x80		;48
	retlw 0x00
	retlw 0xe0
	retlw 0x00
	retlw 0x38
	retlw 0x21
	retlw 0x0d
	retlw 0x2f
   retlw 0x00		;56
	retlw 0x51
	retlw 0x50
	retlw 0x39
	retlw 0x1d
	retlw 0x22
	retlw 0x0e
	retlw 0x30
   retlw 0x52		;64
	retlw 0x00
	retlw 0x00
	retlw 0x31
	retlw 0x1b
	retlw 0x23
	retlw 0x0f
	retlw 0x35
   retlw 0x4f		;72
	retlw 0x00
	retlw 0x00
	retlw 0x46
	retlw 0x06
	retlw 0x24
	retlw 0x33
	retlw 0x2a 	;80


; -------------------------------------------------------------
; memset : simple implementation to loop and initialize memory
;      
; --------------------------------------------------------------

defaultChangeBuffer
	movlw	0x00
	BANKSEL	gWyseChangeBuffer
	movwf	gWyseChangeBuffer
	movwf	gWyseChangeBuffer+1
	movwf	gWyseChangeBuffer+2
	movwf	gWyseChangeBuffer+3
	movwf	gWyseChangeBuffer+4
	movwf	gWyseChangeBuffer+5
	movwf	gWyseChangeBuffer+6
	movwf	gWyseChangeBuffer+7
	movwf	gWyseChangeBuffer+8
	movwf	gWyseChangeBuffer+9
	movwf	gWyseChangeBuffer+10
	movwf	gWyseChangeBuffer+11
	movwf	gWyseChangeBuffer+12
	movwf	gWyseChangeBuffer+13
	movwf	gWyseChangeBuffer+14
	movwf	gWyseChangeBuffer+15
	movwf	gWyseChangeBuffer+16
	movwf	gWyseChangeBuffer+17
	movwf	gWyseChangeBuffer+18
	movwf	gWyseChangeBuffer+19
	return

clearReadBuffer
	movlw	0x00
	BANKSEL gWyseReadBuffer
	movwf	gWyseReadBuffer
	movwf	gWyseReadBuffer+1
	movwf	gWyseReadBuffer+2
	movwf	gWyseReadBuffer+3
	movwf	gWyseReadBuffer+4
	movwf	gWyseReadBuffer+5
	movwf	gWyseReadBuffer+6
	movwf	gWyseReadBuffer+7
	movwf	gWyseReadBuffer+8
	movwf	gWyseReadBuffer+9
	movwf	gWyseReadBuffer+10
	movwf	gWyseReadBuffer+11
	movwf	gWyseReadBuffer+12
	movwf	gWyseReadBuffer+13
	movwf	gWyseReadBuffer+14
	movwf	gWyseReadBuffer+15
	movwf	gWyseReadBuffer+16
	movwf	gWyseReadBuffer+17
	movwf	gWyseReadBuffer+18
	movwf	gWyseReadBuffer+19
	return


; -------------------------------------
; FindBit : gTestByte
; On return W includes the bit index
; Bit has been cleared from gTestByte
; -------------------------------------

FindBit
	btfss	gTestByte,0
	goto	_fb_one
	bcf		gTestByte,0
	retlw	0
_fb_one
	btfss	gTestByte,1
	goto	_fb_two
	bcf		gTestByte,1
	retlw	1
_fb_two
	btfss	gTestByte,2
	goto	_fb_three
	bcf		gTestByte,2
	retlw	2
_fb_three
	btfss	gTestByte,3
	goto	_fb_four
	bcf		gTestByte,3
	retlw	3
_fb_four	
	btfss	gTestByte,4
	goto	_fb_five
	bcf		gTestByte,4
	retlw	4
_fb_five
	btfss	gTestByte,5
	goto	_fb_six
	bcf		gTestByte,5
	retlw	5
_fb_six
	btfss	gTestByte,6
	goto	_fb_seven
	bcf		gTestByte,6
	retlw	6
_fb_seven	; only bit left
	bcf		gTestByte,7
	retlw	7
	

; --------------------------------------------------------------
; PrepareTxBuffer(void)
;     Report any pending kepressed ('n' at a time) to the HOST. 
;     Note some keys are represented in the bitmask of byte 0.
;     Once all possible keys have been considered, reset the  gDataPending
;     to false, allowing the scanning loop to rescan for more keypresses.
; ---------------------------------------------------------------
PrepareTxBuffer:
	logch	'P',0
	BANKSEL HIDTxBuffer
	clrf	HIDTxBuffer      ; Modifier Key Bits (we are not using any)
	clrf	HIDTxBuffer+1    ; Second Byte always 0 (Padding Byte)
	clrf	HIDTxBuffer+2    ; Second Byte always 0 (Padding Byte)
		; Overhead to calculate contents outweighs "just zero it"
	clrf	HIDTxBuffer+3
	clrf	HIDTxBuffer+4
	clrf	HIDTxBuffer+5
	clrf	HIDTxBuffer+6
	clrf	HIDTxBuffer+7

; uint8_t i=2;
	; Point FSR0 at HIDTxBuffer (buffer to be populated)
	movlw	high HIDTxBuffer
	movwf	FSR0H
	movlw	low HIDTxBuffer+2
	movwf	FSR0L

	; Point FSR1 at the Change Buffer (looking at each byte in sequence)
	movlw	high	gWyseChangeBuffer
	movwf	FSR1H
	movlw	low		gWyseChangeBuffer
	addwf	gScanIndex,W ; Offset to the byte we were looking at previously
	movwf	FSR1L	   ; Pointer to byte under review

	; Nothing pending.. so readin the next
ptb_check_byte
	moviw	0[FSR1]		;Load W with the byte under review
	movwf	gTestByte	; assign to gTestByte

ptb_nextbit	; Reload from the (modified) byte under test
	movf	gTestByte,W
	btfsc	STATUS,Z	; Change is empty.. 
	goto	ptb_nextbyte	; Move on to next byte

	PAGESEL	FindBit		; Find bit returns ZERO or bit index
	call	FindBit
	PAGESEL $
	; On return W contains the offset index 
	addwf	gScanBitIndex,W ; gScanBitIndex is always a factor of 8
	movwf	gCalcKeyIndex   ; Contains the calculated index

	movf	gTestByte,W     ; Write the modified bitmap to the buffer
	movwi	0[FSR1]
	goto	ptb_handle_key

ptb_nextbyte:
	incf	FSR1,F	; Point at the next byte to inspect
	movlw	8		; Increment the gScanBitIndex by 8
	addwf	gScanBitIndex,f ; 
	incf	gScanIndex,f

	; Exit condition: are we at the end of the HID message?
	movlw	HIDTxBuffer+HID_REPORT_BYTE_COUNT
	xorwf	FSR0L,W
	btfsc	STATUS,Z
	goto	ptb_eop ; End of Packet -jump down and check if we are done
	movf	gScanIndex,W
	xorlw	0x0B	; KEYMAP maxes out at 80.. so we can stop here.
	btfss	STATUS,Z
	goto	ptb_check_byte
	goto    ptb_eop 
	
   ;while (i<HID_REPORT_BYTE_COUNT) {
        ;uint8_t bstatus = (gWyseChangeBuffer[gKeyIndex>>3]>>(gKeyIndex & 7))&1;
        ;// While I haven't found a bit that matches.. increment gKeyIndex

        ;while((bstatus) && (gKeyIndex<sizeof(KEYMAP))) {
            ;gKeyIndex++;
            ;bstatus = (gWyseChangeBuffer[gKeyIndex>>3]>>(gKeyIndex & 7))&1;
        ;}
        ;// Either I have a match, or overflow (gKeyIndex=BUFFER_BITS-16)
        ;if (!bstatus) {
             ;// Lookup this gKeyIndex in our character map
             ;switch(gKeyIndex) {
ptb_handle_key
check_32:
        ;case 32: HIDTxBuffer[0] |= KEY_R_SHIFT; break;
	movf	gCalcKeyIndex,W
	xorlw	0x20
	btfss	STATUS,Z
	goto	check_40
	bsf		HIDTxBuffer,KEY_R_SHIFT
	goto 	ptb_nextbit

check_40:
        ;case 40: HIDTxBuffer[0] |= KEY_R_ALT; break;
	movf	gCalcKeyIndex,W
	xorlw	40
	btfss	STATUS,Z
	goto	check_41
	bsf		HIDTxBuffer,KEY_R_ALT
	goto	ptb_nextbit

check_41:
        ;case 41: HIDTxBuffer[0] |= KEY_L_SHIFT; ;break;
	movf	gCalcKeyIndex,W
	xorlw	41
	btfss	STATUS,Z
	goto	check_48
	bsf		HIDTxBuffer,KEY_L_SHIFT
	goto	ptb_nextbit

check_48:
        ;case 48: HIDTxBuffer[0] |= KEY_R_WIN; break;
	movf	gCalcKeyIndex,W
	xorlw	48
	btfss	STATUS,Z
	goto	check_50
	bsf		HIDTxBuffer,KEY_R_WIN
	goto	ptb_nextbit

check_50:
        ;case 50: HIDTxBuffer[0] |= KEY_L_CTRL; break;
	movf	gCalcKeyIndex,W
	xorlw	48
	btfss	STATUS,Z
	goto	handle_default
	bsf		HIDTxBuffer,KEY_L_CTRL
	goto	ptb_nextbit

handle_default
        ; WREG = KEYMAP[gCalcKeyIndex];
	movf	gCalcKeyIndex,W
	pagesel	KEYMAP2
	call 	KEYMAP2
	pagesel	$

	; W contains the keypress
	movwi	FSR0++ ; Put byte into the HIDTxBuffer, and increment to point to next char
	
	; Need to test if FSR0 is now at the end of buffer
	movlw	HIDTxBuffer+HID_REPORT_BYTE_COUNT
	xorwf	FSR0L,W
	btfsc	STATUS,Z
	goto	ptb_eop ; Buffer full - exit loop
	goto	ptb_nextbit ; Not end of message yet

	; HIDTxBuff is full 
	; Drop through to check of gScanBitIndex

ptb_eop ; end of packet 
	; no special handling.. already zero's out the packet
	movf	gScanBitIndex,W
	xorlw	88
	btfss	STATUS,Z
	goto	ptb_exit
	;
	; Reset the buffers.. (copy READY to Changed)
	; Scan index should alreayd be 0x0A
	; Copy from Read to Changed
	movlw	low	gWyseReadBuffer+0x0A
	movwf	FSR0L
	movlw	high gWyseReadBuffer
	movwf	FSR0H
ptb_done_loop
	moviw	FSR0--
	movwi	--FSR1
	decfsz	gScanIndex,F
	goto	ptb_done_loop

	bcf		StatusBits,gDataPending

ptb_exit
	return

; -------------------------------------------------------
; ProcessIncommingData
;    Parse the LED status from the incomming HID control 
;    bitmask. 
; -------------------------------------------------------
ProcessIncommingData:
	BANKSEL HIDRxBuffer
	movf	HIDRxBuffer,W

    ; Windows Will send only a single Byte
    ; with statuses of leds
    ; first bit for num lock, second for caps etc..
	BANKSEL LATA
      	;LED_NUMLOCK   = (HIDRxBuffer[0] & 0x01)==0;
	bcf		LED_NUMLOCK
	btfss	WREG,0
	bsf		LED_NUMLOCK
        ; LED_CAPSLOCK  = (HIDRxBuffer[0] & 0x02)==0;
	bcf		LED_CAPSLOCK
	btfss	WREG,1
	bsf		LED_CAPSLOCK
        ; LED_SCROLLOCK = (HIDRxBuffer[0] & 0x04)==0;
	bcf		LED_SCROLLOCK
	btfss	WREG,3
	bsf		LED_SCROLLOCK

        ; gCapsLock = (HIDRxBuffer[0] & 0x02)>0;
	bcf 	StatusBits,gCapsLock
	btfsc	WREG,1
	bsf		StatusBits,gCapsLock

	return


; -----------------------------------------------------
; CheckUsb
;    Poll the USB subsystem to determine if we have received
;    an incoming control request. 
; -----------------------------------------------------
CheckUsb:
    ; if(IsUsbDataAvailable() > 0 )
	PAGESEL IsUsbDataAvailable
	call	IsUsbDataAvailable
	PAGESEL	$
	btfsc	STATUS,Z
	return
    ; {
    ;     ProcessIncommingData();
    ;     ReArmInterface();
    ;  }
	PAGESEL ProcessIncommingData
	call	ProcessIncommingData
	PAGESEL	ReArmInterface
	call	ReArmInterface
	PAGESEL	$
	return

; ----------------- WYSE KEYBOARD DRIVER ----------------
PulseClock:
	BANKSEL LATA
	bcf   KBD_CLOCK
	nop
	nop
	nop
	nop  ;; Arbitrary number .. can improve this with testing
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	bsf KBD_CLOCK
	return

ReadWyseKeyboard:
	logch	'R',0
	; Might interfere with USB .. but lets try..
	bcf	INTCON, GIE
	; Use FSR0 to point to the gWyseBuffer
	movlw	low gWyseReadBuffer
	movwf	FSR0L
	movlw	high gWyseReadBuffer
	movwf	FSR0H

	movlw	BUFFER_SIZE-1
	movwf	gWyseTemp     ; Number of bytes to review

_ReadByteLoop
	clrf	gPollBuffer
	; Starting bit to check 
	movlw	1
	bcf		STATUS,C
_ReadBitLoop
    ;for ( uint8_t i = 0; i < 19*8/*BUFFER_BITS*/; ++i ) {
    ; Disable interrupts while clocking
;	bcf	INTCON,GIE  ; Global Interrupt Disable
   ; clear CLOCK
	pagesel	PulseClock
	call	PulseClock
	pagesel $

        ;INTCONbits.GIE = 1;     // Global Interrupt Enable
;	bsf	INTCON,GIE
	  ;// Store the bit in the gWyseReadBuffer 
	  ;gWyseReadBuffer[i >> 3] |= (v << (i & 7));
	BANKSEL PORTA
	btfss	KBD_DATA
	iorwf	gPollBuffer,f
	rlf		WREG,W
	btfss	STATUS,C
	goto	_ReadBitLoop

	; End of the byte.. store it
	movf	gPollBuffer,W
	movwi	FSR0++
	decfsz	gWyseTemp,f  
	goto	_ReadByteLoop

    ;// CapsLock state - if enabled, add an additional CLOCK pulse
    ;if (gCapsLock) {
	btfss	StatusBits,gCapsLock
	goto	RWK_exit
	;INTCONbits.GIE = 0;     // Global Interrupt Disable
;	bcf	INTCON,GIE
	pagesel	PulseClock
	call	PulseClock
	pagesel $

        ;INTCONbits.GIE = 1;     // Global Interrupt Enable
	bsf	INTCON,GIE
	
RWK_exit
	bcf		KBD_CLOCK
	bsf		INTCON,GIE
	return

InitializeWyse:
	BANKSEL TRISA
	
   	; KBD_Clock as output
   	bcf KBD_CLOCK_TRIS
   	; KBD_DATA as input
   	bsf KBD_DATA_TRIS

   	; Default global variables & buffers
	bcf	StatusBits,gDataPending
	bcf	StatusBits,gCapsLock

	pagesel	defaultChangeBuffer
	call	defaultChangeBuffer
	pagesel	$
	
	return
; ------------------- End of WYSE Keyboard Code ---------------

; ------------------
; ProcessIO(void)
;    Check if the USB is ready for us to send, and check if 
;    there is data *to* send. If yes to both, prepare a payload
;    and queue it for transmission.
; ------------------
ProcessIO:
    	;  Check USB for incomming Commands
	;  if ((DeviceState == 0x05) && (UCONbits.SUSPND==0))
	BANKSEL DeviceState
	movf	DeviceState,W
	xorlw	0x05
	btfss	STATUS,Z
	goto	PIO_exit
	BANKSEL	UCON
	btfsc	UCON,SUSPND
	goto	PIO_exit
	PAGESEL	CheckUsb
	call	CheckUsb
	PAGESEL	$

PIO_check_keyboard 	; Check Status Of the Keyboard 
	btfss	StatusBits,gDataPending
	goto	PIO_exit

        ; If Data Pending then send some of it to HOST
	PAGESEL	PrepareTxBuffer
	call	PrepareTxBuffer
	logch	'T',0
	PAGESEL HIDSend
	call	HIDSend
	PAGESEL	$

PIO_exit
	return


; ---------------------------------------
; MAIN: (main or booloader app_main)
;    Loop forever, polling for USB events 
;    and/or keypresses.
; ---------------------------------------
	nop ; Investiage not jumping to correct address
_app_main:

	; INIT CODE
	PAGESEL	InitializeWyse
	call	InitializeWyse
	
	PAGESEL	InitializeSystem
	call	InitializeSystem

 if LOGGING_ENABLED
	PAGESEL	uart_init
	call	uart_init
	PAGESEL	log_init
	call	log_init
	logch	'^', LOG_NEWLINE
	PAGESEL	log_service
	call	log_service
	PAGESEL	$
 endif
	
	PAGESEL	InitializeUSB
	call	InitializeUSB

	PAGESEL	EnableUSBModule
	call	EnableUSBModule

	PAGESEL EnableInterrupts
	call	EnableInterrupts

    ; Setup an approximate 5ms timer (frequency of polling)
	BANKSel	T1CON
	movlw	0x05
	movwf	T1CON
	clrf	TMR1H
	clrf	TMR1L

_main_loop:
	PAGESEL	ProcessIO
	call	ProcessIO

 if LOGGING_ENABLED
	PAGESEL	log_service
	call	log_service
	PAGESEL	$
 endif
	
       ;if (gDataPending == 0) { // Idle: Poll for keystrokes
	btfsc	StatusBits,gDataPending
	goto	_main_loop
	
	clrf	gScanIndex
	clrf	gScanBitIndex
	PAGESEL	clearReadBuffer
	call	clearReadBuffer
	PAGESEL	$

	PAGESEL	ProcessIO
	call	ProcessIO

	PAGESEL	ReadWyseKeyboard
	call	ReadWyseKeyboard
	loghex	10,LOG_SPACE
	logf	gWyseReadBuffer
	logf	gWyseReadBuffer+1
	logf	gWyseReadBuffer+2
	logf	gWyseReadBuffer+3
	logf	gWyseReadBuffer+4
	logf	gWyseReadBuffer+5
	logf	gWyseReadBuffer+6
	logf	gWyseReadBuffer+7
	logf	gWyseReadBuffer+8
	logf	gWyseReadBuffer+9
	logch	'.',LOG_NEWLINE
	logch	'C',0
	loghex	10,LOG_SPACE
	logf	gWyseChangeBuffer
	logf	gWyseChangeBuffer+1
	logf	gWyseChangeBuffer+2
	logf	gWyseChangeBuffer+3
	logf	gWyseChangeBuffer+4
	logf	gWyseChangeBuffer+5
	logf	gWyseChangeBuffer+6
	logf	gWyseChangeBuffer+7
	logf	gWyseChangeBuffer+8
	logf	gWyseChangeBuffer+9
	logch	'.',LOG_NEWLINE

	PAGESEL	ProcessIO
	call	ProcessIO
 
;          // Check if data has gDataPending 
;          for (uint8_t i=0;i< BUFFER_SIZE-2;i++) {
;             if (gWyseReadBuffer[i] != gWyseChangeBuffer[i]) {
;               gDataPending=1;
;               gWyseChangeBuffer[i] = gWyseReadBuffer[i];
	movlw	0x0A
	movwf	gWyseTemp    ; Number of bytes to review

	movlw	low gWyseReadBuffer
	movwf	FSR0L
	movlw	high gWyseReadBuffer
	movwf	FSR0H

	movlw	low gWyseChangeBuffer
	movwf	FSR1L
	movlw	high gWyseChangeBuffer
	movwf	FSR1H
	
_main_test_loop
	moviw	0[FSR0]		; gWyseReadBuffer[n] - temp copy to test against
	xorwf	INDF1,W   	; XOR with gWyseChangedBuffer
	btfss	STATUS,Z	; If they match don't set dirty bit
	bsf		StatusBits,gDataPending
	moviw	FSR0++
	movwi	FSR1++		; FSR1 - now matches the READ state
	decfsz	gWyseTemp,f
	goto	_main_test_loop

_main_delay
      ; 5ms delay before next poll for keystrokes 
      ; ** This delay encompasses the minimum 34 us delay required 
      ;    to reset the keyboard shift register
	BANKSEL TMR1H
	clrf	TMR1H
	clrf	TMR1L
	bcf		PIR1,TMR1IF

; --- pulling this out
	movlw	1
	movwf	gWyseTemp       ; can adjust the delay by increasing number of ticks to track
 ;     while (tick_count >0) {
_main_spin
	BANKSEL	PIR1
	btfss	PIR1,TMR1IF
	goto	_main_spin_action

	bcf		PIR1,TMR1IF
	decfsz	gWyseTemp,f
	goto	_main_spin
	goto	_main_loop
_main_spin_action
        ; While spinning: process any pending keystrokes
	PAGESEL	ProcessIO
	call	ProcessIO
 if LOGGING_ENABLED
	PAGESEL	log_service
	call	log_service
	PAGESEL	$
 endif

	goto	_main_spin

	end
