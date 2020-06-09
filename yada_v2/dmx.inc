; asmsyntax=pic
;********************************************************************************************* 
; Software License Agreement                                                                 * 
; The software supplied herewith by Microchip Technology Incorporated                        * 
; (the "Company") is intended and supplied to you, the Company's                             * 
; customer, for use solely and exclusively on Microchip products.                            * 
;                                                                                            * 
; The software is owned by the Company and/or its supplier, and is                           * 
; protected under applicable copyright laws. All rights are reserved.                        * 
; Any use in violation of the foregoing restrictions may subject the                         * 
; user to criminal sanctions under applicable laws, as well as to                            * 
; civil liability for the breach of the terms and conditions of this                         * 
; license.                                                                                   * 
;                                                                                            * 
; THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,                          * 
; WHETHER EXPRESS, IMPLIED OR STATU-TORY, INCLUDING, BUT NOT LIMITED                         * 
; TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                                * 
; PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,                          * 
; IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR                                 * 
; CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.                                          * 
;                                                                                            * 
;********************************************************************************************* 
; File:  DMX512TrmtDemo.asm 
; DMX512 Transmitter demo 
; 
; This source code uses the PIC18F2420 to transmit a DMX-512 packet via 
; the EUSART peripheral.  An external 16MHz clock input is used. 
; The DMX transmitter code is written as a polled state machine with 
; 4 states.  The state machine is called periodically from the main 
; software loop and a jump table determines the present state. 
; Timer0 is used to control the state machine timing, including length 
; of the break signal and the spacing between transmitted bytes. 
; written to the first data slot in the DMX frame to control a remote 
; device. 

; Exported interfaces are...
;    DmxIrqHandler
;    DmxTransmit
;    DmxSetup
; 
; The shared memory segment for the DMX Universe
;    DmxUniverse 
;
;

#ifdef STANDALONE
	radix dec
	list n=0,st=off
	include "p16f1455.inc"		; include processor specific definition  
        nolist
	include "macros.inc"
	list
        errorlevel -302
#endif



; Timing constants (assuming 48MHz clock input and assigned prescaler  
; values to produce 1us tick) Uses 1:8 pre-scalar
#define T0CON_PRESCALAR 0x82
#define T100US	.106        ; preload value for TMR0 to roll over in 100us 
#define T30US 	.211        ; 30us value

#define DMX_BREAK_PIN 	LATA,5
#define DMX_BREAK_DIR	TRISA,5
#define DMX_LED_MASK    0x04

; Variables memory allocation  
;;dmxdata UDATA
 CBLOCK 0x70 ; Global Variables
        DmxTxState		; State Variable
        CountH
	CountL			; 16-bit counter
	FSR0_DMXL		; Save Context outside of IRQ Handler
	FSR0_DMXH		;
	DMX_BLINK		; 44 packets per second
	DMX_SOF_BYTE		;
	YADA_STATUS		; 0 = DMX, 1 = PASSTHROUGH, 2 = ??
	end_bank0_minor_vars:0
 ENDC
 if end_bank0_minor_vars > 0x80
	error "Bank 0 Global  Variable Space overrun"
 endif

;	YADA_STATUS FLAGS
;
;	Bit 0 - 0x01 -   Mode  0 = DMX  1 = PASSTHROUGH
;       Bit 7 - 0x80 -   Buffer Mode 0 = available 1 = Busy
;
YADA_MODE_BIT	equ	(0)
YADA_BUSY_BIT	equ	(7)

;DmxUniverse equ 0x2200 ; 512 bytes of DMX Universe Values

;**************************************************************************************** 
;DMX Transmit state machine  
 
DmxTransmit: 
	; The DMX transmit code is driven by the TMR0 roll-over 
	; events.  Just return if a roll-over has not occured. 
	btfss	INTCON,TMR0IF	; wait until TIMER0 roll-over 
	return

	bcf	INTCON,TMR0IF	; clear the flag
	movlw 	high(JumpTable)
	movwf 	PCLATH
	movf	DmxTxState,W
	andlw	0x07		; reduce offset to valid range (0-14) 
	addwf	PCL,F		; computed jump  
 
JumpTable:
	bra	SENDMBB		; 0 IDLE period after each complete frame 
	bra	SENDDATA	; 1 send one byte of data 
	bra	SENDMAB	       	; 2 IDLE period between BREAK and START slot 
	bra	SENDBREAK	; 3 BREAK synchronization signal 
	reset                 	; not used 
	reset     		; not used 
	reset                  	; not used 
	reset			; not used 
	 
; DmxTxState = 3. Generates a Break Signal (100uSec) 
SENDBREAK
	BANKSEL TRISA ; BANK 1
	bsf     DMX_BREAK_DIR  	; tri-state the BREAK to end the break signal

	BANKSEL TMR0  ; BANK 0
	movlw	T30US          	; start timer for MARK AFTER BREAK
	movwf	TMR0

	decf	DmxTxState,F	; proceed to State2 SENDMAB 

 	; Check for BLINK routine
	decfsz  DMX_BLINK,F
	return

	movlw	44		; Every 44 packets.. should toggle 1HZ (ish)
	movwf	DMX_BLINK

	clrf	DMX_SOF_BYTE	; Default SOF byte
	clrf	YADA_STATUS	; Default SOF byte

	movlw	DMX_LED_MASK
	BANKSEL LATC 		; BANK 2
	xorwf	LATC,F			; Turn on DMX LED,F

	return 
 
; DmxTxState = 2. Mark After Break (line IDLE for 100uSec) send a start code 
SENDMAB
	clrf	CountL			; init 16-bit counter		 
	clrf	CountH 
	movlw	low DmxUniverse	; init pointer to transmit buffer
	movwf	FSR0_DMXL
	movlw	high DmxUniverse
	movwf	FSR0_DMXH	
	decf	DmxTxState,f	; proceed to State1  SENDDATA

	movf	DMX_SOF_BYTE,W
	BANKSEL TXREG ; BANK 3
	movwf   TXREG          		; send NULL START CODE 

	BANKSEL PIE1  ; BANK 1
	bsf	PIE1,TXIE		; Enable UART TX interrupt


	return 
 
; DmxTxState = 1. wait for UART to complete transmission of current byte and an 
; additional short amount of time 
SENDDATA 
	return 				; Do nothing, the IRQ handler will push state transition
 
;DmxTxState = 0. sends Mark Before repeating the frame transmission 
SENDMBB
	BANKSEL TMR0  ; BANK 0
	movlw	T100US			; pre-load the timer for 100us BREAK 
	movwf	TMR0			 
	bcf	INTCON,TMR0IF		; clear the flag 

	BANKSEL TRISA ; BANK 1
	bcf     DMX_BREAK_DIR		; make RB0 an output

	BANKSEL LATA  ; BANK 2
	bcf     DMX_BREAK_PIN		; pull pin RB0 low to force a break condition
	movlw	.3			; proceed to State3 SENDBREAK 
	movwf	DmxTxState

	return 
 
 ;**************************************************************************************** 
DmxIrqHandler
	; Is the payload complete?
	btfsc	CountH,1
	bra	DIH_TxDone
	
	; Increase the count - we test for range on next TX irq
	incf	CountL,F
	btfsc	STATUS,Z  ; When ZERO increment the H order byte
	incf	CountH,F

	movf	FSR0_DMXL,W 	; Load the indirect access pointer(FSR0)
	movwf	FSR0L
	movf	FSR0_DMXH,W
	movwf	FSR0H

	moviw	FSR0++		; Copy the data, and increment the FSR0 index
	BANKSEL TXREG ;  BANK 3
	movwf	TXREG		; Write the value to the UART engine

	movf	FSR0L,W		; Save the modified FSR0 index, for the next 
	movwf	FSR0_DMXL	; time in this IRQ/LOOP
	movf	FSR0H,W
	movwf	FSR0_DMXH
	; return from IRQ
	return

DIH_TxDone
	BANKSEL PIE1 ; BANK 1
	bcf	PIE1, TXIE 		; Disable the interrupts

	BANKSEL TMR0 ; BANK 0 		; Start the MARK B4 BREAK timer
	movlw	T100US
	movwf	TMR0

	clrf	DmxTxState		; proceed to State0 - SENDMBB
	bcf 	INTCON,TMR0IF		; Clear any pending TMR0 event
	; return from IRQ
	return

;**************************************************************************************** 
;Setup Serial port  
 
SetupSerial 
	BANKSEL TRISC ; BANK 1
	bsf	TRISC,5		; allow the UART RX to control pin RC5  
	bsf	TRISC,4		; allow the UART TX to control pin RC4  

	BANKSEL TXSTA ; BANK 3
	movlw	0x65		; enable TX, 9-bit mode, high speed mode, 9th bit =1 (2 stop) 
	movwf	TXSTA			  
 
	movlw	0x80		; enable serial port, disable receiver 
	movwf	RCSTA			 
 
	bsf	BAUDCON,BRG16	; select EUART 16-bit Asynchronous mode operation 
 	bsf	BAUDCON,BRGH	;
	movlw	.47		; init baud rate generator for 250k baud (assume Fosc=48MHz)
	movwf	SPBRGL		 
	
	; Setup for BLINKING at 1HZ (44 BREAKS per second) 
	movlw	44
	movwf	DMX_BLINK
	return 
	 

;**************************************************************************************** 
DmxSetup 
	call	SetupSerial	; initialize serial comm  
	call	InitTX		; initialize the I/O ports and TMR0 
 	return

;**************************************************************************************** 
;InitTX		init Timer0, clear TXbuffer, init state machine 
 
InitTX 
	clrf	CountL		; init 16-bit counter 
	clrf	CountH 
	 
; clear Transmit buffer
	movlw	low DmxUniverse	; use IND0 pointer to address the RAM buffer 
	movwf	FSR0L
	movwf	FSR0_DMXL	; store copy of the pointer for when we need to SEND the payload
	movlw	high DmxUniverse
	movwf	FSR0H
	movwf	FSR0_DMXH	

	movlw	0x30		; DEBUG value.. set at 0x00 for initialization
CBloop 
	movwi	FSR0++		; clear the location pointed to by IND0 then increment pointer 

	incfsz	CountL,F	; increment 16-bit counter 
	bra	CBloop
	incf	CountH,F 
	btfss	CountH,1	; check if counter >= 512 
	bra	CBloop 

; init Timer0
	BANKSEL OPTION_REG ; BANK 1
	movlw	T0CON_PRESCALAR ; enable Timer0, as an 8-bit timer, use prescaler 1:2 
	movwf	OPTION_REG	; (0.4us tick@20MHz) 

	BANKSEL TRISA
	bcf	DMX_BREAK_DIR	; make pin RB0 an output

	BANKSEL LATA ; BANK 2
	bcf	DMX_BREAK_PIN	; pull RB0 low to force a break condition

	BANKSEL TMR0  ; BANK 0
	movlw	T100US		; preload timer for 100us interval to roll over 
	movwf	TMR0 
	bcf	INTCON,TMR0IF	; clear roll over flag 
 
; init state machine 	 
	movlw	.03		; Start with BREAK state 
	movwf	DmxTxState		 
	return 
