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

movff	macro x,y 
	movf	x,f
	movwf	y
	endm	


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
#define T0CON_PRESCALAR 0xC2
#define T100US	.106        ; preload value for TMR0 to roll over in 100us 
#define T30US 	.211        ; 30us value

#define DMX_BREAK_PIN 	LATA,5
#define DMX_BREAK_DIR	TRISA,5
#define DMX_LED 	LATC,2


; Variables memory allocation  
;;dmxdata UDATA
 CBLOCK 0x70 ; Global Variables
        DmxTxState		; State Variable
        CountH
	CountL			; 16-bit counter
	FSR0_DMXL		; Save Context outside of IRQ Handler
	FSR0_DMXH		;
	end_bank0_minor_vars:0
 ENDC
 if end_bank0_minor_vars > 0x80
	error "Bank 0 Global  Variable Space overrun"
 endif

;DmxUniverse equ 0x2200 ; 512 bytes of DMX Universe Values


;**************************************************************************************** 
DmxSetup: 
	call	SetupSerial		    ; initialize serial comm  
	call	InitTX			    ; initialize the I/O ports and TMR0 
 	return
;**************************************************************************************** 
;DMX Transmit state machine  
 
DmxTransmit: 
	; The DMX transmit code is driven by the TMR0 roll-over 
	; events.  Just return if a roll-over has not occured. 
	btfss	INTCON,TMR0IF	; wait until TIMER0 roll-over 
	return
	bcf	INTCON,TMR0IF	; clear the flag
	movf	DmxTxState,W
	movlw 	high(JumpTable)
	movwf 	PCLATH
	rlf	DmxTxState,W	; state x2 to account for PC byte  
				; addressing 
	andlw	0x0E		; reduce offset to valid range (0-14) 
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
	return 
 
; DmxTxState = 2. Mark After Break (line IDLE for 100uSec) send a start code 
SENDMAB
	clrf	CountL			; init 16-bit counter		 
	clrf	CountH 
	movlw	low DmxUniverse	; init pointer to transmit buffer
	movwf	FSR0L
	movwf	FSR0_DMXL
	movlw	high DmxUniverse
	movwf	FSR0H
	movwf	FSR0_DMXH	
	bcf	INTCON, TMR0IE 		; disable the timer (for now)
	BANKSEL TXREG ; BANK 3
	clrf    TXREG          		; send NULL START CODE 
	BANKSEL PIE1  ; BANK 1
	bsf	PIE1,TXIE		; Enable UART TX interrupt
	clrf    DmxTxState
	return 
 
; DmxTxState = 1. wait for UART to complete transmission of current byte and an 
; additional short amount of time 
SENDDATA 
	return 
 
;DmxTxState = 0. sends Mark Before repeating the frame transmission 
SENDMBB
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
	btfsc	STATUS,C
	incf	CountH,F
	movff   FSR0_DMXL, FSR0L
	movff   FSR0_DMXH, FSR0H
	moviw	FSR0++
	BANKSEL TXREG ;  BANK 3
	movwf	TXREG
	movff   FSR0L, FSR0_DMXL
	movff   FSR0H, FSR0_DMXH
	BANKSEL LATA ; BANK 2
	bcf	DMX_LED			; Turn on DMX LED
	; return from IRQ
	return

DIH_TxDone
	BANKSEL PIE1 ; BANK 1
	bcf	PIE1, TXIE 		; Disable the interrupts
	BANKSEL TMR0 ; BANK 0
	movlw	T100US
	movwf	TMR0
	bsf	INTCON, TMR0IE
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
	 
	return 
	 
;**************************************************************************************** 
;InitTX		init Timer0, clear TXbuffer, init state machine 
 
InitTX 
	clrf	CountL		; init 16-bit counter 
	clrf	CountH 
	 
; clear Transmit buffer
	movlw	low DmxUniverse	; use IND0 pointer to address the RAM buffer 
	movwf	FSR0L
	movwf	FSR0_DMXL
	movlw	high DmxUniverse
	movwf	FSR0H
	movwf	FSR0_DMXH	
	movlw	0x00
CBloop 
	movwi	FSR0++		; clear the location pointed to by IND0 then increment pointer 
	incf	CountL,F	; increment 16-bit counter 
	btfss	STATUS,C 
	bra	CBloop
	incf	CountH,F 
	btfss	CountH,1	; check if counter >= 512 
	bra	CBloop 

; init Timer0
	BANKSEL OPTION_REG ; BANK 1
	movlw	T0CON_PRESCALAR ; enable Timer0, as an 8-bit timer, use prescaler 1:2 
	movwf	OPTION_REG	; (0.4us tick@20MHz) 
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
