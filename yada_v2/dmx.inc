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
;
; ------------------------------------------------------------------------------------------
; Updates by LabRat (Andrew Williams) 2020
;   a. Modified for use on the P16F1455
;   b. Modified to allow for PASS THROUGH mode (in progress)
;   c. Modified to use an Irq Handler to deal with large block of data (DMA ish)
;      Fire & Forget from the applications point of view.
;   d. Add RxIrq handling
;
; Exported interfaces are...
;    DmxIrqHandler - invoked via 'call' from the application IrqHandler
;    DmxTransmit
;    DmxSetup
;
; The shared memory segment for the DMX Universe
;    DmxUniverse
;
;    ***Note: DmxUniverse can be an imported address.
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


;; ------------------------  TX State Machine Constants  -------------------------
; Timing constants (assuming 48MHz clock input and assigned prescaler
; values to produce 1us tick) Uses 1:8 pre-scalar
#define T0CON_PRESCALAR 0x82
#define T100US	.106        ; preload value for TMR0 to roll over in 100us
#define T30US 	.211        ; 30us value

#define DMX_BREAK_PIN 	LATA,5
#define DMX_BREAK_DIR	TRISA,5

;; ------------------------  RX State Machine Constants  -------------------------
#define RX_STATE_IDLE 		0x00
#define RX_STATE_W4BREAK	0x01
#define RX_STATE_W4START	0x02
#define RX_STATE_READ		0x03

; Variables memory allocation
;;dmxdata UDATA
 CBLOCK 0x70 ; Global Variables
  ;; TX State Machine Variables
        DmxTxState		; 0x70 Tx State Machine
        CountH                  ; 0x71
	CountL			; 0x72 16-bit counter
	FSR0_DMXL		; 0x73 Save Context outside of IRQ Handler
	FSR0_DMXH		; 0x74
	DMX_SOF_BYTE		; 0x75
	DMX_BLINK		; 0x76 Count 44 packets per second (1Hz blink)
	USB_BLINK		; 0x77 Count 44 frame '0' per second (1Hz blink)
	PWR_BLINK		; 0x78 BLINK power led at 1 HZ (APP vs Bootloader behaviour)

  ;; Shared between Rx & Tx state
	TEMP			; 0x79 Used for copy loop
	YADA_STATUS		; 0x7A Bit Field of runtime status options

  ;; Rx State Machine Variables
	DmxRxState		; 0x7B Rx State Machine
	FSR0_RXL		; 0x7C Save Rx pointer outside of IRQ Handler
	FSR0_RXH		; 0x7D
	DmxReadByte		; 0x7E

	end_bank0_minor_vars:0
 ENDC
 if end_bank0_minor_vars > 0x80
	error "Bank 0 Global  Variable Space overrun"
 endif

;DmxUniverse equ 0x21F0 ; 512 bytes of DMX Universe Values

;****************************************************************************************
;DMX Transmit state machine

DmxTransmit:
	; The DMX transmit code is driven by the TMR0 roll-over
	; events.  Just return if a roll-over has not occured.
	btfss	INTCON,TMR0IF	; wait until TIMER0 roll-over
	return

	bcf	INTCON,TMR0IF	; clear the flag
	movlw 	high(TxStateTable)
	movwf 	PCLATH
	movf	DmxTxState,W
	andlw	0x03		; reduce offset to valid range (0-14)
	addwf	PCL,F		; computed jump

TxStateTable:
	bra	SENDMBB		; 0 IDLE period after each complete frame
	bra	SENDDATA	; 1 send one byte of data
	bra	SENDMAB	       	; 2 IDLE period between BREAK and START slot
	bra	SENDBREAK	; 3 BREAK synchronization signal

; DmxTxState = 3. Generates a Break Signal (100uSec)
SENDBREAK
	BANKSEL TRISA ; BANK 1
	bsf     DMX_BREAK_DIR  	; tri-state the BREAK to end the break signal

	BANKSEL TMR0  ; BANK 0
	movlw	T30US          	; start timer for MARK AFTER BREAK
	movwf	TMR0

	decf	DmxTxState,F	; proceed to State2 SENDMAB

ifdef DEBUG_WORKAROUND
 	; Check for BLINK routine
	decfsz  DMX_BLINK,F
	return

	movlw	44		; Every 44 packets.. should toggle 1HZ (ish)
	movwf	DMX_BLINK


	movlw	LED_MASK_DMX
	BANKSEL LED_PORT_DMX 	; BANK 2
	xorwf	LED_PORT_DMX,F	; Toggle DMX LED
endif
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

 ; *********************************************************
DmxIrqHandler
	btfss	YADA_STATUS, YADA_DIR_BIT ; Receiving DMX?
	call	DmxRxIrqHandler

 	BANKSEL PIE1
        btfss   PIE1,TXIE   ; Are we IRQ enabled Nope.. exit
        return

        BANKSEL PIR1
        btfsc   PIR1,TXIF   ; Did the TXIF interrupt go off?
	call	DmxTxIrqHandler
	return

 ; *********************************************************
DmxRxIrqHandler
	movlw 	high(RxStateTable)
	movwf 	PCLATH
	movf	DmxRxState,W
	andlw	0x03		; reduce offset to valid range (0-3)
	addwf	PCL,F		; computed jump
RxStateTable:
	bra	DmxRx_IDLE	; 0 IDLE : No RX selected
	bra	DmxRx_W4BREAK	; 1 W4BREAK: Monitor serial Rx for FERR
	bra	DmxRx_W4START  	; 2 W4START: Look for Rx of 0x00 - else error
	bra	DmxRx_READ	; 3 READ until done (break)

DmxRx_IDLE
	; Should never be here...
	return

DmxRx_W4BREAK
	BANKSEL PIR1	; BANK 0
	btfsc	PIR1,RCIF	; If a byte is received
	call	flush_reg	; Throw it away
	BANKSEL RCSTA
	btfss	RCSTA, FERR	; Woot woot.. break found
	return	; not found ..
	movf	RCREG,W		; Read the Receive bufer to clear the error
	incf	DmxRxState,F
	return
flush_reg
	BANKSEL RCREG
	movf	RCREG,W
	return

DmxRx_W4START
	BANKSEL PIR1
	btfss	PIR1, RCIF	; No Byte received = return
	return			;
	BANKSEL RCSTA
	btfsc	RCSTA, FERR	; BREAK = Keep spinning  ?? Clear the error?
	return			;


	movf	RCREG, W
	andlw	0xFF
	bnz	DmxRx_Err
        ;; Start Byte encountered - read the payload
	movlw	LOW DmxUniverse	; init pointer to transmit buffer
	movwf	FSR0_RXL
	movlw	HIGH DmxUniverse
	movwf	FSR0_RXH
        incf	DmxRxState,F	; Change state to RX_STATE_READ
	return

DmxRx_Err
	movlw	RX_STATE_W4BREAK
	movwf	DmxRxState

	return

DmxRx_READ
	BANKSEL RCSTA
	btfsc	RCSTA,FERR
	goto	DmxRx_Err

        BANKSEL PIR1
	btfss	PIR1,RCIF	; Byte recieved?
	return			; No Byte = return and keep waiting

	movf	FSR0_RXL,W      ; Point to location in buffer
	movwf	FSR0L
	movf	FSR0_RXH,W
	movwf	FSR0H

	BANKSEL RCREG
	movf	RCREG,W 	; Copy Incoming byte into DmxUniverse
	movwi	FSR0++

	movf	FSR0L,W
	movwf	FSR0_RXL
	movf	FSR0H,W
	movwf	FSR0_RXH

	return

 ;****************************************************************************************
DmxTxIrqHandler
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

	movlw	0x00		; DEBUG value.. set at 0x00 for initialization
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

	clrf	DMX_SOF_BYTE	; Default SOF byte
; init state machines
	clrf	DmxRxState
	movlw	.03		; Start with BREAK state
	movwf	DmxTxState
	return
