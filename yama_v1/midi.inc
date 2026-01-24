; asmsyntax=pic
; File:  Midi.inc
; MIDI Tranceiver demo
;
; This source code uses the PIC16F1455 to RX and TX midi data via the
; the EUSART peripheral.  Using an internal 48MHz clock.
;
;; // LabRat - update description once I have it figured out
;  16 byte rxUart queue-->255 midEncoded Queue--> usbINendpoint buffer.
; usbOUTendpoint ---> 255 uart Tx queue -->Uart
;
; ------------------------------------------------------------------------------------------
; Updates by LabRat (Andrew Williams) 2026
;   a. Based on the YADA_v2 code base
;   b. Modified for use on the P16F1455
;   c. Midi state machine based on work by Thorsten Klose (18F2550)
;
; Exported interfaces are...
;    midiUartIrqHandler - invoked via 'call' from the application IrqHandler
;    MidiPoll - Polling call from the YAMA main loop - is it necessary anymore?
;             - Perhaps uRX polling here? (optional?)
;    MidiSetup - Setup Serial port for MIDI
;    MidiTx - If uTX is idle - Pop a byte and transmit it (kick starts the TXENGINE)
;    uartTxPush - If there is room in the uartTxBuffer - enqueue the byte
;    uartRxPop  - If there is a byte in the uartRxBuffer - pop and return it
;    MidiRxSize - return number of bytes in the RxQueue
;
; Uart Queues - Need pointer to start of buffer, startIndex, and Length for each.
;    uartRxBuffer
;    uartTxBuffer
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


; Timer0 timing constants (assuming 48MHz clock input and assigned prescaler
; values to produce ~5ms tick) Uses 1:256 pre-scalar
; (That's 2 timers running at 5ms .. should they be combined?)
#define T0CON_PRESCALAR 0x87
#define T5MS	.22         ; preload value for TMR0 to roll over ~5ms
#define T300COUNT .60

; Variables memory allocation
 CBLOCK 0x70 ; Global Variables
        AUTO_SENSE_COUNT        ; 0x70
	STATUS_FLAGS		; 0x71 
	USB_BLINK		; 0x73 44 frame '0' per second
	TEMP			; 0x74 Used for copy loop

    ; UART RX/TX Buffer Indices
        uRxTail                 ; 0x75 - uartRxBufferStart & Length
        uRxLength               ; 0x76
        uTxTail                 ; 0x77 - uartTxBufferStart & Length
        uTxLength               ; 0x78
	mRxTail
	mRxLength
	uInChar			; 0x79 - Incoming Rx Character
	uOutChar		; 0x7A - Outgoing Tx Char

	PWR_BLINK		; 0x7B - Might harmonize and use status bits
	MIDI_PACKET_COUNT	; 0x7C - Simple counter of 4-byte packets in USB payload
	end_bank0_minor_vars:0
 ENDC
 if end_bank0_minor_vars > 0x80
	error "Bank 0 Global  Variable Space overrun"
 endif

;	STATUS_FLAGS FLAGS (Note: Unused in YAMA code at this time)
;
;	Bit 0 - 0x01 -   EP1_OUT_BUFFER ready for processing 0 = idle 1 = present
;       Bit 5 - 0x20 -   MIDI ACTIVE SENSE 1 = NO DATA TRANSMITTED
;     TO DO: Ping Pong buffer support - will need these additional bits
;       Bit 6 - 0x40 -   USB Buffer Mode 0 = available 1 = Busy
;       Bit 7 - 0x80 -   0=ODD, 1=EVEN
;
MIDI_EP1_OUT 	equ	(0)
MIDI_IDLE_BIT	equ	(5)
MIDI_BUSY_BIT	equ	(6)
USB_EVEN_BIT    equ     (7)

#define MIDI_INx_BYTE0          0
#define MIDI_INx_BYTE1          1
#define MIDI_INx_BYTE2          2
#define MIDI_INx_RUNNING_STATUS 3
#define MIDI_INx_EXPECTED_BYTES 4
#define MIDI_INx_WAIT_BYTES     5
#define MIDI_INx_SYSEX_CTR      6

;****************************************************************************************
;MIDI Transmit state machine

MidiPoll:
        ; Poll the uartRx jitter buffer and pass bytes to the Midi parsing state machine
	; The MIDI transmit code is driven by UART Interrupt events
	;
        ; Is there a uartRx byte?
        movf    uRxLength,W	; Test if length != 0
        bz      _mp_active_sense; Empty .. keep polling

        call    uartRxPop 	; retrieve the byte
        call    INT_MIDI_Parse 	; Pass the byte to the MIDI parser state machine

_mp_active_sense
        ; Polling - active sense - if NO transmission within 300ms
        ;                        - Enque a 0xFE and reset the timer
	btfss	INTCON,TMR0IF	; wait until TIMER0 roll-over
	return
	bcf	INTCON,TMR0IF	; clear the flag
	movlw	T5MS		; preload timer for 5ms interval to roll over
	movwf	TMR0

        ; Skip until we have waited approx 300ms
        decfsz	AUTO_SENSE_COUNT,F
	return
        ; Reset the 300 ms counter
        movlw   T300COUNT
        movwf	AUTO_SENSE_COUNT

	; 3 Hz flash - indicating MAIN LOOP is operational
	movlw	LED_MASK_MIDI
	BANKSEL LED_PORT_MIDI	; BANK 2
	xorwf	LED_PORT_MIDI,F	; Toggle on MIDI LED,F

	; Check if device was idle the entire 300ms
        btfss	STATUS_FLAGS, MIDI_IDLE_BIT
        goto	_mp_active_sense_done

        ;; We were idle since last TMR0IF
        movlw	0xFE
        call 	uartTxPush
_mp_active_sense_done
        bsf 	STATUS_FLAGS, MIDI_IDLE_BIT
	BANKSEL PIE1
	bsf	PIE1,TXIE
	return

;****************************************************************************************
;  UART Tx and Rx interrupt handling
;
midiUartIrqHandler
        BANKSEL PIE1
        btfss   PIE1,TXIE   ; Are we IRQ enabled
        goto    _midiIrq_CheckRx

        BANKSEL PIR1
        btfss   PIR1,TXIF   ; Did the TXIF interrupt go off?
        goto    _midiIrq_CheckRx

       ; IF Tx enabled and Tx set..
	call	_midiIrqTx

_midiIrq_CheckRx
       ; IF Rx set.. (Rx ALWAYS enabled)
        BANKSEL PIR1
        btfsc   PIR1,RCIF   ; Did the RCIF interrupt go off?
	call	_midiIrqRx

	; return from IRQ
	return


_midiIrqTx
	; Test if TxLength is NOT zero
        movf	uTxLength,F ; Move to self, and set the ZERO flag
	bz	_midiIrqTxDone

	; Load FSR0 - &(uartTxBuffer[uTxTail])
        movlw	HIGH uartTxBuffer
	movwf	FSR0H
	movf	uTxTail,W
	movwf	FSR0L

    	;TXREG = uTxBuffer[uTxTail++]
	incf	uTxTail,W       ; Don't use FSR0++ as it could impact FSR0H
	andlw	UART_TX_BUFF_SIZE-1
	movwf	uTxTail

	moviw	FSR0		; Copy the data

        ; Enqueue the byte
	BANKSEL TXREG ;  BANK 3
	movwf	TXREG

	decf	uTxLength,F
        bcf	STATUS_FLAGS, MIDI_IDLE_BIT ; Flag that TX occurred
	return

_midiIrqTxDone
	BANKSEL PIE1 ; BANK 1
	bcf	PIE1, TXIE 		; Disable the interrupts
        return

; midi Rx process (uart receive)
; Copy bytes into the Rx Jitter buffer (as fast as possible)
_midiIrqRx
	BANKSEL	RCREG 	; BANK 3
	movf	RCREG,W
	movwf	uInChar ; Read the Character

	btfsc	RCSTA,FERR
	goto	_midiIrqRxErr
	btfsc	RCSTA,OERR
	goto	_midiIrqRxErr
	;  Check for Room in the buffer
	;
        movf   	uRxLength,W ; if 0x0F then buffer is full
	xorlw	UART_RX_BUFF_SIZE-1
	bz 	_midiRxFullErr

_midiIrqRxByte
	; Copy to the jitter buffer
	;
	movlw	HIGH uartRxBuffer
	movwf	FSR0H
	movf	uRxTail,W
	addwf	uRxLength,W
	andlw	UART_RX_BUFF_SIZE-1 ; Wrap Around if needed
	movwf	FSR0L

	movf	uInChar,W
	movwi	FSR0

	incf	uRxLength,f

_midiIrqRxExit	; Normal end of interrupt
	BANKSEL PIR1
	bcf	PIR1,RCIF
	return

_midiIrqRxErr 	; uart Rx errors
	bcf	RCSTA,CREN ; Clear Error(if any)
	bsf	RCSTA,CREN ; Enable Receiver **LABRAT: Is this needed? receiver already enabled??
	BANKSEL	LED_PORT_ERR
	movlw	LED_MASK_ERR
	xorwf	LED_PORT_ERR,F   ; Toggle ERR led
	goto	_midiIrqRxExit

_midiRxFullErr 	; Jitter buffer full error
	banksel LED_PORT_MIDI
	movlw	LED_MASK_MIDI
	xorwf	LED_PORT_MIDI,F   ; Toggle MIDI led - (indicating error)
	goto	_midiIrqRxExit

;; END of the IRQ Handling

;****************************************************************************************
;midiRxPop :  Pop byte (if available) NOTE: check length>0 before calling
;             As buffer size is 256.. don't need to deal with wrap around.
midiRxPop
	movlw 	HIGH midiRxBuffer
	movwf	FSR0H
	movf	mRxTail,W
	movwf	FSR0L
	incf	mRxTail,f
	decf	mRxLength,f
        moviw	FSR0
	return

;****************************************************************************************
;MidiRxPush : Push byte (if space)
;             As buffer size is 255 .. wrap around taken care of
midiRxPush
	movwf	TEMP ; Store the character until needed
	; Is there room? (if length = buffersize -1 .. buffer full)
	movf	mRxLength,W
	xorlw	MIDI_RX_BUFF_SIZE-1
	bz 	_mrx_buffer_full
	;Setup Pointer to next empty location in Tx Queue
	movlw 	HIGH midiRxBuffer
	movwf	FSR0H
	movf	mRxLength,W
	addwf	mRxTail,W       ; As buffer is 255 bytes, wrap around is taken care of
	movwf   FSR0L 		; Point to next available buffer

	; Store the byte
	movf	TEMP,W
	movwi	FSR0

	; Increment Pointer
	incf	mRxLength,f
	return
_mrx_buffer_full
	BANKSEL LED_PORT_ERR
	movlw	LED_MASK_ERR
	xorwf	LED_PORT_ERR,F   ; Toggle ERR led - for now
	return

;****************************************************************************************
;uartRxPop :  Pop byte (if available) NOTE: check length>0 before calling
;             As buffer size is 16 .. must &0x0F to deal with wrap around
uartRxPop
	movlw 	HIGH uartRxBuffer
	movwf	FSR0H

	; --- Start Critical Section
	bcf	INTCON,GIE

	movf	uRxTail,W
	movwf	FSR0L
	incf	uRxTail,W
	andlw	UART_RX_BUFF_SIZE-1
	movwf	uRxTail
	decf	uRxLength,f

	; --- End Critical Section
	bsf	INTCON,GIE

        moviw	FSR0
	return


;****************************************************************************************
;uartTxPush : Push Byte into the TxBuffer if space available
;             As buffer size is 255 .. wrap around taken care of
uartTxPush
	movwf	uOutChar ; Store the character until needed

	; Is there room? (if length = buffersize -1 .. buffer full)
	movf	uTxLength,W
	xorlw	UART_TX_BUFF_SIZE-1
	bz 	_tx_buffer_full

	; --- Start Critical Section
	bsf	INTCON,GIE
	;Setup Pointer to next empty location in Tx Queue
	movlw 	HIGH uartTxBuffer
	movwf	FSR0H
	movf	uTxLength,W
	addwf	uTxTail,W
	andlw	UART_TX_BUFF_SIZE-1 ; Mod the 128 byte length
	movwf   FSR0L

	; Store the byte
	movf	uOutChar,W
	movwi	FSR0

	; Increment Pointer
	incf	uTxLength,F

	bsf	INTCON,GIE
	; --- End Critical Section
	return
_tx_buffer_full
	BANKSEL LED_PORT_ERR
	movlw	LED_MASK_ERR
	xorwf	LED_PORT_ERR,F   ; Toggle ERR led - for now
	return

;
; MIDI Parse from Thorsten Klose 18f2550 project
;
;; --------------------------------------------------------------------------
;;  FUNCTION: INT_MIDI_Parse
;;  DESCRIPTION: parses for incoming MIDI events
;;               function should be called from MPROC_NotifyReceivedByte
;;  IN:	  incoming byte in WREG
;;  OUT:  -
;;  USES: -
;; --------------------------------------------------------------------------
INT_MIDI_Parse
	movwf	uInChar		; store byte -> uInChar

	;; Banksel the MIDI_IN_INTERNAL
	BANKSEL MIDI_IN_INTERNAL

	;; new status byte?
	btfss	uInChar, 7
	goto	INT_MIDI_Parse_NoNewStatus
_INT_MIDI_Parse_NewStatus
	;; messages >= 0xf8 don't affect the running status and can just be forwarded
	movlw	0xf8
	subwf	uInChar,W	; uInChar-0xF8
	btfsc	STATUS,C	; NEGATIVE flag if uInChar<F8
	goto	INT_MIDI_Parse_GE_F8

INT_MIDI_Parse_LT_F8 ; MIDI BYTE is <0xF8
	;; store running status
	movfw	uInChar
	movwf	MIDI_IN_INTERNAL+MIDI_INx_RUNNING_STATUS ; Store uInChar @offset
	;; get number of expected bytes
	swapf	uInChar, W
	andlw	0x07
	call	_EXPECTED_BYTES_COMMON_TABLE	; Determine Expected Number of Bytes
	bz	INT_MIDI_Parse_System		; If zero - get number expected bytes from system table

	;; "normal" MIDI event - store number of expected bytes (1 byte has already been received!)
	decf	WREG, W
	movwf   MIDI_IN_INTERNAL+MIDI_INx_EXPECTED_BYTES
	movwf	MIDI_IN_INTERNAL+MIDI_INx_WAIT_BYTES ; store same number in WAIT_BYTES counter
	goto	INT_MIDI_Parse_End

INT_MIDI_Parse_System
	;; get number of expected byte from system table (COMMON_TABLE = 0)
	movfw	uInChar
	andlw	0x0f
	call	_EXPECTED_BYTES_SYSTEM_TABLE
	decf	WREG, W		; Subtract one for *this* byte

	movwf   MIDI_IN_INTERNAL+MIDI_INx_EXPECTED_BYTES
	movwf	MIDI_IN_INTERNAL+MIDI_INx_WAIT_BYTES ; store same numberin WAIT_BYTES counter

	;; special treatment for F0
	movfw	uInChar	;  If uInChar is 0xF0
	xorlw	0xf0
	bnz	INT_MIDI_Parse_System_NoF0	; NOT 0xF0 ..

INT_MIDI_Parse_System_F0
	movfw	uInChar  ; midix->byte0 only used by SysEx handler for continuous data streams!
	movfw	MIDI_IN_INTERNAL+MIDI_INx_BYTE0

	clrf	MIDI_IN_INTERNAL+MIDI_INx_SYSEX_CTR
	incf	MIDI_IN_INTERNAL+MIDI_INx_SYSEX_CTR,F
	goto	INT_MIDI_Parse_End

INT_MIDI_Parse_System_NoF0
	;; special treatment for F7
	movfw	uInChar
	xorlw	0xf7
	bnz	INT_MIDI_Parse_System_NoF7
INT_MIDI_Parse_System_F7
	;; branch depending on number of unsent SysEx bytes
	movfw	MIDI_IN_INTERNAL+MIDI_INx_SYSEX_CTR
	clrf	MIDI_IN_INTERNAL+MIDI_INx_SYSEX_CTR ; ensure that next F7 will just send F7
	bz	INT_MIDI_Parse_System_F7_0
	addlw	-1
	bz	INT_MIDI_Parse_System_F7_1
	goto	INT_MIDI_Parse_System_F7_2
INT_MIDI_Parse_System_F7_0
	movlw	0x05			; 5: SysEx ends with single byte
	movwf	_int_midi_rx_package+0	; CIN
	movlw	0xf7
	movwf	_int_midi_rx_package+1	; first byte
	clrf	_int_midi_rx_package+2	; second byte
	clrf	_int_midi_rx_package+3	; third byte
	goto	INT_MIDI_Parse_SendPacket

INT_MIDI_Parse_System_F7_1
	movlw	0x06			; 6: SysEx ends with two bytes
	movwf	_int_midi_rx_package+0	; CIN

	movfw	MIDI_IN_INTERNAL+MIDI_INx_BYTE0
	movwf	_int_midi_rx_package+1 ; first byte

	movlw	0xf7
	movwf	_int_midi_rx_package+2	; second byte
	clrf	_int_midi_rx_package+3	; third byte
	goto	INT_MIDI_Parse_SendPacket

INT_MIDI_Parse_System_F7_2
	movlw	0x07			; 7: SysEx ends with three bytes
	movwf	_int_midi_rx_package+0	; CIN
	movfw	MIDI_IN_INTERNAL+MIDI_INx_BYTE0
	movwf	_int_midi_rx_package+1 ; first byte
	movfw	MIDI_IN_INTERNAL+MIDI_INx_BYTE1
	movwf	_int_midi_rx_package+2 ; second byte
	movlw	0xf7
	movwf	_int_midi_rx_package+3	; third byte
	goto	INT_MIDI_Parse_SendPacket

INT_MIDI_Parse_System_NoF7
	goto	INT_MIDI_Parse_End

INT_MIDI_Parse_GE_F8
	;; realtime messages don't change the running status and can be sent immediately
	movlw	0x0f			; f: send single byte
	movwf	_int_midi_rx_package+0	; CIN
	movfw	uInChar
	movwf	_int_midi_rx_package+1	; first byte
	clrf	_int_midi_rx_package+2	; second byte
	clrf	_int_midi_rx_package+3	; third byte
	goto	INT_MIDI_Parse_SendPacket

INT_MIDI_Parse_NoNewStatus
	;; special treatment for sysex streams
	movfw	MIDI_IN_INTERNAL+MIDI_INx_RUNNING_STATUS
	xorlw	0xf0
	bnz	INT_MIDI_Parse_NoRS_F0
INT_MIDI_Parse_RS_F0
	movfw	MIDI_IN_INTERNAL+MIDI_INx_SYSEX_CTR
	incf	WREG,W       ; increment sysex counter and store incoming byte in appr. register
	movwf	MIDI_IN_INTERNAL+MIDI_INx_SYSEX_CTR
	decf	WREG,W
	bz	INT_MIDI_Parse_RS_F0_1
	addlw	-1
	bz	INT_MIDI_Parse_RS_F0_2
	goto	INT_MIDI_Parse_RS_F0_3
INT_MIDI_Parse_RS_F0_1
	movfw	uInChar
	movwf	MIDI_IN_INTERNAL+MIDI_INx_BYTE0
	goto	INT_MIDI_Parse_End

INT_MIDI_Parse_RS_F0_2
	movfw	uInChar
	movwf	MIDI_IN_INTERNAL+MIDI_INx_BYTE1
	goto	INT_MIDI_Parse_End

INT_MIDI_Parse_RS_F0_3
	movfw	uInChar
	movwf	MIDI_IN_INTERNAL+MIDI_INx_BYTE2

	;; clear counter
	clrf	MIDI_IN_INTERNAL+MIDI_INx_SYSEX_CTR

	;; send three byte sysex package
	movlw	0x04			; 4: SysEx starts and continues
	movwf	_int_midi_rx_package+0	; CIN

	movfw	MIDI_IN_INTERNAL+MIDI_INx_BYTE0
	movwf	_int_midi_rx_package+1 ; first byte
	movfw	MIDI_IN_INTERNAL+MIDI_INx_BYTE1
	movwf	_int_midi_rx_package+2 ; first byte
	movfw	MIDI_IN_INTERNAL+MIDI_INx_BYTE2
	goto	INT_MIDI_Parse_SendPacket


INT_MIDI_Parse_NoRS_F0
	;; reload WAIT_BYTES counter?
	movfw	MIDI_IN_INTERNAL+MIDI_INx_WAIT_BYTES
	bnz	INT_MIDI_Parse_NoWBRel
INT_MIDI_Parse_WBRel
	movfw	MIDI_IN_INTERNAL+MIDI_INx_EXPECTED_BYTES
	movwf	MIDI_IN_INTERNAL+MIDI_INx_WAIT_BYTES
INT_MIDI_Parse_NoWBRel
	;; always decrement counter (no error - we've already received the second byte!)
	decf	MIDI_IN_INTERNAL+MIDI_INx_WAIT_BYTES,F

	;; store new byte
	decfsz	MIDI_IN_INTERNAL+MIDI_INx_EXPECTED_BYTES,W
	goto	INT_MIDI_Parse_ExpNE1
INT_MIDI_Parse_ExpEQ1	; 1 byte expected
	movfw	uInChar
	movwf	MIDI_IN_INTERNAL+MIDI_INx_BYTE1
	clrf	MIDI_IN_INTERNAL+MIDI_INx_BYTE2
	goto	INT_MIDI_Parse_ExpEQ1_Cont
INT_MIDI_Parse_ExpNE1	; 2 bytes expected
        ;; NOTE: Z flag is set based on contents of MIDI_INx_WAIT_BYTES
	movf	MIDI_IN_INTERNAL+MIDI_INx_WAIT_BYTES,W
        bz 	_do_byte2

_do_byte1
	movfw	uInChar
	movwf	MIDI_IN_INTERNAL+MIDI_INx_BYTE1
	goto 	INT_MIDI_Parse_ExpEQ1_Cont
_do_byte2
	movfw	uInChar
	movwf	MIDI_IN_INTERNAL+MIDI_INx_BYTE2


INT_MIDI_Parse_ExpEQ1_Cont
	;; put received bytes into packet once WAIT_BYTES is 0
	movfw	MIDI_IN_INTERNAL+MIDI_INx_WAIT_BYTES
	bnz	INT_MIDI_Parse_NotComplete
INT_MIDI_Parse_Complete
	;; write header (CIN)
	movfw	MIDI_IN_INTERNAL+MIDI_INx_RUNNING_STATUS
	andlw	0xf0
	xorlw	0xf0
	bz	INT_MIDI_Parse_CompFx
INT_MIDI_Parse_CompNotFx
	swapf   MIDI_IN_INTERNAL+MIDI_INx_RUNNING_STATUS,W ; common MIDI event
	andlw	0x0f
	goto	INT_MIDI_Parse_Comp_Cont

INT_MIDI_Parse_CompFx
	;; determine how many bytes should be transmitted
	movfw	MIDI_IN_INTERNAL+MIDI_INx_EXPECTED_BYTES
	bz	INT_MIDI_Parse_CompFx_1
	addlw	-1
	bz	INT_MIDI_Parse_CompFx_2
	goto	INT_MIDI_Parse_CompFx_3

INT_MIDI_Parse_CompFx_1
	movlw	0x05		; SysEx common with one byte
	goto	INT_MIDI_Parse_Comp_Cont

INT_MIDI_Parse_CompFx_2
	movlw	0x02		; SysEx common with two bytes
	goto	INT_MIDI_Parse_Comp_Cont

INT_MIDI_Parse_CompFx_3
	movlw	0x03		; SysEx common with three bytes
	;; 	goto	INT_MIDI_Parse_Comp_Cont

INT_MIDI_Parse_Comp_Cont
	movwf	_int_midi_rx_package+0	; CIN

	movfw	MIDI_IN_INTERNAL+MIDI_INx_RUNNING_STATUS ; first byte
	movwf	_int_midi_rx_package+1
	movfw	MIDI_IN_INTERNAL+MIDI_INx_BYTE1 ; second byte
	movwf	_int_midi_rx_package+2
	movfw	MIDI_IN_INTERNAL+MIDI_INx_BYTE2 ; third byte
	movwf	_int_midi_rx_package+3

INT_MIDI_Parse_SendPacket
	;; nothing more to do here (we can recognize if a package
	;; is complete by checking _int_midi_rx_package+0 != 0
	movf	_int_midi_rx_package,W
	bz	INT_MIDI_Parse_NotComplete
	call 	midiRxPush
	movf	_int_midi_rx_package+1,W
	call 	midiRxPush
	movf	_int_midi_rx_package+2,W
	call 	midiRxPush
	movf	_int_midi_rx_package+3,W
	call 	midiRxPush
INT_MIDI_Parse_NotComplete
INT_MIDI_Parse_End
	return

;****************************************************************************************
; Lookup tables 
;
_EXPECTED_BYTES_COMMON_TABLE
        brw
        retlw   3 ;0
        retlw   3 ;1
        retlw   3 ;2
        retlw   3 ;3
        retlw   2 ;4
        retlw   2 ;5
        retlw   3 ;6
        retlw   0 ;7

_EXPECTED_BYTES_SYSTEM_TABLE
        brw
        retlw  1
        retlw  2
        retlw  3
        retlw  2
        retlw  1
        retlw  1
        retlw  1
        retlw  1
        retlw  1
        retlw  1
        retlw  1
        retlw  1
        retlw  1
        retlw  1
        retlw  1
        retlw  1


;****************************************************************************************
;MidiSetup : Setup Serial port for MIDI
MidiSetup
        ; Pinmode handled in main app
	BANKSEL TXSTA ; BANK 3
	movlw	0x20		; enable TX, high speed mode
	movwf	TXSTA

	movlw	0x80		; enable serial port, disable receiver
	movwf	RCSTA

	bcf	BAUDCON,BRG16	; select EUART 8-bit Asynchronous mode operation
 	bcf	BAUDCON,BRGH	;
	movlw	.23		; init baud rate generator for 31.25 Kbaud (Fosc=48MHz)
	movwf	SPBRGL


	clrf	uTxLength
	clrf	uRxLength
	clrf	mRxLength
	clrf	mRxTail
	clrf	uRxTail
	clrf	uTxTail
	movf	RCREG,W		; Flush receive buffer
	bsf	RCSTA,CREN 	; Enable Receiver
; init Timer0
	BANKSEL OPTION_REG ; BANK 1
	movlw	T0CON_PRESCALAR ; enable Timer0, as an 8-bit timer, use prescaler 1:2
	movwf	OPTION_REG	; (0.4us tick@20MHz)
        bsf	PIE1, RCIE

	BANKSEL TMR0  ; BANK 0
        btfsc   PIR1, RCIF
        bcf	PIR1, RCIF
	movlw	T5MS		; preload timer for 5ms interval to roll over
	movwf	TMR0

        movlw   T300COUNT
        movwf	AUTO_SENSE_COUNT
	bcf	INTCON,TMR0IF	; clear roll over flag

	return

