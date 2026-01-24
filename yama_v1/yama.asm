; vim:noet:sw=8:ts=8:ai:syn=pic
; asmsyntax=pic
;
; Based on the USB 512-Word CDC Bootloader for PIC16(L)F1454/5/9
; Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
; v1.0, February 12, 2015
; Released under a 3-clause BSD license: see the accompanying LICENSE file.
;
; DMX Code from Andrew Williams Copyright (c) 2020
; YADA Command Handlers from Andrew Williams Copyright (c) 2020
; YAMA Based on YADA code base from Andrew Williams Copyright (c) 2022
;
;
;
; Build flag - re-use CDC code from the bootloader
USE_BOOTLOADER_CDC	equ 0

; Build flags - use interrupts for USB code (vs polling)
USB_INTERRUPTS          equ 1

	radix dec
	list n=0,st=off
	include "p16f1455.inc"
	nolist
	include "macros.inc"
	include "bdt.inc"
	include "usb.inc"
	include "protocol_constants.inc"
	include "import_list.inc"
	list
	errorlevel -302


;;; Constants and varaiable addresses
SERIAL_NUMBER_DIGIT_CNT	equ	4
	ifndef SERIAL_NUMBER
		variable SERIAL_NUMBER=42	; Why doesnt 'equ' work here? Go figure
	endif
;; TMR1 -
;;   48Mhz, Default T1CON options TMR1=5536 = 0.005s Timeout
;;   BANKSEL 0x00 TMR1H : 0x15 TMR1L: 0xA0

;; -----------------------------
;; LED & SWITCH Definitions
;; -----------------------------
#define BUTTON_PORT PORTA
#define BUTTON  PORTA,3

  ; GREEN LED : POWER
	#define LED_PWR  LATA,4
	#define LED_PORT_PWR  LATA
	#define LED_DIR_PWR  TRISA,4
	#define LED_MASK_PWR  0x10

  ; AMBER LED : MIDI
	#define LED_MIDI LATC,2
	#define LED_PORT_MIDI LATC
	#define LED_DIR_MIDI TRISC,2
	#define LED_MASK_MIDI 0x04

  ; BLUE LED  : USB
	#define LED_USB  LATC,3
	#define LED_PORT_USB  LATC
	#define LED_DIR_USB  TRISC,3
	#define LED_MASK_USB  0x08

  ; RED LED   : ERROR
	#define LED_ERR  LATA,5
	#define LED_PORT_ERR  LATA
	#define LED_DIR_ERR  TRISA,5
	#define LED_MASK_ERR  0x20

;; ----------------------------
;; Code Space Sizes
;; ----------------------------

;#define MAX_CODE_SIZE    ( 0x1F80 )	; Leave space for HIGH availability FLASH
; YAMA - move to earlier page boundary .. to keep descriptor on a single code page
#define MAX_CODE_SIZE    ( 0x1EFF )	; Leave space for HIGH availability FLASH
#define HIGH_AVAIL_FLASH ( 0x1F80 )	; Address for HIGH availability FLASH

; -----------------------------------------------------------------------
; LabRat - updated with VID/PID assigned to YAMA project by Microchip
;          THANK YOU MICROCHIP!! You ROCK!
; -----------------------------------------------------------------------
USB_VENDOR_ID          equ     0x04D8
USB_PRODUCT_ID         equ     0xE6D1

; -------------------------------
; Descriptor Lengths & Locations
; -------------------------------
DEVICE_DESC_LEN		equ	18	; device descriptor length
CONFIG_DESC_TOTAL_LEN	equ	101	; total length of configuration descriptor
SERIAL_NUM_DESC_LEN	equ	2+(SERIAL_NUMBER_DIGIT_CNT*2)
STRINGS_DESC_LEN	equ	0x3A
ALL_DESCS_TOTAL_LEN	equ	DEVICE_DESC_LEN+CONFIG_DESC_TOTAL_LEN+SERIAL_NUM_DESC_LEN+STRINGS_DESC_LEN

; Endpoint Buffer Sizes
EP0_BUF_SIZE 		equ	8 	; endpoint 0 buffer size
EP1_OUT_BUF_SIZE	equ	64	; endpoint 1 OUT (MIDI data) buffer size
EP1_IN_BUF_SIZE		equ	64	; endpoint 1 IN (MIDI data) buffer size

; Since we're only using 5 endpoints, use the 4 bytes normally occupied by the
; EP2 OUT buffer descriptor for variables,and the BDT area for buffers.
USB_STATE		equ	BANKED_EP2OUT+0
EP0_DATA_IN_PTR		equ	BANKED_EP2OUT+1	; pointer to descriptor to be sent (low byte only)
EP0_DATA_IN_COUNT	equ	BANKED_EP2OUT+2	; remaining bytes to be sent
APP_POWER_CONFIG	equ	BANKED_EP2OUT+3	; application power config byte

; Allocate space for BUFFERs pointed to by USB descriptors
; 0x20::0x2000 to 0x2F::0x200F : Buffer Descriptors
; 0x30::0x2010 to 0x33::0x2013 : Used for 4 variables above
; 0x34::0x2014 to 0x3B::0x201B : EP0_OUT_BUF[8]
; 0x3C::0x201C to 0x43::0x2023 : EP0_IN_BUF[8]
; 0x44::0x2024 to 0x83::0x2063 : EP1_OUT_BUF[64]
; 0xA0::0x2050 to 0xDF::0x208F : EP1_IN_BUF[64]
; *NOTE: Shifted to next Bank to maintain same CPU Bank Address
; 0xA0::0x2050 to 0xDF::0x208F
EP0OUT_BUF		equ	EP2IN
BANKED_EP0OUT_BUF	equ	BANKED_EP2IN	; buffers go immediately after EP2 OUT's buffer descriptor
EP0IN_BUF		equ	EP0OUT_BUF+EP0_BUF_SIZE
BANKED_EP0IN_BUF	equ	BANKED_EP0OUT_BUF+EP0_BUF_SIZE

EP1IN_BUF		equ	EP0IN_BUF+EP0_BUF_SIZE
BANKED_EP1IN_BUF	equ	BANKED_EP0IN_BUF+EP0_BUF_SIZE

; Shift to start of next Bank - to keep buffer contiguous
EP1OUT_BUF		equ	0x2050 	
BANKED_EP1OUT_BUF	equ	0xA0	
MIDI_IN_INTERNAL	equ	BANKED_EP1OUT_BUF+EP1_OUT_BUF_SIZE ; Temp storage for MIDI Rx State Machine
; Aligns with start of next Bank - 0x1A0
_int_midi_rx_package    equ	0x1A0 ; was MIDI_IN_INTERNAL+0x10 ; 16 bytes: Start of NEXT bank (20F0)

; High byte of all endpoint buffers.
EPBUF_ADRH		equ	(EP0OUT_BUF>>8)

	if ((EP0IN_BUF>>8) != (EP0OUT_BUF>>8)) || ((EP1OUT_BUF>>8) != (EP0OUT_BUF>>8)) || ((EP1IN_BUF>>8) != (EP0OUT_BUF>>8))
		error "Endpoint buffers must be in the same 256-word region"
	endif

; Total length of all RAM (variables, buffers, BDT entries) used by the bootloader,
USED_RAM_LEN		equ	EP1OUT_BUF+EP1_OUT_BUF_SIZE+0x10+0x04-BDT_START

;; ------------------------------------------
;; RAM Blocks to Hold MID in/out UART Buffers
;; ------------------------------------------
uartRxBuffer	equ	0x2140
UART_RX_BUFF_SIZE equ	128   ; 128 byte jitter buffer - IRQ in, polling OUT


;; NOTE: Documentation Error: Linear block starts at 0x21F0 .. else last 16 bytes read as zero
;;       Opted to keep the blocks on the same HIGH address pages
midiRxBuffer	equ	0x2200 ; Packets [64 4-byte packets]
MIDI_RX_BUFF_SIZE equ   256

uartTxBuffer	equ	midiRxBuffer+MIDI_RX_BUFF_SIZE
UART_TX_BUFF_SIZE equ   128

; -------------------
; USB_STATE bit flags
; -------------------
IS_CONTROL_WRITE	equ	0	; current endpoint 0 transaction is a control write
ADDRESS_PENDING		equ	1	; need to set address in next IN transaction
DEVICE_CONFIGURED	equ	2	; the device is configured

; - - - - - - - - - - - - - -
; API from bootloader
; - - - - - - - - - - - - - -
  org APP_ENTRY_POINT
  PAGESEL _app_main
  goto    _app_main

  org APP_CONFIG
  PAGESEL _app_config
  goto    _app_config ; must end with 'retlw' instruction

  org APP_INTERRUPT
  PAGESEL _app_interrupt
  call    _app_interrupt
  retfie

; - - - - - - - - - - - - - -

_app_config
	retlw	0x0a


_app_interrupt
	PAGESEL	midiUartIrqHandler
        call	midiUartIrqHandler
        PAGESEL _app_interrupt ; Restore the context

_midi_irq_done
	if USB_INTERRUPTS
	else
	  return
	endif

usb_event_handler
	if USB_INTERRUPTS
	  BANKSEL PIR2
          btfss	PIR2,USBIF
	  return
	else
	endif

        BANKSEL UIR
; USB reset?
        btfss   UIR,URSTIF
        goto    _utrans         ; not a reset? just start servicing transactions
        lcall   usb_init        ; if so, reset the USB interface (clears interrupts)
        PAGESEL _app_interrupt

        if USB_INTERRUPTS
          BANKSEL PIE2
          bsf     PIE2,USBIE	; reenable USB interrupts
        endif

        BANKSEL UIR
        bcf     UIR,URSTIF      ; clear the flag
; service transactions
_utrans
        BANKSEL UIR
        btfss   UIR,TRNIF
        goto    _usdone
        movf	USTAT,W		; stash the status in a temp register
        movwf   FSR1H
        bcf     UIR,TRNIF       ; clear flag and advance USTAT fifo
        BANKSEL BANKED_EP0OUT_STAT
        andlw   USTAT_ENDP_MASK ; check endpoint number
        bnz     _umidi          ; if not endpoint 0, it's a MIDI payload message
        lcall   usb_service_ep0 ; handle the control message
        PAGESEL _app_interrupt
        goto    _utrans
; clear USB interrupt
_usdone
        BANKSEL PIR2
        bcf     PIR2,USBIF
        return                  ; Bootloader frames calls this as standard CALL (!retfie)
_umidi
        call    usb_service_midi ; USTAT value is still in FSR1H
        goto    _utrans


; -------------------------------
; Main Loop - this is the inner
; tight loop for the application
; -------------------------------
main_loop
	bsf	INTCON,PEIE
	bsf	INTCON,GIE	; enable interrupts

        ;; initialize timer1
        movlw   (1 << TMR1ON)   ; turn on Timer1 (aiming at every 5 ms tick)
        movwf   T1CON
	
	BANKSEL LATA
	bcf	LED_PWR  ; PWR LED ON
	bsf	LED_USB  ; USB LED OFF
	bsf	LED_MIDI ; MIDI LED OFF
	bsf	LED_ERR	 ; ERR LED OFF

_loop
	clrwdt 	; clear the watchdog timer

	if USB_INTERRUPTS
	else
	  call	usb_event_handler ; Polling to check forUSB events
	endif

	PAGESEL	MidiPoll
        call	MidiPoll ; Polling Incoming UART for MIDI frame
	PAGESEL $

	btfsc	STATUS_FLAGS,MIDI_EP1_OUT
	call	user_process_packet

_czeck_midi_packet_to_usb
        ; is There data to send?
	movf	mRxLength,W
	sublw	0x03
	btfsc	STATUS,C ;  if 4 or more bytes, there is a packet to send
	goto	_czeck_button

        call	_copy_midi_to_usb_payload
	iorlw	0x00 	; Force Z status bit to be set
        btfsc	STATUS,Z
        goto	_czeck_button
	BANKSEL BANKED_EP1IN_BUF
	; UPDATE HERE TO SELECT BUFFERS BASED ON ODD/EVEN SELECTION
	movwf	BANKED_EP1IN_CNT
	bsf	BANKED_EP1IN_STAT,UOWN

	movlw	LED_MASK_USB  ; Blink the USB LED
	BANKSEL LED_PORT_USB
	xorwf	LED_PORT_USB,F

_czeck_button
        BANKSEL BUTTON_PORT
        btfss   BUTTON  ; Is the button pressed?
	goto	_handle_reset

_czeck_led
	BANKSEL PIR1
	btfsc	PIR1, TMR1IF
	call	led_handler

	goto	_loop

_handle_reset
	BANKSEL LATA	 ; Turn on PWR LEDs and hand control back to the BOOTLOADER
	bsf	LED_USB
	bcf	LED_PWR
	bsf	LED_MIDI
	bsf	LED_ERR
        reset


;;; Handles a control transfer on endpoint 0.
;;; arguments:	expects USTAT value in FSR1H
;;;		BSR=0
;;; returns:	none
;;; clobbers:	W, FSR1H
usb_service_ep0
	btfsc	FSR1H,DIR			; is it an IN transfer or an OUT/SETUP?
	goto	_usb_ctrl_in
; it's an OUT or SETUP transfer
	movf	BANKED_EP0OUT_STAT,W
	andlw	BDnSTAT_PID_MASK		; isolate PID bits
	sublw	PID_SETUP			; is it a SETUP packet?
	bnz	arm_ep0_out			; if not, it's a regular OUT, just rearm the buffer
	; it's a SETUP packet--fall through

; Handles a SETUP control transfer on endpoint 0.
; BSR=0
_usb_ctrl_setup
	bcf	USB_STATE,IS_CONTROL_WRITE
; get bmRequestType, but don't bother checking whether it's standard/class/vendor...
; the CDC and standard requests we'll receive have distinct bRequest numbers
; Windows 10 patch
	bcf	BANKED_EP0OUT_STAT,UOWN	; dearm the OUT endpoint
	bcf	BANKED_EP0IN_STAT,UOWN	; dearm the OUT endpoint

	movf	BANKED_EP0OUT_BUF+bmRequestType,W
	btfss	BANKED_EP0OUT_BUF+bmRequestType,7	; is this host->device?
	bsf	USB_STATE,IS_CONTROL_WRITE		; if so, this is a control write
; check request number: is it Get Descriptor?
	movlw	GET_DESCRIPTOR
	subwf	BANKED_EP0OUT_BUF+bRequest,w
	bz	_usb_get_descriptor
; is it Set Address?
	movlw	SET_ADDRESS
	subwf	BANKED_EP0OUT_BUF+bRequest,w
	bz	_usb_set_address
; is it Set_Configuration?
	movlw	SET_CONFIG
	subwf	BANKED_EP0OUT_BUF+bRequest,w
	bz	_usb_set_configuration
; is it Get Configuration?
	movlw	GET_CONFIG
	subwf	BANKED_EP0OUT_BUF+bRequest,w
	bz	_usb_get_configuration
; unhandled request? fall through to _usb_ctrl_invalid
; Missing:
;  GET_STATUS
;   SetupPkt.Recipient == USB_SETUP_RECIPIENT_DEVICE_BITFIELD
;     Return bits - self/bus powered, RemoteWakeup Disable/Enable
;   SetupPkt.Recipient == USB_SETUP_RECIPIENT_INTERFACE_BITFIELD
;     No data to update
;   SetupPkt.Recipient == USB_SETUP_RECIPIENT_ENDPOINT_BITFIELD
;     Return bits - Halt Status
;   if pipe is busy enumerate pipe info
;  CLEAR_FEATURE/SET_FEATURE
;    - Check for valid REMOTE WAKEUP request
;    - Check for valid ENDPOINT HALT
;    - Check for PingPong buffering - buffer even/odd
;    - In/Out BD entries update
;    - Check for SetFeature & I own the buffer
;            - Terminate the xferA
;      (** and much more ***)
;  GET_INTERFACE
;  SET_INTERFACE
;  SET_DESCRIPTOR - calls routine that looks for:
;     EVENT_TRANSFER - do nothing
;     EVENT_SOF - increment an unused counter
;     EVENT_RESUME - do nothing
;     EVENT_CONFIGURED - Enable EP1 IN & OUT
;     EVENT_SET_DESCRIPTOR - do nothing
;     EVENT_P0_REQUEST - do nothing
;     EVENT_BUS_ERROR - do nothing
;     EVENT_TRANSFER_TERMINATED - do nothing
;  SYNCH_FRAMER

; Finishes a rejected SETUP transaction: the endpoints are stalled
_usb_ctrl_invalid
	BANKSEL	UCON
	bcf	UCON,PKTDIS	; reenable packet processing
	BANKSEL	BANKED_EP0IN_STAT
	movlw	_DAT0|_DTSEN|_BSTALL
	call	arm_ep0_in_with_flags
arm_ep0_out
	movlw	_DAT0|_DTSEN|_BSTALL
arm_ep0_out_with_flags			; W specifies STAT flags
	movwf	BANKED_EP0OUT_STAT
	movlw	EP0_BUF_SIZE		; reset the buffer count
	movwf	BANKED_EP0OUT_CNT
	bsf	BANKED_EP0OUT_STAT,UOWN	; arm the OUT endpoint
	return

; Finishes a successful SETUP transaction.
_usb_ctrl_complete
	BANKSEL	UCON
	bcf	UCON,PKTDIS		; reenable packet processing
	BANKSEL	USB_STATE
	btfsc	USB_STATE,IS_CONTROL_WRITE
	goto	_cwrite
; this is a control read; prepare the IN endpoint for the data stage
; and the OUT endpoint for the status stage
_cread	
	call	ep0_read_in		; read data into IN buffer
	movlw	_DAT1|_DTSEN		; OUT buffer will be ready for status stage
; value in W is used to specify the EP0 OUT flags
_armbfs
	call	arm_ep0_out_with_flags
	movlw	_DAT1|_DTSEN		; arm IN buffer
arm_ep0_in_with_flags			; W specifies STAT flags
	movwf	BANKED_EP0IN_STAT
	bsf	BANKED_EP0IN_STAT,UOWN
	return

; this is a control write: prepare the IN endpoint for the status stage
; and the OUT endpoint for the next SETUP transaction
_cwrite	bcf	BANKED_EP0IN_STAT,UOWN	; ensure we have ownership of the buffer
	clrf	BANKED_EP0IN_CNT	; we'll be sending a zero-length packet
	movlw	_DAT0|_DTSEN|_BSTALL	; make OUT buffer ready for next SETUP packet
	goto	_armbfs			; arm OUT and IN buffers


; Handles a Get Descriptor request.
; BSR=0
_usb_get_descriptor
; check descriptor type
	movlw	DESC_CONFIG
	subwf	BANKED_EP0OUT_BUF+wValueH,w
	bz	_config_descriptor
	movlw	DESC_STRING
	subwf	BANKED_EP0OUT_BUF+wValueH,w
	bz	_string_descriptor
	movlw	DESC_DEVICE
	subwf	BANKED_EP0OUT_BUF+wValueH,w
	bnz	_usb_ctrl_invalid

_device_descriptor
	movlw	low DEVICE_DESCRIPTOR
	movwf	EP0_DATA_IN_PTR
	movlw	DEVICE_DESC_LEN
	goto	_set_data_in_count_from_w

_config_descriptor
	movlw	low CONFIGURATION_DESCRIPTOR
	movwf	EP0_DATA_IN_PTR
	movlw	CONFIG_DESC_TOTAL_LEN	; length includes the sub-descriptors
	goto	_set_data_in_count_from_w

_string_descriptor
; Check wValueL for which string descriptor is requested
	movf    BANKED_EP0OUT_BUF+wValueL,w
	bz	_string_sd000  ; 0x00 - which string to return
        decf	WREG,W	
	bz	_string_mfg    ; 0x01
	decf	WREG,W
	bz	_string_prod   ; 0x02
	goto	_string_serial ; Default to SerialNo.

_string_mfg
	movlw	low sd003
	movwf	EP0_DATA_IN_PTR
	movlw	IMFG_SIZE
	goto	_set_data_in_count_from_w

_string_prod
	movlw	low sd002
	movwf	EP0_DATA_IN_PTR
	movlw 	IPROD_SIZE	
	goto	_set_data_in_count_from_w

_string_serial
	movlw	low sd001      ; SERIAL_NUMBER_STRING_DESCRIPTOR
	movwf	EP0_DATA_IN_PTR
	movlw	SERIAL_NUM_DESC_LEN
	goto	_set_data_in_count_from_w

_string_sd000
	movlw 	low sd000
	movwf	EP0_DATA_IN_PTR
	movlw	0x04

 	;Drop through	

_set_data_in_count_from_w
	movwf	EP0_DATA_IN_COUNT
; the count needs to be set to the minimum of the descriptor's length (in W)
; and the requested length
;	subwf	BANKED_EP0OUT_BUF+wLengthL,w	; just ignore high byte...
;	bc	_usb_ctrl_complete		; if W <= f, no need to adjust
; Windows 10 patch
	tstf	BANKED_EP0OUT_BUF+wLengthH		; test high byte...
	bnz	_usb_ctrl_complete		        ; use length of descriptor
	subwf	BANKED_EP0OUT_BUF+wLengthL,w
	bc	_usb_ctrl_complete

	movf	BANKED_EP0OUT_BUF+wLengthL,W
	movwf	EP0_DATA_IN_COUNT
	goto	_usb_ctrl_complete

; Handles a Set Address request.
; The address is actually set in the IN status stage.
_usb_set_address
	bsf	USB_STATE,ADDRESS_PENDING	; address will be assigned in the status stage
	goto	_usb_ctrl_complete

; Handles a Set Configuration request.
; For now just accept any nonzero configuration.
; BSR=0
_usb_set_configuration
	bcf	USB_STATE,DEVICE_CONFIGURED	; temporarily clear flag
	tstf	BANKED_EP0OUT_BUF+wValueL	; anything other than 0 is valid
	skpz
	bsf	USB_STATE,DEVICE_CONFIGURED
	call	endpoint_init
	goto	_usb_ctrl_complete

; Handles a Get Configuration request.
; BSR=0
_usb_get_configuration
; load a pointer to either a 0 or a 1 in ROM
; the 0 and 1 have been chosen so that they are adjacent
	movlw	low CONFIGURATION_0_CONSTANT
	btfsc	USB_STATE,DEVICE_CONFIGURED
	incw
	movwf	EP0_DATA_IN_PTR
	movlw	1
	movwf	EP0_DATA_IN_COUNT
	goto	_usb_ctrl_complete

; Handles an IN control transfer on endpoint 0.
; BSR=0
_usb_ctrl_in
	btfsc	USB_STATE,IS_CONTROL_WRITE	; is this a control read or write?
	goto	_check_for_pending_address
; fetch more data and re-arm the IN endpoint
	call	ep0_read_in
	movlw	_DTSEN
	btfss	BANKED_EP0IN_STAT,DTS	; toggle DTS
	bsf	WREG,DTS
	goto	arm_ep0_in_with_flags	; arm the IN buffer
	
; if this is the status stage of a Set Address request, assign the address here.
; The OUT buffer has already been armed for the next SETUP.
_check_for_pending_address
	btfss	USB_STATE,ADDRESS_PENDING
	return
; read the address out of the setup packed in the OUT buffer
	bcf	USB_STATE,ADDRESS_PENDING
	movf	BANKED_EP0OUT_BUF+wValueL,W
	BANKSEL	UADDR
	movwf	UADDR
	return



;;; Reads descriptor data from EP0_DATA_IN_PTR, copies it to the EP0 IN buffer,
;;; and decrements EP0_DATA_IN_COUNT.
;;; arguments:	BSR=0
;;; returns:	EP0_DATA_IN_PTRL advanced
;;;		EP0_DATA_IN_COUNT decremented
;;; clobbers:	W, FSR0, FSR1
ep0_read_in
	bcf	BANKED_EP0IN_STAT,UOWN	; make sure we have ownership of the buffer
	clrf	BANKED_EP0IN_CNT	; initialize buffer size to 0
	tstf	EP0_DATA_IN_COUNT	; do nothing if there are 0 bytes to send
	skpnz
	return
	movf	EP0_DATA_IN_PTR,W	; set up source pointer
	movwf	FSR0L
	movlw	DESCRIPTOR_ADRH|0x80
	movwf	FSR0H
	ldfsr1d	EP0IN_BUF		; set up destination pointer
	clrw
; byte copy loop
_bcopy	sublw	EP0_BUF_SIZE		; have we filled the buffer?
	bz	_bcdone
	moviw	FSR0++
	movwi	FSR1++
	incf	BANKED_EP0IN_CNT,f	; increase number of bytes copied
	movf	BANKED_EP0IN_CNT,W	; save to test on the next iteration
	decfsz	EP0_DATA_IN_COUNT,f	; decrement number of bytes remaining
	goto	_bcopy
; write back the updated source pointer
_bcdone	movf	FSR0L,W
	movwf	EP0_DATA_IN_PTR
	return

;;; Initializes the buffers for the MIDI endpoints (1 OUT, 1 IN, and 2 IN).
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR=0
endpoint_init
	BANKSEL	BANKED_EP1OUT_STAT
	call	arm_ep1_out
	; arm EP1 IN buffer, clearing data toggle bit
	clrw

; arms endpoint 1 IN, toggling DTS if W=(1<<DTS)
arm_ep1_in
	clrf	BANKED_EP1IN_CNT	; next packet will have 0 length (unless another OUT is received)
	andwf	BANKED_EP1IN_STAT,f	; clear all bits (except DTS if bit is set in W)
	xorwf	BANKED_EP1IN_STAT,f	; update data toggle (if bit is set in W)
	;bsf	BANKED_EP1IN_STAT,UOWN
	return


;;; Services a transaction on one of the midi endpoints.
;;; arguments:	USTAT value in FSR1H
;;;		BSR=0
;;; returns:	none
;;; clobbers:	W, FSR0, FSR1
usb_service_midi
    ; BSR: BANKED_EP1OUT_CNT
	movlw	(1<<DTS)
	btfsc	FSR1H,ENDP1		; ignore endpoint 2
	return
	btfsc	FSR1H,DIR		; if endpoint 1 IN, return - allow polling loop to rearm
        goto	arm_ep1_in
	movf	BANKED_EP1OUT_CNT,f	; test for a zero-length packet
	bz	arm_ep1_out		; (just ignore them and rearm the OUT buffer)
	bsf	STATUS_FLAGS,MIDI_EP1_OUT
        return

; Need to know which buffers were used last time, and update odd/even
user_process_packet

        BANKSEL BANKED_EP0OUT_STAT

; TODO: ADD ODD/EVEN endpoint buffers: will need to update prior to call of  _yama_cmd
	movlw	high EP1OUT_BUF
	movwf	FSR1H
	movlw	low EP1OUT_BUF
	movwf	FSR1L

	call	_yama_cmd		; execute command- OUTPUT length returned in W
	bcf	STATUS_FLAGS,MIDI_EP1_OUT
	BANKSEL BANKED_EP1IN_BUF
	; UPDATE HERE TO SELECT BUFFERS BASED ON ODD/EVEN SELECTION
	;movwf	BANKED_EP1IN_CNT
	;bsf	BANKED_EP1IN_STAT,UOWN
	; fall through to arm_ep1_out

arm_ep1_out
	movlw	EP1_OUT_BUF_SIZE		; set CNT
	movwf	BANKED_EP1OUT_CNT
	clrf	BANKED_EP1OUT_STAT		; ignore data toggle
	bsf	BANKED_EP1OUT_STAT,UOWN	; rearm OUT buffer
	return

;;; Parse the USB midi and queue outgoing to the UART
;;; arguments:	command payload in EP1 OUT buffer
;;; 		BSR=0
;;; returns:	status code in W
;;; clobbers:	W, BSR, FSR0, FSR1
_yama_cmd
; USB MIDI PACKET parser goes here
_midi_out_payload
	; Somewhat Based on USB to MIDI code from Thorsten Klose
	; Shift PacketCount lsrf 2x .. to leave number f packets to process
	movf	BANKED_EP1OUT_CNT,W
	movwf	MIDI_PACKET_COUNT
	lsrf	MIDI_PACKET_COUNT,F
	lsrf	MIDI_PACKET_COUNT,F

	; Check if destination cable number is 0
	movlw	0xF0
	andwf	INDF1,W
	bnz	 _mo_next_packet

_midi_out_czeck_cable
	movlw	0x0F
	andwf	INDF1,W  ; Lower Nibble
	; Retrieve # of bytes to Enqueue
	call  	get_midi_x_size ; midi10.pdf Table 4-1
	movwf	TEMP
	iorlw	0	; Ensure Zero bit is set

	bz	_midi_out_next_packet

	; Check if there is room for the packet in the TxBuffer?
	addwf	uTxLength,W
	btfsc	STATUS,C
	goto	_txOverrun ; Drop the packet and show an error

	moviw	1[FSR1]
	call	uartTxPush
	decf	TEMP,F
	bz	_midi_out_next_packet
	moviw	2[FSR1]
	call	uartTxPush
	decf	TEMP,F
	bz	_midi_out_next_packet
	moviw	3[FSR1]
	call	uartTxPush

_midi_out_next_packet
        BANKSEL PIR1 			; IF idle - kickstart the transmitter
        btfss	PIR1, TXIF
        goto	_mo_next_packet
        BANKSEL PIE1
        bsf	PIE1, TXIE
_mo_next_packet
	BANKSEL BANKED_EP1OUT_BUF 	; Restore the BANKSEL register (is this needed?)
	; Add 4 to FSR1 - until we have seen all 64 bytes
	addfsr	FSR1,4
	decfsz	MIDI_PACKET_COUNT,F	; Count down of packets to process
	retlw   0x00
        goto	_midi_out_czeck_cable

_txOverrun
	movlw	LED_MASK_ERR  ; Power LED MASK
	BANKSEL LED_PORT_ERR
	xorwf	LED_PORT_ERR,F ; Toggle the LED status
        goto	_mo_next_packet

_copy_midi_to_usb_payload
	; Check if U1EP1IN is free (UOWN = 0)
	BANKSEL BANKED_EP1IN_BUF
	btfsc	BANKED_EP1IN_STAT,UOWN
	retlw	0x00 ; USB owns it.. return payload size as 0
	; If 0, CPU owns it. Safe to write data.

        BANKSEL BANKED_EP1IN_BUF
	; Populate the EP1 IN buffer
	movlw	high EP1IN_BUF
	movwf	FSR1H
	movlw	low EP1IN_BUF
	movwf	FSR1L

	lsrf	mRxLength,W
	lsrf	WREG,W       ; Convert used size to 4 byte payloads
	andlw	0x0f
	btfsc	STATUS,Z
	retlw	0x00 ; Nothing to send

	; We have payloads to copy over
	movwf	TEMP     ; Store number of packets to enqueue
	clrf	MIDI_PACKET_COUNT

_midi_reply_loop
	call	midiRxPop
	movwi	FSR1++		; Cable Number
	call	midiRxPop
	movwi	FSR1++
	call	midiRxPop
	movwi	FSR1++
	call	midiRxPop
	movwi	FSR1++
	incf	MIDI_PACKET_COUNT,F
	decfsz  TEMP,F
	goto	_midi_reply_loop

	lslf	MIDI_PACKET_COUNT,W
	lslf	WREG,W 	; Convert Packet Count to Byte Count (x4)
	;return  ; DROP through... W contains number of packets enqueued
	

ret	return ; Needed for burning 6 instructions up above
;;; Main function
; Now entering application code: initialize the USB interface and wait for commands.
_app_main
	call	reset_timer1

        BANKSEL TRISA ; BANK 1
        bcf     LED_DIR_ERR
        bcf     LED_DIR_PWR
        bcf     LED_DIR_USB
        bcf     LED_DIR_MIDI

        bcf     TRISC,4 ; EUSART - TX
        bsf     TRISC,5 ; EUSART - RX

        BANKSEL LATA ; BANK 2
        bcf     LED_PWR  ; GREEN ; Turn ON  PWR LED
        bsf     LED_MIDI ; AMBER ; Turn OFF DMX LED
        bsf     LED_USB  ; BLUE  ; Turn OFF USB LED
        bsf     LED_ERR  ; RED   ; Turn OFF ERR LED

	movlw	44
	movwf	USB_BLINK

        BANKSEL ANSELA ; BANK 3
        clrf    ANSELA

; Initialize USB
	lcall	usb_init ;(on return BSR=UNKNOWN)

	PAGESEL	_app_main

; Attach to the bus (could be a subroutine, but inlining it saves 2 instructions)
_usb_attach
	BANKSEL	UCON		; reset UCON
	clrf	UCON
	if USB_INTERRUPTS
	  BANKSEL	PIE2
	  bsf	PIE2,USBIE	; enable USB interrupts
	  bsf	INTCON,PEIE
	endif
	BANKSEL	UCON
_usben
	bsf	UCON,USBEN	; enable USB module and wait until ready
	btfss	UCON,USBEN
	goto	_usben

; Enable interrupts and enter an idle loop
; (Loop code is located at the top of the file, in the first 256 words of
; program memory)
	PAGESEL	MidiSetup
        call    MidiSetup
	PAGESEL $

	goto	main_loop

; Handle a TIMER1 Reset Event (approx evey 5ms)
reset_timer1
        BANKSEL TMR1 ; BANK 0
	movlw	0x15
	movwf	TMR1H
	movlw	0xA0
	movwf	TMR1L
	bcf	PIR1, TMR1IF
	return

; LED Event Handling
; Use TMR1 roll over event (@200Hz) to blink power LED at 1HZ
; This allows the operator to recognize APP execution vs
; BOOTLOADER execution.
;
led_handler
     ; Code to handle an LED "tick" event
	call reset_timer1
	decfsz PWR_BLINK,F
	return

	movlw	.200
	movwf	PWR_BLINK  ; Setup a 1s LED status polling loop

	movlw	LED_MASK_PWR  ; Power LED MASK
	BANKSEL LED_PORT_PWR
	xorwf	LED_PORT_PWR,F ; Toggle the LED status
	return

;;; Gets the application's power config byte and stores it in APP_POWER_CONFIG.
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR, FSR0
get_app_power_config
	BANKSEL	APP_POWER_CONFIG
	movlw	0x33			; default value: bus-powered, max current 100 mA
	movwf	APP_POWER_CONFIG
	return


;;; Initializes the USB system and resets all associated registers.
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR, FSR0, FSR1H
usb_init
; disable USB interrupts
	BANKSEL	PIE2
	bcf	PIE2,USBIE
; clear USB registers
	BANKSEL	UEIR
	clrf	UEIR
	clrf	UIR
; disable endpoints we won't use
	clrf	UEP3
	clrf	UEP4
	clrf	UEP5
	clrf	UEP6
	clrf	UEP7
; set configuration
	clrf	UEIE		; don't need any error interrupts
	movlw	(1<<UPUEN)|(1<<FSEN)
	movwf	UCFG		; enable pullups, full speed, no ping-pong buffering
	movlw	(1<<TRNIE)|(1<<URSTIE)
	movwf	UIE			; only need interrupts for transaction complete and reset
; clear all BDT entries, variables, and buffers
	clrf	FSR0L
	movlw	high BDT_START	; BDT starts at 0x2000
	movwf	FSR0H
	movlw	USED_RAM_LEN
	movwf	FSR1H		; loop count
	movlw	0
_ramclr	
	movwi	FSR0++
	decfsz	FSR1H,f
	goto	_ramclr
; get the app's power configuration (if it's present)
	call	get_app_power_config
; reset ping-pong buffers and address
	BANKSEL	UCON
	bsf	UCON,PPBRST
	clrf	UADDR
	bcf	UCON,PKTDIS	; enable packet processing
	bcf	UCON,PPBRST	; clear ping-pong buffer reset flag
; flush pending transactions
_tflush	
	btfss	UIR,TRNIF
	goto	_initep
	bcf	UIR,TRNIF
	call	ret		; need at least 6 cycles before checking TRNIF again
	goto	_tflush
; initialize endpoints:
; 0 for control
; 1 for CDC bulk data
; 2 for CDC notifications (though it's never actually used)
; my intuition was that I should wait until a SET_CONFIGURATION is received
; before setting up endpoints 1 and 2... but there seemed to be a timing issue
; when doing so, so I moved them here
_initep	
	movlw	(1<<EPHSHK)|(1<<EPOUTEN)|(1<<EPINEN)
	movwf	UEP0
	movlw	(1<<EPHSHK)|(1<<EPCONDIS)|(1<<EPOUTEN)|(1<<EPINEN)
	movwf	UEP1
; initialize endpoint buffers and counts
	BANKSEL	BANKED_EP0OUT_ADRL
	movlw	low EP0OUT_BUF	; set endpoint 0 OUT address low
	movwf	BANKED_EP0OUT_ADRL
	movlw	low EP0IN_BUF	; set endpoint 0 IN address low
	movwf	BANKED_EP0IN_ADRL
	movlw	low EP1OUT_BUF	; set endpoint 1 OUT address low
	movwf	BANKED_EP1OUT_ADRL
	movlw	low EP1IN_BUF	; set endpoint 1 IN address low
	movwf	BANKED_EP1IN_ADRL
	movlw	EPBUF_ADRH	; set all ADRH values
	movwf	BANKED_EP0OUT_ADRH
	movwf	BANKED_EP0IN_ADRH
	movwf	BANKED_EP1OUT_ADRH
	movwf	BANKED_EP1IN_ADRH
	goto	arm_ep0_out

;;; Table Lookup
get_midi_x_size
        ;;      taken from Table 4-1 of USB MIDI spec v1.0:
	andlw 0x0f
        brw
        retlw   0 ;0: invalid/reserved event
        retlw   0 ;1: invalid/reserved event
        retlw   2 ;2: two-byte system common messages like MTC, Song Select, etc.
        retlw   3 ;3: three-byte system common messages like SPP, etc.
        retlw   3 ;4: SysEx starts or continues
        retlw   1 ;5: SysEx ends with following single byte
        retlw   2 ;6: SysEx ends with following two bytes
        retlw   3 ;7: SysEx ends with following three bytes
        retlw   3 ;8: Note Off
        retlw   3 ;9: Note On
        retlw   3 ;a: Poly-Key Press
        retlw   3 ;b: Control Change
        retlw   2 ;c: Program Change
        retlw   2 ;d: Channel Pressure
        retlw   3 ;e: PitchBend Change
        retlw   1 ;f: single byte like MIDI Clock/Start/Stop/Continue

;;; Includes
	include "midi.inc"


;;; Descriptors

; Place all the descriptors at the end of the bootloader region.
; This serves 2 purposes: 1) as long as the total length of all descriptors is
; less than 256, we can address them with an 8-bit pointer,
; and 2) the assembler will raise an error if space is exhausted.
	org	MAX_CODE_SIZE-ALL_DESCS_TOTAL_LEN
DESCRIPTOR_ADRH	equ	high $
DEVICE_DESCRIPTOR
	dt	DEVICE_DESC_LEN	; bLength
	dt	0x01		; bDescriptorType
	dt	0x00, 0x02	; bcdUSB (USB 2.0)
	dt	0x00		; bDeviceClass (custom device)
	dt	0x00		; bDeviceSubclass
	dt	0x00		; bDeviceProtocol
	dt	EP0_BUF_SIZE	; bMaxPacketSize0 (8 bytes)
	dt	low USB_VENDOR_ID,  high USB_VENDOR_ID	; idVendor
	dt	low USB_PRODUCT_ID, high USB_PRODUCT_ID	; idProduct
	dt	0x01, 0x00	; bcdDevice (1) release
	dt	0x01		; iManufacturer
	dt	0x02		; iProduct
	dt	0x03		; iSerialNumber
	dt	0x01		; bNumConfigurations

CONFIGURATION_DESCRIPTOR
	dt	0x09		; bLength
	dt	0x02		; bDescriptorType
	dt	CONFIG_DESC_TOTAL_LEN, 0x00	; wTotalLength
	dt	0x02		; bNumInterfaces
	dt	0x01		; bConfigurationValue
	dt	0x00		; iConfiguration
	dt	0x80		; bmAttributes
	dt	0x32		; bMaxPower

INTERFACE_DESCRIPTOR_0
	dt	0x09		; bLength of Interface Desc
	dt	0x04		; bDescriptorType (INTERFACE)
	dt	0x00		; bInterfaceNumber
	dt	0x00		; bAlternateSetting
CONFIGURATION_0_CONSTANT
	dt	0x00		; bNumEndpoints
CONFIGURATION_1_CONSTANT
	dt	0x01		; bInterfaceClass ( 1=Audio )
	dt	0x01		; bInterfaceSubclass (1= Control Device )
	dt	0x00		; bInterfaceProtocol (V.25ter, common AT commands)
	dt	0x00		; iInterface

	if (CONFIGURATION_0_CONSTANT>>8) != (CONFIGURATION_1_CONSTANT>>8)
	error "CONSTANT_0 and CONSTANT_1 must be in the same 256-word region"
	endif

   	; MIDI Adapter Class-specific Audio Control (AC) Interface Descriptor
    	dt	0x09       	; bLength
    	dt	0x24       	; bDescriptorType - CS_INTERFACE
    	dt	0x01       	; bDescriptorSubtype - HEADER
    	dt	0x00,0x01  	; bcdADC
    	dt	0x09,0x00  	; wTotalLength
    	dt	0x01       	; bInCollection
    	dt	0x01       	; baInterfaceNr(1)

    	; MIDI Adapter Standard MIDI Streaming (MS) Interface Descriptor
    	dt	0x09       	; bLength
    	dt	0x04       	; bDescriptorType
    	dt	0x01       	; bInterfaceNumber
    	dt	0x00       	; bAlternateSetting
    	dt	0x02       	; bNumEndpoints
    	dt	0x01       	; bInterfaceClass (Audio)
    	dt	0x03       	; bInterfaceSubclass (MIDI Streaming)
    	dt	0x00       	; bInterfaceProtocol
    	dt	0x00       	; iInterface

	; MIDI Adapter Class-specific MIDI Streaming (MS) Interface Descriptor
    	dt	0x07		; bLength
    	dt	0x24		; bDescriptorType - CS_INTERFACE
    	dt	0x01		; bDescriptorSubtype - MS_HEADER
    	dt	0x00,0x01  	; BcdADC
    	dt	0x41,0x00  	; wTotalLength ?? LABRAT - 65 vs 81

    	;  MIDI Adapter MIDI IN Jack Descriptor (Embedded)
    	dt	0x06		; bLength
    	dt	0x24		; bDescriptorType - CS_INTERFACE
    	dt	0x02		; bDescriptorSubtype - MIDI_IN_JACK
    	dt	0x01		; bJackType - EMBEDDED
    	dt	0x01		; bJackID ?? LABRAT - 2 for Cheap USB Device
    	dt	0x00		; iJack

    	;  MIDI Adapter MIDI IN Jack Descriptor (External)
    	dt	0x06		; bLength
    	dt	0x24		; bDescriptorType - CS_INTERFACE
    	dt	0x02		; bDescriptorSubtype - MIDI_IN_JACK
    	dt	0x02		; bJackType - EXTERNAL
    	dt	0x02		; bJackID ?? LABRAT - 6 for Cheap USB Device
    	dt	0x00		; iJack

    	;  MIDI Adapter MIDI OUT Jack Descriptor (Embedded)
    	dt	0x09		; bLength
    	dt	0x24		; bDescriptorType - CS_INTERFACE
    	dt	0x03		; bDescriptorSubtype - MIDI_OUT_JACK
    	dt	0x01		; bJackType - EMBEDDED
    	dt	0x03		; bJackID ?? LABRAT - 7 for Cheap USB Device
    	dt	0x01		; bNrInputPins
    	dt	0x02		; BaSourceID(1) ?? LABRAT - 6 for Cheap USB Device
    	dt	0x01		; BaSourcePin(1)
    	dt	0x00		; iJack

    	;  MIDI Adapter MIDI OUT Jack Descriptor (External)
    	dt	0x09		; bLength
    	dt	0x24		; bDescriptorType - CS_INTERFACE
    	dt	0x03		; bDescriptorSubtype - MIDI_OUT_JACK
    	dt	0x02		; bJackType - EXTERNAL
    	dt	0x04		; bJackID
    	dt	0x01		; bNrInputPins
    	dt	0x01		; BaSourceID(1) ?? LABRAT - 2 for Cheap USB Device
    	dt	0x01		; BaSourcePin(1)
    	dt	0x00		; iJack

ENDPOINT_DESCRIPTOR_1_OUT
    	;  MIDI Adapter Standard Bulk OUT Endpoint Descriptor
    	dt	0x09		; bLength
    	dt	0x05		; bDescriptorType (ENDPOINT)
    	dt	0x01		; bEndpointAddress - OUT
    	dt	0x02		; bmAttributes (transfer Type BULK)
	dt	low EP1_OUT_BUF_SIZE, 0x00	; wMaxPacketSize (64)
    	dt	0x00   		; bInterval
    	dt	0x00		; bRefresh
    	dt	0x00     	; bSynchAddress

    	; MIDI Adapter Class-specific Bulk OUT Endpoint Descriptor
    	dt	0x05      	; bLength
    	dt	0x25	 	; bDescriptorType - CS_ENDPOINT
    	dt	0x01   		; bDescriptorSubtype - MS_GENERAL
    	dt	0x01      	; bNumEmbMIDIJack
    	dt	0x01      	; BaAssocJackID(1)

ENDPOINT_DESCRIPTOR_1_IN
    	; MIDI Adapter Standard Bulk IN Endpoint Descriptor
    	dt	0x09      	; bLength
    	dt	0x05     	; bDescriptorType (ENDPOINT)
    	dt	0x81      	; bEndpointAddress - IN
    	dt	0x02     	; bmAttributes (transfer Type BULK)
    	dt	low EP1_IN_BUF_SIZE,0x00 	; wMaxPacketSize (64)
    	dt	0x00      	; bInterval
    	dt	0x00     	; bRefresh
    	dt	0x00 		; bSynchAddress

    	; MIDI Adapter Class-specific Bulk IN Endpoint Descriptor
    	dt	0x05 		; bLength
    	dt	0x25 		; bDescriptorType - CS_ENDPOINT
    	dt	0x01 		; bDescriptorSubtype - MS_GENERAL
    	dt	0x01 		; bNumEmbMIDIJack
    	dt	0x03 		; BaAssocJackID(1)

; extract nibbles from serial number
SN1	equ	(SERIAL_NUMBER>>12) & 0xF
SN2	equ	(SERIAL_NUMBER>>8) & 0xF
SN3	equ	(SERIAL_NUMBER>>4) & 0xF
SN4	equ	SERIAL_NUMBER & 0xF

sd001
SERIAL_NUMBER_STRING_DESCRIPTOR
	dt	SERIAL_NUM_DESC_LEN	; bLength
	dt	0x03		; bDescriptorType (STRING)
	dt	'0'+SN1+((SN1>9)*7), 0x00	; convert hex digits to ASCII
	dt	'0'+SN2+((SN2>9)*7), 0x00
	dt	'0'+SN3+((SN3>9)*7), 0x00
	dt	'0'+SN4+((SN4>9)*7), 0x00

; String Descriptor 0 contains list of all supported languages
sd000
	dt	0x04			; sizeof(SD000)
	dt	0x03			; DSC_STR
	dt	0x09, 0x04		;

; Using String 1 for Manufactured String Index
IMFG_SIZE 	equ	0x18
sd003
	dt	IMFG_SIZE ; sizeof( SD003)
	dt	0x03
	dt	'L',0,'a',0,'b',0,'r',0,'a',0,'t',0,' ',0
	dt	'L',0,'a',0,'b',0,'s',0

IPROD_SIZE 	equ	0x1E
sd002
	dt 	IPROD_SIZE ; sizeof( SD002)
	dt	0x03
	dt 	'Y',0,'A',0,'M',0,'A',0,'*',0,' ',0,'(',0,'C',0,')',0,' ',0
	dt	'2',0,'0',0,'2',0,'6',0


; Raise an error if the descriptors aren't properly aligned. (This means you
; changed the descriptors without updating the definition of ALL_DESCS_TOTAL_LEN.)
	if $!=MAX_CODE_SIZE
	error "Descriptors must be aligned with the end of the bootloader region"
	endif

	end

