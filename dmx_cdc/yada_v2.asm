; vim:noet:sw=8:ts=8:ai:syn=pic
; asmsyntax=pic
;
; Based on the USB 512-Word CDC Bootloader for PIC16(L)F1454/5/9
; Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
; v1.0, February 12, 2015
; Released under a 3-clause BSD license: see the accompanying LICENSE file.
;
; This is the APPLICATION that works in conjunction with the USB BOOTLOADER.
; It has been designd to share (what it can) from the BOOTLOADER code space. 
; Developed by LabRat  (Andrew Williams) as part of the  YADA2 project. 
; 
; Bootloader can be invoked by pressing the reset/prog button. 
;
; Protocol for talking to the firmware:
;     DMX_PAYLOAD          (0x00) [PAYLOAD]
;     ADMIN_SET_MODE       (0x01) [MODE_DMX, MODE_ADMIN, MODE_PASSTHROUGH]
;     ADMIN_PASS_THROUGH   (0x01) [PAYLOAD]
;     ADMIN_PROG_SERIAL    (0x02) [ 4 digit Serial No. ]
;     ADMIN_RESET          (0x03) [0x42 <-- Magic request byte]
;
;
; Code notes:
; - Labels that do not begin with an underscore can be called as functions.
;   Labels that begin with an underscore are not safe to call, they should only
;   be reached via goto.
;
; - FSR0L, FSR0H, FSR1L, and FSR1H are used as temporary registers in several
;   places, e.g. as loop counters. They're accessible regardless of the current
;   bank, and automatically saved/restored on interrupt. Neato!
;
; - As much stuff as possible is packed into bank 0 of RAM. This includes the
;   buffer descriptors, bootloader state, endpoint 0 OUT and IN buffers,
;   the endpoint 1 IN buffer (only a single byte is used), and the beginning of
;   the 64-byte endpoint 1 OUT buffer.
;
; - Notification endpoint 2 is enabled, but never used. The endpoint 2 IN
;   buffer descriptor is left uninitialized. The endpoint 2 OUT buffer
;   descriptor is used as 4 bytes of RAM.
;
; - The programming protocol is described in the 'usb16f1prog' script. It is
;   very minimal, but does provide checksum verification. Writing the ID words
;   (0x8000-8003) is not supported at this time, and writing the configuration
;   words is not possible via self-programming.

; Use this only for debugging!
; For more info, see log_macros.inc and log.asm.
LOGGING_ENABLED		equ	0
USE_BOOTLOADER_CDC	equ 	1
USB_INTERRUPTS		equ	1


	radix dec
	list n=0,st=off
	include "p16f1455.inc"
	nolist
	include "macros.inc"
	include "bdt.inc"
	include "usb.inc"
	include "protocol_constants.inc"
	include "log_macros.inc"
        include "import_list.inc"
	list
	errorlevel -302



;;; Constants and varaiable addresses
SERIAL_NUMBER_DIGIT_CNT	equ	4
	ifndef SERIAL_NUMBER
	variable SERIAL_NUMBER=0	; Why doesnt 'equ' work here? Go figure
	endif

#define BUTTON_PORT PORTA
#define BUTTON	PORTA,3

#define LED_PWR  LATA,4
#define LED_USB  LATC,3
#define LED_DMX  LATC,2

#define LED_PWR_DIR TRISA,4
#define LED_USB_DIR TRISC,3
#define LED_DMX_DIR TRISC,2

DEVICE_DESC_LEN		equ	18	; device descriptor length
CONFIG_DESC_TOTAL_LEN	equ	67	; total length of configuration descriptor and sub-descriptors
SERIAL_NUM_DESC_LEN	equ	2+(SERIAL_NUMBER_DIGIT_CNT*2)
ALL_DESCS_TOTAL_LEN	equ	DEVICE_DESC_LEN+CONFIG_DESC_TOTAL_LEN+SERIAL_NUM_DESC_LEN

EP0_BUF_SIZE 		equ	8	; endpoint 0 buffer size
EP1_OUT_BUF_SIZE	equ	64	; endpoint 1 OUT (CDC data) buffer size
EP1_IN_BUF_SIZE		equ	1	; endpoint 1 IN (CDC data) buffer size (only need 1 byte to return status codes)

; Since we're only using 5 endpoints, use the BDT area for buffers,
; and use the 4 bytes normally occupied by the EP2 OUT buffer descriptor for variables.
USB_STATE		equ	BANKED_EP2OUT+0
EP0_DATA_IN_PTR		equ	BANKED_EP2OUT+1	; pointer to descriptor to be sent (low byte only)
EP0_DATA_IN_COUNT	equ	BANKED_EP2OUT+2	; remaining bytes to be sent
APP_POWER_CONFIG	equ	BANKED_EP2OUT+3	; application power config byte
EP0OUT_BUF		equ	EP3OUT
BANKED_EP0OUT_BUF	equ	BANKED_EP3OUT	; buffers go immediately after EP2 IN's buffer descriptor
EP0IN_BUF		equ	EP0OUT_BUF+EP0_BUF_SIZE
BANKED_EP0IN_BUF	equ	BANKED_EP0OUT_BUF+EP0_BUF_SIZE

; Use another byte to store the checksum we use to verify writes
EXTRA_VARS_LEN		equ	1
EXPECTED_CHECKSUM	equ	BANKED_EP0IN_BUF+EP0_BUF_SIZE	; for saving expected checksum

EP1IN_BUF		equ	EP0IN_BUF+EP0_BUF_SIZE+EXTRA_VARS_LEN
BANKED_EP1IN_BUF	equ	BANKED_EP0IN_BUF+EP0_BUF_SIZE+EXTRA_VARS_LEN

EP1OUT_BUF		equ	EP1IN_BUF+EP1_IN_BUF_SIZE	; only use 1 byte for EP1 IN
BANKED_EP1OUT_BUF	equ	BANKED_EP1IN_BUF+EP1_IN_BUF_SIZE

; High byte of all endpoint buffers.
EPBUF_ADRH		equ	(EP0OUT_BUF>>8)
	if ((EP0IN_BUF>>8) != (EP0OUT_BUF>>8)) || ((EP1OUT_BUF>>8) != (EP0OUT_BUF>>8)) || ((EP1IN_BUF>>8) != (EP0OUT_BUF>>8))
	error "Endpoint buffers must be in the same 256-word region"
	endif

; Total length of all RAM (variables, buffers, BDT entries) used by the bootloader,
USED_RAM_LEN		equ	EP1OUT_BUF+EP1_OUT_BUF_SIZE-BDT_START

DmxUniverse 		equ 	0x2200

;--------------------------------------------------------
; global declarations
;--------------------------------------------------------


; - - - - - - - - - - - - - - 
; API from bootloader
; - - - - - - - - - - - - - - 
 org APP_ENTRY_POINT
  pagesel _app_main
  goto    _app_main

 org APP_CONFIG
  pagesel _app_config
  goto    _app_config ; must end with 'retlw' instruction

 org APP_INTERRUPT
  pagesel _app_interrupt
  call    _app_interrupt
  retfie

; - - - - - - - - - - - - - - 

; code
_app_config
        retlw 0x0a 


_app_interrupt
	BANKSEL PIE1
	btfss	PIE1,TXIE   ; Are we IRQ enabled 
	goto	_dmx_irq_done
	BANKSEL PIR1
	btfss	PIR1,TXIF   ; Did the TXIF interrupt go off?
	goto	_dmx_irq_done
	bcf	PIR1,TXIF   ; 
	call	DmxIrqHandler
_dmx_irq_done
	if USB_INTERRUPTS
	else
	return
	endif

usb_event_handler
	banksel	UIR
; USB reset?
	btfss	UIR,URSTIF
	goto	_utrans		; not a reset? just start servicing transactions
	lcall	usb_init	; if so, reset the USB interface (clears interrupts)
	pagesel _app_interrupt
	if USB_INTERRUPTS
	banksel	PIE2
	bsf	PIE2,USBIE	; reenable USB interrupts
	endif
	banksel	UIR
	bcf	UIR,URSTIF	; clear the flag
; service transactions
_utrans	
        banksel	UIR
	btfss	UIR,TRNIF
	goto	_usdone
	movfw	USTAT		; stash the status in a temp register
	movwf	FSR1H
	bcf	UIR,TRNIF	; clear flag and advance USTAT fifo
	banksel	BANKED_EP0OUT_STAT
	andlw	USTAT_ENDP_MASK ; check endpoint number
	bnz	_ucdc		; if not endpoint 0, it's a CDC message
	lcall	usb_service_ep0	; handle the control message
	pagesel _app_interrupt
	goto	_utrans
; clear USB interrupt
_usdone	
  	banksel	PIR2
	bcf	PIR2,USBIF
	return			; Bootloader frames calls this as standar CALL (!retfie)
_ucdc	
	call	usb_service_cdc	; USTAT value is still in FSR1H
	goto	_utrans



;;; Idle loop. In bootloader mode, the MCU just spins here, and all USB
;;; communication is interrupt-driven.
;;; This snippet is deliberately located within the first 256 words of program
;;; memory, so we can easily check in the interrupt handler if the interrupt
;;; occurred while executing application code or bootloader code.
;;; (TOSH will be 0x00 when executing bootloader code, i.e. this snippet)
bootloader_main_loop
	bsf	INTCON,PEIE
	bsf	INTCON,GIE	; enable interrupts (TMR interrupts)

	BANKSEL LATA
	bcf	LED_PWR  ; GREEN only
	bsf	LED_USB
	bsf	LED_DMX
_loop
	if USB_INTERRUPTS
	else
	  call 	usb_event_handler ; Polling to check for USB events
	endif

	if LOGGING_ENABLED
	; Print any pending characters in the log
	  call	log_service
	endif

	call	DmxTransmit ; Polling check for DMX frame

	BANKSEL BUTTON_PORT
	btfss	BUTTON  ; Is the button pressed?
	RESET

	BANKSEL PIR1 ; BANK 0
	btfss	PIR1,TMR1IF
	goto	_main_no_t1_event
	bcf	PIR1,TMR1IF
	movlw	0xec
	addwf	TMR1H,F
	clrf	TMR1L

	BANKSEL LATA
	bcf	LED_USB
	bcf	LED_DMX
_main_no_t1_event
	goto	_loop


; arms endpoint 1 IN, toggling DTS if W=(1<<DTS)
arm_ep1_in_old
	clrf	BANKED_EP1IN_CNT	; next packet will have 0 length (unless another OUT is received)
	andwf	BANKED_EP1IN_STAT,f	; clear all bits (except DTS if bit is set in W)
	xorwf	BANKED_EP1IN_STAT,f	; update data toggle (if bit is set in W)
	bsf	BANKED_EP1IN_STAT,UOWN
	return


;;; Services a transaction on one of the CDC endpoints.
;;; arguments:	USTAT value in FSR1H
;;;		BSR=0
;;; returns:	none
;;; clobbers:	W, FSR0, FSR1
usb_service_cdc
	movlw	(1<<DTS)
	retbfs	FSR1H,ENDP1		; ignore endpoint 2
	bbfs	FSR1H,DIR,arm_ep1_in_old; if endpoint 1 IN, rearm buffer
	movf	BANKED_EP1OUT_CNT,f	; test for a zero-length packet
	bz	arm_ep1_out		; (just ignore them and rearm the OUT buffer)
	bcf	BANKED_EP1IN_STAT,UOWN
	call	bootloader_exec_cmd	; execute command; status returned in W
	banksel	BANKED_EP1IN_BUF
	movwf	BANKED_EP1IN_BUF	; copy status to IN buffer
	movlw	1
	movwf	BANKED_EP1IN_CNT	; output byte count is 1
	bsf	BANKED_EP1IN_STAT,UOWN
	; fall through to arm_ep1_out

arm_ep1_out
	movlw	EP1_OUT_BUF_SIZE	; set CNT
	movwf	BANKED_EP1OUT_CNT
	clrf	BANKED_EP1OUT_STAT	; ignore data toggle
	bsf	BANKED_EP1OUT_STAT,UOWN	; rearm OUT buffer
	return


;;; LABRAT says - this is where our Command handler will go. 
;;; Protocol support goes here... 

;;; Executes a bootloader command.
;;; arguments:	command payload in EP1 OUT buffer
;;; 		BSR=0
;;; returns:	status code in W
;;; clobbers:	W, BSR, FSR0, FSR1
bootloader_exec_cmd
	bcf	LED_USB
	logch	'C',0
        retlw	BSTAT_OK ; Quit with "SUCCESS" add more code in time

; The Bootloader supports 3 messages, and uses the packet length packet length
; to denote which is which.

	movlw	BCMD_SET_PARAMS_LEN ; SIZE=4 ADRL, ADRH, CKSUM, ERASE
	subwf	BANKED_EP1OUT_CNT,w
	bz	_bootloader_set_params
	movlw	BCMD_WRITE_LEN      ; SIZE=64 
	subwf	BANKED_EP1OUT_CNT,w
	bz	_bootloader_write
	movlw	BCMD_RESET_LEN      ; SIZE=1 'R'  (invoke a reset)
	subwf	BANKED_EP1OUT_CNT,w
	bz	_bootloader_reset
	retlw	BSTAT_INVALID_COMMAND

; Resets the device if the received byte matches the reset character ('R').
_bootloader_reset
	movlw	BCMD_RESET_CHAR
	subwf	BANKED_EP1OUT_BUF,w	; check received character
	skpz
	retlw	BSTAT_INVALID_COMMAND
; command is valid, reset the device
	BANKSEL PORTA
	reset

; Sets the write address, expected checksum of the next 32 words,
; and erases the row at that address if the last byte of the command matches
; the "erase" character.
; BSR=0
_bootloader_set_params
	movfw	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_CKSUM	; expected checksum
	movwf	EXPECTED_CHECKSUM			; save for verification during write command
	movfw	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_ERASE
	movwf	FSR1L	; temp
	movfw	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_ADRL	; address lower bits
	movwf	FSR1H	; temp
	movfw	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_ADRH	; address upper bits 
	banksel	PMADRH
	movwf	PMADRH
	movfw	FSR1H	; bring lower bits out of temp
	movwf	PMADRL
; do we need to erase?
	movlw	BCMD_ERASE_CHAR
	subwf	FSR1L,w
	skpz
	retlw	BSTAT_OK	; if no reset command is given, return OK

; Erases the row of flash in PMADRH:PMADRL.
; BSR=3
_bootloader_erase
	movlw	(1<<FREE)|(1<<WREN)	; enable write and erase to program memory
	movwf	PMCON1
	lcall	flash_unlock		; stalls until erase finishes
        pagesel _bootloader_erase
_wdone	bcf	PMCON1,WREN		; clear write enable flag
	retlw	BSTAT_OK

; Verifies that the checksum of the 32 words (64 bytes) in the EP1 OUT buffer
; matches the previously sent value. If so, the 32 bytes are then written to
; flash memory at the address in PMADRH:PMADRL. (set by a prior command)
; BSR=0
_bootloader_write
; The expected checksum is the two's complement of the sum of the bytes.
; If the data is valid, we can add the checksum to the sum of the bytes and
; the result will be 0. We initialize a temporary register with the expected
; checksum, and then add each byte to it as it's processed.
; If the value in the temp register is 0 after all 64 bytes have been copied
; to the write latches, proceed with the write.
	movfw	EXPECTED_CHECKSUM
	movwf	FSR1L			; use a temp for the running checksum
	ldfsr0d	EP1OUT_BUF		; set up read pointer
	movlw	(1<<LWLO)|(1<<WREN)	; write to latches only
	banksel	PMCON1
	movwf	PMCON1
; simultaneously compute the checksum of the 32 words and copy them to the
; write latches
	movlw	32			; number of words to write minus 1
	movwf	FSR1H			; used for loop count
_wloop	moviw	FSR0++			; load lower byte
	addwf	FSR1L,f			; add lower byte to checksum
	movwf	PMDATL			; copy to write latch
	moviw	FSR0++			; load upper byte
	addwf	FSR1L,f			; add upper byte to checksum
	movwf	PMDATH			; copy to write latch
; after writing the 32nd word to PMDATH:PMDATL, don't execute the unlock sequence
; or advance the address pointer!
	decf	FSR1H,f			; decrement loop count
	bz	_wcksum			; if 0, we're done writing to the latches
; still have more words to go
	lcall	flash_unlock		; execute unlock sequence
        pagesel _bootloader_write
	incf	PMADRL,f		; increment write address
	goto	_wloop
; verify the checksum
_wcksum	clrf	PMCON1
	tstf	FSR1L
	skpz
	retlw	BSTAT_INVALID_CHECKSUM	; if there's a mismatch, abort the write
; checksum is valid, write the data
	bsf	PMCON1,WREN
	lcall	flash_unlock		; stalls until write finishes
        pagesel _wcksum
; verify the write: compare each byte in the buffer to its counterpart that
; was just written to flash.
; we do this backwards so we don't waste instructions resetting the pointers.
; (note: PMADRH:PMADRL is already pointing at the last written word, but FSR0
; is pointing to one byte past the end of the buffer)
	clrf	PMCON1			; clear write enable
	bsf	FSR1H,5			; set loop count to 32 (just need to set one bit because it's already 0)
_vloop	bsf	PMCON1,RD		; read word from flash
	nop				; 2 required nops
	nop
	moviw	--FSR0			; get high byte of expected word
	subwf	PMDATH,w		; compare with high byte written to flash
	skpz
	retlw	BSTAT_VERIFY_FAILED
	moviw	--FSR0			; get low byte of expected word
	subwf	PMDATL,w		; compare with low byte written to flash
	skpz
	retlw	BSTAT_VERIFY_FAILED
	decf	PMADRL,f		; decrement read address
	decfsz	FSR1H,f			; decrement loop count
	goto	_vloop
	retlw	BSTAT_OK


;;; Main function
_app_main
	banksel TRISA ; BANK 1
	bcf 	LED_PWR_DIR
	bcf 	LED_USB_DIR
	bcf 	LED_DMX_DIR 

        bcf	TRISC,4 ; EUSART - TX
	bsf	TRISC,5 ; EUSART - RX 

	banksel LATA ; BANK 2
	bcf	LED_PWR ; GREEN ; Turn OFF PWR LED
	bcf	LED_DMX ; AMBER ; Turn ON DMX LED
	bcf	LED_USB ; BLUE  ; Turn OFF USB LED

	if LOGGING_ENABLED
		call	uart_init
; Print a power-on character
		call	log_init
		logch	":",LOG_NEWLINE
		logch	':',0
	endif

	call	DmxSetup

	BANKSEL ANSELA ; BANK 3
	clrf	ANSELA

	BANKSEL T1CON
	; Initialize timer1
	movlw	(1 << TMR1ON)
	movwf	T1CON

; Print a power-on character
	logch	'M',0

; Initialize USB
	lcall	usb_init
        pagesel _app_main

; Attach to the bus (could be a subroutine, but inlining it saves 2 instructions)
_usb_attach
	logch	'A',0
	banksel	UCON		; reset UCON
	clrf	UCON
	if USB_INTERRUPTS
	banksel	PIE2
	bsf	PIE2,USBIE	; enable USB interrupts
	bsf	INTCON,PEIE
	endif
	banksel	UCON
_usben	
        bsf	UCON,USBEN	; enable USB module and wait until ready
	btfss	UCON,USBEN
	goto	_usben
	logch	'!',0

; Enable interrupts and enter an idle loop
; (Loop code is located at the top of the file, in the first 256 words of
; program memory)
	goto	bootloader_main_loop


;;; Includes
	if LOGGING_ENABLED
	include "log.asm"
	endif

	include "dmx.inc"

	end
