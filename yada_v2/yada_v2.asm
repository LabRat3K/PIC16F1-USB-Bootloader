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
;
;
;
; Build flag - re-use CDC code from the bootloader
USE_BOOTLOADER_CDC	equ 0

; Build flags - use interrupts for USB code (vs polling)
USB_INTERRUPTS          equ 1


; Should come from the import_list.inc
;BOOTLOADER_SIZE	equ	0x200

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
;; -----------------------------
;; LED Definitions
;; -----------------------------

#define BUTTON_PORT PORTA
#define BUTTON  PORTA,3

#define LED_PWR  LATA,4
#define LED_USB  LATC,3
#define LED_DMX  LATC,2

#define LED_PWR_DIR TRISA,4
#define LED_USB_DIR TRISC,3
#define LED_DMX_DIR TRISC,2

#define LED_MASK_USB 0x08
#define LED_MASK_DMX 0x04

;; ----------------------------
;; Code Space Sizes
;; ----------------------------

#define MAX_CODE_SIZE    ( 0x1F80 )	; Leave space for HIGH availability FLASH
#define HIGH_AVAIL_FLASH ( 0x1F80 )	; Address for HIGH availability FLASH

; -----------------------------------------------------------------------
; LabRat - updated with VID/PID assigned to YADA2 project by Microchip
;          THANK YOU MICROCHIP!! You ROCK!
; -----------------------------------------------------------------------
USB_VENDOR_ID          equ     0x04D8
USB_PRODUCT_ID         equ     0xEBC2

; -------------------------------
; Descriptor Lengths & Locations
; -------------------------------
DEVICE_DESC_LEN		equ	18	; device descriptor length
CONFIG_DESC_TOTAL_LEN	equ	32	; total length of configuration descriptor
SERIAL_NUM_DESC_LEN	equ	2+(SERIAL_NUMBER_DIGIT_CNT*2)
STRINGS_DESC_LEN	equ	0x3A
ALL_DESCS_TOTAL_LEN	equ	DEVICE_DESC_LEN+CONFIG_DESC_TOTAL_LEN+SERIAL_NUM_DESC_LEN+STRINGS_DESC_LEN

EP0_BUF_SIZE 		equ	16	; endpoint 0 buffer size
EP1_OUT_BUF_SIZE	equ	64	; endpoint 1 OUT (CDC data) buffer size
EP1_IN_BUF_SIZE		equ	16	; endpoint 1 IN (CDC data) buffer size 

; Since we're only using 5 endpoints, use the 4 bytes normally occupied by the 
; EP2 OUT buffer descriptor for variables,and the BDT area for buffers.
USB_STATE		equ	BANKED_EP2OUT+0
EP0_DATA_IN_PTR		equ	BANKED_EP2OUT+1	; pointer to descriptor to be sent (low byte only)
EP0_DATA_IN_COUNT	equ	BANKED_EP2OUT+2	; remaining bytes to be sent
APP_POWER_CONFIG	equ	BANKED_EP2OUT+3	; application power config byte

EP0OUT_BUF		equ	EP2IN
BANKED_EP0OUT_BUF	equ	BANKED_EP2IN	; buffers go immediately after EP2 OUT's buffer descriptor
EP0IN_BUF		equ	EP0OUT_BUF+EP0_BUF_SIZE
BANKED_EP0IN_BUF	equ	BANKED_EP0OUT_BUF+EP0_BUF_SIZE

; Use another byte to store the checksum we use to verify writes
EXTRA_VARS_LEN		equ	1
EXPECTED_CHECKSUM	equ	BANKED_EP0IN_BUF+EP0_BUF_SIZE	; for saving expected checksum

EP1IN_BUF		equ	EP0IN_BUF+EP0_BUF_SIZE+EXTRA_VARS_LEN
BANKED_EP1IN_BUF	equ	BANKED_EP0IN_BUF+EP0_BUF_SIZE+EXTRA_VARS_LEN

EP1OUT_BUF		equ	0x2050 	;EP1IN_BUF+EP1_IN_BUF_SIZE	; only use 1 byte for EP1 IN
BANKED_EP1OUT_BUF	equ	0xA0	;BANKED_EP1IN_BUF+EP1_IN_BUF_SIZE

; High byte of all endpoint buffers.
EPBUF_ADRH		equ	(EP0OUT_BUF>>8)

	if ((EP0IN_BUF>>8) != (EP0OUT_BUF>>8)) || ((EP1OUT_BUF>>8) != (EP0OUT_BUF>>8)) || ((EP1IN_BUF>>8) != (EP0OUT_BUF>>8))
		error "Endpoint buffers must be in the same 256-word region"
	endif

; Total length of all RAM (variables, buffers, BDT entries) used by the bootloader,
USED_RAM_LEN		equ	EP1OUT_BUF+EP1_OUT_BUF_SIZE-BDT_START

;; ----------------------------
;; RAM Block to Hold DMX Buffer
;; ----------------------------
DmxUniverse		equ	0x2200

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
        BANKSEL PIE1
        btfss   PIE1,TXIE   ; Are we IRQ enabled 
        goto    _dmx_irq_done

        BANKSEL PIR1
        btfss   PIR1,TXIF   ; Did the TXIF interrupt go off?
        goto    _dmx_irq_done

	PAGESEL	DmxIrqHandler
        call	DmxIrqHandler
        PAGESEL _app_interrupt

_dmx_irq_done
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
        bnz     _ucdc           ; if not endpoint 0, it's a CDC message
        lcall   usb_service_ep0 ; handle the control message
        PAGESEL _app_interrupt
        goto    _utrans
; clear USB interrupt
_usdone
        BANKSEL PIR2
        bcf     PIR2,USBIF
        return                  ; Bootloader frames calls this as standard CALL (!retfie)
_ucdc
        call    usb_service_cdc ; USTAT value is still in FSR1H
        goto    _utrans


; -------------------------------
; Main Loop - this is the inner 
; tight loop for the application
; -------------------------------
main_loop
	bsf	INTCON,PEIE
	bsf	INTCON,GIE	; enable interrupts

	BANKSEL LATA
	bcf	LED_PWR ; PWR LED ON 
	bsf	LED_USB ; USB LED OFF
	bsf	LED_DMX ; DMX LED OFF

_loop
	CLRWDT
	if USB_INTERRUPTS
	else
	  call	usb_event_handler ; Polling to check forUSB events
	endif

	PAGESEL	DmxTransmit
        call	DmxTransmit ; Polling check for DMX frame
	PAGESEL $

        BANKSEL BUTTON_PORT
        btfss   BUTTON  ; Is the button pressed?
	goto	_handle_reset

	goto	_loop

_handle_reset
	BANKSEL LATA
	bsf	LED_USB
	bcf	LED_PWR
	bsf	LED_DMX
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
	bz	_string_sd000  ;0x00
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
	movlw	low SERIAL_NUMBER_STRING_DESCRIPTOR
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
	call	cdc_init
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
	clrf	BANKED_EP0IN_CNT		; initialize buffer size to 0
	tstf	EP0_DATA_IN_COUNT		; do nothing if there are 0 bytes to send
	skpnz
	return
	movf	EP0_DATA_IN_PTR,W		; set up source pointer
	movwf	FSR0L
	movlw	DESCRIPTOR_ADRH|0x80
	movwf	FSR0H
	ldfsr1d	EP0IN_BUF			; set up destination pointer
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

;;; Initializes the buffers for the CDC endpoints (1 OUT, 1 IN, and 2 IN).
;;; arguments:	none
;;; returns:	none
;;; clobbers:	W, BSR=0
cdc_init
	BANKSEL	BANKED_EP1OUT_STAT
	call	arm_ep1_out
	; arm EP1 IN buffer, clearing data toggle bit
	clrw

; arms endpoint 1 IN, toggling DTS if W=(1<<DTS)
arm_ep1_in
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
	btfsc	FSR1H,ENDP1		; ignore endpoint 2
	return
	btfsc	FSR1H,DIR		; if endpoint 1 IN, rearm buffer
	goto	arm_ep1_in
	movf	BANKED_EP1OUT_CNT,f	; test for a zero-length packet
	bz	arm_ep1_out		; (just ignore them and rearm the OUT buffer)
	bcf	BANKED_EP1IN_STAT,UOWN
	call	_yada_cmd		; execute command; OUTPUT length returned in W
	BANKSEL BANKED_EP1IN_BUF
;	skpz				; no output - rearm the ep1 buffer
	movwf	BANKED_EP1IN_CNT	; output byte count as returned from call
	bsf	BANKED_EP1IN_STAT,UOWN
	; fall through to arm_ep1_out

arm_ep1_out
	movlw	EP1_OUT_BUF_SIZE		; set CNT
	movwf	BANKED_EP1OUT_CNT
	clrf	BANKED_EP1OUT_STAT		; ignore data toggle
	bsf	BANKED_EP1OUT_STAT,UOWN	; rearm OUT buffer
	return

;;; Executes a bootloader command.
;;; arguments:	command payload in EP1 OUT buffer
;;; 		BSR=0
;;; returns:	status code in W
;;; clobbers:	W, BSR, FSR0, FSR1
; A New Paradigm
;  BANKED_EP1OUT_BUFF[0]
;	0 - DMX data - if MODE = 0, apply to DmxUniverse
;	    Note: Fixed payload length - 32 bytes + 2 byte overhead
;           Reply: None - keep the bandwidth clear.
;	1 - Passthrough Data - if MODE =1 & Buffer Available - copy data 
;           Note: Variabe length data payload (1 to 63)
;           Reply <C1> - ACK
;	2 - Admin : <00> - Set Mode DMX : <SOF-Byte> 
;                          Reply <C2> <00> <00/01> - ACK/NACK
;	            <01> - Set Mode PassThrough : <SOF-Byte> 
;                          Reply <C2> <01> <00/01> - ACK/NACK
;                   <02> - Query Device <MAJ>.<MIN>, <DevIdH><DevIdL>
;                          Reply <C2> <02> <MAJ>.<MIN>, <DevIdH><DevIdL>
;                   <03> - Query Serial No.
;                          Reply <C2> <03> <MAJ>.<MIN>, <DevIdH><DevIdL>
;	            <04> - <B1><B2><B3><B4><CSUM> : Write DeviceInfo
;                          Reply <C2> <04> <00/01> - ACK/NACK
;	            <05> - <B1><B2><B3><B4><CSUM> : Write SerialNo.
;                          Reply <C2> <05> <00/01> - ACK/NACK
;       3- Reset Device
;
_yada_cmd
	BANKSEL BANKED_EP1OUT_BUF	; Is DMX frame?
	movlw	0x40
	subwf	BANKED_EP1OUT_BUF,W
	bz	_dmx_packet

	movlw	0x41			; Is PASSTHROUGH frame?
	subwf	BANKED_EP1OUT_BUF,W
	bz	_pass_through_packet
	
	movlw	0x42			; Is ADMIN frame?
	subwf	BANKED_EP1OUT_BUF,W
	bz	_admin_packet

	movlw	0x43 			; Is Reset Request?
	subwf	BANKED_EP1OUT_BUF,W
	bz	_yada_cmd_reset

	BANKSEL	BANKED_EP1IN_BUF
	movlw	BSTAT_INVALID_COMMAND
	movwf	BANKED_EP1IN_BUF	; copy status to IN buffer
	retlw	1
	

_admin_packet
	; 0x42 00 <SOF>
	movf	BANKED_EP1OUT_BUF+1,F	; Set Mode to DMX?
	bz	_admin_set_dmx

	; 0x42 01 <SOF> 
	movlw	0x01
	subwf	BANKED_EP1OUT_BUF+1,W	; Set Mode to Passthrough?
	bz	_admin_set_passthrough

	; 0x42 02
	movlw	0x02
	subwf	BANKED_EP1OUT_BUF+1,W	; Qry Device INFO
	bz	_admin_qry_device

	; 0x42 03
	movlw	0x03
	subwf	BANKED_EP1OUT_BUF+1,W	; Qry Device Serial No.
	bz	_admin_qry_serial

	; 0x42 04 <0xYY> <0xYY> <0xYY> <0xYY> 
	movlw	0x04
	subwf	BANKED_EP1OUT_BUF+1,W	; Set Device INFO
	bz	_admin_set_devinfo

	; 0x42 05 <0xYY> <0xYY> <0xYY> <0xYY> 
	movlw	0x05
	subwf	BANKED_EP1OUT_BUF+1,W	; Set SerialNo
	bz	_admin_set_serialno

	BANKSEL	BANKED_EP1IN_BUF
	movlw	BSTAT_INVALID_COMMAND
	movwf	BANKED_EP1IN_BUF	; copy status to IN buffer
	retlw	1

_admin_set_dmx
	movf	BANKED_EP1OUT_BUF+2,W
	movwf	DMX_SOF_BYTE
	bcf	YADA_STATUS,YADA_MODE_BIT	; Mode 0 = DMX
	BANKSEL	BANKED_EP1IN_BUF
	movlw	0xC2
	movwf	BANKED_EP1IN_BUF
	movlw	0x00
	movwf	BANKED_EP1IN_BUF+1
	movlw	BSTAT_OK
	movwf	BANKED_EP1IN_BUF+2	; copy status to IN buffer
	retlw	3

_admin_set_passthrough
	movf	BANKED_EP1OUT_BUF+2,W
	movwf	DMX_SOF_BYTE
	bsf	YADA_STATUS,YADA_MODE_BIT
	BANKSEL	BANKED_EP1IN_BUF
	movlw	0xC2
	movwf	BANKED_EP1IN_BUF
	movlw	0x01
	movwf	BANKED_EP1IN_BUF+1
	movlw	BSTAT_OK
	movwf	BANKED_EP1IN_BUF+2	; copy status to IN buffer
	retlw	3

_admin_qry_device
	BANKSEL BANKED_EP1IN_BUF ; 0x42 03
	movlw	0xC2
	movwf	BANKED_EP1IN_BUF
	movlw 	0x02
	movwf	BANKED_EP1IN_BUF+1
	movlw	0x00
	movwf	BANKED_EP1IN_BUF+2
	movlw	0x01
	movwf	BANKED_EP1IN_BUF+3
	movlw	0x02
	movwf	BANKED_EP1IN_BUF+4
	movlw	0x03
	movwf	BANKED_EP1IN_BUF+5
	retlw	0x06

_admin_qry_serial
	BANKSEL BANKED_EP1IN_BUF ; 0x42 03
	movlw	0xC2
	movwf	BANKED_EP1IN_BUF
	movlw 	0x03
	movwf	BANKED_EP1IN_BUF+1
	movlw	0x04
	movwf	BANKED_EP1IN_BUF+2
	movlw	0x05
	movwf	BANKED_EP1IN_BUF+3
	movlw	0x06
	movwf	BANKED_EP1IN_BUF+4
	movlw	0x07
	movwf	BANKED_EP1IN_BUF+5
	retlw	0x06

_admin_set_devinfo	;0x42 04
	BANKSEL	BANKED_EP1IN_BUF
	movlw	0xC2
	movwf	BANKED_EP1IN_BUF
	movlw	0x04
	movwf	BANKED_EP1IN_BUF+1
	movlw	BSTAT_INVALID_COMMAND
	movwf	BANKED_EP1IN_BUF+2	; copy status to IN buffer
	retlw	3

_admin_set_serialno	;0x42 05
	BANKSEL	BANKED_EP1IN_BUF
	movlw	0xC2
	movwf	BANKED_EP1IN_BUF
	movlw	0x05
	movwf	BANKED_EP1IN_BUF+1
	movlw	BSTAT_INVALID_COMMAND
	movwf	BANKED_EP1IN_BUF+2	; copy status to IN buffer
	retlw	3

#ifdef OLD_CODE
; Used Packet length to determine packet function
	movlw	0x22   ; Length of the DMX packet
	subwf	BANKED_EP1OUT_CNT,W
	bz	_dmx_packet

	movlw	BCMD_SET_PARAMS_LEN
	subwf	BANKED_EP1OUT_CNT,w
	bz	_flash_set_params

	movlw	BCMD_WRITE_LEN
	subwf	BANKED_EP1OUT_CNT,w
	bz	_flash_write

	movlw	BCMD_RESET_LEN
	subwf	BANKED_EP1OUT_CNT,w
	bz	_yada_cmd_reset

	BANKSEL	BANKED_EP1IN_BUF
	movlw	BSTAT_INVALID_COMMAND
	movwf	BANKED_EP1IN_BUF	; copy status to IN buffer
	retlw	1
#endif

_pass_through_packet	; 0x41
_dmx_packet		; 0x40
	movlw	high DmxUniverse
	movwf	FSR0H
	movlw	low DmxUniverse
	movwf	FSR0L
	BANKSEL BANKED_EP1OUT_BUF+1   ; If DMX & 1st packet of frame..
	movf	BANKED_EP1OUT_BUF+1,W ;    Jump to dmx_led_cnt
	bz	_dmx_led_cnt
_dmx_skip_loop
	addfsr	FSR0, 16	; Multiplication/Addition loop (32 x # )
	addfsr	FSR0, 16
	decfsz	WREG,W
	goto	_dmx_skip_loop
_dmx_copy_payload
	movlw	low EP1OUT_BUF+2
	movwf	FSR1L
	movlw	high EP1OUT_BUF
	movwf	FSR1H
	movlw	0x20
	; SNEAKY - use BANKED_EP1_OUT_BUF[0] (formerly 0x40) as a temp counter
	movwf	BANKED_EP1OUT_BUF
_dmx_copy_loop
	moviw	FSR1++
	movwi	FSR0++
	decfsz	BANKED_EP1OUT_BUF,F
	goto	_dmx_copy_loop
	retlw	0x00	; *** ZERO LENGTH REPLY ***

_dmx_led_cnt
	decfsz	USB_BLINK,F
	goto	_dmx_copy_payload
	movlw	44
	movwf	USB_BLINK
	movlw	LED_MASK_USB 
	BANKSEL	LATC
	xorwf	LATC,F
	BANKSEL BANKED_EP1OUT_BUF
	goto	_dmx_copy_payload

; Resets the device if the received byte matches the reset character.
_yada_cmd_reset
	BANKSEL	BANKED_EP1IN_BUF
	movlw	0xC3
	movwf	BANKED_EP1IN_BUF
	movlw	BSTAT_INVALID_COMMAND
	movwf	BANKED_EP1IN_BUF+1	; copy status to IN buffer
	
        BANKSEL BANKED_EP1OUT_BUF
	movlw	BCMD_RESET_CHAR
	subwf	BANKED_EP1OUT_BUF+1,w	; check received character
	skpz
	retlw	2 	; Length of reply 
; command is valid, reset the device
	lcall	_handle_reset

; Sets the write address, expected checksum of the next 32 words,
; and erases the row at that address if the last byte of the command matches
; the "erase" character.
; BSR=0
_flash_set_params
	BANKSEL	BANKED_EP1IN_BUF
	movlw	BSTAT_OK
	movwf	BANKED_EP1IN_BUF	; copy status to IN buffer

	BANKSEL BANKED_EP1OUT_BUF
	movf	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_CKSUM,W	; expected checksum
	movwf	EXPECTED_CHECKSUM			; save for verification during write command
	movf	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_ERASE,W
	movwf	FSR1L		; temp
	movf	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_ADRL,W	; address lower bits
	movwf	FSR1H		; temp
	movf	BANKED_EP1OUT_BUF+BCMD_SET_PARAMS_ADRH,W	; address upper bits 
	BANKSEL	PMADRH
	movwf	PMADRH
	movf	FSR1H,W		; bring lower bits out of temp
	movwf	PMADRL
; do we need to erase?
	movlw	BCMD_ERASE_CHAR
	subwf	FSR1L,w
	skpz
	retlw	1	; if no reset command is given, return OK

; Erases the row of flash in PMADRH:PMADRL.
; BSR=3
_bootloader_erase
	movlw	(1<<FREE)|(1<<WREN)	; enable write and erase to program memory
	movwf	PMCON1
	call	flash_unlock		; stalls until erase finishes
_wdone
	bcf	PMCON1,WREN		; clear write enable flag
	retlw	1	; lenght of OK packet

; Verifies that the checksum of the 32 words (64 bytes) in the EP1 OUT buffer
; matches the previously sent value. If so, the 32 bytes are then written to
; flash memory at the address in PMADRH:PMADRL. (set by a prior command)
; BSR=0
_flash_write
; The expected checksum is the two's complement of the sum of the bytes.
; If the data is valid, we can add the checksum to the sum of the bytes and
; the result will be 0. We initialize a temporary register with the expected
; checksum, and then add each byte to it as it's processed.
; If the value in the temp register is 0 after all 64 bytes have been copied
; to the write latches, proceed with the write.
	movf	EXPECTED_CHECKSUM,W
	movwf	FSR1L			; use a temp for the running checksum
	ldfsr0d	EP1OUT_BUF		; set up read pointer
	movlw	(1<<LWLO)|(1<<WREN)	; write to latches only
	BANKSEL	PMCON1
	movwf	PMCON1
; simultaneously compute the checksum of the 32 words and copy them to the
; write latches
	movlw	32			; number of words to write minus 1
	movwf	FSR1H			; used for loop count
_wloop
	moviw	FSR0++			; load lower byte
	addwf	FSR1L,f			; add lower byte to checksum
	movwf	PMDATL			; copy to write latch
	moviw	FSR0++			; load upper byte
	addwf	FSR1L,f			; add upper byte to checksum
	movwf	PMDATH			; copy to write latch
; after writing the 32nd word to PMDATH:PMDATL, don't execute the unlock sequence
; or advance the address pointer!
	decf	FSR1H,f			; decrement loop count
	bz		_wcksum			; if 0, we're done writing to the latches
; still have more words to go
	call	flash_unlock		; execute unlock sequence
	incf	PMADRL,f		; increment write address
	goto	_wloop
; verify the checksum
_wcksum	
	clrf	PMCON1
	tstf	FSR1L
	skpnz 	
	goto	_oksum
	BANKSEL	BANKED_EP1IN_BUF
	movlw	BSTAT_INVALID_CHECKSUM	; if there's a mismatch, abort the write
	movwf	BANKED_EP1IN_BUF	; copy status to IN buffer
	retlw	1	; return length of reply packet (1)
_oksum
; checksum is valid, write the data
	bsf	PMCON1,WREN
	call	flash_unlock		; stalls until write finishes
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
	skpnz
	goto	_verifyok
	BANKSEL	BANKED_EP1IN_BUF
	movlw	BSTAT_VERIFY_FAILED	; reply with verify failed message 
	movwf	BANKED_EP1IN_BUF	; copy status to IN buffer
	retlw	1	; return length of reply packet (1)
_verifyok
	decf	PMADRL,f		; decrement read address
	decfsz	FSR1H,f			; decrement loop count
	goto	_vloop
	retlw	1	; return length of OK packet (already configured)


;;; Executes the flash unlock sequence, performing an erase or write.
;;; arguments:	PMCON1 bits CFGS, LWLO, FREE and WREN set appropriately
;;;		BSR=3
;;; returns:	none
;;; clobbers:	W
flash_unlock
	movlw	0x55
	movwf	PMCON2
	movlw	0xAA
	movwf	PMCON2
	bsf	PMCON1,WR
	nop
	nop
ret	return

;;; Main function
; Now entering application code: initialize the USB interface and wait for commands.
_app_main
        BANKSEL TRISA ; BANK 1
        bcf     LED_PWR_DIR
        bcf     LED_USB_DIR
        bcf     LED_DMX_DIR

        bcf     TRISC,4 ; EUSART - TX
        bsf     TRISC,5 ; EUSART - RX 

        BANKSEL LATA ; BANK 2
        bcf     LED_PWR ; GREEN ; Turn ON  PWR LED
        bsf     LED_DMX ; AMBER ; Turn OFF DMX LED
        bsf     LED_USB ; BLUE  ; Turn OFF USB LED

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
	PAGESEL	DmxSetup
        call    DmxSetup
	PAGESEL $

	goto	main_loop


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
	;movlw	(1<<EPHSHK)|(1<<EPCONDIS)|(1<<EPINEN)
	;movwf	UEP2
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

;;; Includes
	include "dmx.inc"


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
	dt	0x01, 0x00	; bcdDevice (1) 
	dt	0x01		; iManufacturer
	dt	0x02		; iProduct
	dt	0x03		; iSerialNumber
	dt	0x01		; bNumConfigurations

CONFIGURATION_DESCRIPTOR
	dt	0x09		; bLength
	dt	0x02		; bDescriptorType
	dt	CONFIG_DESC_TOTAL_LEN, 0x00	; wTotalLength
	dt	0x01		; bNumInterfaces
	dt	0x01		; bConfigurationValue
	dt	0x00		; iConfiguration
	dt	0x80		; bmAttributes
	dt	0x32		; bMaxPower 

INTERFACE_DESCRIPTOR_0
	dt	0x09		; bLength
	dt	0x04		; bDescriptorType (INTERFACE)
	dt	0x00		; bInterfaceNumber
CONFIGURATION_0_CONSTANT
	dt	0x00		; bAlternateSetting
CONFIGURATION_1_CONSTANT
	dt	0x02		; bNumEndpoints
	dt	0x00		; bInterfaceClass (communication)
	dt	0x00		; bInterfaceSubclass (abstract control model)
	dt	0x00		; bInterfaceProtocol (V.25ter, common AT commands)
	dt	0x00		; iInterface

	if (CONFIGURATION_0_CONSTANT>>8) != (CONFIGURATION_1_CONSTANT>>8)
	error "CONSTANT_0 and CONSTANT_1 must be in the same 256-word region"
	endif

ENDPOINT_DESCRIPTOR_1_OUT
	dt	0x07		; bLength
	dt	0x05		; bDescriptorType (ENDPOINT)
	dt	0x01		; bEndpointAddress (1 OUT)
	dt	0x02		; bmAttributes (transfer type: bulk)
	dt	low EP1_OUT_BUF_SIZE, 0x00	; wMaxPacketSize (64)
	dt	0x00		; bInterval

ENDPOINT_DESCRIPTOR_1_IN
	dt	0x07		; bLength
	dt	0x05		; bDescriptorType (ENDPOINT)
	dt	0x81		; bEndpointAddress (1 IN)
	dt	0x02		; bmAttributes (transfer type: bulk)
	dt	low EP1_IN_BUF_SIZE, 0x00	; wMaxPacketSize (16)
	dt	0x00		; bInterval

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
	dt 	'Y',0,'A',0,'D',0,'A',0,'2',0,' ',0,'(',0,'C',0,')',0,' ',0
	dt	'2',0,'0',0,'2',0,'0',0


; Raise an error if the descriptors aren't properly aligned. (This means you
; changed the descriptors without updating the definition of ALL_DESCS_TOTAL_LEN.)
	if $!=MAX_CODE_SIZE
	error "Descriptors must be aligned with the end of the bootloader region"
	endif

	end

