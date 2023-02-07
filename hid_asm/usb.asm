; asmsyntax=pic
; File:   Usb.asm
; Hand coded by Andrew Williams, based on C source by
; Author: Szymon Roslowski
;
; Created on 13 October 2014, 17:46
; Tweaked Jan 23, 2023 by Andrew Williams
;
; Firmware framework for USB I/O on PIC 16F1455 (and siblings)
;
;

;LOGGING_ENABLED		equ 1

	radix dec
	list n=0,st=off
	include "p16f1455.inc"
	nolist
	include "bdt.inc"
	include "log_macros.inc"
	list
	errorlevel -302

;***********************/
;* Local Definitions   */
;***********************/
; Commands
#define GET_STATUS                  0x00
#define CLEAR_FEATURE               0x01
#define SET_FEATURE                 0x03
#define SET_ADDRESS                 0x05
#define GET_DESCRIPTOR              0x06
#define SET_DESCRIPTOR              0x07
#define GET_CONFIGURATION           0x08
#define SET_CONFIGURATION           0x09
#define GET_INTERFACE               0x0A
#define SET_INTERFACE               0x0B
#define SYNCH_FRAME                 0x0C

; Descriptor Types
#define DEVICE_DESCRIPTOR           0x01
#define CONFIGURATION_DESCRIPTOR    0x02
#define STRING_DESCRIPTOR           0x03
#define INTERFACE_DESCRIPTOR        0x04
#define ENDPOINT_DESCRIPTOR         0x05

; Class Descriptor Types
#define HID_DESCRIPTOR              0x21
#define REPORT_DESCRIPTOR           0x22
#define PHYSICAL_DESCRIPTOR         0x23

; HID Class specific requests
#define GET_REPORT                  0x01
#define GET_IDLE                    0x02
#define GET_PROTOCOL                0x03
#define SET_REPORT                  0x09
#define SET_IDLE                    0x0A
#define SET_PROTOCOL                0x0B

; Standard Feature Selectors
#define DEVICE_REMOTE_WAKEUP        0x01
#define ENDPOINT_HALT               0x00

; Device states (Chap 9.1.1)
#define STATE_DETACHED              0x00
#define STATE_ATTACHED              0x01
#define STATE_POWERED               0x02
#define STATE_DEFAULT               0x03
#define STATE_ADDRESS               0x04
#define STATE_CONFIGURED            0x05

; Interrupts
#define USB_URST                    0x01

; Control Transfer Stages - see USB spec chapter 5
#define STAGE_SETUP                 0x00 ; Start of a control transfer (followed by 0 or more data stages)
#define STAGE_DATA_OUT              0x01 ; Data from host to device
#define STAGE_DATA_IN               0x02 ; Data from device to host
#define STAGE_STATUS                0x03 ; Unused - if data I/O went ok, then back to Setup


#define E0SZ (8)

;/ Bit index of Status Bits
#define  status_remote_wakeup 	 (1)
#define  status_self_powered     (2)
#define  status_request_handled  (3)
#define  status_transfer_type    (4)

#define type_RAM (0x00)
#define type_ROM (1<<status_transfer_type)

 include "usb_descriptors.inc"


;***********************/
;* Local Variables     */
;***********************/
 CBLOCK 0x70            ; Global Variables
	StatusBits	; 0x70
	outPtrL		; 0x71 SRC & Destination pointers for in/out going buffers
	outPtrH         ; 0x72
	inPtrL          ; 0x73
	inPtrH          ; 0x74
	wCount		; 0x75 counter for copying in/out data
	DeviceState	; 0x76
	bufferSize	; 0x77 - working counter
	end_bank0_minor_vars:0
	reserved:7
 ENDC
 if end_bank0_minor_vars > 0x080
        error "Bank 0 Global  Variable Space overrun"
 endif

; Bank/Linear Shared Variables


; Banked Variables
 CBLOCK 0xA0
 	ControlTransferBuffer:E0SZ ; 8 Byte buffer
	SetupPacket: 9 		   ; 9 Byte buffer
	CtrlTransferStage	   ; Hold the current stage in a control transfer
	DeviceAddress
	CurrentConfiguration
	HidIdleRate
	HidProtocol	           ; [0] Boot Protocol [1] Report Protocol

	HIDRxBuffer:HID_REPORT_BYTE_COUNT
	HIDTxBuffer:HID_REPORT_BYTE_COUNT
	end_bank1_minor_vars:0
 ENDC
 if end_bank1_minor_vars > 0x0F0
        error "Bank 0 Global  Variable Space overrun"
 endif

#define SETUP_bmRequestType (SetupPacket)   ; D7: Direction, D6..5: Type, D4..0: Recipient
#define SETUP_bRequest      (SetupPacket+1) ; Specific request
#define SETUP_wValue0       (SetupPacket+2) ; LSB of wValue
#define SETUP_wValue1       (SetupPacket+3) ; MSB of wValue
#define SETUP_wIndex0       (SetupPacket+4) ; LSB of wIndex
#define SETUP_wIndex1       (SetupPacket+5) ; MSB of wIndex
#define SETUP_wLengthL      (SetupPacket+6)
#define SETUP_wLengthH      (SetupPacket+7) ; Number of bytes to transfer if there's a data stage
#define SETUP_extra	    	(SetupPacket+8) ; Fill out to same size as Endpoint 0 max buffer (E0SZ-7)


	org	0x400
;***********************/
;* Implementation      */
;***********************/
IsUsbDataAvailable:
	; Note: Demo only ever used EP1, so this is optimized down from "any" endpoint to only
	; checking if EP1 has something for us.
	; return 0 or the EP1Count
	BANKSEL BANKED_EP1OUT_STAT
	btfsc	BANKED_EP1OUT_STAT,UOWN
	retlw	0x00
    movf	BANKED_EP1OUT_CNT,W
	return

ReArmInterface:
	; As with above, this is optimized down re-arm EP1 only.
	; First check that SIE doesn't already have control of the BD
	BANKSEL BANKED_EP1OUT_STAT
	btfsc	BANKED_EP1OUT_STAT,UOWN
	return

	; Setup the buffer size
	movlw	HID_REPORT_BYTE_COUNT
	movwf	BANKED_EP1OUT_CNT

	; Setup the STAT flags (Data0/Data1 toggling)
 	movlw	(1<<DTSEN)
	btfss	BANKED_EP1OUT_STAT,DTS
	iorlw	(1<<DTS)
	movwf	BANKED_EP1OUT_STAT
	; Hand control of the BD to the SIE
	bsf		BANKED_EP1OUT_STAT,UOWN
	return

HIDSend:
	; As with above, this is optimized down to only SENDing for EP1
	; Check the SIE doesn't still have control of the BD
	BANKSEL BANKED_EP1IN_STAT
	btfsc	BANKED_EP1IN_STAT,UOWN
	return

	; Record the outgoing byte count
	movlw	HID_REPORT_BYTE_COUNT
	movwf	BANKED_EP1IN_CNT

	; Setup the STAT flags (Data0/Data1 toggling)
 	movlw	(1<<DTSEN)
	btfss	BANKED_EP1IN_STAT,DTS
	iorlw	(1<<DTS)
	movwf	BANKED_EP1IN_STAT
	bsf		BANKED_EP1IN_STAT,UOWN
	return

; After configuration is complete, this routine is called to initialize
; the endpoints (e.g., assign buffer addresses).
; Note: only using EP0 & EP1 so didn't create a loop
HIDInitEndpoints:
	BANKSEL UEP1
	movlw 	0x1E
	movwf 	UEP1

	BANKSEL BANKED_EP1OUT_STAT
	movlw 	HID_REPORT_BYTE_COUNT
	movwf 	BANKED_EP1OUT_CNT

	movlw 	low HIDRxBuffer
	movwf	BANKED_EP1OUT_ADRL
	movlw	high HIDRxBuffer
	movwf   BANKED_EP1OUT_ADRH

	movlw   (1<<DTSEN)
	movwf	BANKED_EP1OUT_STAT
	bsf		BANKED_EP1OUT_STAT,UOWN

	movlw	low HIDTxBuffer
	movwf	BANKED_EP1IN_ADRL
	movlw	high HIDTxBuffer
	movwf	BANKED_EP1IN_ADRH
	movlw   (1<<DTS);LabRat .. found this didn't match the C code .. but should it be DTS or DTSEN?
	movwf	BANKED_EP1IN_STAT

	return

; Process HID specific requests
ProcessHIDRequest:
	; Was this Request for a HID interface..
	;   SetupPacket.wIndex must be zero
	BANKSEL SetupPacket
	movf	SETUP_wIndex0,W  ; if SetupPacket.wIndex != 0 : return
	btfss	STATUS,Z
	return

	; The bmRequestType must 1
	movf	SETUP_bmRequestType,W ; if (SetupPacket.bmRequestType & 0x1F) != 0x01
	andlw	0x1F
	decfsz	WREG,W
	return

	; Was this HID request a GET_DESCRIPTOR
	movf	SETUP_bRequest,W
	xorlw	GET_DESCRIPTOR
	btfss	STATUS,Z
	goto	_PHR_check_RequestType
	; What type of descriptor...
	; Was it a HID_DESCRIPTOR
	movf	SETUP_wValue1,W
	xorlw	HID_DESCRIPTOR    ; Was this  HID Descriptor
	btfss	STATUS,Z
	goto	_PHR_check_REPORT_DESC

	; then update outPtr to point at the HIDDescriptor
	bsf		StatusBits, status_request_handled ; Flag this was handled
	movlw	low HIDDescriptor                  ; Point to the HID Descriptor
	movwf	outPtrL
	movlw	high HIDDescriptor
	movwf	outPtrH
	movlw	HID_DESC_LEN
	movwf	wCount
	; Do we need to set RAM vs ROM?
	goto	_PHR_check_RequestType

_PHR_check_REPORT_DESC:
	; Was it a REPORT_DESCRIPTOR
	movf	SETUP_wValue1,W
	xorlw	REPORT_DESCRIPTOR
	btfss	STATUS,Z
	goto	_PHR_check_PHYSICAL

	; then update the outPtr to point to the HIDReport
	bsf		StatusBits, status_request_handled
	movlw 	low HIDReport
	movwf	outPtrL
	movlw	high HIDReport
	movwf	outPtrH
	movlw	HID_REPORT_LEN
	movwf	wCount
	; Do we need to type to ROM?
	goto	_PHR_check_RequestType

_PHR_check_PHYSICAL:
	; Do Nothing
_PHR_check_RequestType:
	; if the (requestType &0x60) is not 0x20
	movf	SETUP_bmRequestType,W
	andlw	0x60
	xorlw	0x20
	btfss	STATUS,Z
	return


_PHR_check_GET_REPORT:
	; Was the request equal to GET_REPORT
	movf	SETUP_bRequest,W
	xorlw	GET_REPORT
	btfsc   STATUS,Z ;Note checking it WASN't and skipping
    goto	_PHR_exit ; exit if it matched

	; Was bRequest a SET_REPORT
	movf	SETUP_bRequest,W
	xorlw	SET_REPORT
	btfss	STATUS,Z
	goto	_PHR_check_GET_IDLE
	bsf		StatusBits,status_request_handled
	; Note: HidPostProcess removed - as it was never used
	goto	_PHR_exit

_PHR_check_GET_IDLE:
	; Was bRequest a GET_IDLE request...
	movf	SETUP_bRequest,W
	xorlw	GET_IDLE
	btfss	STATUS,Z
	goto	_PHR_check_SET_IDLE

	;	Update outPtr and wCount for the payload
	bsf		StatusBits,status_request_handled
	movlw	low HidIdleRate
	movwf	outPtrL
	movlw	high HidIdleRate
	movwf	outPtrH
	movlw	1
	movwf	wCount
	goto	_PHR_exit

_PHR_check_SET_IDLE:
	; Was bRequest a SET_IDLE request...
	movf	SETUP_bRequest,W
	xorlw	SET_IDLE
	btfss	STATUS,Z
	goto	_PHR_check_GET_PROTOCOL

	;	Update HidIdleRate based on incomming payload
	bsf		StatusBits,status_request_handled
	movf	SETUP_wValue1,W
	BANKSEL	HidIdleRate
	movwf	HidIdleRate
	goto	_PHR_exit

_PHR_check_GET_PROTOCOL:
	; Was bRequest a GET_PROTOCOL request...
	movf	SETUP_bRequest,W
	xorlw	GET_PROTOCOL
	btfss	STATUS,Z
	goto	_PHR_check_SET_PROTOCOL
	; update outPtr to point to the HidProtocol payload
	movlw	low HidProtocol
	movwf	outPtrL
	movlw	high HidProtocol
	movwf	outPtrH
	movlw	1
	movwf	wCount
	goto	_PHR_exit

_PHR_check_SET_PROTOCOL:
	; Was the bRequest set to SET_PROTOCOL
	movf	SETUP_bRequest,W
	xorlw	SET_PROTOCOL
	btfss	STATUS,Z
	goto	_PHR_unknown

	; Store value of HidProtocol from incomming payload
	bsf		StatusBits, status_request_handled
	movf	SETUP_wValue0,W
	BANKSEL	HidProtocol
	movwf	HidProtocol
	goto	_PHR_exit

_PHR_unknown:
	; Turn on the RED LED??
_PHR_exit:
	return

; Process GET_DESCRIPTOR
GetDescriptor:
	; Was this a GET_DESCRIPTOR request (seems to be redundant)
	BANKSEL SetupPacket
	movf	SETUP_bmRequestType,W
	xorlw	0x80
	btfss	STATUS,Z
	goto	_GD_exit
	; Then was it for a DEVICE_DESCRIPTOR
	movf	SETUP_wValue1,W
	xorlw	DEVICE_DESCRIPTOR
	btfss	STATUS,Z
	goto	_GD_check_CONFIG
	; then setup outPtr to point at the DeviceDescriptor
	bsf		StatusBits, status_request_handled
	movlw	low DeviceDescriptor
	movwf	outPtrL
	movlw	high DeviceDescriptor
	movwf	outPtrH
	; update wCount to hold the length of the descriptor
	movlw	DEVICE_DESC_LEN
	movwf	wCount
	logch	'D', 0
	goto	_GD_exit

_GD_check_CONFIG:
	; Was this a request for a CONFIGURATION_DESCRIPTOR
	movf	SETUP_wValue1,W
	xorlw	CONFIGURATION_DESCRIPTOR
	btfss	STATUS,Z
	goto	_GD_check_STRING
	; Then setup outPtr to point at the CONFIG DESCRIPTOR
	bsf		StatusBits, status_request_handled
	movlw	low ConfigurationDescriptor
	movwf	outPtrL
	movlw	high ConfigurationDescriptor
	movwf	outPtrH
	; update wCount to hold the length of the descriptor
	movlw	CONFIG_DESC_LEN+HID_DESC_LEN
	movwf	wCount
	logch	'C', 0
	goto	_GD_exit

_GD_check_STRING:
	; Was this a request for a STRING_DESCRIPTOR
	movf	SETUP_wValue1,W
	xorlw	STRING_DESCRIPTOR
	btfss	STATUS,Z
	goto	_GD_exit

	bsf		StatusBits, status_request_handled

	; Update outPtrH .. and then determine outPtrL based on which
	; string the host is asking for
	movlw	high StringDescriptor0
	movwf	outPtrH

	; Was it String0
	movf	SETUP_wValue0,W
	btfsc	STATUS,Z
	goto	_GD_Str0 ; Request for STR000
	decf	WREG,W
	; Or String1
	btfsc	STATUS,Z
	goto	_GD_Str1 ; Request for STR001
	decf	WREG,W
	; or String2
	btfsc	STATUS,Z
	goto	_GD_Str2 ; Request for STR002
	decf	WREG,W
	; or String3
	btfsc	STATUS,Z
	goto	_GD_Str3 ; Request for STR002
	; Default back to String0 for all others
	goto	_GD_Str0 ; Error condition jump to 0x0
_GD_Str3
	movlw	low StringDescriptor3
	movwf	outPtrL
	movlw	STR_DESC3_SIZE
	movwf	wCount
	logch	'3', 0
	goto	_GD_exit
_GD_Str2
	movlw	low StringDescriptor2
	movwf	outPtrL
	movlw	STR_DESC2_SIZE
	movwf	wCount
	logch	'2', 0
	goto	_GD_exit
_GD_Str1
	movlw	low StringDescriptor1
	movwf	outPtrL
	movlw	STR_DESC1_SIZE
	movwf	wCount
	logch	'1', 0
	goto	_GD_exit
_GD_Str0
	movlw	low StringDescriptor0
	movwf	outPtrL
	movlw	STR_DESC0_SIZE
	movwf	wCount
	; Fall through to exit
	logch	'*', 0
_GD_exit:
	return

; Process GET_STATUS
GetStatus:
	; Clear first two bytes of reply payload
	BANKSEL ControlTransferBuffer
	clrf	ControlTransferBuffer
	clrf	ControlTransferBuffer+1

	; Mask out the recipient bits
	BANKSEL	SetupPacket
	movf	SETUP_bmRequestType,W
	andlw	0x1F

	; Was the recipient 0x00...
	btfss	STATUS,Z
	goto	gs_check1

	; Show Request was handled
	bsf		StatusBits, status_request_handled

	; Update reply with self-powered state
	btfsc	StatusBits, status_self_powered
	bsf		ControlTransferBuffer,1
	; Update reply with remote-wakeup state

	btfsc	StatusBits, status_remote_wakeup
	bsf		ControlTransferBuffer,2
	logch	'S', 0
	goto	_gs_common

gs_check1:
	; Was the recipient 0x01...
	decfsz  WREG,W
	goto	gs_check2
	; then do nothing, but show request was recognized/handled
	bsf		StatusBits, status_request_handled
	goto	_gs_common
gs_check2:
	; Was the recipient 0x02...
	decfsz	WREG,W
	goto	_gs_common
	; then this was to an endpoint..
	; Update status to show request was handled
	bsf		StatusBits, status_request_handled
	; Update inPtr to point to appropriate buffer
	;  inPtr = (uint8_t *)&Interfaces[0].Output + (endpointNum * 8);
	BANKSEL SetupPacket
	movf	SETUP_wIndex0,W
	andlw	0x0F
	; W contains the endpoint number
	; copy to bufferSize (a scratch pad copy)
	movwf	bufferSize
	; Was this for EP0 then no need for further math
	btfsc	STATUS,Z
	goto	_gs_have_offset
	clrf	WREG
_gs_ep_loop
	addlw	0x08		; quick and dirty multiplication loop (keep adding 8)
	decfsz	bufferSize,f
	goto	_gs_ep_loop

_gs_have_offset ; W contains endpointNum * 8
	; Add offset to start of buffer descriptors
	addlw	BANKED_EP0OUT_STAT
	movwf	inPtrL
	movlw	high BANKED_EP0OUT_STAT
	movwf	inPtrH
	movwf	FSR0H

	; Add 4 if this is an IN endpoint
	;  Start with ADD 0 then override with 4 if wIndex0&0x80 is true
	clrf	WREG
	btfsc	SETUP_wIndex0,7
	movlw	0x04
	addwf	inPtrL,f


	; if(*inPtr & BSTALL)
	;  ControlTransferBuffer[0] = 0x01;
	movf	inPtrL,W
	movwf	FSR0L
	; Retrieve inPtr[0] (indirection)
	moviw	0[FSR0]
	btfss	WREG,BSTALL
	goto	_gs_common

	; BSTALL is flagged, so set ControlTransferBuffer[0] to 1
	movlw	0x01
	BANKSEL ControlTransferBuffer
	movwf	ControlTransferBuffer
	logch	'B', 0
	goto	_gs_common

_gs_common:
	; If the the request was handled ...
	btfss	StatusBits, status_request_handled
	goto	_gs_exit
	; Then update outPtr to point at ControlTransferBuffer
	movlw	low ControlTransferBuffer
	movwf	outPtrL
	movlw	high ControlTransferBuffer
	movwf	outPtrH
	; And set the length to 2
	movlw	0x02
	movwf	wCount
	; Set type to RAM
	goto	_gs_exit

_gs_exit
	return

; Process SET_FEATURE and CLEAR_FEATURE
SetFeature:
;    uint8_t recipient = SetupPacket.bmRequestType & 0x1F;
;    uint8_t feature = SetupPacket.wValue0;
;
	; Was this to recipient 0x00
	BANKSEL SETUP_bmRequestType
	movf	SETUP_bmRequestType,W
	btfss	STATUS,Z
	goto	_SF_check_02

	; Device handling
	;   Was it a DEVICE_REMOTE_WAKEUP...
	movf	SETUP_wValue0,W
	xorlw	DEVICE_REMOTE_WAKEUP
	btfss	STATUS,Z
	goto	_SF_exit
	; Show request was handled
	bsf		StatusBits, status_request_handled

	; Store RemoteWakeup flag based on bRequest flag
	movf	SETUP_bRequest,W
	xorlw	SET_FEATURE
	bsf		StatusBits,status_remote_wakeup
	btfss	STATUS,Z
	bcf		StatusBits,status_remote_wakeup
	goto	_SF_exit
_SF_check_02
	; Was the recipient an endpoint (0x02)...
	xorlw	0x02
	btfss	STATUS,Z
	goto	_SF_exit

	; then if it was an enpoint halt for EP *NOT* 0
	; Note feature is located in wValue0
	movf	SETUP_wValue0,W
	xorlw	ENDPOINT_HALT
	btfss	STATUS,Z
	goto	_SF_exit

	movf	SETUP_wIndex0,W
	andlw	0x0F
	btfsc	STATUS,Z	; if not Zero
	goto	_SF_exit

	; w contains wIndex0&0x0F (aka "Endpoint Num" which is NON-zero)

	; Halt endpoint (as long as it isn't endpoint 0)
	; Show request was handled
	bsf		StatusBits,status_request_handled
	; inPtr = (uint8_t *)&Interfaces[0].Output + (endpointNum * 8);
	; Retrieve the endpoint Number from wIndex0&0x0F
	movwf	bufferSize     ;endpoint numb

	clrf	WREG
	; multiplication loop, add 8 until bufferSize counts down to zero
_SF_ep_loop
	addlw	0x08
	decfsz 	bufferSize,F
	goto	_SF_ep_loop
_SF_have_offset
	;W now contains offset to add
	addlw	BANKED_EP0OUT
	movwf	inPtrL
	movlw	high BANKED_EP0OUT
	movwf	inPtrH
	; Determine if we need to add 4 for an IN (vs OUT) endpoint
	;    if (endpointDir)
	;       inPtr += 4;
	;   (here we check if we add 0 or 4 )
	clrf	WREG
	btfsc	SETUP_wIndex0,7
	movlw	0x04
	addwf	inPtrL,f

	; Setup FSR0 for indirect access to inPtr
	movf	inPtrL,W
	movwf	FSR0L
	movf	inPtrH,W
	movwf	FSR0H

	;  Was bRequest equal to SET_FEATURE...
	movf	SETUP_bRequest,W
	xorlw	SET_FEATURE
	btfss	STATUS,Z
	goto	_SF_check_dir
	; then write 0x84
	movlw	0x84
	movwi	0[FSR0]
	goto	_SF_exit

_SF_check_dir
	; Was wIndex0 direction (bit 7) a zero or one
	;   if 1 then inPtr=0x00
	;   else
	;   *inPtr = 0x88
	;  Setup W to be 0x00, and over-ride if endpointDir 1
	movlw	0x00
	btfsc 	SETUP_wIndex0,7
	movlw	0x88
	movwi	0[FSR0]
        ; Drop through to SF_exit
_SF_exit
	return

ProcessStandardRequest:
	;   Was this a non-standard request...
 	;  if((SetupPacket.bmRequestType & 0x60) != 0x00) {
	BANKSEL SetupPacket
	movf	SETUP_bmRequestType,W
	andlw	0x60
	btfss	STATUS,Z
	;  Not a standard request - don't process here.  Class or Vendor
	;  requests have to be handled seperately.
	;  return;
	return

	; Was this a SET_ADDRESS request ...
	movf	SETUP_bRequest,W
	xorlw	SET_ADDRESS
	btfss	STATUS,Z
	goto	_PSR_GET_DESCR
	; Set the address of the device.  All future requests
	; will come to that address.  Can't actually set UADDR
	; to the new address yet because the rest of the SET_ADDRESS
	; transaction uses address 0.

	; Show request has been handled
	bsf		StatusBits, status_request_handled
	logch	'A', 0
	;  Update DeviceState to STATE_ADDRESS;
	movlw	STATE_ADDRESS
	movwf	DeviceState
	; Retrieve DeviceAddress from the SetupPacket wValue0
	movf	SETUP_wValue0,W
	movwf	DeviceAddress
	goto	_PSR_exit
_PSR_GET_DESCR
	;  Was this a GET_DESCRIPTOR request...
	movf	SETUP_bRequest,W
	xorlw	GET_DESCRIPTOR
	btfss	STATUS,Z
	goto	_PSR_SET_CONFIG

	; Then call GetDescriptor to update the points, and wCount
	pagesel GetDescriptor
	call	GetDescriptor
	pagesel $
	goto	_PSR_exit

_PSR_SET_CONFIG
	; Was this a SET_CONFIGURATION request ...
	movf	SETUP_bRequest,W
	xorlw	SET_CONFIGURATION
	btfss	STATUS,Z
	goto	_PSR_GET_CONFIG
	logch	'c', 0
	; Then handle the request
	; Show request has been handled
	bsf		StatusBits, status_request_handled
	; CurrentConfiguration = SetupPacket.wValue0;
	movf	SETUP_wValue0,W
	; Was this set to configuration zero (0)...
	BANKSEL CurrentConfiguration
	movwf	CurrentConfiguration
	btfss 	STATUS,Z
	goto	_PSR_config_not_0
	; If configuration value is zero, device is put in
	; address state (USB 2.0 - 9.4.7)
	;  DeviceState = STATE_ADDRESS;
	movlw	STATE_ADDRESS
	movwf	DeviceState
	goto	_PSR_exit

_PSR_config_not_0
	; Not zero config - store state as CONFIGURED
	movlw	STATE_CONFIGURED
	movwf	DeviceState
	; Initialize the HID Endpoints
	pagesel	HIDInitEndpoints
	call	HIDInitEndpoints
	pagesel $
	; TBD: Add initialization code here for any additional
	; interfaces beyond the one used for the HID
	goto	_PSR_exit

_PSR_GET_CONFIG
	; Was this a GET_CONFIGURATION request ...
	movf	SETUP_bRequest,W
	xorlw	GET_CONFIGURATION
	btfss	STATUS,Z
	goto	_PSR_GET_STATUS
	; GET_CONFIG reuqest - retrieve configuration and reply
	; Show message was handled
	bsf     StatusBits,status_request_handled
	; Copy configuration to the outPtr buffer
	; 	outPtr = (uint8_t*)&CurrentConfiguration;
	movf    outPtrL,W
	movwf   FSR0L
	movf    outPtrH,W
	movwf   FSR0H
	; Can pull the byte directly, don't need to use indirection
	BANKSEL CurrentConfiguration
	movf    CurrentConfiguration,W
	movwi   0[FSR0]
	; Record size of reply as 1 byte long
	movlw   1
	movwf   wCount
	goto    _PSR_exit

_PSR_GET_STATUS
	; Was this a GET_STATUS request...
	movf	SETUP_bRequest,W
	xorlw	GET_STATUS
	btfss	STATUS,Z
	goto	_PSR_FEATURE
	; then call GetStatus() to handle it
	pagesel	GetStatus
	call	GetStatus
	pagesel $
	goto	_PSR_exit

_PSR_FEATURE
	; Was this a CLEAR_FEATURE Or SET_FEATURE request ...
	movf	SETUP_bRequest,W
	xorlw	SET_FEATURE
	btfsc	STATUS,Z
	goto	_PSR_SetFeature
	movf	SETUP_bRequest,W
	xorlw	CLEAR_FEATURE
	btfss	STATUS,Z
	goto	_PSR_GET_INTF
_PSR_SetFeature
	; then call SetFeature() to handle it
	pagesel	SetFeature
	call	SetFeature
	pagesel $
	goto	_PSR_exit

_PSR_GET_INTF
	; Was this a GET_INTERFACE request ...
	movf	SETUP_bRequest,W
	xorlw	GET_INTERFACE
	btfss	STATUS,Z
	goto	_PSR_SET_INTF
	; It was .. but we don't support it so
	; send back a ZERO to the host
	; Show the request was handled
	bsf	StatusBits, status_request_handled

	; ControlTransferBuffer[0] = 0;
	BANKSEL	ControlTransferBuffer
	clrf	ControlTransferBuffer

	;  outPtr = (uint8_t*) &ControlTransferBuffer;
	movlw	low ControlTransferBuffer
	movwf	outPtrL
	movlw	high ControlTransferBuffer
	movwf	outPtrH
	; set the length to 1
	movlw	1
	movwf	wCount
	goto	_PSR_exit

_PSR_SET_INTF
	; Was this a SET_INTERFACE request...
	movf	SETUP_bRequest,W
	xorlw	SET_INTERFACE
	btfss	STATUS,Z
	goto	_PSR_exit
	; No support for alternate interfaces - just ignore.
	; Show request was handled
	bsf		StatusBits, status_request_handled
_PSR_exit
	return

; Data stage for a Control Transfer that sends data to the host
InDataStage:
	;    Determine how many bytes are going to the host
	;    if(wCount < E0SZ)
	;        bufferSize = wCount;
	;    else
	;        bufferSize = E0SZ;
	;
	movlw	E0SZ
	subwf	wCount,W
	btfsc	STATUS,C
	goto	_IDS_buffer_E0SZ
	; bufferSize = wCount
	movf	wCount,W
	movwf	bufferSize
	goto	_IDS_clear_BD
_IDS_buffer_E0SZ
	; else
	;   bufferSize = E0SZ
	movlw	E0SZ
	movwf	bufferSize
    ; Fall through to IDS_clear_BD
	; W to hold 'bufferSize'
_IDS_clear_BD
	;   Load the high two bits of the byte count into BC8:BC9
	;	for our purposes these will always be ZERO so clear them
	BANKSEL BANKED_EP0IN_STAT
	bcf		BANKED_EP0IN_STAT,BC8
	bcf		BANKED_EP0IN_STAT,BC9
	; Copy Buffer size to the buffer descriptor
	movwf	BANKED_EP0IN_CNT
	; Update the number of bytes that still need to be sent.  Getting
	; all the data back to the host can take multiple transactions, so
	; we need to track how far along we are.
	; 		wCount = wCount - bufferSize;
	; W still holds 'bufferSize'
	subwf  wCount,f
	;    Interfaces[0].Input.Addr = PTR16(&ControlTransferBuffer);
	movlw	low ControlTransferBuffer
	movwf	BANKED_EP0IN_ADRL
	movwf	FSR0L	 ; also point FSR0 at inPtr (aka ControlTransferBuffer)
	movlw	high ControlTransferBuffer
	movwf	BANKED_EP0IN_ADRH
	movwf	FSR0H

	; Check that there *is* data to copy
	movf	bufferSize,W
	btfsc	STATUS,Z
	goto	_IDS_exit

	; Setup FSR1 for indirect access to outPtr
	movf	outPtrL,W
	movwf	FSR1L
	movf	outPtrH,W
	movwf	FSR1H

	; Move data to the USB output buffer from wherever it sits now.
	; inPtr = (uint8_t *) &ControlTransferBuffer;
	; NOTE: I removed the difference between ROM and RAM copies
	; as I think it was holdover from some previous project on a different
    ; device.

_IDS_copy_loop
	; Copy from outPtr (FSR1) to the ControlTransferBuffer (inPtr::FSR0)
	moviw	FSR1++
	movwi	FSR0++
	decfsz	bufferSize,F
	goto	_IDS_copy_loop

	; NOTE: inPtr context is not maintained across calls to this function
	; it ALWAYS starts at the beginning of the ControlTransferBuffer
	; so there is not need to store the context.
	; Store outPtr context
	movf	FSR1L,W
	movwf	outPtrL
	movf	FSR1H,W
	movwf	outPtrH

_IDS_exit
	return

;; Data stage for a Control Transfer that reads data from the host
OutDataStage:
	; Note: ASSUMES we only ever get a small (within 1 byte) buffer size
	; Setup for indirect access to inPtr
	movf	inPtrL,W
	movwf	FSR0L
	movf	inPtrH,W
	movwf	FSR0H
	; Setup for indirect access to ControlTransferBuffer
	movlw	low ControlTransferBuffer
	movwf	FSR1L
	movlw	high ControlTransferBuffer
	movwf	FSR1H

	; Pull message length from the SIE BD's
	BANKSEL BANKED_EP0OUT_STAT
	movf	BANKED_EP0OUT_CNT,W
	; keep a local copy (bufferSize is not maintained across function calls)
	movwf	bufferSize
	; Add length to wCount
	addwf	wCount,F

	; Loop copying wCount f
ods_loop
	moviw	FSR1++
	movwi	FSR0++
	decfsz	bufferSize,f
	goto	ods_loop

	; Save the inPtr Context
	movf	FSR0L,W
	movwf	inPtrL
	movf	FSR0H,W
	movwf	inPtrH
	; Save the outPtr Context
	movf	FSR1L,W
	movwf	outPtrL
	movf	FSR1H,W
	movwf	outPtrH
	return

; Process the Setup stage of a control transfer.  This code initializes the
; flags that let the firmware know what to do during subsequent stages of
; the transfer.
SetupStage:
    ; Note: Microchip says to turn off the UOWN bit on the IN direction as
    ; soon as possible after detecting that a SETUP has been received.
	BANKSEL BANKED_EP0IN_STAT
	bcf		BANKED_EP0IN_STAT,UOWN
	bcf		BANKED_EP0OUT_STAT,UOWN

    ; Initialize the transfer process
	movlw	STAGE_SETUP
	BANKSEL CtrlTransferStage
	movwf	CtrlTransferStage
 	bcf  	StatusBits, status_request_handled ; Clear handled bit
	clrf	wCount         ; No bytes copied yet

    ; See if this is a standard (as definded in USB chapter 9) request
	pagesel	ProcessStandardRequest
	call	ProcessStandardRequest

    ; See if the HID class can do something with it.
	pagesel	ProcessHIDRequest
	call	ProcessHIDRequest
	pagesel	$

    ; If the request wasn't handled ...
	btfsc	StatusBits,status_request_handled
	goto	_ss_device_to_host
    ; ... then the Service Was not handled - stall endpoint 0
	logch	'U', 0
	BANKSEL BANKED_EP0OUT_STAT
	movlw	E0SZ
	movwf	BANKED_EP0OUT_CNT
	movlw	low SetupPacket
	movf	BANKED_EP0OUT_ADRL,W
	movlw	high SetupPacket
	movwf	BANKED_EP0OUT_ADRH
	movlw	(_BSTALL)
	movwf	BANKED_EP0OUT_STAT
	movwf	BANKED_EP0IN_STAT
	; Hand the BD's to the SIE
	bsf		BANKED_EP0OUT_STAT,UOWN
	bsf		BANKED_EP0IN_STAT,UOWN
	goto	_SetupStage_exit

_ss_device_to_host:
	; Was this a device to host transfer (0x8n)
	BANKSEL SetupPacket
	btfss	SETUP_bmRequestType,7 ; if &0x80
	goto	_host_to_device

    ; Device-to-host
	; Trim wCount if it is larger than SetupPacket.wLength
	BANKSEL SETUP_wLengthL
	movf	SETUP_wLengthL,W
	BANKSEL	wCount
	subwf	wCount,w   ; wCount > wLength C=0
	btfss   STATUS,C
    goto    _ss_in_data
	; wLength is smaller, so use that instead
	BANKSEL SETUP_wLengthL
	movf	SETUP_wLengthL,W
	BANKSEL wCount
	movwf	wCount
_ss_in_data:
	; Copy data to outgoing buffer
	pagesel	InDataStage
	call	InDataStage
	pagesel $

	; Update state to STAGE_DATA_IN
	movlw	STAGE_DATA_IN
	BANKSEL	CtrlTransferStage
	movwf	CtrlTransferStage

	; Reset BD's for EP0OUT
	BANKSEL BANKED_EP0OUT_STAT
	movlw	E0SZ
	movwf	BANKED_EP0OUT_CNT
	movlw  	low SetupPacket
	movwf	BANKED_EP0OUT_ADRL
	movlw	high SetupPacket
	movwf	BANKED_EP0OUT_ADRH
	; Hand the BD to the SIE
	movlw	(1<<UOWN)
	movwf	BANKED_EP0OUT_STAT

	; Update BD's for EP0IN (to pull data from the RAM buffer)
	movlw 	low ControlTransferBuffer
	movwf	BANKED_EP0IN_ADRL
	movlw	high ControlTransferBuffer
	; Determine Data0 vs Data1 flag (toggling back and forth)
	movwf	BANKED_EP0IN_ADRH
	movlw   (1<<DTS) |(1<<DTSEN)
	movwf	BANKED_EP0IN_STAT
	; Hand the BD to the SIE
	bsf		BANKED_EP0IN_STAT,UOWN
	goto	_SetupStage_exit

_host_to_device:
	; Update state to refelect receiving data
	logch	'H', 0
	movlw	STAGE_DATA_OUT
	BANKSEL	CtrlTransferStage
	movwf	CtrlTransferStage

    ; Clear the input buffer descriptor
	BANKSEL	BANKED_EP0IN_STAT
	clrf	BANKED_EP0IN_CNT
	movlw	(1<<DTS)|(1<<DTSEN)
	movwf	BANKED_EP0IN_STAT
	; Hand the BD to the SIE
	bsf		BANKED_EP0IN_STAT,UOWN

    ; Set the out buffer descriptor on endpoint 0 to receive data
	movlw	E0SZ
	movwf	BANKED_EP0OUT_CNT
	movlw	low ControlTransferBuffer
	movwf	BANKED_EP0OUT_ADRL
	movlw	high ControlTransferBuffer
	movwf	BANKED_EP0OUT_ADRH

    ; Give to SIE, DATA1 packet, enable data toggle checks
	movlw	(1<<DTS)|(1<<DTSEN)
	movwf	BANKED_EP0OUT_STAT
	; Hand the BD to the SIE
	bsf		BANKED_EP0OUT_STAT,UOWN

_SetupStage_exit:
	BANKSEL	UCON
   	bcf	UCON,PKTDIS
	return

; Configures the buffer descriptor for endpoint 0 so that it is waiting for
; the status stage of a control transfer.
WaitForSetupStage:
	movlw	STAGE_SETUP
	BANKSEL CtrlTransferStage
	movwf	CtrlTransferStage
	; Update the EP0OUT BD - ready to receive SETUP packet
	BANKSEL BANKED_EP0OUT_STAT
	movlw	E0SZ
	movwf	BANKED_EP0OUT_CNT
	movlw	low SetupPacket
	movwf	BANKED_EP0OUT_ADRL
	movlw	high SetupPacket
	movwf	BANKED_EP0OUT_ADRH
	movlw	(_DTSEN)
	movwf	BANKED_EP0OUT_STAT
	bsf		BANKED_EP0OUT_STAT,UOWN
	; Update the EP1IN BD
	clrf	BANKED_EP0IN_STAT
	return

; This is the starting point for processing a Control Transfer.  The code directly
; follows the sequence of transactions described in the USB spec chapter 5.  The
; only Control Pipe in this firmware is the Default Control Pipe (endpoint 0).
; Control messages that have a different destination will be discarded.
ProcessControlTransfer:
	; If USTAT == 0 ...
	BANKSEL USTAT
	movf	USTAT,W
	btfss	STATUS,Z
    goto	_PCT_EP0IN
	; then this was Endpoing 0:OUT
 	; Was this PID = 0x0D
	BANKSEL	BANKED_EP0OUT_STAT
	movf	BANKED_EP0OUT_STAT,W
	andlw	0x3C  ; Mask PID from middle of BD0STAT
	xorlw	(0x0D<<2) ; (PID = 0x0D <<2)
	btfss 	STATUS,Z
	goto	_PCT_1
	; Then call SetupStage as a transaction is starting
	pagesel	SetupStage
	call	SetupStage
	goto	_PCT_exit

_PCT_1:
	; If we are in STAGE_DATA_OUT...
	logch '>',0
	BANKSEL CtrlTransferStage
	movf	CtrlTransferStage,W
	xorlw	STAGE_DATA_OUT
	btfss 	STATUS,Z
	goto	_PCT_2
	; Then copy data to outbuffer, and setup the BD for xfer
	pagesel	OutDataStage
	call	OutDataStage
	pagesel	$
	; Are we in phase DATA0 or DATA1 (toggles back and forth
	movlw	(1<<DTSEN)
	BANKSEL BANKED_EP0OUT_STAT
	btfss	BANKED_EP0OUT_STAT,DTS
	iorlw	(1<<DTS)
	movwf	BANKED_EP0OUT_STAT
	; Hand buffer descriptor to the SIE
	bsf		BANKED_EP0OUT, UOWN
	goto	_PCT_exit
_PCT_2:
	; Else ... Prepare for the next Setup Stage control transfer
	logch 'W',0
	pagesel WaitForSetupStage
	call	WaitForSetupStage
	pagesel	$
	goto	_PCT_exit

_PCT_EP0IN:
	; Was USTAT == 0x04 ...
	xorlw	0x04
	btfss	STATUS,Z
	goto	_PCT_exit
	logch '<',0
	; Then this was Endpoint 0: IN
	; Was this a set address packet
	; 	((UADDR==0) && (DeviceState == STATE_ADDRESS))
	movf 	UADDR,W
	btfss	STATUS,Z
	goto	_PCT_data_in
	movf	DeviceState,W
	xorlw	STATE_ADDRESS
	btfss	STATUS,Z
	goto	_PCT_data_in
	; Then read the address from the packet, and assign it.
	BANKSEL	SETUP_wValue0
	movf	SETUP_wValue0,W
	; Tell the SIE about the assigned address
	BANKSEL	UADDR
	movwf	UADDR
	; Was the assigned address 0...
	btfsc	STATUS,Z
	goto	_PCT_data_in
	; then drop to the default state
	movlw	STATE_DEFAULT
	movwf	DeviceState

_PCT_data_in:
	; Are we in STAGE_DATA_IN mode
	BANKSEL	CtrlTransferStage
	movf	CtrlTransferStage,W
	xorlw	STAGE_DATA_IN
	btfss	STATUS,Z
	goto	_PCT_w4s
	; Then  copy the data to the outgoing packet
	pagesel	InDataStage
	call	InDataStage
	pagesel	$
	; Are we in phase DATA0 or DATA1 (toggles back and forth
	movlw	(1<<DTSEN)
	BANKSEL	BANKED_EP0IN_STAT
	btfss	BANKED_EP0IN_STAT,DTS
	iorlw	(1<<DTS)
	movwf	BANKED_EP0IN_STAT
	; Hand the BD over to the SIE
	bsf	BANKED_EP0IN_STAT,UOWN
	goto	_PCT_exit

_PCT_w4s:
	; Else not DATA_IN_STAGE
	; Prepare for next Setup packey
	pagesel	WaitForSetupStage
	call	WaitForSetupStage
	pagesel $

_PCT_exit:
	return

InitializeUSB:
	; Init global variables

	; Clear the SETUP packet (aids with debugging)
	BANKSEL	SETUP_bmRequestType
	clrf	SETUP_bmRequestType
	clrf	SETUP_bRequest
	clrf	SETUP_wValue0
	clrf	SETUP_wValue1
	clrf	SETUP_wIndex0
	clrf	SETUP_wIndex1
	clrf	SETUP_wLengthL
	clrf	SETUP_wLengthH


	; Enable PULLUP resistors; full speed mode; no Ping-Pong
	BANKSEL UCFG
	movlw 0x14
	movwf UCFG

	movlw STATE_DETACHED
	movwf DeviceState
	bcf   StatusBits,status_remote_wakeup
	BANKSEL CurrentConfiguration
	clrf  CurrentConfiguration

	; Reset the USB address
	BANKSEL UADDR
	clrf  UADDR
	; Clear all USB Error Interrupt Flags
	clrf  UEIR
    ; Reset PP buffers
	bsf   UCON, PPBRST
	bcf   UCON, PPBRST
	; Enable Packet Transfers
    bcf   UCON,PKTDIS
	return

EnableUSBModule:
    ; TBD: Check for voltage coming from the USB cable and use that
    ; as an indication we are attached.
	BANKSEL UCON
	btfsc	UCON,USBEN
	goto	_eum_next
	clrf	UCON
	clrf	UIE
	bsf		UCON,USBEN
	movlw	STATE_ATTACHED
	movwf	DeviceState
_eum_next:
	btfsc   UCON,SE0 ; Single ended zero is set, spin until initial power-up
	goto	$-1

	; Once attached, and SE0 has cleared, can transition to POWERED state
	clrf	UIR
	clrf	UIE
	; USB Reset Interupt Enable
	bsf		UIE,URSTIE
	; USB Idle Interrupt Enable
	bsf		UIE,IDLEIE
	movlw	STATE_POWERED
	movwf	DeviceState
	return

; Unsuspend the device
UnSuspend:
	; Bring the USB Module out of power conserve state
      BANKSEL UCON
      bcf 	UCON,SUSPND
      bcf 	UIE, ACTVIE
      bcf 	UIR, ACTVIF  ; Note: replaced two line & with this bcf
      ;movlw 0xFB
      ;andwf UIR,F
	  return

; Full speed devices get a Start Of Frame (SOF) packet every 1 millisecond.
; Nothing is currently done with this interrupt (it is simply masked out).
StartOfFrame:
      BANKSEL UIR
      bcf UIR, SOFIF ; This is redundant, as bit is cleared after the call here
      return

; This routine is called in response to the code stalling an endpoint.
Stall:
	; If stall condition on EP0
	BANKSEL UEP0
	btfss	UEP0, EPSTALL
	goto  	_stall_exit
	; then prepare for the Setup stage of a control transfer
	pagesel WaitForSetupStage
    call	WaitForSetupStage
	pagesel $
	BANKSEL UEP0
	bcf		UEP0, EPSTALL
_stall_exit:
	bcf		UIR, STALLIF
	return

; Suspend all processing until we detect activity on the USB bus
Suspend:
	; Enable the bus activity interrupt
	BANKSEL UIE
	bsf	UIE, ACTVIE
	movlw	0xEF
	andwf	UIR,f	; This AND should have no effect ??(High order bit is NA)
	; Place the USB module in a power conserve state
	bsf		UCON, SUSPND
	return

BusReset:
	BANKSEL UEIR
	clrf	UEIR ; Clear any pending Error Interrupts
	clrf 	UIR  ; Clear any pending USB interrupts
	movlw	0x9F ; Enable ALL Error Interrupts
	movwf	UEIE
	movlw	0x7b ; Enable all *but* the ACTVIE interrupts
	movwf	UIE
	clrf	UADDR
	movlw	0x16
	movwf	UEP0 ; Set endpoint 0 as a control pipe

        ; Flush any pending transactions
	bcf 	UIR, TRNIF
	btfsc	UIR, TRNIF
	goto 	$-2	; loop to clear the FIFO

	bcf     UCON, PKTDIS ; Enable packet processing

	; Prepare for the Setup Stage of a control transfer
	pagesel	WaitForSetupStage
	call	WaitForSetupStage
	pagesel	$

	; Remote wakeup is off by default
	bcf		StatusBits,status_remote_wakeup
	; Self powered is off by default
	bcf		StatusBits,status_self_powered
	; Clear active configuration
	BANKSEL CurrentConfiguration
	clrf 	CurrentConfiguration

	movlw	STATE_DEFAULT
	movwf	DeviceState
	return

; Main entry point for USB tasks.  Checks interrupts, then checks for transactions.
ProcessUSBTransactions:
	; See if device is connected yet
	BANKSEL DeviceState
 	movf	DeviceState,W
	xorlw   STATE_DETACHED
	btfss   STATUS,Z
	goto	_PUT_UNSUSPEND
 	; then clear all interrupts
	BANKSEL	UIR
	clrf	UIR
	BANKSEL PIR2
	bcf		PIR2,USBIF
	; and exit
	return

_PUT_UNSUSPEND:
    ; If the USB became active ...
	BANKSEL UIR
	btfss	UIR,ACTVIF
	goto	_PUT_SUSPEND
	btfss	UIE,ACTVIE
	goto	_PUT_SUSPEND
	; Wakeup from suspend
	pagesel UnSuspend
	call	UnSuspend
	pagesel	$
  	BANKSEL UIR
	bcf		UIR,ACTVIF

_PUT_SUSPEND:
    ; If we are supposed to be suspended...
	btfss	UCON,SUSPND
	goto	_PUT_RESET
	; then cease performing any processing
	clrf	UIR
	BANKSEL PIR2
	bcf		PIR2,USBIF
	return

_PUT_RESET:
     ; If there was a bus reset...
	btfss	UIR, URSTIF
	goto	_PUT_IDLE
	btfss	UIE, URSTIE
	goto	_PUT_IDLE
	; then process a bus reset
	pagesel BusReset
	call	BusReset
	pagesel $
	BANKSEL UIR
	bcf		UIR,URSTIF

_PUT_IDLE:
    ; If there was  No bus activity for a while...
	btfss 	UIR,IDLEIF
	goto	_PUT_SOF
	btfss	UIE,IDLEIE
	goto	_PUT_SOF
 	; then suspend the firmware
	pagesel Suspend
	call	Suspend
	pagesel $

	BANKSEL UIR
  	bcf		UIR,IDLEIF

_PUT_SOF:
    ; If this was a Start of Frame (SOF)...
	btfss	UIR, SOFIF
	goto	_PUT_STALL
	btfss	UIE, SOFIE
	goto	_PUT_STALL
	; Then clear the SOF
	pagesel StartOfFrame
	call	StartOfFrame
	pagesel $
	BANKSEL	UIR
	bcf		UIR,SOFIF

_PUT_STALL:
	; If this was a STALL request ...
	btfss	UIR,STALLIF
	goto	_PUT_UERR
	btfss	UIE,STALLIE
	goto	_PUT_UERR
	; ... then invoke a STALL
	pagesel Stall
	call	Stall
	pagesel $
	BANKSEL	UIR
	bcf		UIR,STALLIF

_PUT_UERR:
	; If this was a USB error
	btfss	UIR,UERRIF
	goto	_PUT_OTHER
	btfss	UIE,UERRIE
	goto	_PUT_OTHER
	; Then clear the error.
	bcf		UIR,UERRIF
	clrf	UEIR
	bcf		UIR,UERRIF	; This seems redundant.. probably can remove

_PUT_OTHER:
	; If we are state is < STATE_DEFAULT ...
	movlw	STATE_DEFAULT
	subwf	DeviceState,W
	btfsc	STATUS,C
	goto	_PUT_PCT
	; Then there is no need to keep processing
	BANKSEL	UIR
	clrf	UIR  ; Clear All Interrupt Flags
	BANKSEL PIR2
	bcf		PIR2,USBIF ; Clear Global USB Interrupt Flag
	return

_PUT_PCT:
	; If a usb transaction has finished
	btfss	UIR, TRNIF
	goto	_PUT_exit
	btfss	UIE, TRNIE
	goto	_PUT_exit

	; then hand the transaction to the usb event state machine
	mlog
	mloghex 8,LOG_SPACE
	mlogf	SetupPacket
	mlogf	SetupPacket+1
	mlogf	SetupPacket+2
	mlogf	SetupPacket+3
	mlogf	SetupPacket+4
	mlogf	SetupPacket+5
	mlogf	SetupPacket+6
	mlogf	SetupPacket+7
	mlogend
	pagesel	ProcessControlTransfer
	call	ProcessControlTransfer
	pagesel $

;	logch	'|',0
;	mlog
;	mloghex 8,LOG_SPACE
;	mlogf	ControlTransferBuffer
;	mlogf	ControlTransferBuffer+1
;	mlogf	ControlTransferBuffer+2
;	mlogf	ControlTransferBuffer+3
;	mlogf	ControlTransferBuffer+4
;	mlogf	ControlTransferBuffer+5
;	mlogf	ControlTransferBuffer+6
;	mlogf	ControlTransferBuffer+7
;	mlogend
;
	loghex	1,LOG_SPACE
	logf	BANKED_EP0IN_STAT
	logch '.',LOG_NEWLINE
	BANKSEL	UIR
	bcf		UIR,TRNIF

_PUT_exit:
	BANKSEL PIR2
	bcf		PIR2,USBIF ; Clear Global USB Interrupt Flag
	return

 include "log.asm"
;------------------------- End of USB.INC -----------------------
