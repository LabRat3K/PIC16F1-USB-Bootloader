; asmsyntax=pic
; File:   Usb.asm
; Hand coded by Andrew Williams, based on C source by 
; Author: Szymon Roslowski
;
; Created on 13 October 2014, 17:46
; Tweaked Jan 23, 2023 by Andrew Williams (SDCC compatible)
;
; Firmware framework for USB I/O on PIC 16F1455 (and siblings)
;
;


	radix dec
	list n=0,st=off
	include "p16f1455.inc"
	nolist
	include "bdt.inc"
	list
	errorlevel -302

#define CONVERTEDCODE (0)
;***********************/
;* Local Definitions   */
;***********************/
; Commands
#define GET_STATUS                  0x00
#define CLEAR_FEATURE               0x01
#define SET_FEATURE                 0x03
#define SET_STATE_ADDRESS           0x05
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
#define SETUP_wLengthH      (SetupPacket+6)
#define SETUP_wLengthL      (SetupPacket+7) ; Number of bytes to transfer if there's a data stage
#define SETUP_extra	    (SetupPacket+8) ; Fill out to same size as Endpoint 0 max buffer (E0SZ-7)

;const uint8_t *ROMoutPtr;   Data to send to the host

	org 0x300
;***********************/
;* Implementation      */
;***********************/
IsUsbDataAvailable:
	BANKSEL BANKED_EP1OUT_STAT
	btfss	BANKED_EP1OUT_STAT,UOWN 
	retlw	0x00
        movf	BANKED_EP1OUT_CNT,W
	return

ReArmInterface:
	BANKSEL BANKED_EP1OUT_STAT
	btfsc	BANKED_EP1OUT_STAT,UOWN
	return
	movlw	HID_REPORT_BYTE_COUNT
	movwf	BANKED_EP1OUT_CNT

 	movlw	(1<<DTSEN)
	btfss	BANKED_EP1OUT_STAT,DTS
	iorlw	(1<<DTS)
	movwf	BANKED_EP1OUT_STAT
	bsf	BANKED_EP1OUT_STAT,UOWN

HIDSend:
	BANKSEL BANKED_EP1IN_STAT
	btfsc	BANKED_EP1IN_STAT,UOWN
	return

	movlw	HID_REPORT_BYTE_COUNT
	movwf	BANKED_EP1IN_CNT

 	movlw	(1<<DTSEN)
	btfss	BANKED_EP1IN_STAT,DTS
	iorlw	(1<<DTS)
	movwf	BANKED_EP1IN_STAT
	bsf	BANKED_EP1IN_STAT,UOWN

; After configuration is complete, this routine is called to initialize
; the endpoints (e.g., assign buffer addresses).
HIDInitEndpoints:
	BANKSel UEP1
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
	bsf	BANKED_EP1OUT_STAT,UOWN

	movlw	low HIDTxBuffer
	movwf	BANKED_EP1IN_ADRL
	movlw	high HIDTxBuffer
	movwf	BANKED_EP1IN_ADRH
       	movlw   (1<<DTSEN)
	movwf	BANKED_EP1IN_STAT

; Process HID specific requests
ProcessHIDRequest:

	BANKSEL SetupPacket
	movf	SETUP_wIndex0,W  ; is SetupPacket.wIndex != 0 : return
	btfss	STATUS,Z
	return
	movf	SETUP_bmRequestType,W ; if (SetupPacket.bmRequestType & 0x1F) != 0x01
	andlw	0x1F
	decfsz	WREG,W
	return

	movf	SETUP_bRequest,W
	xorlw	GET_DESCRIPTOR
	btfss	STATUS,Z
	goto	PHR_check_RequestType

	movf	SETUP_wValue1,W
	xorlw	HID_DESCRIPTOR    ; Was this  HID Descriptor
	btfss	STATUS,Z
	goto	PHR_check_REPORT_DESC

	bsf	StatusBits, status_request_handled ; Flag this was handled
	movlw	low HIDDescriptor                  ; Point to the HID Descriptor
	movwf	outPtrL
	movlw	high HIDDescriptor
	movwf	outPtrH
	movlw	HID_DESC_LEN
	movwf	wCount
	; Do we need to set RAM vs ROM?
	goto	PHR_check_RequestType

PHR_check_REPORT_DESC:
	movf	SETUP_wValue1,W
	xorlw	REPORT_DESCRIPTOR
	btfss	STATUS,Z
	goto	PHR_check_PHYSICAL

	bsf	StatusBits, status_request_handled
	movlw 	low HIDReport
	movwf	outPtrL
	movlw	high HIDReport
	movwf	outPtrH
	movlw	HID_REPORT_LEN
	movwf	wCount
	; Do we need to type to ROM?

PHR_check_PHYSICAL:
	; Do Nothing
PHR_check_RequestType:
	movf	SETUP_bmRequestType,W
	andlw	0x60
	xorlw	0x20
	btfss	STATUS,Z
	return

PHR_check_GET_REPORT:
	movf	SETUP_bRequest,W
	xorlw	GET_REPORT
	btfsc   STATUS,Z
        goto	PHR_exit
	movf	SETUP_bRequest,W
	xorlw	SET_REPORT
	btfss	STATUS,Z
	goto	PHR_check_GET_IDLE
	bsf	StatusBits,status_request_handled
	goto	PHR_exit
PHR_check_GET_IDLE:
	movf	SETUP_bRequest,W
	xorlw	GET_IDLE
	btfss	STATUS,Z
	goto	PHR_check_SET_IDLE
	bsf	StatusBits,status_request_handled
	movlw	low HidIdleRate
	movwf	outPtrL
	movlw	high HidIdleRate
	movwf	outPtrH
	movlw	1
	movwf	wCount
	; Set type to RAM?
	goto	PHR_exit

PHR_check_SET_IDLE:
	movf	SETUP_bRequest,W
	xorlw	SET_IDLE
	btfss	STATUS,Z
	goto	PHR_check_GET_PROTOCOL
	bsf	StatusBits,status_request_handled
	movf	SETUP_wValue1,W
	BANKSEL	HidIdleRate
	movwf	HidIdleRate
	goto	PHR_exit

PHR_check_GET_PROTOCOL:
	movf	SETUP_bRequest,W
	xorlw	GET_PROTOCOL
	btfss	STATUS,Z
	goto	PHR_check_SET_PROTOCOL
	movlw	low HidProtocol
	movwf	outPtrL
	movlw	high HidProtocol
	movwf	outPtrH
	movlw	1
	movwf	wCount
	; Set t ype to RAM?
	goto	PHR_exit

PHR_check_SET_PROTOCOL:
;
	movf	SETUP_bRequest,W
	xorlw	SET_PROTOCOL
	btfss	STATUS,Z
	goto	PHR_unknown
	bsf	StatusBits, status_request_handled
	movf	SETUP_wValue0,W
	BANKSEL	HidProtocol
	movwf	HidProtocol
	goto	PHR_exit

PHR_unknown:
	; Turn on the RED LED
PHR_exit:
	return

; Process GET_DESCRIPTOR
GetDescriptor:
	BANKSEL SetupPacket
	movf	SETUP_bmRequestType,W
	xorlw	0x80
	btfss	STATUS,Z
	goto	GD_exit

	movf	SETUP_wValue1,W
	xorlw	DEVICE_DESCRIPTOR
	btfss	STATUS,Z
	goto	GD_check_CONFIG
	bsf	StatusBits, status_request_handled
	movlw	low DeviceDescriptor
	movwf	outPtrL
	movlw	high DeviceDescriptor
	movwf	outPtrH
	movlw	DEVICE_DESC_LEN
	movwf	wCount
	; type = ROM?
	goto	GD_exit
GD_check_CONFIG:
	movf	SETUP_wValue1,W
	xorlw	CONFIGURATION_DESCRIPTOR
	btfss	STATUS,Z
	goto	GD_check_STRING
	bsf	StatusBits, status_request_handled
	movlw	low ConfigurationDescriptor
	movwf	outPtrL
	movlw	high ConfigurationDescriptor
	movwf	outPtrH
	movlw	CONFIG_DESC_LEN
	movwf	wCount
	; type = ROM
	goto	GD_exit
GD_check_STRING:
	movf	SETUP_wValue1,W
	xorlw	CONFIGURATION_DESCRIPTOR
	btfss	STATUS,Z
	goto	GD_exit
	bsf	StatusBits, status_request_handled

	movlw	high StringDescriptor0
	movwf	outPtrH

	movf	SETUP_wValue0,W
	btfsc	STATUS,Z
	goto	GD_Str0 ; Request for STR000
	decf	WREG,W
	btfsc	STATUS,Z
	goto	GD_Str1 ; Request for STR001
	decf	WREG,W
	btfsc	STATUS,Z 
	goto	GD_Str2 ; Request for STR002
	decf	WREG,W
	btfsc	STATUS,Z
	goto	GD_Str3 ; Request for STR002
	goto	GD_Str0 ; Error condition jump to 0x0
GD_Str3
	movlw	low StringDescriptor3
	movwf	outPtrL
	movlw	STR_DESC3_SIZE
	movwf	wCount
	goto	GD_exit
GD_Str2
	movlw	low StringDescriptor2
	movwf	outPtrL
	movlw	STR_DESC2_SIZE
	movwf	wCount
	goto	GD_exit
GD_Str1
	movlw	low StringDescriptor1
	movwf	outPtrL
	movlw	STR_DESC1_SIZE
	movwf	wCount
	goto	GD_exit
GD_Str0
	movlw	low StringDescriptor0
	movwf	outPtrL
	movlw	STR_DESC0_SIZE
	movwf	wCount
	; Fall through to exit
GD_exit:
	return
; Process GET_STATUS
GetStatus:
	BANKSEL ControlTransferBuffer
	clrf	ControlTransferBuffer
	clrf	ControlTransferBuffer+1

	BANKSEL	SetupPacket
	movf	SETUP_bmRequestType,W
	andlw	0x1F

	btfss	STATUS,Z
	goto	gs_check1
	bsf	StatusBits, status_request_handled
	btfsc	StatusBits, status_self_powered
	bsf	ControlTransferBuffer,1
	btfsc	StatusBits, status_remote_wakeup
	bsf	ControlTransferBuffer,2
	goto	gs_common

gs_check1:
        decfsz  WREG,W
	goto	gs_check2
	bsf	StatusBits, status_request_handled
	goto	gs_common
gs_check2:
;    else if (recipient == 0x02)
	decfsz	WREG,W
	goto	gs_common
;        StatusBits |= (1<< status_request_handled);
	bsf	StatusBits, status_request_handled


;        // Endpoint descriptors are 8 bytes long, with each in and out taking 4 bytes
;        // within the endpoint. (See PIC datasheet.)
;        inPtr = (uint8_t *)&Interfaces[0].Output + (endpointNum * 8);
	BANKSEL SetupPacket
	movf	SETUP_wIndex0,W
	andlw	0x0F

	movwf	bufferSize ; scratch pad copy of endpoint num
	btfsc	STATUS,Z
	goto	gs_have_offset
	clrf	WREG
gs_ep_loop
	addlw	0x08
	decfsz	bufferSize,f
	goto	gs_ep_loop

gs_have_offset ; W containt endpointNum * 8
	addlw	BANKED_EP0OUT_STAT
	movwf	inPtrL
	movlw	high BANKED_EP0OUT_STAT
	movwf	inPtrH
	movwf	FSR0H

;        if (endpointDir)
;            inPtr += 4;
	clrf	WREG
	btfsc	SETUP_wIndex0,7
	movlw	0x04
	addwf	inPtrL,f

;        if(*inPtr & BSTALL)
;            ControlTransferBuffer[0] = 0x01;
	movf	inPtrL,W
	movwf	FSR0L

	moviw	FSR0
	btfss	WREG,BSTALL
	goto	gs_common

	movlw	0x01
	BANKSEL ControlTransferBuffer
	movwf	ControlTransferBuffer
	goto	gs_common

gs_common:
	;if (RequestHandled)
	btfss	StatusBits, status_request_handled
	movlw	low ControlTransferBuffer
	movwf	outPtrL
	movlw	high ControlTransferBuffer
	movwf	outPtrH
	movlw	0x02
	movwf	wCount
	; Set type to RAM
	goto	gs_exit

gs_exit
	return

; Process SET_FEATURE and CLEAR_FEATURE
SetFeature:
;    uint8_t recipient = SetupPacket.bmRequestType & 0x1F;
;    uint8_t feature = SetupPacket.wValue0;
;
	BANKSEL SETUP_bmRequestType
	movf	SETUP_bmRequestType,W
	btfss	STATUS,Z
	goto	SF_check_02

	movf	SETUP_wValue0,W
	xorlw	DEVICE_REMOTE_WAKEUP
	btfss	STATUS,Z
	goto	SF_exit
	bsf	StatusBits, status_request_handled

	movf	SETUP_bRequest,W
	xorlw	SET_FEATURE
	bsf	StatusBits,status_remote_wakeup
	btfss	STATUS,Z
	bcf	StatusBits,status_remote_wakeup
	goto	SF_exit
SF_check_02
	xorlw	0x02
	btfss	STATUS,Z
	goto	SF_exit


;        if ((feature == ENDPOINT_HALT) && (endpointNum != 0))
;        {
	movf	SETUP_wValue0,W
	xorlw	ENDPOINT_HALT
	btfss	STATUS,Z
	goto	SF_exit
	movf	SETUP_wIndex0,W
	andlw	0x0F
	btfsc	STATUS,Z
	goto	SF_exit

;            // Halt endpoint (as long as it isn't endpoint 0)
;            StatusBits |= (1<< status_request_handled);
	bsf	StatusBits,status_request_handled
;            // Endpoint descriptors are 8 bytes long, with each in and out taking 4 bytes
;            // within the endpoint. (See PIC datasheet.)
;            inPtr = (uint8_t *)&Interfaces[0].Output + (endpointNum * 8);
	movf	SETUP_wIndex0,W
	andlw	0x0F
	movwf	bufferSize     ;endpoint numb
	
	btfsc	STATUS,Z
	goto	SF_have_offset
	clrf	WREG
SF_ep_loop
	addlw	0x08
	decfsz bufferSize,F
	goto	SF_ep_loop
SF_have_offset
	;W now contains offset to add  
	addlw	BANKED_EP0OUT
	movwf	inPtrL
	movlw	high BANKED_EP0OUT
	movwf	inPtrH
;            if (endpointDir)
;                inPtr += 4;
	clrf	WREG
	btfsc	SETUP_wIndex0,7
	movlw	0x04
	addwf	inPtrL,f

	movf	inPtrL,W
	movwf	FSR0L
	movf	inPtrH,W
	movwf	FSR0H

;            if(SetupPacket.bRequest == SET_FEATURE)
	movf	SETUP_bRequest,W
	xorlw	SET_FEATURE
	btfss	STATUS,Z
	goto	SF_check_dir
;                *inPtr = 0x84;
	movlw	0x84
	movwi	FSR0
	goto	SF_exit
	
;            {
SF_check_dir
;                if(endpointDir == 1)
;                    *inPtr = 0x00;
;                else
;                    *inPtr = 0x88;
	movlw	0x00
	btfsc 	SETUP_wIndex0,7
	movlw	0x88
	movwi	FSR0
        ; Drop through to SF_exit	
SF_exit
	return

ProcessStandardRequest:
;    uint8_t request = SetupPacket.bRequest;
;
;    if((SetupPacket.bmRequestType & 0x60) != 0x00) {
	BANKSEL SetupPacket
	movf	SETUP_bmRequestType,W
	andlw	0x60
	btfss	STATUS,Z
;        // Not a standard request - don't process here.  Class or Vendor
;        // requests have to be handled seperately.
;        return;
	return
;    }

;    if (request == SET_STATE_ADDRESS)
;    {
;            // Set the address of the device.  All future requests
;            // will come to that address.  Can't actually set UADDR
;            // to the new address yet because the rest of the SET_STATE_ADDRESS
;            // transaction uses address 0.
;            StatusBits |= (1<< status_request_handled);
	movf	SETUP_bRequest,W
	xorlw	SET_STATE_ADDRESS
	btfss	STATUS,Z
	goto	PSR_GET_DESCR
	bsf	StatusBits, status_request_handled
;            DeviceState = STATE_ADDRESS;
	movlw	STATE_ADDRESS
	movwf	DeviceState
;            DeviceAddress = SetupPacket.wValue0;
	movf	SETUP_wValue0,W
	movwf	DeviceAddress
	goto	PSR_exit
;    }
PSR_GET_DESCR
;    else if (request == GET_DESCRIPTOR)
	movf	SETUP_bRequest,W
	xorlw	GET_DESCRIPTOR
	btfss	STATUS,Z
	goto	PSR_SET_CONFIG
;{
;            GetDescriptor();
;}
	pagesel GetDescriptor
	call	GetDescriptor
	pagesel $
	goto	PSR_exit

PSR_SET_CONFIG
;    else if (request == SET_CONFIGURATION)
;    {
	movf	SETUP_bRequest,W
	xorlw	SET_CONFIGURATION
	btfss	STATUS,Z
	goto	PSR_GET_CONFIG

;            StatusBits |= (1<< status_request_handled);
	bsf	StatusBits, status_request_handled
;            CurrentConfiguration = SetupPacket.wValue0;
	movf	SETUP_wValue0,W
;            if (CurrentConfiguration == 0)
	BANKSEL CurrentConfiguration
	movwf	CurrentConfiguration
	btfss 	STATUS,Z
	goto	PSR_config_not_0
;                // If configuration value is zero, device is put in
;                // address state (USB 2.0 - 9.4.7)
;                DeviceState = STATE_ADDRESS;
	movlw	STATE_ADDRESS
	movwf	DeviceState
	goto	PSR_exit
PSR_config_not_0	
;                // Set the configuration.
;                DeviceState = STATE_CONFIGURED;
;
;                // Initialize the endpoints for all interfaces
;                HIDInitEndpoints();
;
;                // TBD: Add initialization code here for any additional
;                // interfaces beyond the one used for the HID
	movlw	STATE_CONFIGURED
	movwf	DeviceState
	pagesel	HIDInitEndpoints
	call	HIDInitEndpoints
	pagesel $
	goto	PSR_exit

PSR_GET_CONFIG
	movf	SETUP_bRequest,W
	xorlw	GET_CONFIGURATION
	btfss	STATUS,Z
	goto	PSR_GET_STATUS


;    else if (request == GET_CONFIGURATION)
;    {
;            StatusBits |= (1<< status_request_handled);
;            outPtr = (uint8_t*)&CurrentConfiguration;
;            wCount = 1;
;            StatusBits |= type_RAM;
;    }
PSR_GET_STATUS
;    else if (request == GET_STATUS)
	movf	SETUP_bRequest,W
	xorlw	GET_STATUS
	btfss	STATUS,Z
	goto	PSR_FEATURE
;    {
;            GetStatus();
;    }
	pagesel	GetStatus
	call	GetStatus
	pagesel $
	goto	PSR_exit

PSR_FEATURE
;    else if ((request == CLEAR_FEATURE) ||
;        (request == SET_FEATURE))
	movf	SETUP_bRequest,W
	xorlw	SET_FEATURE
	btfsc	STATUS,Z
	goto	PSR_SetFeature
	movf	SETUP_bRequest,W
	xorlw	CLEAR_FEATURE
	btfss	STATUS,Z
	goto	PSR_GET_INTF
PSR_SetFeature
;    {
;            SetFeature();
;    }
	pagesel	SetFeature
	call	SetFeature
	pagesel $
	goto	PSR_exit

PSR_GET_INTF
;    else if (request == GET_INTERFACE)
	movf	SETUP_bRequest,W
	xorlw	GET_INTERFACE
	btfss	STATUS,Z
	goto	PSR_SET_INTF
;            // No support for alternate interfaces.  Send
;            // zero back to the host.
;            StatusBits |= (1<< status_request_handled);
	bsf	StatusBits, status_request_handled
;            ControlTransferBuffer[0] = 0;
	BANKSEL	ControlTransferBuffer
	clrf	ControlTransferBuffer
;            outPtr = (uint8_t*) &ControlTransferBuffer;
	movlw	low ControlTransferBuffer
	movwf	outPtrL
	movlw	high ControlTransferBuffer
	movwf	outPtrH
;            wCount = 1;
	movlw	1
	movwf	wCount
	; Memory typeRAM
;            StatusBits |= type_RAM;
	goto	PSR_exit
;    }
PSR_SET_INTF
;    else if (request == SET_INTERFACE)
	movf	SETUP_bRequest,W
	xorlw	SET_INTERFACE
	btfss	STATUS,Z
;            // No support for alternate interfaces - just ignore.
;            StatusBits |= (1<< status_request_handled);
	goto	PSR_exit
	bsf	StatusBits, status_request_handled
PSR_exit
	return

; Data stage for a Control Transfer that sends data to the host
InDataStage:
;;; LABRAT Re-check this SUBLW .. 

;    // Determine how many bytes are going to the host
;    if(wCount < E0SZ)
;        bufferSize = wCount;
;    else
;        bufferSize = E0SZ;
;
	movlw	E0SZ
	subwf	wCount,W
	btfsc	STATUS,C
	goto	IDS_clear_BD
	; bufferSize = wCount
	movf	wCount,W
	movwf	bufferSize
	goto	IDS_clear_BD
IDS_buffer_E0SZ
	movlw	E0SZ
	movwf	bufferSize
        ; Fall through to IDS_clear_BD	
	; W to hold 'bufferSize'
IDS_clear_BD
;    // Load the high two bits of the byte count into BC8:BC9
;    Interfaces[0].Input.Stat &= ~(BC8 | BC9); // Clear BC8 and BC9
	BANKSEL BANKED_EP0IN_STAT
	bcf	BANKED_EP0IN_STAT,BC8
	bcf	BANKED_EP0IN_STAT,BC9
;    Interfaces[0].Input.Stat |= (uint8_t)((bufferSize & 0x0300) >> 8);
	; Do NOthing.. (or do I need to clear the bits?)
;    Interfaces[0].Input.Cnt = (uint8_t)(bufferSize & 0xFF);
	; Copy Buffer size to buffer descriptor
	movwf	BANKED_EP0IN_CNT
;    // Update the number of bytes that still need to be sent.  Getting
;    // all the data back to the host can take multiple transactions, so
;    // we need to track how far along we are.
;    wCount = wCount - bufferSize;
	; W still holds 'bufferSize'
	subwf  wCount,W
;    Interfaces[0].Input.Addr = PTR16(&ControlTransferBuffer);
	movlw	low ControlTransferBuffer
	movwf	BANKED_EP0IN_ADRL
	movwf	FSR0L	 ; point FSR0 at inPtr (aka ControlTransferBuffer)
	movlw	high ControlTransferBuffer
	movwf	BANKED_EP0IN_ADRH
	movwf	FSR0H
	movf	outPtrL,W ; point FSR1 at outPtr
	movwf	FSR1L
	movf	outPtrH,W
	movwf	FSR1H

;    // Move data to the USB output buffer from wherever it sits now.
;    inPtr = (uint8_t *) &ControlTransferBuffer;
;    NOTE: I removed the difference between ROM and RAM copies
;	if(StatusBits & type_ROM) 
;             for(i=0;i<bufferSize;i++) *inPtr++ = *ROMoutPtr++;
;	else 
;             for(i=0;i<bufferSize;i++) *inPtr++ = *outPtr++;
IDS_copy_loop
	moviw	FSR1++ ; copy from outPtr
	movwi	FSR0++ ; copy to inPtr
	decfsz	bufferSize,F
	goto	IDS_copy_loop
	movf	bufferSize,W
IDS_exit
	return	

;; Data stage for a Control Transfer that reads data from the host
OutDataStage:
	; Note: ASSUMES we only ever get a small (within 1 byte) buffer size

	movf	inPtrL,W
	movwf	FSR0L
	movf	inPtrH,W
	movwf	FSR0H
	movlw	low ControlTransferBuffer
	movwf	FSR1L
	movlw	high ControlTransferBuffer
	movwf	FSR1H
	

	BANKSEL BANKED_EP0OUT_STAT
	movf	BANKED_EP0OUT_CNT,W
	addwf	wCount,F

ods_loop
	moviw	FSR1++
	movwi	FSR0++
	decfsz	WREG,W
	goto	ods_loop
	return

; Process the Setup stage of a control transfer.  This code initializes the
; flags that let the firmware know what to do during subsequent stages of
; the transfer.
SetupStage:
    ; Note: Microchip says to turn off the UOWN bit on the IN direction as
    ; soon as possible after detecting that a SETUP has been received.
	BANKSEL BANKED_EP0IN_STAT
	bcf	BANKED_EP0IN_STAT,UOWN
	bcf	BANKED_EP0OUT_STAT,UOWN
    ; Initialize the transfer process
	movlw	STAGE_SETUP
	movwf	CtrlTransferStage
 	bcf  	StatusBits, status_request_handled ; Clear handled bit 
	clrf	wCount         ; No bytes transferred

    ; See if this is a standard (as definded in USB chapter 9) request
	pagesel	ProcessStandardRequest
	call	ProcessStandardRequest

    ; See if the HID class can do something with it.
    	pagesel	ProcessHIDRequest
    	call	ProcessHIDRequest
	pagesel	$

    ; If not request handled
	btfsc	StatusBits,status_request_handled
        goto	ss_device_to_host
        ; Service Was not handled - stall endpoint 0
	BANKSEL BANKED_EP0OUT_STAT
	movlw	E0SZ
	movwf	BANKED_EP0OUT_CNT
	movlw	low SetupPacket
	movf	BANKED_EP0OUT_ADRL,W
	movwf	high SetupPacket
	movwf	BANKED_EP0OUT_ADRH
	movlw	(_BSTALL)
	movwf	BANKED_EP0OUT_STAT
	movwf	BANKED_EP0IN_STAT
	bsf	BANKED_EP0OUT_STAT,UOWN
	bsf	BANKED_EP0IN_STAT,UOWN
	goto	SetupStage_exit

ss_device_to_host:
	BANKSEL SetupPacket
	movwf	SETUP_bmRequestType
	xorlw	0x80
	btfsc   STATUS,Z
	goto	host_to_device

        ; Device-to-host   *** LABRAT: come back and re-check this
        ; NOTE: L vs H *may* be incorrect
	BANKSEL SETUP_wLengthL
	movf	SETUP_wLengthL,W
	BANKSEL	wCount
	subwf	wCount,w   ; wCount > wLength C=0
	btfsc   STATUS,C
        goto    ss_in_data 
	BANKSEL SETUP_wLengthL
	movwf	SETUP_wLengthL
	BANKSEL wCount
	movwf	wCount
ss_in_data:
	pagesel	InDataStage
	call	InDataStage
	pagesel $

	movlw	STAGE_DATA_IN
	movwf	CtrlTransferStage

	BANKSEL BANKED_EP0OUT_STAT
	movlw	E0SZ
	movwf	BANKED_EP0OUT_CNT
	movlw  	low SetupPacket
	movwf	BANKED_EP0OUT_ADRL
	movlw	high SetupPacket
	movwf	BANKED_EP0OUT_ADRH
	movlw	(1<<UOWN)
	movwf	BANKED_EP0OUT_STAT

	movlw 	low ControlTransferBuffer
	movwf	BANKED_EP0IN_ADRL
	movlw	high ControlTransferBuffer
	movwf	BANKED_EP0IN_ADRH
	movlw   (1<< DTS) | (1<<DTSEN)
	movwf	BANKED_EP0IN_STAT
	bsf	BANKED_EP0IN_STAT,UOWN
host_to_device:

	movlw	STAGE_DATA_OUT
	movwf	CtrlTransferStage

        ; Clear the input buffer descriptor
	BANKSEL	BANKED_EP0IN_STAT
	clrf	BANKED_EP0IN_CNT
	movlw	(1<<DTS)|(1<<DTSEN)
	movwf	BANKED_EP0IN_STAT
	bsf	BANKED_EP0IN_STAT,UOWN

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
	bsf	BANKED_EP0OUT_STAT,UOWN

SetupStage_exit:
	BANKSEL	UCON
    	bcf	UCON,PKTDIS 
	
; Configures the buffer descriptor for endpoint 0 so that it is waiting for
; the status stage of a control transfer.
WaitForSetupStage:
	movlw	STAGE_SETUP
	movwf	CtrlTransferStage
	BANKSEL BANKED_EP0OUT_STAT
	movlw	E0SZ
	movwf	BANKED_EP0OUT_CNT
	movlw	low SetupPacket
	movwf	BANKED_EP0OUT_ADRL
	movlw	high SetupPacket
	movwf	BANKED_EP0OUT_ADRH
	movlw	(1<<DTSEN)
	movwf	BANKED_EP0OUT_STAT
	bsf	BANKED_EP0OUT_STAT,UOWN
	clrf	BANKED_EP0IN_STAT

; This is the starting point for processing a Control Transfer.  The code directly
; follows the sequence of transactions described in the USB spec chapter 5.  The
; only Control Pipe in this firmware is the Default Control Pipe (endpoint 0).
; Control messages that have a different destination will be discarded.
ProcessControlTransfer:
	BANKSEL USTAT
	movf	USTAT,W
	btfss	STATUS,Z
     	goto	PCT_EP0IN
   
 	; Was this PID = 0x0D 
	BANKSEL	BANKED_EP0OUT_STAT 
	movf	BANKED_EP0OUT_STAT,W
	andlw	0x3C  ; Mask PID from middle of BD0STAT
	xorlw	(0x0D<<2) ; (PID = 0x0D <<2)
	btfss 	STATUS,Z
	goto	PCT_1
	pagesel	SetupStage  ; if PID==0x0D then call SetupStage
	call	SetupStage
	goto	PCT_exit

PCT_1:
	; else  are we in STAGE_DATA_OUT
	movf	CtrlTransferStage,W
	xorlw	STAGE_DATA_OUT
	btfss 	STATUS,Z
	goto	PCT_2
	pagesel	OutDataStage
	call	OutDataStage
	pagesel	$
	movlw	(1<<DTSEN)
	BANKSEL BANKED_EP0OUT_STAT
	btfss	BANKED_EP0OUT_STAT,DTS
	xorlw	(1<<DTS)
	movwf	BANKED_EP0OUT_STAT
	bsf	BANKED_EP0OUT, UOWN
	goto	PCT_exit
PCT_2:
	; Prepare for the Setup Stage control transfer
	pagesel WaitForSetupStage
	call	WaitForSetupStage
	pagesel	$
	goto	PCT_exit

PCT_EP0IN:
	xorlw	0x04 ; Endpoint 0: IN
	btfss	STATUS,Z
	goto	PCT_exit

	; if ((UADDR==0) && (DeviceState == STATE_ADDRESS))
	movf 	UADDR,W
	btfss	STATUS,Z
	goto	PCT_data_in
	movf	DeviceState,W
	xorlw	STATE_ADDRESS
	btfss	STATUS,Z
	goto	PCT_data_in

	BANKSEL	SETUP_wValue0
	movf	SETUP_wValue0,W
	BANKSEL	UADDR
	movwf	UADDR
	btfsc	STATUS,Z
	goto	PCT_data_in
	movlw	STATE_DEFAULT
	movwf	DeviceState

PCT_data_in:
	; if (CtrlTransferStage == STAGE_DATA_IN)
	movf	CtrlTransferStage,W
	xorlw	STAGE_DATA_IN
	btfss	STATUS,Z
	goto	PCT_w4s
	pagesel	InDataStage
	call	InDataStage
	pagesel	$

	movlw	(1<<DTSEN)
	BANKSEL	BANKED_EP0IN_STAT
	btfss	BANKED_EP0IN_STAT,DTS
	xorlw	(1<<DTS)
	movwf	BANKED_EP0IN_STAT
	bsf	BANKED_EP0IN_STAT,UOWN
	goto	PCT_exit

PCT_w4s:
	pagesel	WaitForSetupStage
	call	WaitForSetupStage
	pagesel $

PCT_exit:

InitializeUSB:
        BANKSEL StatusBits
	clrf	StatusBits	
	BANKSEL UCFG
	movlw 0x14
	movwf UCFG  ; Enable Pullup resistors; full speed mode; No PingPong
	movlw STATE_DETACHED
	movwf DeviceState
	bcf   StatusBits,status_remote_wakeup
	clrf  CurrentConfiguration
	clrf  UADDR ; Reset USB Address
	clrf  UEIR  ; Clear all USB Error Interrupt Flags
      ; Reset PP buffers
	bsf   UCON, PPBRST
	bcf   UCON, PPBRST

        bcf   UCON,PKTDIS  ; Enable Packet Transfers

EnableUSBModule:
    ; TBD: Check for voltage coming from the USB cable and use that
    ; as an indication we are attached.
	BANKSEL UCON
	btfss	UCON,USBEN
	goto	eum_next
	clrf	UCON
	clrf	UIE
	bsf	UCON,USBEN
	movlw	STATE_ATTACHED
	movwf	DeviceState
eum_next:
        btfsc   UCON,SE0
        goto	$-1

	clrf	UIR
	clrf	UIE
	bsf	UIE,URSTIE
	bsf	UIE,IDLEIE
	movlw	STATE_POWERED
	movwf	DeviceState

; Unsuspend the device
UnSuspend:
      BANKSEL UCON
      bcf UCON,SUSPND
      bcf UIE, ACTVIE
      movlw 0xFB
      andwf UIR,F

; Full speed devices get a Start Of Frame (SOF) packet every 1 millisecond.
; Nothing is currently done with this interrupt (it is simply masked out).
StartOfFrame:
      BANKSEL UIR
      bcf UIR, SOFIF

; This routine is called in response to the code stalling an endpoint.
Stall:
	BANKSEL UEP0
	btfss	UEP0, EPSTALL
	goto  	stall_exit
	pagesel WaitForSetupStage
        call	WaitForSetupStage
	BANKSEL UEP0
	bcf	UEP0, EPSTALL
stall_exit:
        bcf	UIR, STALLIF

; Suspend all processing until we detect activity on the USB bus
Suspend:
	BANKSEL UIE
	bsf	UIE, ACTVIE
	movlw	0xEF
	andwf	UIR,f
	bsf	UCON, SUSPND

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
	goto 	$-2

        bcf     UCON, PKTDIS ; Enable packet processing

	bcf	StatusBits,status_remote_wakeup ; Remote wakeup is off by default
	bcf	StatusBits,status_self_powered  ; Self powered is off by default
	clrf 	CurrentConfiguration ; Clear active configuration

	movlw	STATE_DEFAULT
	movwf	DeviceState

; Main entry point for USB tasks.  Checks interrupts, then checks for transactions.
ProcessUSBTransactions:
	; See if device is connected yet
	BANKSEL DeviceState
 	movf	DeviceState,W
	xorlw   STATE_DETACHED
	btfss   STATUS,Z
	goto	PUT_UNSUSPEND
        
	BANKSEL	UIR ; Clear all interrupts
	clrf	UIR

	BANKSEL PIR2 ; Clear Global USB Interrupt
	bcf	PIR2,USBIF
	return

PUT_UNSUSPEND:
    ; If the USB became active then wake up from suspend
	BANKSEL UIR
	btfss	UIR,ACTVIF
	goto	PUT_SUSPEND
	btfss	UIE,ACTVIE
	goto	PUT_SUSPEND

	pagesel UnSuspend	
	call	UnSuspend
	pagesel	$
	
  	BANKSel UIR
	bcf	UIR,ACTVIF

PUT_SUSPEND:
    ; If we are supposed to be suspended, then cease performing any processing
	btfss	UCON,SUSPND
	goto	PUT_RESET
	
	clrf	UIR

	BANKSEL PIR2
	bcf	PIR2,USBIF
	return

PUT_RESET:
     ; Process a bus reset
	btfss	UIR, URSTIF
	goto	PUT_IDLE
	btfss	UIE, URSTIE
	goto	PUT_IDLE
	pagesel BusReset
	call	BusReset
	pagesel $

	BANKSEL UIR
	bcf	UIR,URSTIF

PUT_IDLE:
        ; Check for No bus activity for a while - suspend the firmware
	btfss 	UIR,IDLEIF
	goto	PUT_SOF
	btfss	UIE,IDLEIE
	goto	PUT_SOF
	pagesel Suspend
	call	Suspend
	pagesel $
	
	BANKSEL UIR
  	bcf	UIR,IDLEIF	

PUT_SOF:
    	; Start of Frame (SOF)
	btfss	UIR, SOFIF
	goto	PUT_STALL
	btfss	UIE, SOFIE
	goto	PUT_STALL
	pagesel StartOfFrame
	call	StartOfFrame
	pagesel $

	BANKSEL	UIR
	bcf	UIR,SOFIF

PUT_STALL:
	; Check for STALL
	btfss	UIR,STALLIF
	goto	PUT_UERR
	btfss	UIE,STALLIE
	goto	PUT_UERR
	pagesel Stall
	call	Stall
	pagesel $

	BANKSEL	UIR
	bcf	UIR,STALLIF

PUT_UERR:
	btfss	UIR,UERRIF
	goto	PUT_OTHER
	btfss	UIE,UERRIE
	goto	PUT_OTHER

	bcf	UIR, UERRIF
	clrf	UEIR

	bcf	UIR,UERRIF

PUT_OTHER:
	; Unless we have been reset by the host, no need to keep processing
	movlw	STATE_DEFAULT
	subwf	DeviceState,W

	btfsc	STATUS,C
	goto	PUT_PCT
	
	BANKSEL	UIR
	clrf	UIR  ; Clear All Interrupt Flags

	BANKSEL PIR2
	bcf	PIR2,USBIF ; Clear Global USB Interrupt Flag
	return

PUT_PCT:
	btfss	UIR, TRNIF
	goto	PUT_exit
	btfss	UIE, TRNIE
	goto	PUT_exit
	pagesel	ProcessControlTransfer
	call	ProcessControlTransfer
	pagesel $

	BANKSEL	UIR
	bcf	UIR,TRNIF

PUT_exit:
	BANKSEL PIR2
	bcf	PIR2,USBIF ; Clear Global USB Interrupt Flag
	return

	end
