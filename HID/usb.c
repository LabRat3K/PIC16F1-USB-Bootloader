/*
 * File:   Usb.asm
 * Hand coded by Andrew Williams, based on C source by 
 * Author: Szymon Roslowski
 *
 * Created on 13 October 2014, 17:46
 * Tweaked Jan 23, 2023 by Andrew Williams (SDCC compatible)
 *
 * Firmware framework for USB I/O on PIC 16F1455 (and siblings)
 *
 */


#include "usb.h"
#include "usb_descriptors.h"
#include "pic16f1455.h"

/***********************/
/* Local Definitions   */
/***********************/
// Commands
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

// Descriptor Types
#define DEVICE_DESCRIPTOR           0x01
#define CONFIGURATION_DESCRIPTOR    0x02
#define STRING_DESCRIPTOR           0x03
#define INTERFACE_DESCRIPTOR        0x04
#define ENDPOINT_DESCRIPTOR         0x05

// Class Descriptor Types
#define HID_DESCRIPTOR              0x21
#define REPORT_DESCRIPTOR           0x22
#define PHYSICAL_DESCRIPTOR         0x23

// HID Class specific requests
#define GET_REPORT                  0x01
#define GET_IDLE                    0x02
#define GET_PROTOCOL                0x03
#define SET_REPORT                  0x09
#define SET_IDLE                    0x0A
#define SET_PROTOCOL                0x0B

// Standard Feature Selectors
#define DEVICE_REMOTE_WAKEUP        0x01
#define ENDPOINT_HALT               0x00

// Device states (Chap 9.1.1)
#define STATE_DETACHED              0x00
#define STATE_ATTACHED              0x01
#define STATE_POWERED               0x02
#define STATE_DEFAULT               0x03
#define STATE_ADDRESS               0x04
#define STATE_CONFIGURED            0x05

/* Interrupt */
#define USB_SOF                     0x40
#define USB_STALL                   0x20
#define USB_IDLE                    0x10
#define USB_TRN                     0x08
#define USB_RESUM                   0x04
#define USB_UERR                    0x02
#define USB_URST                    0x01

// Buffer Descriptor bit masks (from PIC datasheet)
#define UOWN                        0x80 // USB Own Bit
#define DTS                         0x40 // Data Toggle Synchronization Bit
#define KEN                         0x20 // BD Keep Enable Bit
#define INCDIS                      0x10 // Address Increment Disable Bit
#define DTSEN                       0x08 // Data Toggle Synchronization Enable Bit
#define BSTALL                      0x04 // Buffer Stall Enable Bit
#define BC9                         0x02 // Byte count bit 9
#define BC8                         0x01 // Byte count bit 8

// Control Transfer Stages - see USB spec chapter 5
#define STAGE_SETUP                 0x00 // Start of a control transfer (followed by 0 or more data stages)
#define STAGE_DATA_OUT              0x01 // Data from host to device
#define STAGE_DATA_IN               0x02 // Data from device to host
#define STAGE_STATUS                0x03 // Unused - if data I/O went ok, then back to Setup

// Hardware
#define USB_RESET_FLAG              UIRbits.URSTIF
#define USB_RESUME_FLAG             UIRbits.ACTVIF
#define USB_IDLE_FLAG               UIRbits.IDLEIF
#define USB_STALL_FLAG              UIRbits.STALLIF
#define USB_SOF_FLAG                UIRbits.SOFIF
#define USB_ERROR_FLAG              UIRbits.UERRIF
#define USB_TRANSACTION_FLAG        UIRbits.TRNIF

/***********************/
/* Structures          */
/***********************/

typedef struct _BDT
{
    uint8_t Stat;
    uint8_t Cnt;
    uint16_t Addr;
} BDT; //Buffer Descriptor Table

typedef struct _Interface
{
    BDT Output;
    BDT Input;
} Interface;

// Every device request starts with an 8 byte setup packet (USB 2.0, chap 9.3)
// with a standard layout.  The meaning of wValue and wIndex will
// vary depending on the request type and specific request.
typedef struct _setupPacketStruct
{
    uint8_t bmRequestType; // D7: Direction, D6..5: Type, D4..0: Recipient
    uint8_t bRequest;      // Specific request
    uint8_t wValue0;       // LSB of wValue
    uint8_t wValue1;       // MSB of wValue
    uint8_t wIndex0;       // LSB of wIndex
    uint8_t wIndex1;       // MSB of wIndex
    uint16_t wLength;       // Number of bytes to transfer if there's a data stage
    uint8_t extra[1];      // Fill out to same size as Endpoint 0 max buffer (E0SZ-7)
} setupPacketStruct;

/***********************/
/* Local Variables     */
/***********************/
// Bit index of Status Bits
#define  status_remote_wakeup 	 (1)
#define  status_self_powered     (2)
#define  status_request_handled  (3)
#define  status_transfer_type    (4)

#define type_RAM (0x00)
#define type_ROM (1<<status_transfer_type)

uint8_t StatusBits;    // Global in the 0x70-7F range

uint8_t DeviceAddress;
uint8_t CtrlTransferStage; // Holds the current stage in a control transfer
uint8_t CurrentConfiguration;

uint8_t HidIdleRate;
uint8_t HidProtocol;       // [0] Boot Protocol [1] Report Protocol

const uint8_t *ROMoutPtr;  // Data to send to the host
uint8_t *outPtr;           // Data to send to the host
uint8_t *inPtr;            // Data from the host
uint16_t wCount;           // Number of bytes of data

volatile setupPacketStruct SetupPacket;
volatile uint8_t ControlTransferBuffer[E0SZ];

volatile uint8_t DeviceState;

// !!! It is ABSOLUTELY VITAL for the start of BDTs to point to 0x2000.
// !!! Won't work without it.
// *** LABRAT: SDCC needed to declare the BANKED start address (as we are only using EP0 & 1 .. fits in a single bank so no need to worry)
//volatile Interface Interfaces[InterfaceCount + 1] @ 0x2000;
volatile __data __at (0x020) Interface Interfaces[InterfaceCount + 1];
// ... The hours I've waisted before I found out... :(


/***********************/
/* Implementation      */
/***********************/
uint8_t IsUsbDataAvailable()
{
   __asm
	BANKSEL BANKED_EP1OUT_STAT
	btfsc BANKED_EP1OUT_STAT,UOWN
	retlw 0x00
        movfw BANKED_EP1OUT_CNT
	return
   __endasm;
   return 0x00;
}

void ReArmInterface()
{
  __asm
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
  __endasm;
}

void HIDSend()
{
  __asm
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
  __endasm;
}

// After configuration is complete, this routine is called to initialize
// the endpoints (e.g., assign buffer addresses).
void HIDInitEndpoints(void)
{
   __asm
	BANKSel UEP1
	movlw 	0x1E
	movwf 	UEP1

        BANKSEL BANKED_EP1OUT_STAT
	movlw 	HID_REPORT_BYTE_COUNT
        movwf 	BANKED_EP1OUT_CNT

	movlw 	low _HIDRxBuffer
	movwf	BANKED_EP1OUT_ADRL
	movlw	high _HIDRxBuffer
	movwf   BANKED_EP1OUT_ADRH

       	movlw   (1<<DTSEN)
	movwf	BANKED_EP1OUT_STAT
	bsf	BANKED_EP1OUT_STAT,UOWN

	movlw	low _HIDTxBuffer
	movwf	BANKED_EP1IN_ADRL
	movlw	high _HIDTxBuffer
	movwf	BANKED_EP1IN_ADRH
       	movlw   (1<<DTSEN)
	movwf	BANKED_EP1IN_STAT
  __endasm;
}

// Process HID specific requests
void ProcessHIDRequest(void)
{
    uint8_t bRequest;

     // Has to be to the HID interface
    if((SetupPacket.bmRequestType & 0x1F) != 0x01 || (SetupPacket.wIndex0 != 0x00)) return;

    bRequest = SetupPacket.bRequest;

    if (bRequest == GET_DESCRIPTOR)
    {
        // Request for a descriptor.
        uint8_t descriptorType  = SetupPacket.wValue1;
        if (descriptorType == HID_DESCRIPTOR)
        {
            StatusBits |= (1<< status_request_handled);
            ROMoutPtr = (const uint8_t*) &ConfigurationDescriptor.HIDDescriptor;
            wCount = sizeof(ConfigurationDescriptor.HIDDescriptor);
            StatusBits |=type_ROM;
        }
        else if (descriptorType == REPORT_DESCRIPTOR)
        {
            StatusBits |= (1<< status_request_handled);
            ROMoutPtr = (const uint8_t*) HIDReport;
            wCount = sizeof(HIDReport);
            StatusBits |=type_ROM;
        }
        else if (descriptorType == PHYSICAL_DESCRIPTOR)
        {   // Do Nothing
        }
        else
        {   // Unsupported Descriptor
        }
    }

    if ((SetupPacket.bmRequestType & 0x60) != 0x20)
    {
        return;
    }

    // HID-specific requests.
    if (bRequest == GET_REPORT)
    {   // Do Nothing
    }

    else if (bRequest == SET_REPORT)
    {
       StatusBits |= (1<< status_request_handled);
    }

    else if (bRequest == GET_IDLE)
    {
        StatusBits |= (1<< status_request_handled);
        outPtr = &HidIdleRate;
        wCount = 1;
        StatusBits |= type_RAM;
    }

    else if (bRequest == SET_IDLE)
    {
       StatusBits |= (1<< status_request_handled);
       HidIdleRate = SetupPacket.wValue1;
    }

    else if (bRequest == GET_PROTOCOL)
    {
       StatusBits |= (1<< status_request_handled);
        outPtr = &HidProtocol;
        wCount = 1;
	StatusBits |= type_RAM;
    }

    else if (bRequest == SET_PROTOCOL)
    {
       StatusBits |= (1<< status_request_handled);
        HidProtocol = SetupPacket.wValue0;
    }

    else
    {   // Unknown Request
    }
}

// Process GET_DESCRIPTOR
static void GetDescriptor(void)
{
    if(SetupPacket.bmRequestType == 0x80)
    {
        uint8_t descriptorType  = SetupPacket.wValue1;
        uint8_t descriptorIndex = SetupPacket.wValue0;

        if (descriptorType == DEVICE_DESCRIPTOR)
        {
                StatusBits |= (1<< status_request_handled);
                ROMoutPtr = (const uint8_t *) &DeviceDescriptor;
                wCount = sizeof(DeviceDescriptor);
                StatusBits |=type_ROM;
        }
        else if (descriptorType == CONFIGURATION_DESCRIPTOR)
        {
                StatusBits |= (1<< status_request_handled);
                ROMoutPtr = (const uint8_t*) &ConfigurationDescriptor;
                wCount = sizeof(ConfigurationDescriptor);
		StatusBits |=type_ROM;
        }
        else if (descriptorType == STRING_DESCRIPTOR)
        {
                StatusBits |= (1<< status_request_handled);
                if(descriptorIndex >= StringDescriptorCount)
                    ROMoutPtr = (const uint8_t*) &StringDescriptor0;
                else
                    ROMoutPtr = *(StringDescriptorPointers + descriptorIndex);

                wCount = *ROMoutPtr;
		StatusBits |=type_ROM;
        }
        else
        {   // Unknown Descriptor
        }
    }
}

// Process GET_STATUS
static void GetStatus(void)
{
    // Mask off the Recipient bits
    uint8_t recipient = SetupPacket.bmRequestType & 0x1F;
    ControlTransferBuffer[0] = 0;
    ControlTransferBuffer[1] = 0;

    // See where the request goes
    if (recipient == 0x00)
    {
        // Device
        StatusBits |= (1<< status_request_handled);
        // Set bits for self powered device and remote wakeup.
        if (StatusBits & status_self_powered)
            ControlTransferBuffer[0] |= 0x01;
        if (StatusBits & status_remote_wakeup)
            ControlTransferBuffer[0] |= 0x02;
    }
    else if (recipient == 0x01)
    {
        // Interface
        StatusBits |= (1<< status_request_handled);
    }
    else if (recipient == 0x02)
    {
        // Endpoint
        uint8_t endpointNum = SetupPacket.wIndex0 & 0x0F;
        uint8_t endpointDir = SetupPacket.wIndex0 & 0x80;
        StatusBits |= (1<< status_request_handled);
        // Endpoint descriptors are 8 bytes long, with each in and out taking 4 bytes
        // within the endpoint. (See PIC datasheet.)
        inPtr = (uint8_t *)&Interfaces[0].Output + (endpointNum * 8);
        if (endpointDir)
            inPtr += 4;
        if(*inPtr & BSTALL)
            ControlTransferBuffer[0] = 0x01;
    }

    if (StatusBits & (1<<status_request_handled))
    {
        outPtr = (uint8_t *) &ControlTransferBuffer;
        wCount = 2;
	StatusBits |= type_RAM;
    }
}

// Process SET_FEATURE and CLEAR_FEATURE
static void SetFeature(void)
{
    uint8_t recipient = SetupPacket.bmRequestType & 0x1F;
    uint8_t feature = SetupPacket.wValue0;

    if (recipient == 0x00)
    {
        // Device
        if (feature == DEVICE_REMOTE_WAKEUP)
        {
            StatusBits |= (1<< status_request_handled);
            if (SetupPacket.bRequest == SET_FEATURE)
                StatusBits |= (1<<status_remote_wakeup);
            else
                StatusBits &= ~(1<<status_remote_wakeup);
        }
        // TBD: Handle TEST_MODE
    }
    else if (recipient == 0x02)
    {
        // Endpoint
        uint8_t endpointNum = SetupPacket.wIndex0 & 0x0F;
        uint8_t endpointDir = SetupPacket.wIndex0 & 0x80;
        if ((feature == ENDPOINT_HALT) && (endpointNum != 0))
        {
            // Halt endpoint (as long as it isn't endpoint 0)
            StatusBits |= (1<< status_request_handled);
            // Endpoint descriptors are 8 bytes long, with each in and out taking 4 bytes
            // within the endpoint. (See PIC datasheet.)
            inPtr = (uint8_t *)&Interfaces[0].Output + (endpointNum * 8);
            if (endpointDir)
                inPtr += 4;

            if(SetupPacket.bRequest == SET_FEATURE)
                *inPtr = 0x84;
            else
            {
                if(endpointDir == 1)
                    *inPtr = 0x00;
                else
                    *inPtr = 0x88;
            }
        }
    }
}

void ProcessStandardRequest(void)
{
    uint8_t request = SetupPacket.bRequest;

    if((SetupPacket.bmRequestType & 0x60) != 0x00) {
        // Not a standard request - don't process here.  Class or Vendor
        // requests have to be handled seperately.
        return;
    }


    if (request == SET_STATE_ADDRESS)
    {
            // Set the address of the device.  All future requests
            // will come to that address.  Can't actually set UADDR
            // to the new address yet because the rest of the SET_STATE_ADDRESS
            // transaction uses address 0.
            StatusBits |= (1<< status_request_handled);
            DeviceState = STATE_ADDRESS;
            DeviceAddress = SetupPacket.wValue0;
    }
    else if (request == GET_DESCRIPTOR)
    {
            GetDescriptor();
    }
    else if (request == SET_CONFIGURATION)
    {
            StatusBits |= (1<< status_request_handled);
            CurrentConfiguration = SetupPacket.wValue0;
            // TBD: ensure the new configuration value is one that
            // exists in the descriptor.
            if (CurrentConfiguration == 0)
                // If configuration value is zero, device is put in
                // address state (USB 2.0 - 9.4.7)
                DeviceState = STATE_ADDRESS;
            else
            {
                // Set the configuration.
                DeviceState = STATE_CONFIGURED;

                // Initialize the endpoints for all interfaces
                HIDInitEndpoints();

                // TBD: Add initialization code here for any additional
                // interfaces beyond the one used for the HID
            }
    }
    else if (request == GET_CONFIGURATION)
    {
            StatusBits |= (1<< status_request_handled);
            outPtr = (uint8_t*)&CurrentConfiguration;
            wCount = 1;
            StatusBits |= type_RAM;
    }
    else if (request == GET_STATUS)
    {
            GetStatus();
    }
    else if ((request == CLEAR_FEATURE) ||
        (request == SET_FEATURE))
    {
            SetFeature();
    }
    else if (request == GET_INTERFACE)
    {
            // No support for alternate interfaces.  Send
            // zero back to the host.
            StatusBits |= (1<< status_request_handled);
            ControlTransferBuffer[0] = 0;
            outPtr = (uint8_t*) &ControlTransferBuffer;
            wCount = 1;
            StatusBits |= type_RAM;
    }
    else if (request == SET_INTERFACE)
    {
            // No support for alternate interfaces - just ignore.
            StatusBits |= (1<< status_request_handled);
    }
/* LabRat commented out these empty options
    else if (request == SET_DESCRIPTOR)
    {
    }
    else if (request == SYNCH_FRAME)
    {
    }
    else
    {
    }
*/
}

// Data stage for a Control Transfer that sends data to the host
void InDataStage(void)
{
    uint8_t i;
    uint16_t bufferSize;

    // Determine how many bytes are going to the host
    if(wCount < E0SZ)
        bufferSize = wCount;
    else
        bufferSize = E0SZ;

    // Load the high two bits of the byte count into BC8:BC9
    Interfaces[0].Input.Stat &= ~(BC8 | BC9); // Clear BC8 and BC9
    Interfaces[0].Input.Stat |= (uint8_t)((bufferSize & 0x0300) >> 8);
    Interfaces[0].Input.Cnt = (uint8_t)(bufferSize & 0xFF);
    Interfaces[0].Input.Addr = PTR16(&ControlTransferBuffer);

    // Update the number of bytes that still need to be sent.  Getting
    // all the data back to the host can take multiple transactions, so
    // we need to track how far along we are.
    wCount = wCount - bufferSize;

    // Move data to the USB output buffer from wherever it sits now.
    inPtr = (uint8_t *) &ControlTransferBuffer;
	if(StatusBits & type_ROM) 
             for(i=0;i<bufferSize;i++) *inPtr++ = *ROMoutPtr++;
	else 
             for(i=0;i<bufferSize;i++) *inPtr++ = *outPtr++;
}

// Data stage for a Control Transfer that reads data from the host
void OutDataStage(void)
{
    uint16_t i, bufferSize;

    bufferSize = ((0x03 & Interfaces[0].Output.Stat) << 8) | Interfaces[0].Output.Cnt;

    // Accumulate total number of bytes read
    wCount = wCount + bufferSize;

    outPtr = (uint8_t*) &ControlTransferBuffer; // This is at *MOST* 8 bytes

    for (i=0;i<bufferSize;i++)
    {
        *inPtr++ = *outPtr++;
    }
}

// Process the Setup stage of a control transfer.  This code initializes the
// flags that let the firmware know what to do during subsequent stages of
// the transfer.
void SetupStage(void)
{
  __asm
    ; Note: Microchip says to turn off the UOWN bit on the IN direction as
    ; soon as possible after detecting that a SETUP has been received.
	BANKSEL BANKED_EP0INPUT_STAT
	bcf	BANKED_EP0INPUT_STAT,UOWN
	bcf	BANKED_EP0OUT_STAT,UOWN
    ; Initialize the transfer process
	movlw	STAGE_SETUP
	movwf	_CtrlTransferStage
 	bcf  	_StatusBits, status_request_handled ; Clear handled bit 
	clrf	_wCount         ; No bytes transferred

    ; See if this is a standard (as definded in USB chapter 9) request
	pagesel	_ProcessStandardRequest
	call	_ProcessStandardRequest

    ; See if the HID class can do something with it.
    	pagesel	_ProcessHIDRequest
    	call	_ProcessHIDRequest
	pagesel	$

    ; If not request handled
	btfsc	_StatusBits,status_request_handled
        goto	czeck_device_to_host
        ; Service Was not handled - stall endpoint 0
	BANKSEL BANKED_EP0OUT_STAT
	movlw	_E0SZ
	movwf	BANKED_EP0OUT_CNT
	movlw	low _SetupPacket
	movf	BANKED_EP0OUT_ADRL,W
	movwf	high _SetupPacket
	movwf	BANKED_EP0OUT_ADRH
	movlw	(_BSTALL)
	movwf	BANKED_EP0OUT_STAT
	movwf	BANKED_EP0IN_STAT
	bsf	BANKED_EP0OUT_STAT,UOWN
	bsf	BANKED_EP0IN_STAT,UOWN
	goto	SetupStage_exit

czeck_device_to_host:
	BANKSEL BANKED_SETUP_PACKET
	movwf	SETUP_PACKET_bmRequestType
	xorlw	0x80
	btfsc   STATUS,Z
	goto	host_to_device

        ; Device-to-host
	BANKSEL	_wCount
	movf	SETUP_PACKET_wLength,W
	subwf	SETUP_PACKET_wLength,w   ; wCount > wLength C=0
	btfsc   STATUS,C
        goto    $4
	movwf	SETUP_PACKET_wLength
	BANKSEL _wCount
	movwf	SETUP_PACKEt_wLength
	
	pagesel	_InDataStage
	call	_InDataStage
	pagesel $

	movlw	STAGE_DATA_IN
	movwf	_CtrlTransferStage

	BANKSEL BANKED_EP0OUT_STAT
	movlw	E0SZ
	movwf	BANKED_EP0OUT_CNT
	movlw  	low _SetupPacket
	movwf	BANKED_EP0OUT_ADRL
	movlw	high _SetupPacket
	movwf	BANKED_EP0OUT_ADRH
	movlw	_UOWN
	movwf	BANKED_EP0OUT_STAT

	movlw 	low _ControlTransferBuffer
	movwf	BANKED_EP0IN_ADRL
	movlw	high _ControlTransferBuffer
	movwf	BANKED_EP0IN_ADRH
	movlw   (1<< DTS) | (1<<DTSEN)
	movwf	BANKED_EP0IN_STAT
	bsf	BANKED_EP0IN_STAT,UOWN
host_to_device:

	movlw	STAGE_DATA_OUT
	movwf	_CtrlTransferStage

        ; Clear the input buffer descriptor
	BANKSEL	BANKED_EP0IN_STAT
	clrf	BANKED_EP0IN_CNT
	movlw	(1<<DTS)|(1<<DTSEN)
	movwf	BANKED_EP0IN_STAT
	bsf	BANKED_EP0IN_STAT,UOWN

        ; Set the out buffer descriptor on endpoint 0 to receive data
        movlw	E0SZ
	movwf	BANKED_EP0OUT_CNT
	movlw	low _ControlTransferBuffer
	movwf	BANKED_EP0OUT_ADRL
	movlw	high _ControlTransferBuffer
	movwf	BANKED_EP0OUT_ADRH

        // Give to SIE, DATA1 packet, enable data toggle checks
	movlw	(1<<DTS)|(1<<DTSEN)
	movwf	BANKED_EP0OUT_STAT
	bsf	BANKED_EP0OUT_STAT,UOWN

SetupStage_exit:
	BANKSEL	UCON
    	bcf	UCON,PKTDIS 
	
  __endasm;
}

// Configures the buffer descriptor for endpoint 0 so that it is waiting for
// the status stage of a control transfer.
void WaitForSetupStage(void)
{
  __asm
	movlw	STAGE_SETUP
	movwf	_CtrlTransferStage
	BANKSEL BANKED_EP0OUT_STAT
	movlw	_E0SZ
	movwf	BANKED_EP0OUT_CNT
	movlw	low _SetupPacket
	movwf	BANKED_EP0OUT_ADRL
	movlw	high _SetupPacket
	movwf	BANKED_EP0OUT_ADRH
	movlw	(1<<DTSEN)
	movwf	BANKED_EP0OUT_STAT
	bsf	BANKED_EP0OUT_STAT,UOWN
	clrf	BANKED_EP0IN_STAT
  __endasm;
}

// This is the starting point for processing a Control Transfer.  The code directly
// follows the sequence of transactions described in the USB spec chapter 5.  The
// only Control Pipe in this firmware is the Default Control Pipe (endpoint 0).
// Control messages that have a different destination will be discarded.
void ProcessControlTransfer(void)
{

  __asm
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
	pagesel	_SetupStage  ; if PID==0x0D then call SetupStage
	call	_SetupStage
	goto	PCT_exit

PCT_1:
	; else  are we in STAGE_DATA_OUT
	movfw	_CtrlTransferStage
	xorlw	STAGE_DATA_OUT
	btfss 	STATUS,Z
	goto	PCT_2
	pagesel	_OutDataStage
	call	_OutDataStage
	pagesel	$
	movlw	(1<<DTSEN)
	BANKSEL  BANKED_EP0OUT_STAT
	btfss	BANKED_EP0OUT_STAT,DTS
	xorlw	(1<<DTS)
	movwf	BANKED_EP0OUT_STAT
	bsf	BANKED_EP0OUT, UOWN
	goto	PCT_exit
PCT_2:
	; Prepare for the Setup Stage control transfer
	pagesel _WaitForSetupStage
	call	_WaitForSetupStage
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
	movf	_DeviceState,W
	xorlw	STATE_ADDRESS
	btfss	STATUS,Z
	goto	PCT_data_in

	movf	SetupPacket_wValue0,W
	movwf	UADDR
	btfsc	STATUS,Z
	goto	PCT_data_in
	movlw	STATE_DEFAULT
	movwf	_DeviceState

PCT_data_in:
	; if (CtrlTransferStage == STAGE_DATA_IN)
	movf	_CtrlTransferStage,W
	xorlw	STAGE_DATA_IN
	btfss	STATUS,Z
	goto	PCT_w4s
	pagesel	_InDataStage
	call	_InDataStage
	pagesel	$

	movlw	(1<<DTSEN)
	BANKSEL	BANKED_EP0IN_STAT
	btfss	BANKED_EP0IN_STAT,DTS
	xorlw	(1<<DTS)
	movwf	BANKED_EP0IN_STAT
	bsf	BANKED_EP0IN_STAT,UOWN
	goto	PCT_exit

PCT_w4s:
	pagesel	_WaitForSetupStage
	call	_WaitForSetupStage
	pagesel $

PCT_exit:
  __endasm;
}

void InitializeUSB(void)
{
   __asm
        BANKSEL _StatusBits
	clrf	_StatusBits	
	BANKSEL UCFG
	movlw 0x14
	movwf UCFG  // Enable Pullup resistors; full speed mode; No PingPong
	movlw STATE_DETACHED
	movwf _DeviceState
	bcf   _StatusBits,status_remote_wakeup
	clrf  _CurrentConfiguration
	clrf  UADDR // Reset USB Address
	clrf  UEIR  // Clear all USB Error Interrupt Flags
      ; Reset PP buffers
	bsf   UCON, PPBRST
	bcf   UCON, PPBRST

        bcf   UCON,PKTDIS  // Enable Packet Transfers
   __endasm;
}

void EnableUSBModule(void)
{
    // TBD: Check for voltage coming from the USB cable and use that
    // as an indication we are attached.
   __asm
	BANKSEL UCON
	btfss	UCON,USBEN
	goto	eum_next
	clrf	UCON
	clrf	UIE
	bsf	UCON,USBEN
	movlw	STATE_ATTACHED
	movwf	_DeviceState
eum_next:
        btfsc  UCON,SE0
        goto	$-1

	clrf	UIR
	clrf	UIE
	bsf	UIE,URSTIE
	bsf	UIE,IDLEIE
	movlw	STATE_POWERED
	movwf	_DeviceState
   __endasm;
}

// Unsuspend the device
void UnSuspend(void)
{
   __asm
      BANKSEL UCON
      bcf UCON,SUSPND
      bcf UIE, ACTVIE
      movlw 0xFB
      andwf UIR,F
   __endasm;
}

// Full speed devices get a Start Of Frame (SOF) packet every 1 millisecond.
// Nothing is currently done with this interrupt (it is simply masked out).
void StartOfFrame(void)
{
    // TBD: Add a callback routine to do something
   __asm
      BANKSEL UIR
      bcf UIR, SOFIF
   __endasm;
}

// This routine is called in response to the code stalling an endpoint.
void Stall(void)
{
   __asm
	BANKSEL UEP0
	btfss	UEP0, EPSTALL
	goto  stall_exit
	pagesel _WaitForSetupStage
        call	_WaitForSetupStage
	BANKSEL UEP0
	bcf	UEP0, EPSTALL
stall_exit:
        bcf	UIR, STALLIF
   __endasm;
}

// Suspend all processing until we detect activity on the USB bus
void Suspend(void)
{
   __asm
	BANKSEL UIE
	bsf	UIE, ACTVIE
	movlw	0xEF
	andwf	UIR,f
	bsf	UCON, SUSPND
  __endasm;
}

void BusReset()
{
   __asm
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

	bcf	_StatusBits,status_remote_wakeup ; Remote wakeup is off by default
	bcf	_StatusBits,status_self_powered  ; Self powered is off by default
	clrf 	_CurrentConfiguration ; Clear active configuration

	movlw	STATE_DEFAULT
	movwf	_DeviceState
   __endasm;
}

// Main entry point for USB tasks.  Checks interrupts, then checks for transactions.
void ProcessUSBTransactions(void)
{
  __asm
	; See if device is connected yet
	BANKSEL _DeviceState
 	movfw	_DeviceState	
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

	pagesel _UnSuspend	
	call	_UnSuspend
	pagesel	$
	
  	BANKSel UIR
	bcf	UIR,USB_RESUM

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
	pagesel _BusReset
	call	_BusReset
	pagesel $

	BANKSEL UIR
	bcf	UIR,USB_URST

PUT_IDLE:
        ; Check for No bus activity for a while - suspend the firmware
	btfss 	UIR,IDLEIF
	goto	PUT_SOF
	btfss	UIE,IDLEIE
	goto	PUT_SOF
	pagesel _Suspend
	call	_Suspend
	pagesel $
	
	BANKSEL UIR
  	bcf	UIR,USB_IDLE	

PUT_SOF:
    	; Start of Frame (SOF)
	btfss	UIR, SOFIF
	goto	PUT_STALL
	btfss	UIE, SOFIE
	goto	PUT_STALL
	pagesel _StartOfFrame
	call	_StartOfFrame
	pagesel $

	BANKSEL	UIR
	bcf	UIR,USB_SOF

PUT_STALL:
	; Check for STALL
	btfss	UIR,STALLIF
	goto	PUT_UERR
	btfss	UIE,STALLIE
	goto	PUT_UERR
	pagesel _Stall
	call	_Stall
	pagesel $

	BANKSEL	UIR
	bcf	UIR,USB_STALL

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
	subwf	_DeviceState,W

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
	pagesel	_ProcessControlTransfer
	call	_ProcessControlTransfer
	pagesel $

	BANKSEL	UIR
	bcf	UIR,USB_TRN

PUT_exit:
	BANKSEL PIR2
	bcf	PIR2,USBIF ; Clear Global USB Interrupt Flag
	return
  __endasm;
}
