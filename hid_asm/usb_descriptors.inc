; vim:noet:sw=8:ts=8:ai:syn=pic
; asmsyntax=pic
;
; File:   usb_descriptos.inc
; Based on: UsbDescriptors.h
; by: Szymon Roslowski
;
; Created on 13 October 2014, 18:44
;
; Device and configuration descriptors.  These are used as the
; host enumerates the device and discovers what class of device
; it is and what interfaces it supports.
;

;#include "usb_internal.h"

; Vendor and Product Information
#define VendorId    0x04D8
#define ProductId   0x01A6
#define ReleaseNo   0x0001

;;; Constants and varaiable addresses
SERIAL_NUMBER_DIGIT_CNT equ     4
    ifndef SERIAL_NUMBER
        variable SERIAL_NUMBER=42       ; Why doesnt 'equ' work here? Go figure
    endif

; Definitions
#define InterfaceCount          0x01 ; One Interface - Just Keyboard
#define StringDescriptorCount   0x03 ; Three string descriptors - See Bottom of this file
#define HidDescriptorSize       0x20 ; Size Of HID Descriptor


; Strings
#define SMAN 0x01   ; Manufacturer Name String Index
#define SPRD 0x02   ; Product Name String Index
#define SSER 0x03   ; Serial Number String Index
#define SCON 0x00   ; Configuration String Index

#define MAX_CODE_SIZE (0x1F80)
#define HIGH_AVAIL_FLASH (0x1F80)

#define DEVICE_DESC_LEN  0x12
#define CONFIG_DESC_LEN  0x09
#define HID_DESC_LEN   HidDescriptorSize
#define HID_REPORT_LEN  0x3F


#define ALL_STRINGS_LEN (STR_DESC0_SIZE + STR_DESC1_SIZE + STR_DESC2_SIZE + STR_DESC3_SIZE)
#define ALL_DESC_TOTAL_LEN (DEVICE_DESC_LEN + CONFIG_DESC_LEN + HID_DESC_LEN + + HID_REPORT_LEN +ALL_STRINGS_LEN)

HID_REPORT_BYTE_COUNT equ 0x08

;***********************
;* Descriptors         *
;***********************
	;org	MAX_CODE_SIZE-ALL_DESC_TOTAL_LEN
	org 0x1E00
; Device Descriptor
DESCRIPTOR_ADRH equ high $
DeviceDescriptor
    	dt	0x12   ; Size of this descriptor in bytes
    	dt	0x01   ; DEVICE descriptor type
    	dt	0x00   ; USB Spec Release Number in BCD format LSB
    	dt	0x02   ; USB Spec Release Number in BCD format MSB
    	dt	0x00   ; Class Code
    	dt	0x00   ; Subclass code
    	dt	0x00   ; Protocol code
    	dt	E0SZ   ; Max packet size for EP0 (8)
    	dt	low  VendorId; Vendor ID LSB
    	dt	high VendorId; Vendor ID MSB
    	dt	low  ProductId; Product ID: Custom HID device demo LSB
    	dt	high ProductId; Product ID: Custom HID device demo MSB
    	dt	low  ReleaseNo; Device release number in BCD format LSB
    	dt	high ReleaseNo; Device release number in BCD format MSB
    	dt	SMAN   ; Manufacturer string index
    	dt	SPRD   ; Product string index
    	dt	SSER   ; Device serial number string index
    	dt	0x01   ; Number of possible configurations

; ...Stuck these here to keep the number of files to minimum
ConfigurationDescriptor ; Configuration descriptor
    dt	0x09 ; Size of this descriptor in bytes
    dt	0x02 ; CONFIGURATION descriptor type
    dt	0x29 ; Total length of data for this cfg LSB
    dt	0x00 ; Total length of data for this cfg MSB
    dt	INTF ; Number of interfaces in this cfg
    dt	0x01 ; Index value of this configuration
    dt	SCON ; Configuration string index
    dt	0xA0 ; Attributes
    dt	0x32 ; Max power consumption (50 mA)

HIDDescriptor
        ; Keyboard HID Interface descriptor
    dt	0x09 ; Size of this descriptor in bytes
    dt	0x04 ; INTERFACE descriptor type
    dt	0x00 ; Interface Number
    dt	0x00 ; Alternate Setting Number
    dt	0x02 ; Number of endpoints in this interface
    dt	0x03 ; Class code (HID)
    dt	0x01 ; Subclass code (Sublass Boot(1) as opposed to NONE(0) the rest reserved)
    dt	0x01 ; Protocol code 0-none, 1-Keyboard, 2- Mouse
    dt	0x00 ; Interface String Descriptor Index

        ; Keyboard Class-Specific descriptor
    dt	0x09 ; Size of this descriptor in bytes
    dt	0x21 ; HID descriptor type
    dt	0x11 ; HID Spec Release Number in BCD format (1.11) LSB
    dt	0x01 ; HID Spec Release Number in BCD format (1.11) MSB
    dt	0x00 ; Country Code (0x00 for Not supported)
    dt	0x01 ; Number of class descriptors
    dt	0x22 ; Report descriptor type
    dt	0x3F ; Report Size LSB  (63 bytes)
    dt	0x00 ; Report Size MSB

    	; Keyboard Endpoint 1 In
    dt	0x07 ; Size of this descriptor in bytes
    dt	0x05 ; ENDPOINT descriptor type
    dt	0x81 ; Endpoint Address EP1IN
    dt	0x03 ; Attributes (Interrupt)
    dt	low HID_REPORT_BYTE_COUNT ; Max Packet Size LSB
    dt	0x00 ; Max Packet Size MSB
    dt	0x01 ; Interval (1 millisecond)

    	; Keyboard Endpoint 1 Out
    dt	0x07 ; Size of this descriptor in bytes
    dt	0x05 ; ENDPOINT descriptor type
    dt	0x01 ; Endpoint Address EP1OUT
    dt	0x03 ; Attributes (Interrupt)
    dt	low HID_REPORT_BYTE_COUNT; Max Packet Size LSB
    dt	0x00 ; Max Packet Size MSB
    dt	0x01    ; Interval (1 millisecond)

HIDReport 	; Report For Keyboard
    dt	0x05, 0x01                  ; USAGE_PAGE (Generic Desktop)
    dt	0x09, 0x06                  ; USAGE (Keyboard)
    dt	0xa1, 0x01                  ; COLLECTION (Application)
    dt	0x05, 0x07                  ;   USAGE_PAGE (Keyboard)
    dt	0x19, 0xe0                  ;   USAGE_MINIMUM (Keyboard LeftControl)
    dt	0x29, 0xe7                  ;   USAGE_MAXIMUM (Keyboard Right GUI)
    dt	0x15, 0x00                  ;   LOGICAL_MINIMUM (0)
    dt	0x25, 0x01                  ;   LOGICAL_MAXIMUM (1)
    dt	0x75, 0x01                  ;   REPORT_SIZE (1)
    dt	0x95, 0x08                  ;   REPORT_COUNT (8)
    dt	0x81, 0x02                  ;   INPUT (Data,Var,Abs)
    dt	0x95, 0x01                  ;   REPORT_COUNT (1)
    dt	0x75, 0x08                  ;   REPORT_SIZE (8)
    dt	0x81, 0x03                  ;   INPUT (Cnst,Var,Abs)
    dt	0x95, 0x05                  ;   REPORT_COUNT (5)
    dt	0x75, 0x01                  ;   REPORT_SIZE (1)
    dt	0x05, 0x08                  ;   USAGE_PAGE (LEDs)
    dt	0x19, 0x01                  ;   USAGE_MINIMUM (Num Lock)
    dt	0x29, 0x05                  ;   USAGE_MAXIMUM (Kana)
    dt	0x91, 0x02                  ;   OUTPUT (Data,Var,Abs)
    dt	0x95, 0x01                  ;   REPORT_COUNT (1)
    dt	0x75, 0x03                  ;   REPORT_SIZE (3)
    dt	0x91, 0x03                  ;   OUTPUT (Cnst,Var,Abs)
    dt	0x95, 0x06                  ;   REPORT_COUNT (6)
    dt	0x75, 0x08                  ;   REPORT_SIZE (8)
    dt	0x15, 0x00                  ;   LOGICAL_MINIMUM (0)
    dt	0x25, 0x65                  ;   LOGICAL_MAXIMUM (101)
    dt	0x05, 0x07                  ;   USAGE_PAGE (Keyboard)
    dt	0x19, 0x00                  ;   USAGE_MINIMUM (Reserved (no event indicated))
    dt	0x29, 0x65                  ;   USAGE_MAXIMUM (Keyboard Application)
    dt	0x81, 0x00                  ;   INPUT (Data,Ary,Abs)
    dt	0xc0                           ; END_COLLECTION

#define STR_DESC0_SIZE (0x04)

StringDescriptor0
	dt	0x04 		; Size
	dt	0x03		; DSC_STR
	dt	0x09,0x04	;

; iManufacturer String
#define STR_DESC1_SIZE (0x18)
StringDescriptor1
	dt STR_DESC1_SIZE
	dt 0x03
	dt 'D',0,'I',0,'Y',0,' ',0,'D',0,'e',0,'v',0,'i',0,'c',0,'e',0,'s',0

; iPROD String
#define STR_DESC2_SIZE (0x2A)
StringDescriptor2
	dt STR_DESC2_SIZE
	dt 0x03
	dt '1',0,'6',0,'F',0,'1',0,'4',0,'5',0,'5',0,' ',0
    dt 'H',0,'I',0,'D',0,' ',0,'K',0,'e',0,'y',0,'b',0
 	dt 'o',0,'a',0,'r',0,'d',0

; iSerial Number
; extract nibbles from serial number
SN1     equ     (SERIAL_NUMBER>>12) & 0xF
SN2     equ     (SERIAL_NUMBER>>8) & 0xF
SN3     equ     (SERIAL_NUMBER>>4) & 0xF
SN4     equ     SERIAL_NUMBER & 0xF

#define STR_DESC3_SIZE (0x0A)
StringDescriptor3
        dt      STR_DESC3_SIZE			; bLength
        dt      0x03            		; bDescriptorType (STRING)
        dt      '0'+SN1+((SN1>9)*7), 0x00       ; convert hex digits to ASCII
        dt      '0'+SN2+((SN2>9)*7), 0x00
        dt      '0'+SN3+((SN3>9)*7), 0x00
        dt      '0'+SN4+((SN4>9)*7), 0x00

