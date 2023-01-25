/*
 * File:   usb_internal.h
 * Author: Andrew Williams 
 *
 * Names and externs only. NO VARIABLE DECLARATIONS
 */

#ifndef USBINTERNAL_H
#define	USBINTERNAL_H

#include "usb.h"

// Vendor and Product Information
#define VendorId    0x04D8
#define ProductId   0x01A6
#define ReleaseNo   0x0001

// MACROS 

#define PTR16(x) ((unsigned int)(((unsigned long)x) & 0xFFFF))
#define LSB(x) (x & 0xFF)
#define MSB(x) ((x & 0xFF00) >> 8)
#define ClearUsbInterruptFlag(x)        UIR &= ~(x)
#define VIDL LSB(VendorId)  // Vendor Id Low Byte (LSB)
#define VIDH MSB(VendorId)  // Vendor Id High Byte (MSB)
#define PIDH MSB(ProductId) // Product Id High Byte (MSB)
#define PIDL LSB(ProductId) // Product Id Low Byte (LSB)
#define RELH MSB(ReleaseNo) // Release Number High Byte (MSB)
#define RELL LSB(ReleaseNo) // Release Number Low Byte (LSB)
#define INTF InterfaceCount // Total Count of Interfaces
#define IHID HID_INTERFACE_NUMBER
#define E0SZ Endpoint0BufferSize
#define CONFIG_HEADER_SIZE      0x09 // Configuration descriptor header size (see UsbDescriptors.h) - Pretty much always 9 :)

// Definitions
#define InterfaceCount          0x01 // One Interface - Just Keyboard
#define StringDescriptorCount   0x03 // Three string descriptors - See Bottom of this file
#define Endpoint0BufferSize     0x08 // Endpoint 0 Buffer Size
#define HidDescriptorSize       0x20 // Size Of HID Descriptor

// Strings
#define SMAN 0x01   // Manufacturer Name String Index
#define SPRD 0x02   // Product Name String Index
#define SSER 0x00   // Serial Number String Index
#define SCON 0x00   // Configuration String Index

#define CONFIG_HEADER_SIZE      0x09

// Structures
typedef struct _BufferInfo
{
    uint8_t Size;
    uint8_t *Buffer;
} BufferInfo;

/***********************/
/* Descriptors         */
/***********************/

// -------------------
// Device Descriptor
extern const uint8_t DeviceDescriptor[];

// ...Stuck these here to keep the number of files to minimum
#define HRBC HID_REPORT_BYTE_COUNT
typedef struct _configStruct
{
    uint8_t configHeader[CONFIG_HEADER_SIZE];
    uint8_t HIDDescriptor[HidDescriptorSize];
} ConfigStruct;

// -------------------------
// Configuration descriptor
extern const ConfigStruct ConfigurationDescriptor ;

// --------------------
// Report For Keyboard
extern const uint8_t HIDReport[];


#define UsbInterrupt PIR2bits.USBIF

#endif	/* USBDESCRIPTORS_H */

