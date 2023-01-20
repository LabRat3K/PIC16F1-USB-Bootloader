/*
 * File:   usb_shared.h
 * Author: Andrew Williams 
 *
 * Names and externs only. NO VARIABLE DECLARATIONS
 */

#ifndef USBSHARED_H
#define	USBSHARED_H

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

// Vendor and Product Information
#define VendorId    0x04D8
#define ProductId   0x01A6
#define ReleaseNo   0x0001

// Definitions
#define InterfaceCount          0x01 // One Interface - Just Keyboard
#define StringDescriptorCount   0x03 // Three string descriptors - See Bottom of this file
#define Endpoint0BufferSize     0x08 // Endpoint 0 Buffer Size
#define HidDescriptorSize       0x20 // Size Of HID Descriptor
// HID
#define HidReportByteCount      0x08 // Hid Report Size, also size of Buffers etc. ( Memory usage can go over the roof if not careful with this value)
#define HidInterfaceNumber      0x00 // Interface For our HID

// Strings
#define SMAN 0x01   // Manufacturer Name String Index
#define SPRD 0x02   // Product Name String Index
#define SSER 0x00   // Serial Number String Index
#define SCON 0x00   // Configuration String Index

#define CONFIG_HEADER_SIZE      0x09

// Actual USB Data Buffers
extern volatile uint8_t HIDRxBuffer[HidReportByteCount];
extern volatile uint8_t HIDTxBuffer[HidReportByteCount];


/***********************/
/* Descriptors         */
/***********************/

// Device Descriptor
extern const uint8_t DeviceDescriptor[];

// ...Stuck these here to keep the number of files to minimum
#define HRBC HidReportByteCount
typedef struct _configStruct
{
    uint8_t configHeader[CONFIG_HEADER_SIZE];
    uint8_t HIDDescriptor[HidDescriptorSize];
} ConfigStruct;

// Configuration descriptor
extern const ConfigStruct ConfigurationDescriptor ;

// Report For Keyboard
extern const uint8_t HIDReport[];


#define UsbInterrupt PIR2bits.USBIF

#endif	/* USBDESCRIPTORS_H */

