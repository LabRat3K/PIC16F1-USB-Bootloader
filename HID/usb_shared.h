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

// Definitions
#define HidDescriptorSize       0x20 // Size Of HID Descriptor
// HID
#define HidReportByteCount      0x08 // Hid Report Size, also size of Buffers etc. ( Memory usage can go over the roof if not careful with this value)
#define HidInterfaceNumber      0x00 // Interface For our HID

#define CONFIG_HEADER_SIZE      0x09
#define DeviceDescriptorSize    0x12

// Actual USB Data Buffers
extern volatile uint8_t HIDRxBuffer[HidReportByteCount];
extern volatile uint8_t HIDTxBuffer[HidReportByteCount];


/***********************/
/* Descriptors         */
/***********************/

// Device Descriptor
extern __at (0x1E00) const uint8_t DeviceDescriptor[];

// ...Stuck these here to keep the number of files to minimum
typedef struct _configStruct
{
    uint8_t configHeader[CONFIG_HEADER_SIZE];
    uint8_t HIDDescriptor[HidDescriptorSize];
} ConfigStruct;

// Configuration descriptor
extern __at (0x1E00+DeviceDescriptorSize ) const ConfigStruct ConfigurationDescriptor;

// Report For Keyboard
extern __at (0x1E00+DeviceDescriptorSize + CONFIG_HEADER_SIZE+ HidDescriptorSize) const uint8_t HIDReport[];


#define UsbInterrupt PIR2bits.USBIF

#endif	/* USBDESCRIPTORS_H */
