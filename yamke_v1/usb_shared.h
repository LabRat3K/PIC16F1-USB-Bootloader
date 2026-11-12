/*
 * File:   usb_shared.h
 * Author: Andrew Williams 
 *
 * Names and externs only. NO VARIABLE DECLARATIONS
 */

#ifndef USBSHARED_H
#define	USBSHARED_H

#define INC_CDC

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

// Vendor and Product Information
#define VendorId    0x04D8
#define ProductId   0xE3D6   // Sub-licensed from Microchip YAMKE (Yet Another Mame Keyboard Encoder)
#define ReleaseNo   0x0001

// Definitions
#define StringDescriptorCount   0x04 // Three string descriptors - See Bottom of this file
#define Endpoint0BufferSize     0x08 // Endpoint 0 Buffer Size
// HID
#define HidReportByteCount      0x08 // Hid Report Size, also size of Buffers etc.
#define CdcReportByteCount      0x40 

#define EP0Id   0x00
#define P1HidNo 0x01            // Player1 HID Endpoint Index
#define P2HidNo 0x02            // Player2 HID Endpoint Index
#define CdcNo   0x04            // Serial EndPoint (Bulk)


// Strings
#define SMAN 0x01   // Manufacturer Name String Index
#define SPRD 0x02   // Product Name String Index
#define SSER 0x03   // Serial Number String Index
#define SCON 0x00   // Configuration String Index

#define CONFIG_HEADER_SIZE      0x09
#define DeviceDescriptorSize    0x12



/***********************/
/* Descriptors         */
/***********************/

// Device Descriptor
extern __at (0x1E00) const uint8_t DeviceDescriptor[];

// ...Stuck these here to keep the number of files to minimum
#define HRBC HidReportByteCount
#ifdef INC_CDC
   #define CDC_CONFIG_LENGTH (66)
   #define NUM_CDC_INTF (2)
#else
   #define CDC_CONFIG_LENGTH (0)
   #define NUM_CDC_INTF (0)
#endif

#define HID2_CONFIG_LENGTH (25)

#define HID1_CONFIG_LENGTH (32)
#define HidDescriptorSize  (HID1_CONFIG_LENGTH + HID2_CONFIG_LENGTH + CDC_CONFIG_LENGTH) // Size Of HID Descriptor
#define InterfaceCount     (2+NUM_CDC_INTF)

typedef struct _configStruct
{
    uint8_t configHeader[CONFIG_HEADER_SIZE];
    uint8_t HID1Descriptor[HID1_CONFIG_LENGTH];
    uint8_t HID2Descriptor[HID2_CONFIG_LENGTH];
#ifdef INC_CDC
    uint8_t CDCDescriptor[CDC_CONFIG_LENGTH];
#endif
} ConfigStruct;

// Configuration descriptor
extern __at (0x1E00+DeviceDescriptorSize ) const ConfigStruct ConfigurationDescriptor;

// Report For Keyboard
extern __at (0x1E00+DeviceDescriptorSize + CONFIG_HEADER_SIZE+ HidDescriptorSize) const uint8_t HIDReport[];


#define UsbInterrupt PIR2bits.USBIF

#endif	/* USBDESCRIPTORS_H */
