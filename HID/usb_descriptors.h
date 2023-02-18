/*
 * File:   UsbDescriptors.h
 * Author: Szymon Roslowski
 *
 * Created on 13 October 2014, 18:44
 *
 * Device and configuration descriptors.  These are used as the
 * host enumerates the device and discovers what class of device
 * it is and what interfaces it supports.
 */

#ifndef USBDESCRIPTORS_H
#define	USBDESCRIPTORS_H


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
#define DeviceDescriptorSize    0x12

// Strings
#define SMAN 0x01   // Manufacturer Name String Index
#define SPRD 0x02   // Product Name String Index
#define SSER 0x00   // Serial Number String Index
#define SCON 0x00   // Configuration String Index

// Actual USB Data Buffers
volatile uint8_t HIDRxBuffer[HidReportByteCount];
volatile uint8_t HIDTxBuffer[HidReportByteCount];

BufferInfo Buffers[(InterfaceCount * 2)] =
{
    { HidReportByteCount, (uint8_t*)&HIDTxBuffer },
    { HidReportByteCount, (uint8_t*)&HIDRxBuffer }
};

/***********************/
/* Descriptors         */
/***********************/

// Device Descriptor
__at (0x1E00) const uint8_t DeviceDescriptor[]=
{
    0x12,   // Size of this descriptor in bytes
    0x01,   // DEVICE descriptor type
    0x00,   // USB Spec Release Number in BCD format LSB
    0x02,   // USB Spec Release Number in BCD format MSB
    0x00,   // Class Code
    0x00,   // Subclass code
    0x00,   // Protocol code
    E0SZ,   // Max packet size for EP0 (8)
    VIDL,   // Vendor ID LSB
    VIDH,   // Vendor ID MSB
    PIDL,   // Product ID: Custom HID device demo LSB
    PIDH,   // Product ID: Custom HID device demo MSB
    RELL,   // Device release number in BCD format LSB
    RELH,   // Device release number in BCD format MSB
    SMAN,   // Manufacturer string index
    SPRD,   // Product string index
    SSER,   // Device serial number string index
    0x01    // Number of possible configurations
};

// ...Stuck these here to keep the number of files to minimum
/*
#define HRBC HidReportByteCount
typedef struct _configStruct
{
    uint8_t configHeader[CONFIG_HEADER_SIZE];
    uint8_t HIDDescriptor[HidDescriptorSize];
} ConfigStruct;
*/
// Configuration descriptor
__at (0x1E00+DeviceDescriptorSize) const ConfigStruct ConfigurationDescriptor =
{
    {
        // Configuration descriptor
    0x09,   // Size of this descriptor in bytes
    0x02,   // CONFIGURATION descriptor type
    0x29,   // Total length of data for this cfg LSB
    0x00,   // Total length of data for this cfg MSB
    INTF,   // Number of interfaces in this cfg
    0x01,   // Index value of this configuration
    SCON,   // Configuration string index
    0xA0,   // Attributes
    0x32,   // Max power consumption (50 mA)
    },
    {
        // Keyboard HID Interface descriptor
    0x09,   // Size of this descriptor in bytes
    0x04,   // INTERFACE descriptor type
    IHID,   // Interface Number
    0x00,   // Alternate Setting Number
    0x02,   // Number of endpoints in this interface
    0x03,   // Class code (HID)
    0x01,   // Subclass code (Sublass Boot(1) as opposed to NONE(0) the rest reserved)
    0x01,   // Protocol code 0-none, 1-Keyboard, 2- Mouse
    0x00,   // Interface String Descriptor Index


        // Keyboard Class-Specific descriptor
    0x09,   // Size of this descriptor in bytes
    0x21,   // HID descriptor type
    0x11,   // HID Spec Release Number in BCD format (1.11) LSB
    0x01,   // HID Spec Release Number in BCD format (1.11) MSB
    0x00,   // Country Code (0x00 for Not supported)
    0x01,   // Number of class descriptors
    0x22,   // Report descriptor type
    0x3F,   // Report Size LSB  (63 bytes)
    0x00,   // Report Size MSB

    	// Keyboard Endpoint 1 In
    0x07,   // Size of this descriptor in bytes
    0x05,   // ENDPOINT descriptor type
    0x81,   // Endpoint Address
    0x03,   // Attributes (Interrupt)
    HRBC,   // Max Packet Size LSB
    0x00,   // Max Packet Size MSB
    0x01,   // Interval (1 millisecond)

    	// Keyboard Endpoint 1 Out
    0x07,   // Size of this descriptor in bytes
    0x05,   // ENDPOINT descriptor type
    0x01,   // Endpoint Address
    0x03,   // Attributes (Interrupt)
    HRBC,   // Max Packet Size LSB
    0x00,   // Max Packet Size MSB
    0x01    // Interval (1 millisecond)
    }
};

// Report For Keyboard
__at (0x1E00+DeviceDescriptorSize + CONFIG_HEADER_SIZE+ HidDescriptorSize) const uint8_t HIDReport[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};


const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[1];}StringDescriptor0={sizeof(StringDescriptor0),0x03,{0x0409}};

const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[11];}StringDescriptor1={sizeof(StringDescriptor1),0x03,
{'D','I','Y',' ','D','e','v','i','c','e','s'}};

const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[20];}StringDescriptor2={sizeof(StringDescriptor2),0x03,
{'1','6','F','1','4','5','5',' ','H','I','D',' ','K','e','y','b','o','a','r','d'}};

//Array of string descriptors
const uint8_t *const StringDescriptorPointers[StringDescriptorCount]=
{
    (const uint8_t *const)&StringDescriptor0,
    (const uint8_t *const)&StringDescriptor1,
    (const uint8_t *const)&StringDescriptor2
};

#endif	/* USBDESCRIPTORS_H */
