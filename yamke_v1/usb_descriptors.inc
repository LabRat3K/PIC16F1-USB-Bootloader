/*
 * File:   UsbDescriptors.inc
 * Author: Szymon Roslowski
 *
 * Created on 13 October 2014, 18:44
 *
 * Device and configuration descriptors.  These are used as the
 * host enumerates the device and discovers what class of device
 * it is and what interfaces it supports.
 */

#ifndef USBDESCRIPTORS_INC
#define	USBDESCRIPTORS_INC

#include "usb_shared.h"

// Allocate endpoint payload buffers elsewhere inside the USB RAM (0x2040 - 0x21FF)
/*
__at (0x2040) volatile uint8_t ep0_out_buf[64];
__at (0x2080) volatile uint8_t ep0_in_buf[64];
*/
__at (0x20C0) volatile uint8_t hid1_out_buf[HidReportByteCount];
__at (0x20C8) volatile uint8_t hid1_in_buf[HidReportByteCount];
__at (0x20D0) volatile uint8_t hid2_out_buf[HidReportByteCount];
__at (0x20D8) volatile uint8_t hid2_in_buf[HidReportByteCount];
__at (0x20E0) volatile uint8_t cdc_comm_out_buf[8];
__at (0x20E8) volatile uint8_t cdc_comm_in_buf[8];
__at (0x2100) volatile uint8_t cdc_out_buf[CdcReportByteCount];
__at (0x2140) volatile uint8_t cdc_in_buf[CdcReportByteCount];

BufferInfo Buffers[(InterfaceCount * 2)] =
{
    { HidReportByteCount, (uint8_t*)&hid1_in_buf },
    { HidReportByteCount, (uint8_t*)&hid1_out_buf},
    { HidReportByteCount, (uint8_t*)&hid2_in_buf },
    { HidReportByteCount, (uint8_t*)&hid2_out_buf},
#ifdef INC_CDC
    { 8, (uint8_t*)&cdc_comm_in_buf},
    { 8, (uint8_t*)&cdc_comm_out_buf},
    { CdcReportByteCount, (uint8_t*)&cdc_in_buf },
    { CdcReportByteCount, (uint8_t*)&cdc_out_buf}
#endif
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
    uint8_t HID1Descriptor[Hid1DescriptorSize];
    uint8_t HID2Descriptor[Hid2DescriptorSize];
    uint8_t CDCDescriptor[CdcDescriptorSize];
} ConfigStruct;
*/

// Configuration descriptor
__at (0x1E00+DeviceDescriptorSize) const ConfigStruct ConfigurationDescriptor =
{
    {
    // ==========================================
    // Configuration descriptor
    // ==========================================
    0x09,   // Size of this descriptor in bytes
    0x02,   // CONFIGURATION descriptor type
    9+HID1_CONFIG_LENGTH+CDC_CONFIG_LENGTH+HID2_CONFIG_LENGTH,   // Total length of data for this cfg LSB
    0x00,   // Total length of data for this cfg MSB
    0x02+NUM_CDC_INTF,  // Number of interfaces in this cfg
    0x01,   // Index value of this configuration
    SCON,   // Configuration string index
    0x80,   // Attributes
    0x32,   // Max power consumption (50 mA)
    },
    {
    // ==========================================
    // HID0:: Keyboard HID Interface descriptor
    // ==========================================
    0x09,   // Size of this descriptor in bytes
    0x04,   // INTERFACE descriptor type
    0x00,   // Interface Number
    0x00,   // Alternate Setting Number
    0x02,   // Number of endpoints in this interface
    0x03,   // Class code (HID)
    0x01,   // Subclass code (Sublass Boot(1) as opposed to NONE(0) the rest reserved)
    0x01,   // Protocol code 0-none, 1-Keyboard, 2- Mouse
    0x00,   // Interface String Descriptor Index


    // ==========================================
    // Keyboard Class-Specific descriptor
    // ==========================================
    0x09,   // Size of this descriptor in bytes
    0x21,   // HID descriptor type
    0x11,   // HID Spec Release Number in BCD format (1.11) LSB
    0x01,   // HID Spec Release Number in BCD format (1.11) MSB
    0x00,   // Country Code (0x00 for Not supported)
    0x01,   // Number of class descriptors
    0x22,   // Report descriptor type
    0x3F,   // Report Size LSB  (63 bytes)
    0x00,   // Report Size MSB

    // ==============================
    // Keyboard Endpoint 1 In (0x81)
    // ==============================
    0x07,   // Size of this descriptor in bytes
    0x05,   // ENDPOINT descriptor type
    0x81,   // Endpoint Address
    0x03,   // Attributes (Interrupt)
    HRBC,   // Max Packet Size LSB
    0x00,   // Max Packet Size MSB
    0x01,   // Interval (1 millisecond)

    // ===============================
    // Keyboard Endpoint 1 Out (0x01)
    // ===============================
    0x07,   // Size of this descriptor in bytes
    0x05,   // ENDPOINT descriptor type
    0x01,   // Endpoint Address
    0x03,   // Attributes (Interrupt)
    HRBC,   // Max Packet Size LSB
    0x00,   // Max Packet Size MSB
    0x01,   // Interval (1 millisecond)
    },
    {
    // =============================
    // INTERFACE 1: HID 2  In (0x82)
    // =============================
    0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00, // HID Class
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x3F, 0x00, // HID Descriptor
    0x07, 0x05, 0x82, 0x03, 0x08, 0x00, 0x01,             // Endpoint 2 IN
    },

#ifdef INC_CDC
    {
    // ==============================================
    // INTERFACE ASSOCIATION DESCRIPTOR (IAD) FOR CDC
    // ==============================================
    0x08, 0x0B, 0x02, 0x02, 0x02, 0x02, 0x01, 0x00,

    // ========================================
    // INTERFACE 2: CDC Communication In (0x84)
    // ========================================
    0x09, 0x04, 0x02, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    0x05, 0x24, 0x00, 0x10, 0x01,                         // Header Functional
    0x05, 0x24, 0x01, 0x00, 0x03,                         // Call Management
    0x04, 0x24, 0x02, 0x02,                               // ACM Functional
    0x05, 0x24, 0x06, 0x02, 0x03,                         // Union Functional
    0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0xFF,             // Notification EP 3 IN

    // ========================================
    // INTERFACE 3: CDC Data Out/In (0x03/0x83)
    // ========================================
    0x09, 0x04, 0x03, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    0x07, 0x05, 0x04, 0x02, 0x40, 0x00, 0x00,            // OUT Endpoint 4
    0x07, 0x05, 0x84, 0x02, 0x40, 0x00, 0x00             // IN Endpoint 4

    }
#endif
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

#ifdef OLD_CODE
const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[1];}StringDescriptor0={sizeof(StringDescriptor0),0x03,{0x0409}};

const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[11];}StringDescriptor1={sizeof(StringDescriptor1),0x03,
{'J','S','G',' ','P','r','o','d','u','c','t'}};

const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[19];}StringDescriptor2={sizeof(StringDescriptor2),0x03,
{'Y','e','t',' ','A','n','o','t','h','e','r',' ','M','a','m','e','K','B','D'}};

//Array of string descriptors
const uint8_t *const StringDescriptorPointers[StringDescriptorCount]=
{
    (const uint8_t *const)&StringDescriptor0, // 0x0409 = English
    (const uint8_t *const)&StringDescriptor1,
    (const uint8_t *const)&StringDescriptor2
};
#else 
// Language code string descriptor (4 bytes)
const uint8_t StringDescriptor0[] = {
    0x04, 0x03, 0x09, 0x04
};

// Manufacturer string descriptor: "JSG Product" (2 + 11*2 = 24 bytes = 0x18)
const uint8_t StringDescriptor1[] = {
    24, 0x03,
    'J', 0, 'S', 0, 'G', 0, ' ', 0, 'P', 0, 'r', 0, 'o', 0, 'd', 0, 'u', 0, 'c', 0, 't', 0
};

// Product string descriptor: "Yet Another MameKBD" (2 + 19*2 = 40 bytes = 0x28)
const uint8_t StringDescriptor2[] = {
    40, 0x03,
    'Y', 0, 'e', 0, 't', 0, ' ', 0, 'A', 0, 'n', 0, 'o', 0, 't', 0, 'h', 0, 'e', 0, 'r', 0, ' ', 0,
    'M', 0, 'a', 0, 'm', 0, 'e', 0, 'K', 0, 'B', 0, 'D', 0
};

// Serial number string descriptor: "YAMKE0001" (2 + 9*2 = 20 bytes = 0x16)
const uint8_t StringDescriptor3[] = {
    20, 0x03,
    'Y', 0, 'A', 0, 'M', 0, 'K', 0, 'E', 0, '0', 0, '0', 0, '0', 0, '1', 0
};

#endif

#endif	/* USBDESCRIPTORS_INC */
