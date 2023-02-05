/*
 * File:   Usb.h
 * Author: Szymon Roslowski
 *
 * Created on 13 October 2014, 17:45
 *
 * Firmware framework for USB I/O on PIC 16F1455 (and siblings)
 *
 * Based On
 *
 * Firmware framework for USB I/O on PIC 18F2455 (and siblings)
 * Copyright (C) 2005 Alexander Enzmann
 * adapted to MCC18 by Alberto Maccioni on 1/8/09
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111 USA
 * or see <http://www.gnu.org/licenses/>
 */

#ifndef USB_H
#define	USB_H
#include <pic14regs.h>

#ifndef uint8_t
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
#endif


// Definitions
#define HID_REPORT_BYTE_COUNT  0x08 // Hid Report Size, also size of Buffers etc. ( Memory usage can go over the roof if not careful with this value)
#define HID_INTERFACE_NUMBER   0x00 // Interface For our HID

// Global Variables
extern volatile uint8_t DeviceState;    // Visible device states (from USB 2.0, chap 9.1.1): used in IsUsbReady() macro below.
extern volatile uint8_t HIDTxBuffer[HID_REPORT_BYTE_COUNT];
extern volatile uint8_t HIDRxBuffer[HID_REPORT_BYTE_COUNT];

// Shared Macro calls
#define IsUsbReady ((DeviceState == 0x05) && (UCONbits.SUSPND==0))
#define UsbInterrupt PIR2bits.USBIF

// Exported USB Functions
void InitializeUSB(void);
void EnableUSBModule(void);
void HIDSend(void);
void ProcessUSBTransactions(void);
void ReArmInterface(void);
uint8_t IsUsbDataAvailable(void);

#endif	/* USB_H */

