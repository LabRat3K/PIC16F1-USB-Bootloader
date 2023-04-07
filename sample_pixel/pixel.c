/**
 * USB 512-Word CDC Bootloader Application Code
 * Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
 * v1.0, February 12, 2015
 * Released under a 3-clause BSD license: see the accompanying LICENSE file.
 *
 * PIC blinking LED example for SDCC.
 * Blinks an LED connected to pin RA5.
 * Tested with a PIC16F1454.
 *
 * Application code that is compatible with the bootloader has 5 requirements:
 *
 * - a main function named app_main(), NOT main().
 *   Defining main() will confuse the linker. As of version 3.4.0, SDCC forces
 *   code generation at address 0x0000; excluding main() prevents this.
 *
 * - an interrupt handler named app_interrupt(), even if the application does
 *   not use interrupts. (in that case, just use an empty function body.)
 *
 * - a configuration byte defined with the APP_CONFIG() macro.
 *   This macro is provided in usb_bootloader_config.h.
 *   This specifies whether the device is self-powered or bus-powered, and
 *   the maximum current usage when in bootloader mode.
 *   Ideally, a device should use as little current as possible if the PIC
 *   starts up in bootloader mode.
 *
 * - it must be compiled with the accompanying file crt_bootloader_512.S.
 *
 * - the linker script 16f145x_bootloader_512.lkr must be used.
 */

#include <pic14regs.h>
#include "usb_bootloader_config.h"

APP_CONFIG(USB_BUS_POWERED, 250 MILLIAMPS);
/*
 * File:   main.c
 * Author: boos
 *
 * Created on May 16, 2021, 10:27 PM
 */

// location of the LED, DATA output, and pushbutton

#define DATA LATCbits.LATC3
#define SW1  PORTAbits.RA3


#define uint8_t unsigned char

uint8_t DisplayArray[3*3];

#define BASE_EFFECT_SOLID  0
#define BASE_EFFECT_LARSON 1
#define BASE_EFFECT_NONE   2

// macro to send the bit 'b' (can be either 0 or 1)
void inline send(char b)  {
   DATA=1;
  __asm
	nop
	nop
	nop
 __endasm;
   DATA=b;
  __asm
	nop
	nop
	nop
	nop
 __endasm;
   DATA=0; 
  __asm
	nop
	nop
	nop
	nop
 __endasm;
}


// auxiliary functions to control the WS2812 NeoPixel LEDs
void sendByte (unsigned char b);

void sendArray (void) {
  uint8_t i;
  __asm
     bcf	INTCON,GIE
  __endasm;

  for (i=0;i<sizeof(DisplayArray);i++) {
     sendByte(DisplayArray[i]);
  }
  __asm
     bsf	INTCON,GIE
  __endasm;
}

void setPixel(uint8_t index, uint8_t r,uint8_t g, uint8_t b) {
   uint8_t offset = (index%3)*3;

   if (index<3) {
      DisplayArray[offset] = g;
      DisplayArray[offset+1] =r;
      DisplayArray[offset+2] =b;
   }
}

static void clearPixels() {
  for (int i=0;i<12*3;i++) {
     DisplayArray[i] =0;
  }
}


void app_init(void) {
    // LED is an output
    TRISC = 0x0; 

    // turn off analog to digital converter
    ANSELA = 0;
    ANSELC = 0;
    
    // button input
    TRISA = 0;
    TRISA |= 1<<3; // SW1 (aka PROG)
    
    // Enable TIMER1
    TMR1H = TMR1L = 0;

    // Setup to use TIMER1 - 1:8 , FOSC/4, No Sync, TimerON
    // Note: Get this wrong and all sorts of things don't work. ;o)
    T1CON = 0x35;
   
}
    
#define MAX_BRIGHTNESS 63 
#define MIN_BRIGHTNESS 0

// main function
int app_main (void) {
    // declare variables used for animation
    unsigned char brt = 1; 
    signed char dir = 1;
    int stage = 0;

    app_init();
    
    setPixel(0,100,0,0);
    setPixel(1,0,100,0);
    setPixel(2,0,0,100);
    // main loop
    while (1) {

        // Brightness up/down 
        brt += dir;
        if (brt == MAX_BRIGHTNESS) {
           dir = -1;
        } else if (brt == MIN_BRIGHTNESS) {
           dir = 1;
           stage+=1;
        }

        // send out animation pattern
        // R    RG    G    GB     B   BR 
        // 0 1  2  3  4  5  6  7  8 9 10  11
        setPixel((stage)%3,   brt,    0,      0 );
        setPixel((stage+1)%3, 0,    brt,      0 );
        setPixel((stage+2)%3, 0,      0,     brt);

        TMR1=64036; 

        PIR1bits.TMR1IF=0;

        // Check PROGRAM pin
        while (PIR1bits.TMR1IF==0) {
          if (SW1==0) {
	      __asm
	      reset
	      __endasm;
          }
        }

        PIR1bits.TMR1IF=0;

        sendArray();
    }
    
       return 1;
}

// send out a byte b in WS2812 protocol
void sendByte (unsigned char b) {

    if (b & 0b10000000) { send(1); } else { send(0); }
    if (b & 0b01000000) { send(1); } else { send(0); }
    if (b & 0b00100000) { send(1); } else { send(0); }
    if (b & 0b00010000) { send(1); } else { send(0); }
    if (b & 0b00001000) { send(1); } else { send(0); }
    if (b & 0b00000100) { send(1); } else { send(0); }
    if (b & 0b00000010) { send(1); } else { send(0); }
    if (b & 0b00000001) { send(1); } else { send(0); }
    
}

void app_interrupt( void ) {
}
