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
//#define LED LATCbits.LATC2
#define DATA LATAbits.LATA4
#define SW1 PORTAbits.RA3

#define uint8_t unsigned char

uint8_t DisplayArray[21*3];

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

#define USETHIS
#ifdef USETHIS

// auxiliary functions to control the WS2812 NeoPixel LEDs
void sendByte (unsigned char b);
void sendRGB (unsigned char r, unsigned char g, unsigned char b);

void sendArray (void) {
  uint8_t i;
  for (i=0;i<sizeof(DisplayArray);i++) {
     sendByte(DisplayArray[i]);
  }
}

void setArray(uint8_t index, uint8_t r,uint8_t g, uint8_t b) {
   uint8_t offset = (index%12)*3;

   DisplayArray[offset] = g;
   DisplayArray[offset+1] =r;
   DisplayArray[offset+2] =b;
}

void setBase(uint8_t r,uint8_t g, uint8_t b) {
   uint8_t i;
   for (i=36;i<22*3;i+=3) {
     DisplayArray[i] = g;
     DisplayArray[i+1] = r;
     DisplayArray[i+2] = b;
   }
}

// main function
int app_main (void) {
    
    
    // set up internal oscillator to run at 48 MHz
    
    // LED is an output
    TRISC = 0x00;
    
    // turn off analog to digital converter
    ANSELA = 0;
    
    // button input
    TRISA = 0;
    TRISA |= 1<<3; // SW1 (aka PROG)
    
    // Enable TIMER1
    TMR1H = TMR1L = 0;

    // Setup to use TIMER1 - 1:8 , FOSC/4, No Sync, TimerON
    // Note: Get this wrong and all sorts of things don't work. ;o)
    T1CON = 0x35;
   
 
    // declare variables used for animation
    unsigned char brt = 1; 
    signed char dir = 1;
    int timebase = 0, stage = 0;
    
    __asm
      bcf	INTCON,GIE
    __endasm;
#define MAX_BRIGHTNESS 127 
#define MIN_BRIGHTNESS 10
    for (int j=0;j<12;j++) {
        setArray(j,0,0,0);
    }
    setBase(90,0,90);
    sendArray();

 // Color Wheel... 

// R 0 -> 12*10
//    Amber
// G
//    Teal
// B
//    Violet
//
    // main loop
    while (1) {
        // react to button press
        brt += dir;
        if (brt == MAX_BRIGHTNESS) {
           dir = -1;
        } else if (brt == MIN_BRIGHTNESS) {
           dir = 1;
	   stage = (stage+1)%12;

	   switch (stage) {
             // R    RG    G    GB     B   BR 
             // 0 1  2  3  4  5  6  7  8 9 10  11
             case 0: setBase(48,0,  0); break;
             case 1: setBase(0 ,24, 0); break;
             case 2: setBase(48,48, 0); break;
             case 3: setBase(24,48, 0); break;
             case 4: setBase(0, 48, 0); break;

             case 5: setBase(0, 48,24); break;
             case 6: setBase(0, 48,48); break;
             case 7: setBase(0, 24,48); break;
             case 8: setBase(0,  0,48); break;
             case 9: setBase(24, 0,48); break;
             case 10: setBase(48,0,48); break;
             case 11: setBase(48,0,24); break;

           }

        }

        // time the animation
        /*timebase++;
        if (timebase > 200) {
            timebase = 0;
            stage = (stage+1)&0x07;
        }*/
        
 
        // send out animation pattern
        // R    RG    G    GB     B   BR 
        // 0 1  2  3  4  5  6  7  8 9 10  11
        setArray(stage,   brt,    0,      0 );
        setArray(stage+1, brt>>1, brt>>3, 0 );
        setArray(stage+2, brt>>2, brt>>2, 0 );
        setArray(stage+3, brt>>3, brt>>1, 0 );
        setArray(stage+4, 0     , brt   , 0 );
        setArray(stage+5, 0     , brt>>1, brt>>3 );
        setArray(stage+6, 0     , brt>>2, brt>>2 );
        setArray(stage+7, 0     , brt>>3, brt>>1 );
        setArray(stage+8, 0     , 0     , brt );
        setArray(stage+9, brt>>3, 0     , brt>>1 );
        setArray(stage+10,brt>>2, 0     , brt>>2 );
        setArray(stage+11,brt>>1, 0     , brt>>3 );

        //   __asm
        //	 bsf	INTCON,GIE
        //   __endasm;
            
        // Put some code to check for TMR roll over
         // wait some time
        //    __delay_ms(1);

        TMR1=64036; 
    

        PIR1bits.TMR1IF=0;

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

// send red, green, and blue values in WS2812 protocol
void sendRGB (unsigned char r, unsigned char g, unsigned char b) {

    sendByte(g);
    sendByte(r);
    sendByte(b);
    
}

#endif
void app_interrupt( void ) {
}
