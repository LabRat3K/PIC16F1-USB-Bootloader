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

#define DATA LATAbits.LATA4
#define SW1  PORTAbits.RA3

// Defined as OUTPUTS
#define ROW1 LATCbits.LATC4
#define ROW2 LATCbits.LATC5
#define ROW3 LATAbits.LATA5

// Define as INPUTS
#define COL1 PORTC.RC0
#define COL2 PORTC.RC1
#define COL3 PORTC.RC2
#define COL4 PORTC.RC3

#define uint8_t unsigned char

uint8_t DisplayArray[21*3];
uint8_t base_effect;

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
   uint8_t offset = (index%12)*3;

   if (index<21) {
      DisplayArray[offset] = g;
      DisplayArray[offset+1] =r;
      DisplayArray[offset+2] =b;
   }
}

void setBase(signed char bid, uint8_t r, uint8_t g, uint8_t b) {
   if (bid<0)
     return;

   if (bid>8)
     return;

   uint8_t offset = (12+bid)*3;  // Normallize then add to BASE offset

   DisplayArray[offset] = g;
   DisplayArray[offset+1] = r;
   DisplayArray[offset+2] = b;

}

static void clearPixels() {
  for (int i=0;i<12*3;i++) {
     DisplayArray[i] =0;
  }
}

void floodBase(uint8_t r,uint8_t g, uint8_t b) {
   uint8_t i;
   for (i=36;i<22*3;i+=3) {
     DisplayArray[i] = g;
     DisplayArray[i+1] = r;
     DisplayArray[i+2] = b;
   }
}


static void DrawLarsen(signed char index) {
   setBase(index,   0, 0, 0x10);
   setBase(index+1, 0, 0, 0x20);
   setBase(index+2, 0, 0x10, 0x40);
   setBase(index+3, 0, 0, 0x20);
   setBase(index+4, 0, 0, 0x10);
}

signed char larsen_index;
uint8_t larsen_count;
signed char larsen_dir;  // Animation Direction

void larsen_tick(void) {
   larsen_count--;
   if (larsen_count==0) {
      larsen_index += larsen_dir;
      larsen_count = 3;

      floodBase(0,0,0);
      DrawLarsen(larsen_index);

      if (larsen_index>0x08) {
         larsen_dir = -1;
      } else if (larsen_index == -5) {
         larsen_dir = 1;
      } 
      
   } 
}

void app_init(void) {
    // LED is an output
    //TRISC = 0x0F; //RC0-RC3 (Columns) are INPUTS
    TRISC = 0x0; 
    LATC = LATC&0xF0; 

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
   
    larsen_index = -5;
    larsen_dir = 1;
    larsen_count = 3;
}
    
uint8_t KEYS[3];

const uint8_t ColorMap [12*3] = {
           0x40,    0,      0 ,
           0x40>>1, 0x40>>3, 0 ,
           0x40>>2, 0x40>>2, 0,
           0x40>>3, 0x40>>1, 0,
           0     , 0x40   , 0,
           0     , 0x40>>1, 0x40>>3,
           0     , 0x40>>2, 0x40>>2,
           0     , 0x40>>3, 0x40>>1,
           0     , 0     , 0x40,
           0x40>>3, 0     , 0x40>>1,
           0x40>>2, 0     , 0x40>>2,
           0x40>>1, 0     , 0x40>>3
};


static void checkRow0(void) {
   if (KEYS[0]&0x01) {
      setPixel(0,0x80,0,0);
      larsen_dir = 1;
      base_effect = BASE_EFFECT_SOLID;
    }
   else 
      setPixel(0,0,0,0);

   if (KEYS[0]&0x02)  {
      setPixel(1,0,0x80,0);
      base_effect = BASE_EFFECT_LARSON;
    }
   else
      setPixel(1,0,0,0);

   if (KEYS[0]&0x04)  {
      setPixel(2,0,0,0x80);
      base_effect = BASE_EFFECT_NONE;
    }
   else
      setPixel(2,0,0,0);

   if (KEYS[0]&0x08)
      setPixel(3,0x80,0,0x80);
   else
      setPixel(3,0,0,0);
}

static void checkRow1(void) {
   if (KEYS[1]&0x01) 
      setPixel(4,0x80,0,0);
   else 
      setPixel(4,0,0,0);

   if (KEYS[1]&0x02) 
      setPixel(5,0,0x80,0);
   else
      setPixel(5,0,0,0);

   if (KEYS[1]&0x04) 
      setPixel(6,0,0,0x80);
   else
      setPixel(6,0,0,0);

   if (KEYS[1]&0x08)
      setPixel(7,0x80,0,0x80);
   else
      setPixel(7,0,0,0);
} 

static void checkRow2(void) {
   if (KEYS[2]&0x01) 
      setPixel(8,0x80,0,0);
   else 
      setPixel(8,0,0,0);

   if (KEYS[2]&0x02) 
      setPixel(9,0,0x80,0);
   else
      setPixel(9,0,0,0);

   if (KEYS[2]&0x04) 
      setPixel(10,0,0,0x80);
   else
      setPixel(10,0,0,0);

   if (KEYS[2]&0x08)
      setPixel(11,0x80,0,0x80);
   else
      setPixel(11,0,0,0);
} 

#define MAX_BRIGHTNESS 63 
#define MIN_BRIGHTNESS 10



static void scanKeys() {
   ROW1 = 1;
   ROW2 = 0;
   ROW3 = 0;
   TRISC = 0x0F; //RC0-RC3 (Columns) are INPUTS
   KEYS[0] = PORTC&0x0F;
   TRISC = 0x00; //RC0-RC3 (Columns) are INPUTS
   ROW1 = 0;
   ROW2 = 1;
   TRISC = 0x0F; //RC0-RC3 (Columns) are INPUTS
   KEYS[1] = PORTC&0x0F;
   TRISC = 0x00; //RC0-RC3 (Columns) are INPUTS
   ROW2 = 0;
   ROW3 = 1;
   TRISC = 0x0F; //RC0-RC3 (Columns) are INPUTS
   KEYS[2] = PORTC&0x0F;
   TRISC = 0x00; //RC0-RC3 (Columns) are INPUTS
   ROW3 = 0;
}


// main function
int app_main (void) {
    base_effect = BASE_EFFECT_LARSON; 
    // declare variables used for animation
    unsigned char brt = 1; 
    signed char dir = 1;
    int stage = 0;

    app_init();    
    
    // main loop
    while (1) {
        switch(base_effect) {
          case BASE_EFFECT_LARSON:
	      larsen_tick();
              break;
          case BASE_EFFECT_SOLID:
              floodBase(0x40,0x00,0x0);
              break; 
          default: // BASE_EFFECT_NONE
              floodBase(0x0,0x0,0x0);
              break;
        }

        scanKeys();
	checkRow0();
        checkRow1();
        checkRow2();
        // Brightness up/down 
        brt += dir;
        if (brt == MAX_BRIGHTNESS) {
           dir = -1;
        } else if (brt == MIN_BRIGHTNESS) {
           dir = 1;
	   stage = (stage+1)%12;
        }
/*
        if (KEYS[0]&0x01) {
          setPixel(0,0,0x60,0);
        } else {
           // send out animation pattern
           // R    RG    G    GB     B   BR 
           // 0 1  2  3  4  5  6  7  8 9 10  11
           setPixel(stage,   brt,    0,      0 );
           setPixel(stage+1, brt>>1, brt>>3, 0 );
           setPixel(stage+2, brt>>2, brt>>2, 0 );
           setPixel(stage+3, brt>>3, brt>>1, 0 );
           setPixel(stage+4, 0     , brt   , 0 );
           setPixel(stage+5, 0     , brt>>1, brt>>3 );
           setPixel(stage+6, 0     , brt>>2, brt>>2 );
           setPixel(stage+7, 0     , brt>>3, brt>>1 );
           setPixel(stage+8, 0     , 0     , brt );
           setPixel(stage+9, brt>>3, 0     , brt>>1 );
           setPixel(stage+10,brt>>2, 0     , brt>>2 );
           setPixel(stage+11,brt>>1, 0     , brt>>3 );
        }
*/

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

void app_interrupt( void ) {
}
