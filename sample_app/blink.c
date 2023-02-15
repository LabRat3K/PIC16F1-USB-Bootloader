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

APP_CONFIG(USB_BUS_POWERED, 20 MILLIAMPS);
// Timer1 configured for approx 15 Hz, so loop 7 times to get a short flash
#define LOOP_COUNTDOWN (7)

unsigned char loop_counter = LOOP_COUNTDOWN;
unsigned char display_phase = 0;

void app_interrupt(void)
{
 // Perhaps interrupt from something other than TMR1 ??
  if (PIR1bits.TMR1IF==1) {
     // Clear interrupt flag and toggle LED state
     PIR1bits.TMR1IF=0;

     loop_counter--;

     if (loop_counter==0) {
        // Every 'n' iterations-  update the LED state
        loop_counter=LOOP_COUNTDOWN;
        switch(display_phase) {
          case 0:
            LATC ^= 0x04; // Amber ON
            LATC |= 0x08; // Blue  OFF
            LATA |= 0x10; // Green OFF
            break;
          case 1:
            LATC |= 0x04;  // Amber OFF
            LATC ^= 0x08;  // Blue  ON
            LATA |= 0x10;  // Green OFF
            break;     
          case 2:
            LATC |= 0x0C;  // Amber & Blue OFF
            LATA ^= 0x10;  // Green ON
            break;
          default:
          case 3: // Clear the Three Flags
            LATC |= 0x0C;  // Amber & Blue OFF
            LATA |= 0x10;  // Green OFF
            break;
        }
        display_phase++;
        display_phase%=4; // Mod the loop_counter to 0-3
     }
  }
}


int app_main(void)
{

  // RA4 - is DIGITIAL NOT ANALOG!!
  ANSELA = 0;

  // RC2&3 (AMBER & BLUE LEDS) as outputs
  TRISC &= 0xF3;

  // RA4 (POWER LED (Green)) as an output
  TRISA &= 0xEF;

  // Green
  LATA &= 0xEF; // Green ON
  //LATA |= 0x10; // Green OFF

  //LATC &= 0xF3; // Blue & Amber ON
  LATC |= 0x0C; // Blue & Amber OFF

  // Enable TIMER1
  TMR1H = TMR1L = 0;

  // Setup to use TIMER1 - 1:8 , FOSC/4, No Sync, TimerON
  // Note: Get this wrong and all sorts of things don't work. ;o)
  T1CON = 0x35;

  // TIMER1 Interrupt Enable
  PIE1bits.TMR1IE = 1;

  // Peripheral Interrupts Enable 
  INTCONbits.PEIE = 1;

  // Global Interrupt Enable
  INTCONbits.GIE = 1;
  while (1) {
     if (PORTAbits.RA3 == 0) {
	__asm
	reset
	__endasm;
     }
   }
}

