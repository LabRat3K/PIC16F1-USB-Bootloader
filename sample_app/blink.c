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

#include "log.h"

#define LED LATCbits.LATC2
#define SWITCH PORTAbits.RA3
#define ON (0)
#define OFF (1)

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
            LED = ON; 
            break;
          case 1:
            LED = OFF;
            break;     
        }
        display_phase++;
        display_phase%=2; // Mod the loop_counter to 0-3
     }
  }
}

int app_main(void)
{

  // RA4 - is DIGITIAL NOT ANALOG!!
  ANSELA = 0;

 
  // PORTC
  // TRISC: set to 0 for OUTPUT or 1 for INPUT 
  // LED: C2 - OUTPUT
  // PORT A3 - INPUT (switch) 
  TRISCbits.TRISC2 = 0; // LED as an output

  // RA3 : SWITCH (input)
  TRISAbits.TRISA3 = 1;

  LED = ON; // Turn on STATUS led 

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

  uart_init();
  log_init();
  log_char('t',0);
  log_char('e',0);
  log_char('s',0);
  log_char('t',0);
  log_char(0x8A,FMT_HEX|FMT_SPACE|FMT_EOL);

  set_multi(4, FMT_HEX|FMT_SPACE);
  mlog_hex(0x01);
  mlog_hex(0x02);
  mlog_hex(0x03);
  mlog_hex(0x42);
 
  while (1) {
     log_service();
     if (SWITCH==ON) {
        __asm
	reset
	__endasm;
     }
  }
}

