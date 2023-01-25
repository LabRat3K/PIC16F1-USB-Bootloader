/*
 * File:   Main.c
 * Based on:  HID demo from Szymon Roslowski
 *
 * 2014-10-13 S. Roslowski Created
 * 2023-01-24 A. Williams - added Wyse Keyboard Handler & build for SDCC
 *
 */

#define USE_SDCC

#include "usb.h"

// If building with XC8, include the pic16f1455 register definitions
#ifndef USE_SDCC
   #include <pic16f1455.h>
#endif

#define USE_WYSE

// -----------------------------------------------------------------------------
// Configuration Word : For standalone mode. Not required for bootloaded build.
// -----------------------------------------------------------------------------

#ifndef USE_SDCC
   // CONFIG1
   #pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
   #pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
   #pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
   #pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is Digital Input)
   #pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
   #pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
   #pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
   #pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
   #pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

   // CONFIG2
   #pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
   #pragma config CPUDIV = NOCLKDIV // CPU System Clock Selection Bit (CPU system clock divided by 6)
   #pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
   #pragma config PLLMULT = 3x     // PLL Multipler Selection Bit (3x Output Frequency Selected)
   #pragma config PLLEN = ENABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
   #pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
   #pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
   #pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
   #pragma config LVP = OFF         // Low-Voltage Programming Enable (Low-voltage programming enabled)

   // Work around for converting SDCC ASM to XC8ASM
   #define __asm #asm
   #define __endasm #endasm
#endif


// ------------------------
// Button & LED definitions
// ------------------------
// Button used to trigger the bootloader reset & code
#define PROG_BUTTON              PORTAbits.RA3


// LED definitions
#define LED_NUMLOCK              LATAbits.LATA4
#define LED_CAPSLOCK             LATCbits.LATC3
#define LED_SCROLLOCK            LATCbits.LATC2

// ------------------------
// Bootloader Configuration 
// ------------------------

#define APP_CONFIG(selfpwr, milliamps)  const uint8_t app_config = ((selfpwr)!=0)|(((milliamps)/4)<<1)
APP_CONFIG(0 /*USB_BUS_POWERED*/, 20 );


// ----------------
// Global Variables
// ----------------

static uint8_t gDataPending = 0;

// -----------------------------------------------
// Interrupt Handler : Respond to USB transactions
// 
// Note: SDCC with bootloader, this is a function call
// and exits with a RETURN not a RETFIE
// -----------------------------------------------
#ifdef USE_SDCC
   void app_interrupt( void )
#else
   //static void interrupt ISRCode() 
#endif
{
    if (UsbInterrupt) {
       ProcessUSBTransactions();
    }
}

// -------------------------------------------------
// InitializeSystem(void)
//   System setup: Configure defaults for PIC SFRs
// -------------------------------------------------
static void InitializeSystem(void)
{
    TRISC =         0b00000000;     // Set RC3&RC2 as output 
    TRISA =         0b00101000;     // A5 & A3 as input
    LATC =          0b00001100;     // Clear Port C Latches
    LATA =          0b00010000;     // Clear Port A Latches

    ANSELA =        0x00;
    ANSELC =        0x00;
    OSCTUNE =       0x00;
    OSCCON =        0xFC;           // 16MHz HFINTOSC with 3x PLL enabled (48MHz operation)
    ACTCON =        0x90;           // Enable active clock tuning with USB
    OPTION_REG =    0xC3;           // Set prescaler to 256
}

// --------------------------------------------
// EnableInterrupts(void):
//    Enable Interrupt handler for USB events
// --------------------------------------------
static void EnableInterrupts(void)
{
    UIE = 0x4B;             // Transaction complete, Start Of Frame, Error, Reset
    INTCONbits.PEIE = 1;    // Peripheral Interrupt enable
    INTCONbits.GIE = 1;     // Global Interrupt Enable
    PIE2bits.USBIE = 1;     // Enable Usb Global Interrupt
}

// ------- WYSE Keyboard Driver Code --------
#ifdef USE_WYSE

// --------------------------------------------
// HID Key Modifiers
//    Modifier Keys (First Byte in Keyboard Message) 
// --------------------------------------------
#define KEY_L_CTRL        0x01
#define KEY_L_SHIFT       0x02
#define KEY_L_ALT         0x04
#define KEY_L_WIN         0x08
#define KEY_R_CTRL        0x10
#define KEY_R_SHIFT       0x20
#define KEY_R_ALT         0x40
#define KEY_R_WIN         0x80

// ---------------------
// Wyse Defines 
// ---------------------
   // Clock & Data PIN Definitions
   #define KBD_CLOCK LATC,5
   #define KBD_CLOCK_TRIS  TRISCbits.TRISC5

   #define KBD_DATA  PORTAbits.RA5
   #define KBD_DATA_TRIS TRISAbits.TRISA5

   #define BUFFER_SIZE 20
   #define BUFFER_BITS (BUFFER_SIZE*8)

// ---------------------
// Wyse Global Variables
// ---------------------
static uint8_t gKeyIndex = 0;

volatile uint8_t gWyseReadBuffer[BUFFER_SIZE];
volatile uint8_t gWyseChangeBuffer[BUFFER_SIZE];

// Track CAPSLOCK state (used to turn on/off the CAPSLOCK LED)
uint8_t gCapsLock;

// ------------------------------------------------------
// // KEYMAP: Map keypresses to windows HID events. 
// Some keys can/should appear in the byte 0 bitmask
//    ALT 0xe6 can be in the BIT mask at start of packet
//    CTL 0xe0 "" ""
//    DO = Right Meta (0x80)
//  L&R Shift
// Other mappings: 
//    PRINT = SYSRQ (0x46)
// ------------------------------------------------------
const uint8_t KEYMAP[80] = {
   0x2b,0x08,0x1a,0x14,0x15,0x17,0x1c,0x00,
   0x34,0x28,0x49,0x07,0x2c,0x16,0x00,0x0c,
   0x25,0x26,0x27,0x2d,0x2e,0x00,0x04,0x2a,
   0x19,0x05,0x11,0x10,0x00,0x1e,0x09,0x18,
   0xe5,0x00,0x29,0x00,0x36,0x1f,0x0a,0x12,
   0xe6,0xe5,0x00,0x48,0x37,0x20,0x0b,0x13,
   0x80,0x00,0xe0,0x00,0x38,0x21,0x0d,0x2f,
   0x00,0x51,0x50,0x39,0x1d,0x22,0x0e,0x30,
   0x52,0x00,0x00,0x31,0x1b,0x23,0x0f,0x35,
   0x4f,0x00,0x00,0x46,0x06,0x24,0x33,0x2a 
};
#endif

// -------------------------------------------------------------
// memset : simple impelemtnation to loop and initialize memory
//      
// --------------------------------------------------------------
static void memset (void *buf, unsigned char ch, unsigned char count) 
{
  register unsigned char *ret = buf;

  while (count--)
    {
      *(unsigned char *)ret = ch;
      ++ret;
    }
}


// --------------------------------------------------------------
// PrepareTxBuffer(void)
//     Report any pending kepressed ('n' at a time) to the HOST. 
//     Note some keys are represented in the bitmask of byte 0.
//     Once all possible keys have been considered, reset the  gDataPending
//     to false, allowing the scanning loop to rescan for more keypresses.
// ---------------------------------------------------------------
static void PrepareTxBuffer(void)
{
    uint8_t i=2;

    HIDTxBuffer[0] = 0x00;                          // Modifier Key Bits (we are not using any)
    HIDTxBuffer[1] = 0x00;                          // Second Byte always 0 (Padding Byte)

#ifdef USE_WYSE 
   while (i<HID_REPORT_BYTE_COUNT) {
        uint8_t bstatus = bstatus = (gWyseChangeBuffer[gKeyIndex>>3]>>(gKeyIndex & 7))&1;
        // While I haven't found a bit that matches.. increment gKeyIndex

        while((bstatus) && (gKeyIndex<sizeof(KEYMAP))) {
            gKeyIndex++;
            bstatus = (gWyseChangeBuffer[gKeyIndex>>3]>>(gKeyIndex & 7))&1;
        }
        // Either I have a match, or overflow (gKeyIndex=BUFFER_BITS-16)
        if (!bstatus) {
             // Lookup this gKeyIndex in our character map
             switch(gKeyIndex) {
               case 0: break;
                   case 32: HIDTxBuffer[0] |= KEY_R_SHIFT; break;
                   case 40: HIDTxBuffer[0] |= KEY_R_ALT; break;
                   case 41: 
               HIDTxBuffer[0] |= 0x02; 
               break;
                   case 48: HIDTxBuffer[0] |= KEY_R_WIN; break;
                   case 50: HIDTxBuffer[0] |= KEY_L_CTRL; break;
                   default:   
                     HIDTxBuffer[i] = KEYMAP[gKeyIndex];
                     i++;
                     break;
                }         
              // Advance to Next Character
           gKeyIndex++;
           
        } else {
             // No match.. fill the array with 0x00
             for ( ;i<HID_REPORT_BYTE_COUNT;i++) {
                 HIDTxBuffer[i] = 0x00;
             }
        } //if (bstat)
    }// while (i<HID_REPORT_BYTE_COUNT)

    // All possible indexes have been checked- scan again
    if (gKeyIndex==sizeof(KEYMAP)) {
       gDataPending=0; 
    }
#else
  for ( ;i<HID_REPORT_BYTE_COUNT;i++) {
     HIDTxBuffer[i] = 0x00;
   }
#endif
}

// -------------------------------------------------------
// ProcessIncommingData
//    Parse the LED status from the incomming HID control 
//    bitmask. 
// -------------------------------------------------------
static void ProcessIncommingData(void)
{
    // Windows Will send only a single Byte
    // with statuses of leds
    // first bit for num lock, second for caps etc..
      LED_NUMLOCK   = (HIDRxBuffer[0] & 0x01)==0;
      LED_CAPSLOCK  = (HIDRxBuffer[0] & 0x02)==0;
      LED_SCROLLOCK = (HIDRxBuffer[0] & 0x04)==0;
#ifdef USE_WYSE
   gCapsLock = (HIDRxBuffer[0] & 0x02)>0;
#endif
}

// -----------------------------------------------------
// CheckUsb
//    Poll the USB subsystem to determine if we have received
//    an incoming control request. 
// -----------------------------------------------------
static void CheckUsb(void)
{
    if(IsUsbDataAvailable(HID_INTERFACE_NUMBER) > 0 )
    {
        ProcessIncommingData();
        ReArmInterface(HID_INTERFACE_NUMBER);
    }
}


// ----------------- WYSE KEYBOARD DRIVER ----------------
#ifdef USE_WYSE
void ReadWyseKeyboard(void)
{
    for ( uint8_t i = 0; i < 19*8/*BUFFER_BITS*/; ++i ) {
        INTCONbits.GIE = 0;     // Global Interrupt Disable
   __asm 
   ; clear CLOCK
      banksel LATA
      bcf   KBD_CLOCK
      nop
      nop
      nop
      nop  ;; Arbitrary number .. can improve this with testing
      nop
      nop
      nop
      nop
      nop
      nop
      nop
      nop
      nop
      bsf KBD_CLOCK
   __endasm;
        INTCONbits.GIE = 1;     // Global Interrupt Enable
        uint8_t v = KBD_DATA;  
        // Strore the bit in the gWyseReadBuffer 
        gWyseReadBuffer[i >> 3] |= (v << (i & 7)); 
    }
    // CapsLock state - if enabled, add an additional CLOCK pulse
    if (gCapsLock) {
        INTCONbits.GIE = 0;     // Global Interrupt Disable
   __asm 
   ; clear CLOCK
      banksel LATA
      bcf   KBD_CLOCK
      nop
      nop
      nop
      nop
      nop
      nop
      nop
      nop
      nop
      nop
      nop
      nop
      nop
      bsf KBD_CLOCK
   __endasm;
        INTCONbits.GIE = 1;     // Global Interrupt Enable
    }
 /* BUG ASM not being encountered : USE C to force clock LOW*/
 /* Must stay low for more than 34 us to reset the shift register*/
    LATCbits.LATC5 = 0;

}

static void InitializeWyse() {

   // KBD_Clock as output
   KBD_CLOCK_TRIS = 0;
   // KBD_DATA as input
   KBD_DATA_TRIS = 1;

   // Default global variables & buffers
   gDataPending=0;
   gCapsLock=0;

   memset(gWyseChangeBuffer,0xff, sizeof(gWyseChangeBuffer));
}
#endif
// ------------------- End of WYSE Keyboard Code ---------------

// ------------------
// ProcessIO(void)
//    Check if the USB is ready for us to send, and check if 
//    there is data *to* send. If yes to both, prepare a payload
//    and queue it for transmission.
// ------------------
static void ProcessIO(void)
{
    // Check USB for incomming Commands
    if (IsUsbReady) CheckUsb();


    // Check Status Of the Keyboard 
    if (gDataPending==0) return;

    // If Data Pending then send some of it to HOST
    PrepareTxBuffer();
    HIDSend(HID_INTERFACE_NUMBER);

}


// ---------------------------------------
// MAIN: (main or booloader app_main)
//    Loop forever, polling for USB events 
//    and/or keypresses.
// ---------------------------------------
#ifdef USE_SDCC
int app_main(void) 
#else
static int main(void)
#endif
{
#ifdef USE_WYSE
    uint8_t tick_count;
#endif

#ifdef USE_WYSE
    InitializeWyse();
#endif
    InitializeSystem();
    InitializeUSB();
    EnableUSBModule();
    EnableInterrupts(); 
// Timer to pause between KEYBOARD polling events
#ifdef USE_WYSE
    // Setup a 5ms timer (frequency of polling)
    T1CON = 0x35;
    TMR1H  = TMR1L = 0;
    while(1) {
       ProcessIO();
       if (gDataPending == 0) { // Idle: Poll for keystrokes
          gKeyIndex=0; // Reset counter for exporting HID data
          memset(gWyseReadBuffer,0,sizeof(gWyseReadBuffer));
          ProcessIO();
          ReadWyseKeyboard();
          ProcessIO();
  
          // Check if data has gDataPending 
          for (uint8_t i=0;i< BUFFER_SIZE-2;i++) {
             if (gWyseReadBuffer[i] != gWyseChangeBuffer[i]) {
               gDataPending=1;
               gWyseChangeBuffer[i] = gWyseReadBuffer[i];
             }
          }
       } // End Poll for Keystrokes

      // 5ms delay before next poll for keystrokes 
      // ** This delay encompasses the minimum 34 us delay required 
      //    to reset the keyboard shift register
      TMR1H = TMR1L = 0;
      PIR1bits.TMR1IF = 0;
      tick_count=1;
      while (tick_count >0) {
         if (PIR1bits.TMR1IF==1) {
            PIR1bits.TMR1IF = 0;
            tick_count--;
         } 
         // While spinning: process any pending keystrokes
         ProcessIO();
      }
    }
#else
  while(1) {
   ProcessIO();
  }
#endif
}
