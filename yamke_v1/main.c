/*
 * File:   Main.c
 * Author: LabRat
 *
 * LabRat's MAME encoder
PORTA:
   A3 - RESET SW
   A4 - J1 RIGHT
   A5 - J1 UP

PORTB:
   B4 - J2 FIRE
   B5 - J2 LEFT
   B6 - J2 DOWN
   B7 - J2 RIGHT

PORTC:
   C0 - J1 LED
   C1 - J2 LED
   C2 - J2 SELECT
   C3 - J1 FIRE
   C4 - J1 LEFT
   C5 - J1 DOWN
   C6 - J1 SELECT
   C7 - J2 UP

Player::Port mapping
   0x0,0x0, R, D, L, F, U, S
P1:0x0,0x0,A4,C5,C4,C3,A5,C6
P2:0x0,0x0,B7,B6,B5,B4,C7,C2

LEDMAP: C0,C1
 */

//#define LOGGING
#define HID_LEDS

#include "usb.h"
#include <pic14regs.h>
#include "usb_bootloader_config.h"
#include "usb_hid_keys.h"
#include "log.h"

#define PROG_BUTTON         PORTAbits.RA3
#define P1LED               LATCbits.LATC0
#define P2LED               LATCbits.LATC1

#define SW_MASK  (0x08)
#define OUT_MASK_C (0x03)
/*
#define J1_UP     PORTAbits.RA5
#define J1_RIGHT  PORTAbits.RA4
#define J1_DOWN   PORTCbits.RC5
#define J1_LEFT   PORTCbits.RC4
#define J1_FIRE   PORTCbits.RC3
#define J1_SELECT PORTCbits.RC6

#define J2_UP     PORTCbits.RC7
#define J2_RIGHT  PORTBbits.RB7
#define J2_DOWN   PORTBbits.RB6
#define J2_LEFT   PORTBbits.RB5
#define J2_FIRE   PORTBbits.RB4
#define J2_SELECT PORTCbits.RC2
*/
#define IN_MASK_A (0x38)
#define IN_MASK_B (0xF0)
#define IN_MASK_C (0xFC)


// Global variables tracking final debounced player switch states
volatile uint8_t db_PlayerA = 0;
volatile uint8_t db_PlayerB = 0;

// Internal state tracking for the bitwise filter
static uint8_t last_PlayerA = 0;
static uint8_t last_PlayerB = 0;

// Tx Cache of switch states
static uint8_t cachePlayerA = 0;
static uint8_t cachePlayerB = 0;

// Note: Player Index is EVEN numbered, with key modifier = IDX+1 
// Theses denote the profile associated with each player (0,2,4,6,8)
uint8_t Player1_Idx = 0;
uint8_t Player2_Idx = 2; 


APP_CONFIG(0 /*USB_BUS_POWERED*/, 20 );

// Interrupt
void app_interrupt( void )
{
    if (UsbInterrupt) {
       ProcessUSBTransactions();
    }

    // ------- GPIO POLLING ON TIMER2 -----
    // Check Timer2 Interrupt Flag
    if (PIR1bits.TMR2IF) {
        PIR1bits.TMR2IF = 0; // Clear Flag

        uint8_t current;
        uint8_t stable_mask;

        // --- Process PLAYER1 ---
        //P1:0x0,0x0,A4,C5,C4,C3,A5,C6
        current = (PORTAbits.RA4<<5) | ((PORTC&0x38)>>1) | (PORTAbits.RA5<<1) | PORTCbits.RC6;
        stable_mask = ~(current ^ last_PlayerA); // Bits are 1 if matching previous tick
        last_PlayerA = current;
        db_PlayerA = (db_PlayerA & ~stable_mask) | (current & stable_mask); // Update only stable bits

        // --- Process PLAYER2 ---
        //P2:0x0,0x0,B7,B6,B5,B4,C7,C2
        current = (PORTB&0xF0)>>2 | PORTCbits.RC7<<1 | PORTCbits.RC2;
        stable_mask = ~(current ^ last_PlayerB);
        last_PlayerB = current;
        db_PlayerB = (db_PlayerB & ~stable_mask) | (current & stable_mask);
    }
}

static void InitializeSystem(void)
{
    TRISC =         IN_MASK_C;
    TRISB =         IN_MASK_B;
    TRISA =         SW_MASK|IN_MASK_A;

    LATC =          0x00;     // Clear Port C Latches
    LATB =          0x00;     // Clear Port B Latches
    LATA =          0x00;     // Clear Port A Latches

/*
    ANSELA =        0x00;
    ANSELC =        0x00;
    OSCTUNE =       0x00;
    OSCCON =        0xFC;           // 16MHz HFINTOSC with 3x PLL enabled (48MHz operation)
    ACTCON =        0x90;           // Enable active clock tuning with USB
*/
    OPTION_REG =    0xC3;           // Set prescaler to 256

    // Setup Timer2 for 1ms Debounce Checking
    T2CONbits.TMR2ON = 0;        // Turn off timer for now
    T2CONbits.T2OUTPS = 0b0000;  // 1:1 Postscaler
    T2CONbits.T2CKPS  = 0b01;    // 1:4 Prescaler
    PR2 = 250;                   // Match value for 1ms intervals at 48MHz/4
    PIE1bits.TMR2IE = 1;         // Enable Timer2 Interrupts
    T2CONbits.TMR2ON = 1;        // Turn on timer   

    // Enable Logging
    uart_init();
    log_init();
}

static void EnableInterrupts(void)
{
    UIE = 0x4B;             // Transaction complete, Start Of Frame, Error, Reset
    INTCONbits.PEIE = 1;    // Peripheral Interrupt enable
    INTCONbits.GIE = 1;     // Global Interrupt Enable
    PIE2bits.USBIE = 1;     // Enable Usb Global Interrupt
}



// LabRat's key encoder is limited to 6 possible inputs per player (joystick(4) and 2 buttons)
// So each profile (player 1-4) is listed, with SLOT 5 being for a "COIN DOOR" profile. 
// Key order: 0..7 Select, Up, Fire, Left, Down, Right
uint8_t const kbd_scancodes[6][10] = {
  // Player 1            Player 2         Player 3             Player 4          COIN DOOR
  {KEY_MOD_LCTRL,   1,   KEY_A,      0,   KEY_MOD_RCTRL,   1,   KEY_B,      0,   KEY_5,         0},   // Button 1
  {KEY_UP,          0,   KEY_R,      0,   KEY_I,           0,   KEY_Y,      0,   KEY_6,         0},   // Joystick up
  {KEY_MOD_LALT,    1,   KEY_S,      0,   KEY_MOD_RSHIFT,  1,   KEY_E,      0,   KEY_7,         0},   // Button 2
  {KEY_LEFT,        0,   KEY_D,      0,   KEY_J,           0,   KEY_V,      0,   KEY_8,         0},   // Joystick left
  {KEY_DOWN,        0,   KEY_F,      0,   KEY_K,           0,   KEY_N,      0,   KEY_NONE,      0},   // Joystick down
  {KEY_RIGHT,       0,   KEY_G,      0,   KEY_L,           0,   KEY_U,      0,   KEY_NONE,      0},   // Joystick right
};

// ----------------------------------

void PrepareTxBuffer(uint8_t profile, uint8_t bitfield, uint8_t InterfaceNo)
{
    uint8_t payloadIdx=2;
    uint8_t* HIDTxBuffer = GetEpTxBuffer(InterfaceNo);

    HIDTxBuffer[0] = 0x00;                          // Modifier Key Bits (we are not using any)
    HIDTxBuffer[1] = 0x00;                          // Second Byte always 0 (Padding Byte)

    uint8_t i;
    for (i=0;i<6;i++) {
       if (bitfield | 1<<i) {
          if (kbd_scancodes[profile][i*2+1]) {
             HIDTxBuffer[0] |= kbd_scancodes[profile][i*2];
          } else {
             HIDTxBuffer[payloadIdx]= kbd_scancodes[profile][i*2];
             payloadIdx++;
          }
       }
    }
    for (i=payloadIdx;i<8;i++){
       HIDTxBuffer[i] = 0;
    }
}


void ProcessIncommingData(uint8_t InterfaceNo)
{
    log_byte(':');
    uint8_t* HIDRxBuffer = GetEpRxBuffer(InterfaceNo);

    // Windows Will send only a single Byte
    // with statuses of leds
    // first bit for num lock, second for caps etc..
#ifdef HID_LEDS
// DEBUGGING ONLY : use the SCOLL/CAPSLOCK leds 
    P1LED = (HIDRxBuffer[0] & 0x01)==0;
    P2LED = (HIDRxBuffer[0] & 0x02)==0;
#endif
}


void ProcessSerial(uint8_t InterfaceNo,uint8_t count) 
{
   uint8_t* CdcRxBuffer = GetEpRxBuffer(InterfaceNo);
   uint8_t* CdcTxBuffer = GetEpTxBuffer(InterfaceNo);

#ifndef HID_LEDS
   // TODO: Update this to scan for LED update messages (0 though 4 based on player profile)
    // Parse text stream (line by line?) 
    // If token = CMD 
    //      SET P1 = 0..4
    //    If setting P1 Player1_Idx = new profile <<1;
    //    If setting P2 Player2_Idx = new profile <<1;
    //    If SAVE cmd - write profiles to the EEPROM 
    // Parse text stream looking for LEDx = 1/0 strings
    //   for each LEDx in string
    //     if (x == Player1_Idx <<1 ) P1LED = on/off
    //     if (x == Player2_Idx <<1 ) P2LED = on/off
#else
   // For now do nothing... but will want to reivew the BDT[EP].CNT and then cycle through bytes
   for (uint8_t i=0;i<count;i++){
     *CdcTxBuffer = *CdcRxBuffer;
      CdcRxBuffer++;
      CdcTxBuffer++;
   }

   CDCSend(CdcNo,count);
#endif
}


static void CheckUsb(void)
{
    if(IsUsbDataAvailable(P1HidNo) > 0 )
    {
        log_byte('1');
        ProcessIncommingData(P1HidNo);
        ReArmInterface(P1HidNo);
    }

    if(IsUsbDataAvailable(P2HidNo) > 0 )
    {
        log_byte('2');
        ProcessIncommingData(P2HidNo);
        ReArmInterface(P2HidNo);
    }

    uint8_t cdcCount = IsUsbDataAvailable(CdcNo);
    if(cdcCount > 0 )
    {
        log_byte('~');
        ProcessSerial(CdcNo,cdcCount);
        ReArmInterface(CdcNo);
    }


}

void ProcessIO(void)
{
    // Check USB for incomming Commands
    if (IsUsbReady) CheckUsb();


    // Check Status Of the Keyboard 
    if (cachePlayerA != db_PlayerA) {
        PrepareTxBuffer(Player1_Idx, db_PlayerA, P1HidNo);
        if (HIDSend(P1HidNo)) cachePlayerA = db_PlayerA;
    }

    if (cachePlayerB != db_PlayerB) {
        PrepareTxBuffer(Player2_Idx,db_PlayerB, P2HidNo);
        if (HIDSend(P2HidNo)) cachePlayerB = db_PlayerB;
    }

}

int app_main(void)
{
    InitializeSystem();
    log_byte('S');

    InitializeUSB();
    log_byte('U');

    EnableUSBModule();
    log_byte('E');

    EnableInterrupts(); 
    log_byte('I');

    while(1) {
      ProcessIO();
      log_service();


      // Poll PROG_BUTTON
      if (PROG_BUTTON==0) {
           __asm
           reset
           __endasm;
      }
   } // while(1)
}

