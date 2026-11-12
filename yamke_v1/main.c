/*
 * File:   Main.c
 * Author: LabRat
 *
 * LabRat's MAME encoder
 * 12 Keyboard Inputs all with EXTERNAL pull-up to 5v
 * RA3 has EXTERNAL pull-up and is configured as a digital INPUT in the bootloader
 *
 *
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

LEDMAP: C0,C1 are outputs to toggle MM3906 transistors
 */

//#define HID_LEDS

#include "usb.h"
#include <pic14regs.h>
#include "usb_bootloader_config.h"
#include "usb_hid_keys.h"
#include "stdio.h"
#include "string.h"

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
extern volatile uint8_t hid1_out_buf[];
extern volatile uint8_t hid1_in_buf[];
extern volatile uint8_t hid2_out_buf[];
extern volatile uint8_t hid2_in_buf[];
extern volatile uint8_t cdc_comm_out_buf[];
extern volatile uint8_t cdc_comm_in_buf[];
extern volatile uint8_t cdc_out_buf[];
extern volatile uint8_t cdc_in_buf[];

// Internal state tracking for the bitwise filter
static uint8_t last_PlayerA = 0x00;
static uint8_t last_PlayerB = 0x00;

// Tx Cache of switch states
static uint8_t cachePlayerA = 0x00;
static uint8_t cachePlayerB = 0x00;

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
        current = (~((PORTAbits.RA4<<5) | ((PORTC&0x38)>>1) | (PORTAbits.RA5<<1) | PORTCbits.RC6))&0x3F;
        stable_mask = ~(current ^ last_PlayerA); // Bits are 1 if matching previous tick
        last_PlayerA = current;
        db_PlayerA = (db_PlayerA & ~stable_mask) | (current & stable_mask); // Update only stable bits

        // --- Process PLAYER2 ---
        //P2:0x0,0x0,B7,B6,B5,B4,C7,C2
        current = (~(((PORTB&0xF0)>>2) | (PORTCbits.RC7<<1) | PORTCbits.RC2))&0x3F;
        stable_mask = ~(current ^ last_PlayerB);
        last_PlayerB = current;
        db_PlayerB = (db_PlayerB & ~stable_mask) | (current & stable_mask);
    }
}

#define HEF_ADDR_MAGIC  0x1F80
#define HEF_ADDR_P1     0x1F81
#define HEF_ADDR_P2     0x1F82
#define HEF_ADDR_SN     0x1F83 // 9 bytes: 0x1F83..0x188B
#define HEF_MAGIC_VAL   0xA5

static uint8_t HEF_ReadByte(uint16_t addr)
{
    PMADRL = (uint8_t)(addr & 0xFF);        // Loads 0x80
    PMADRH = (uint8_t)((addr >> 8) & 0xFF); // Loads 0x1F
    PMCON1bits.CFGS = 0; // Access Program Flash (HEF)
    PMCON1bits.RD = 1;
    __asm
    nop
    nop
    __endasm;
    return PMDATL;
}

static void HEF_SaveProfiles(void)
{
    uint8_t gie_state = INTCONbits.GIE;
    uint8_t i;
    uint8_t data[12];
    data[0] = HEF_MAGIC_VAL;
    data[1] = Player1_Idx;
    data[2] = Player2_Idx;
    // Copy the SerialNumber
    for (i=0;i<9;i++) {
       data[3+i] = SerialStringDesc[2+i*2];
    }

    INTCONbits.GIE = 0; // Mandatory during Flash erase/write

    // 1. Erase the 32-word HEF row at 0x1F80..0x1F9F
    PMADRL = (uint8_t)(HEF_ADDR_MAGIC & 0xFF);        // Loads 0x80
    PMADRH = (uint8_t)((HEF_ADDR_MAGIC >> 8) & 0xFF); // Loads 0x1F
    PMCON1bits.CFGS = 0;
    PMCON1bits.FREE = 1;
    PMCON1bits.WREN = 1;
    PMCON2 = 0x55;
    PMCON2 = 0xAA;
    PMCON1bits.WR = 1;
    __asm
    nop
    nop
    __endasm;
    PMCON1bits.WREN = 0;

    // 2. Write the 12 bytes (Magic, Player1_Idx, Player2_Idx, SerialNo) using write latches
    for (i = 0; i < 12; i++) {
        uint16_t addr = HEF_ADDR_MAGIC+i;
        PMADRL = (uint8_t)(addr & 0xFF);
        PMADRH = (uint8_t)((addr+i >> 8) & 0xFF);
        PMDATH = 0x3F; // Upper 6 bits blank (0x3FFF unprogrammed)
        PMDATL = data[i];
        PMCON1bits.CFGS = 0;
        PMCON1bits.FREE = 0;
        PMCON1bits.LWLO = (i < 11) ? 1 : 0; // Load latches for i=0,1; commit on i=2
        PMCON1bits.WREN = 1;
        PMCON2 = 0x55;
        PMCON2 = 0xAA;
        PMCON1bits.WR = 1;
        __asm
        nop
        nop
        __endasm;
    }
    PMCON1bits.WREN = 0;
    INTCONbits.GIE = gie_state;
}


static void InitializeSystem(void)
{
    TRISC =         IN_MASK_C;
    TRISB =         IN_MASK_B;
    TRISA =         SW_MASK|IN_MASK_A;

    LATC =          0x00;     // Clear Port C Latches
    LATB =          0x00;     // Clear Port B Latches
    LATA =          0x00;     // Clear Port A Latches

    ANSELA =        0x00;
    ANSELB =        0x00;
    ANSELC =        0x00;

    // Disable comparators and serial peripherals so all pins act as pure digital GPIOs
    CM1CON0 = 0x00;
    CM2CON0 = 0x00;
    SSP1CON1 = 0x00;
    RCSTA = 0x00;
    TXSTA = 0x00;

    // Set initial LED states (OFF)
    P1LED = 0;
    P2LED = 0;

    OPTION_REG =    0xC3;           // Set prescaler to 256

    // Setup Timer2 for 1ms Debounce Checking
    T2CONbits.TMR2ON = 0;        // Turn off timer for now
    T2CONbits.T2OUTPS = 0b0000;  // 1:1 Postscaler
    T2CONbits.T2CKPS  = 0b11;    // 1:4 Prescaler
    PR2 = 187;                   // Match value for 1ms intervals at 48MHz/4
    PIR1bits.TMR2IF = 0;
    PIE1bits.TMR2IE = 1;         // Enable Timer2 Interrupts
    T2CONbits.TMR2ON = 1;        // Turn on timer

    if (HEF_ReadByte(HEF_ADDR_MAGIC) == HEF_MAGIC_VAL) {
        uint8_t p1 = HEF_ReadByte(HEF_ADDR_P1);
        uint8_t p2 = HEF_ReadByte(HEF_ADDR_P2);
        if (p1 <= 8 && (p1 & 0x01) == 0) Player1_Idx = p1;
        if (p2 <= 8 && (p2 & 0x01) == 0) Player2_Idx = p2;

        uint8_t j;
        for (j=0;j<9;j++) {
            uint8_t ch = HEF_ReadByte(HEF_ADDR_SN + j);
            if (ch >= 0x20 && ch <= 0x7E) {
                SerialStringDesc[2 + (j << 1)] = ch;
            }
        }
    }

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
    volatile uint8_t* HIDTxBuffer = (InterfaceNo == P1HidNo) ?hid1_in_buf: hid2_in_buf;

    HIDTxBuffer[0] = 0x00;                          // Modifier Key Bits (we are not using any)
    HIDTxBuffer[1] = 0x00;                          // Second Byte always 0 (Padding Byte)

    uint8_t i;
    for (i=0;i<6;i++) {
       if (bitfield & (1<<i)) {
          if (kbd_scancodes[i][profile+1]) {
             HIDTxBuffer[0] |= kbd_scancodes[i][profile];
          } else if (payloadIdx<8) {
             HIDTxBuffer[payloadIdx]= kbd_scancodes[i][profile];
             payloadIdx++;
          }
       }
    }
    for (i=payloadIdx;i<8;i++){
       HIDTxBuffer[i] = 0;
    }
}


void ProcessIncommingData(uint8_t leds)
{
    // Windows Will send only a single Byte
    // with statuses of leds
    // first bit for num lock, second for caps etc..
#ifdef HID_LEDS
// DEBUGGING ONLY : use the SCOLL/CAPSLOCK leds
    P1LED = (leds & 0x01)!=0;
    P2LED = (leds & 0x02)!=0;
#endif
}


#ifdef INC_CDC
#define CMD_BUF_LEN 32
static char cmdBuf[CMD_BUF_LEN];
static uint8_t cmdIdx = 0;

static void CDCPrint(const char *str)
{
    volatile uint8_t *dst = cdc_in_buf;
    uint8_t len = 0;

    while (!IsUsbTxReady(CdcNo));

    while (str[len] != '\0' && len < CdcReportByteCount) {
        dst[len] = str[len];
        len++;
    }
    if (len > 0) {
        while (!CDCSend(CdcNo, len)); // Wait if previous IN packet is still pending
    }
}

static void ExecuteCommand(void)
{
    cmdBuf[cmdIdx] = '\0';

    // Trim leading spaces
    char *p = cmdBuf;
    while (*p == ' ') p++;
    if (*p == '\0') return;

    // 1. "LED <idx> <val>" (e.g., "LED 1 1" or "LED 2 0")
    if (p[0] == 'L' && p[1] == 'E' && p[2] == 'D' && p[3] == ' ') {
        uint8_t led = p[4] - '0';
        uint8_t val = p[6] - '0';
        if (p[5] == ' ' && (val == 0 || val == 1)) {
            uint8_t doubled = led << 1;
            if (doubled == Player1_Idx || led == 1) { P1LED = (val != 0); CDCPrint("OK\r\n"); return; }
            if (doubled == Player2_Idx || led == 2) { P2LED = (val != 0); CDCPrint("OK\r\n"); return; }
        }
        CDCPrint("ERR LED\r\n");
    }
    // 2. "SET P1 <0..4>" or "SET P2 <0..4>"
    else if (p[0] == 'S' && p[1] == 'E' && p[2] == 'T' && p[3] == ' ' && p[4] == 'P') {
        uint8_t player = p[5] - '0';
        uint8_t prof = p[7] - '0';
        if (p[6] == ' ' && prof <= 4) {
            if (player == 1) { Player1_Idx = prof << 1; cachePlayerA = 0xFF; CDCPrint("OK\r\n"); return; }
            if (player == 2) { Player2_Idx = prof << 1; cachePlayerB = 0xFF; CDCPrint("OK\r\n"); return; }
        }
        CDCPrint("ERR SET\r\n");
    }
    // 3. "STATUS"
    else if (p[0] == 'S' && p[1] == 'T' && p[2] == 'A' && p[3] == 'T') {
        char resp[16] = "P1:0 P2:0\r\n";
        resp[3] = '0' + (Player1_Idx >> 1);
        resp[8] = '0' + (Player2_Idx >> 1);
        CDCPrint(resp);
    }
    // 4. "SAVE"
    else if (p[0] == 'S' && p[1] == 'A' && p[2] == 'V' && p[3] == 'E') {
        HEF_SaveProfiles();
        CDCPrint("SAVED\r\n");
    }
    // 5. "?" or "HELP"
    else if (p[0] == '?' || (p[0] == 'H' && p[1] == 'E' && p[2] == 'L' && p[3] == 'P')) {

        CDCPrint("CMDS: LED <1-2> <0|1>, SET P<1-2> <0-4>\r\n");
        CDCPrint("      STATUS, SAVE, ?\r\n");
/*
        CDCPrint("CMDS: STATUS - show player profiles\r\n");
        CDCPrint("      LED <1-2> <0|1> - set LED on/off\r\n");
        CDCPrint("      SET P<1-2> <0-4> -select player profile\r\n");
        CDCPrint("      SAVE - store profiles in EEPROM\r\n");
        CDCPrint("      HELP/? - print this text\r\n");
*/
    }
    // 6. "SN <9-char-serial>" (e.g., "SN YAMKE0002" followed by "SAVE")
    else if (p[0] == 'S' && p[1] == 'N' && p[2] == ' ') {
        uint8_t k;
        for (k = 0; k < 9; k++) {
            if (p[3 + k] == '\0') break;
        }
        if (k == 9 && p[12] == '\0') {
            for (k = 0; k < 9; k++) {
                SerialStringDesc[2 + (k << 1)] = p[3 + k];
            }
            CDCPrint("OK (RUN SAVE & REBOOT)\r\n");
        } else {
            CDCPrint("ERR SN (9 CHARS)\r\n");
        }
    }
    // 7. "REBOOT"
    else if (p[0] == 'R' && p[1] == 'E' && p[2] == 'B' && p[3] == 'O' && p[4] == 'O' && p[5] == 'T') {
       __asm
       reset
       __endasm;
    }
    else {
        CDCPrint("ERR CMD\r\n");
    }
}

void ProcessSerial(uint8_t InterfaceNo, uint8_t count)
{
    (void)InterfaceNo;
    uint8_t i;
    for (i = 0; i < count; i++) {
        uint8_t c = cdc_out_buf[i];
        // Convert lowercase a-z to uppercase A-Z for case-insensitive commands
        if (c >= 'a' && c <= 'z') c -= 32;

        if (c == '\r' || c == '\n') {
            if (cmdIdx > 0) {
                ExecuteCommand();
                cmdIdx = 0;
            }
        } else if (c == 0x08 || c == 0x7F) { // Backspace
            if (cmdIdx > 0) cmdIdx--;
        } else if (cmdIdx < (CMD_BUF_LEN - 1)) {
            cmdBuf[cmdIdx++] = c;
        }
    }
}

#endif // INC_CDC


static void CheckUsb(void)
{
   if(IsUsbDataAvailable(P1HidNo) > 0 )
   {
       ProcessIncommingData(hid1_out_buf[0]);
       ReArmInterface(P1HidNo);
   }

   if(IsUsbDataAvailable(P2HidNo) > 0 )
   {
       ProcessIncommingData(hid2_out_buf[0]);
       ReArmInterface(P2HidNo);
   }

#ifdef INC_CDC
   uint8_t cdcCount = IsUsbDataAvailable(CdcNo);
   if(cdcCount > 0 )
   {
       ProcessSerial(CdcNo,cdcCount);
       ReArmInterface(CdcNo);
   }
#endif
}

void ProcessIO(void)
{
    if (!IsUsbReady) return;

    // Check USB for incomming Commands
    CheckUsb();

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

    InitializeUSB();

    EnableUSBModule();

    EnableInterrupts();

    while(1) {
      ProcessIO();


      // Poll PROG_BUTTON
      if (PROG_BUTTON==0) {
           __asm
           reset
           __endasm;
      }
   } // while(1)
}

