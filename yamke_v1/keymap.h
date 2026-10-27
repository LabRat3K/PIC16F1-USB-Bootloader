#include "usb_hid_keys.h"

// LabRat's key encoder is limited to 6 possible inputs per player (joystick(4) and 2 buttons)
// So each profile (player 1-4) is listed, with SLOT 5 being for a "COIN DOOR" profile. 
// Key order: 0..7 Select, Up, Fire, Left, Down, Right
unsigned int8 const kbd_scancodes[6][10] = {
  // Player 1            Player 2         Player 3             Player 4          COIN DOOR
   KEY_MOD_LCTRL,   1,   KEY_A,      0,   KEY_MOD_RCTRL,   1,   KEY_B,      0,   KEY_5,         0,   // Button 1
   KEY_UP,          0,   KEY_R,      0,   KEY_I,           0,   KEY_Y,      0,   KEY_6,         0,   // Joystick up
   KEY_MOD_LALT,    1,   KEY_S,      0,   KEY_MOD_RSHIFT,  1,   KEY_E,      0,   KEY_7,         0,   // Button 2
   KEY_LEFT,        0,   KEY_D,      0,   KEY_J,           0,   KEY_V,      0,   KEY_8,         0,   // Joystick left
   KEY_DOWN,        0,   KEY_F,      0,   KEY_K,           0,   KEY_N,      0,   KEY_NONE,      0,   // Joystick down
   KEY_RIGHT,       0,   KEY_G,      0,   KEY_L,           0,   KEY_U,      0,   KEY_NONE,      0,   // Joystick right
}

// Note: Player Index is EVEN numbered, with key modifier = IDX+1 
int8 Player1_Idx = 0;
int8 Player2_Idx = 2; 
