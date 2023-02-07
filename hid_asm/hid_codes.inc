; vim:noet:sw=8:ts=8:ai:syn=pic
; asmsyntax=pic

;
;Modifier masks - used for the first byte in the HID report.
;NOTE: The second byte in the report is reserved, equ 0x00
;
KEY_MOD_LCTRL  equ 0x01
KEY_MOD_LSHIFT equ 0x02
KEY_MOD_LALT   equ 0x04
KEY_MOD_LMETA  equ 0x08
KEY_MOD_RCTRL  equ 0x10
KEY_MOD_RSHIFT equ 0x20
KEY_MOD_RALT   equ 0x40
KEY_MOD_RMETA  equ 0x80

;
;Scan codes - last N slots in the HID report (usually 6).
;0x00 if no key pressed.
;
;If more than N keys are pressed, the HID reports 
;KEY_ERR_OVF in all slots to indicate this condition.
;

KEY_NONE equ 0x00 ; No key pressed
KEY_ERR_OVF equ 0x01 ;  Keyboard Error Roll Over - used for all slots if too many keys are pressed ("Phantom key")
; equ 0x02 //  Keyboard POST Fail
; equ 0x03 //  Keyboard Error Undefined
KEY_A equ 0x04 ; Keyboard a and A
KEY_B equ 0x05 ; Keyboard b and B
KEY_C equ 0x06 ; Keyboard c and C
KEY_D equ 0x07 ; Keyboard d and D
KEY_E equ 0x08 ; Keyboard e and E
KEY_F equ 0x09 ; Keyboard f and F
KEY_G equ 0x0a ; Keyboard g and G
KEY_H equ 0x0b ; Keyboard h and H
KEY_I equ 0x0c ; Keyboard i and I
KEY_J equ 0x0d ; Keyboard j and J
KEY_K equ 0x0e ; Keyboard k and K
KEY_L equ 0x0f ; Keyboard l and L
KEY_M equ 0x10 ; Keyboard m and M
KEY_N equ 0x11 ; Keyboard n and N
KEY_O equ 0x12 ; Keyboard o and O
KEY_P equ 0x13 ; Keyboard p and P
KEY_Q equ 0x14 ; Keyboard q and Q
KEY_R equ 0x15 ; Keyboard r and R
KEY_S equ 0x16 ; Keyboard s and S
KEY_T equ 0x17 ; Keyboard t and T
KEY_U equ 0x18 ; Keyboard u and U
KEY_V equ 0x19 ; Keyboard v and V
KEY_W equ 0x1a ; Keyboard w and W
KEY_X equ 0x1b ; Keyboard x and X
KEY_Y equ 0x1c ; Keyboard y and Y
KEY_Z equ 0x1d ; Keyboard z and Z

KEY_1 equ 0x1e ; Keyboard 1 and !
KEY_2 equ 0x1f ; Keyboard 2 and @
KEY_3 equ 0x20 ; Keyboard 3 and #
KEY_4 equ 0x21 ; Keyboard 4 and $
KEY_5 equ 0x22 ; Keyboard 5 and %
KEY_6 equ 0x23 ; Keyboard 6 and ^
KEY_7 equ 0x24 ; Keyboard 7 and &
KEY_8 equ 0x25 ; Keyboard 8 and *
KEY_9 equ 0x26 ; Keyboard 9 and (
KEY_0 equ 0x27 ; Keyboard 0 and )

KEY_ENTER equ 0x28 ; Keyboard Return (ENTER)
KEY_ESC equ 0x29 ; Keyboard ESCAPE
KEY_BACKSPACE equ 0x2a ; Keyboard DELETE (Backspace)
KEY_TAB equ 0x2b ; Keyboard Tab
KEY_SPACE equ 0x2c ; Keyboard Spacebar
KEY_MINUS equ 0x2d ; Keyboard - and _
KEY_EQUAL equ 0x2e ; Keyboard = and +
KEY_LEFTBRACE equ 0x2f ; Keyboard [ and {
KEY_RIGHTBRACE equ 0x30 ; Keyboard ] and }
KEY_BACKSLASH equ 0x31 ; Keyboard \ and |
KEY_HASHTILDE equ 0x32 ; Keyboard Non-US # and ~
KEY_SEMICOLON equ 0x33 ; Keyboard ; and :
KEY_APOSTROPHE equ 0x34 ; Keyboard ' and "
KEY_GRAVE equ 0x35 ; Keyboard ` and ~
KEY_COMMA equ 0x36 ; Keyboard , and <
KEY_DOT equ 0x37 ; Keyboard . and >
KEY_SLASH equ 0x38 ; Keyboard / and ?
KEY_CAPSLOCK equ 0x39 ; Keyboard Caps Lock

KEY_F1 equ 0x3a ; Keyboard F1
KEY_F2 equ 0x3b ; Keyboard F2
KEY_F3 equ 0x3c ; Keyboard F3
KEY_F4 equ 0x3d ; Keyboard F4
KEY_F5 equ 0x3e ; Keyboard F5
KEY_F6 equ 0x3f ; Keyboard F6
KEY_F7 equ 0x40 ; Keyboard F7
KEY_F8 equ 0x41 ; Keyboard F8
KEY_F9 equ 0x42 ; Keyboard F9
KEY_F10 equ 0x43 ; Keyboard F10
KEY_F11 equ 0x44 ; Keyboard F11
KEY_F12 equ 0x45 ; Keyboard F12

KEY_SYSRQ equ 0x46 ; Keyboard Print Screen
KEY_SCROLLLOCK equ 0x47 ; Keyboard Scroll Lock
KEY_PAUSE equ 0x48 ; Keyboard Pause
KEY_INSERT equ 0x49 ; Keyboard Insert
KEY_HOME equ 0x4a ; Keyboard Home
KEY_PAGEUP equ 0x4b ; Keyboard Page Up
KEY_DELETE equ 0x4c ; Keyboard Delete Forward
KEY_END equ 0x4d ; Keyboard End
KEY_PAGEDOWN equ 0x4e ; Keyboard Page Down
KEY_RIGHT equ 0x4f ; Keyboard Right Arrow
KEY_LEFT equ 0x50 ; Keyboard Left Arrow
KEY_DOWN equ 0x51 ; Keyboard Down Arrow
KEY_UP equ 0x52 ; Keyboard Up Arrow

KEY_NUMLOCK equ 0x53 ; Keyboard Num Lock and Clear
KEY_KPSLASH equ 0x54 ; Keypad /
KEY_KPASTERISK equ 0x55 ; Keypad *
KEY_KPMINUS equ 0x56 ; Keypad -
KEY_KPPLUS equ 0x57 ; Keypad +
KEY_KPENTER equ 0x58 ; Keypad ENTER
KEY_KP1 equ 0x59 ; Keypad 1 and End
KEY_KP2 equ 0x5a ; Keypad 2 and Down Arrow
KEY_KP3 equ 0x5b ; Keypad 3 and PageDn
KEY_KP4 equ 0x5c ; Keypad 4 and Left Arrow
KEY_KP5 equ 0x5d ; Keypad 5
KEY_KP6 equ 0x5e ; Keypad 6 and Right Arrow
KEY_KP7 equ 0x5f ; Keypad 7 and Home
KEY_KP8 equ 0x60 ; Keypad 8 and Up Arrow
KEY_KP9 equ 0x61 ; Keypad 9 and Page Up
KEY_KP0 equ 0x62 ; Keypad 0 and Insert
KEY_KPDOT equ 0x63 ; Keypad . and Delete

KEY_102ND equ 0x64 ; Keyboard Non-US \ and |
KEY_COMPOSE equ 0x65 ; Keyboard Application
KEY_POWER equ 0x66 ; Keyboard Power
KEY_KPEQUAL equ 0x67 ; Keypad =

KEY_F13 equ 0x68 ; Keyboard F13
KEY_F14 equ 0x69 ; Keyboard F14
KEY_F15 equ 0x6a ; Keyboard F15
KEY_F16 equ 0x6b ; Keyboard F16
KEY_F17 equ 0x6c ; Keyboard F17
KEY_F18 equ 0x6d ; Keyboard F18
KEY_F19 equ 0x6e ; Keyboard F19
KEY_F20 equ 0x6f ; Keyboard F20
KEY_F21 equ 0x70 ; Keyboard F21
KEY_F22 equ 0x71 ; Keyboard F22
KEY_F23 equ 0x72 ; Keyboard F23
KEY_F24 equ 0x73 ; Keyboard F24

KEY_OPEN equ 0x74 ; Keyboard Execute
KEY_HELP equ 0x75 ; Keyboard Help
KEY_PROPS equ 0x76 ; Keyboard Menu
KEY_FRONT equ 0x77 ; Keyboard Select
KEY_STOP equ 0x78 ; Keyboard Stop
KEY_AGAIN equ 0x79 ; Keyboard Again
KEY_UNDO equ 0x7a ; Keyboard Undo
KEY_CUT equ 0x7b ; Keyboard Cut
KEY_COPY equ 0x7c ; Keyboard Copy
KEY_PASTE equ 0x7d ; Keyboard Paste
KEY_FIND equ 0x7e ; Keyboard Find
KEY_MUTE equ 0x7f ; Keyboard Mute
KEY_VOLUMEUP equ 0x80 ; Keyboard Volume Up
KEY_VOLUMEDOWN equ 0x81 ; Keyboard Volume Down
; equ 0x82  Keyboard Locking Caps Lock
; equ 0x83  Keyboard Locking Num Lock
; equ 0x84  Keyboard Locking Scroll Lock
KEY_KPCOMMA equ 0x85 ; Keypad Comma
; equ 0x86  Keypad Equal Sign
KEY_RO equ 0x87 ; Keyboard International1
KEY_KATAKANAHIRAGANA equ 0x88 ; Keyboard International2
KEY_YEN equ 0x89 ; Keyboard International3
KEY_HENKAN equ 0x8a ; Keyboard International4
KEY_MUHENKAN equ 0x8b ; Keyboard International5
KEY_KPJPCOMMA equ 0x8c ; Keyboard International6
; equ 0x8d  Keyboard International7
; equ 0x8e  Keyboard International8
; equ 0x8f  Keyboard International9
KEY_HANGEUL equ 0x90 ; Keyboard LANG1
KEY_HANJA equ 0x91 ; Keyboard LANG2
KEY_KATAKANA equ 0x92 ; Keyboard LANG3
KEY_HIRAGANA equ 0x93 ; Keyboard LANG4
KEY_ZENKAKUHANKAKU equ 0x94 ; Keyboard LANG5
; equ 0x95  Keyboard LANG6
; equ 0x96  Keyboard LANG7
; equ 0x97  Keyboard LANG8
; equ 0x98  Keyboard LANG9
; equ 0x99  Keyboard Alternate Erase
; equ 0x9a  Keyboard SysReq/Attention
; equ 0x9b  Keyboard Cancel
; equ 0x9c  Keyboard Clear
; equ 0x9d  Keyboard Prior
; equ 0x9e  Keyboard Return
; equ 0x9f  Keyboard Separator
; equ 0xa0  Keyboard Out
; equ 0xa1  Keyboard Oper
; equ 0xa2  Keyboard Clear/Again
; equ 0xa3  Keyboard CrSel/Props
; equ 0xa4  Keyboard ExSel

; equ 0xb0  Keypad 00
; equ 0xb1  Keypad 000
; equ 0xb2  Thousands Separator
; equ 0xb3  Decimal Separator
; equ 0xb4  Currency Unit
; equ 0xb5  Currency Sub-unit
KEY_KPLEFTPAREN equ 0xb6 ; Keypad (
KEY_KPRIGHTPAREN equ 0xb7 ; Keypad )
; equ 0xb8  Keypad {
; equ 0xb9  Keypad }
; equ 0xba  Keypad Tab
; equ 0xbb  Keypad Backspace
; equ 0xbc  Keypad A
; equ 0xbd  Keypad B
; equ 0xbe  Keypad C
; equ 0xbf  Keypad D
; equ 0xc0  Keypad E
; equ 0xc1  Keypad F
; equ 0xc2  Keypad XOR
; equ 0xc3  Keypad ^
; equ 0xc4  Keypad %
; equ 0xc5  Keypad <
; equ 0xc6  Keypad >
; equ 0xc7  Keypad &
; equ 0xc8  Keypad &&
; equ 0xc9  Keypad |
; equ 0xca  Keypad ||
; equ 0xcb  Keypad :
; equ 0xcc  Keypad #
; equ 0xcd  Keypad Space
; equ 0xce  Keypad @
; equ 0xcf  Keypad !
; equ 0xd0  Keypad Memory Store
; equ 0xd1  Keypad Memory Recall
; equ 0xd2  Keypad Memory Clear
; equ 0xd3  Keypad Memory Add
; equ 0xd4  Keypad Memory Subtract
; equ 0xd5  Keypad Memory Multiply
; equ 0xd6  Keypad Memory Divide
; equ 0xd7  Keypad +/-
; equ 0xd8  Keypad Clear
; equ 0xd9  Keypad Clear Entry
; equ 0xda  Keypad Binary
; equ 0xdb  Keypad Octal
; equ 0xdc  Keypad Decimal
; equ 0xdd  Keypad Hexadecimal

KEY_LEFTCTRL equ 0xe0 ; Keyboard Left Control
KEY_LEFTSHIFT equ 0xe1 ; Keyboard Left Shift
KEY_LEFTALT equ 0xe2 ; Keyboard Left Alt
KEY_LEFTMETA equ 0xe3 ; Keyboard Left GUI
KEY_RIGHTCTRL equ 0xe4 ; Keyboard Right Control
KEY_RIGHTSHIFT equ 0xe5 ; Keyboard Right Shift
KEY_RIGHTALT equ 0xe6 ; Keyboard Right Alt
KEY_RIGHTMETA equ 0xe7 ; Keyboard Right GUI

KEY_MEDIA_PLAYPAUSE equ 0xe8
KEY_MEDIA_STOPCD equ 0xe9
KEY_MEDIA_PREVIOUSSONG equ 0xea
KEY_MEDIA_NEXTSONG equ 0xeb
KEY_MEDIA_EJECTCD equ 0xec
KEY_MEDIA_VOLUMEUP equ 0xed
KEY_MEDIA_VOLUMEDOWN equ 0xee
KEY_MEDIA_MUTE equ 0xef
KEY_MEDIA_WWW equ 0xf0
KEY_MEDIA_BACK equ 0xf1
KEY_MEDIA_FORWARD equ 0xf2
KEY_MEDIA_STOP equ 0xf3
KEY_MEDIA_FIND equ 0xf4
KEY_MEDIA_SCROLLUP equ 0xf5
KEY_MEDIA_SCROLLDOWN equ 0xf6
KEY_MEDIA_EDIT equ 0xf7
KEY_MEDIA_SLEEP equ 0xf8
KEY_MEDIA_COFFEE equ 0xf9
KEY_MEDIA_REFRESH equ 0xfa
KEY_MEDIA_CALC equ 0xfb

