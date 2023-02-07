; vim:noet:sw=8:ts=8:ai:syn=pic
; asmsyntax=pic
;
; File:   wyse_keymap.c
;
; 2023-01-24 A. Williams - pulled keymap into it's own 
; file and location in the ROM, so that it could
; be flashed independent of the code base
;
;
; ------------------------------------------------------
; 80 byte keymap table
;KEYMAP:
;   dt 0x2b,0x08,0x1a,0x14,0x15,0x17,0x1c,KEY_NONE
;   dt 0x34,0x28,0x49,0x07,0x2c,0x16,KEY_NONE,0x0c
;   dt 0x25,0x26,0x27,0x2d,0x2e,KEY_NONE,0x04,0x2a
;   dt 0x19,0x05,0x11,0x10,KEY_NONE,0x1e,0x09,0x18
;   dt 0xe5,KEY_NONE,0x29,0x00,0x36,0x1f,0x0a,0x12
;   dt 0xe6,0xe5,KEY_NONE,0x48,0x37,0x20,0x0b,0x13
;   dt 0x74,KEY_NONE,0xe0,0x00,0x38,0x21,0x0d,0x2f
;   dt KEY_NONE,0x51,0x50,0x39,0x1d,0x22,0x0e,0x30
;   dt 0x52,KEY_NONE,0x00,0x31,0x1b,0x23,0x0f,0x35
;   dt 0x4f,KEY_NONE,0x00,0x46,0x06,0x24,0x33,0x2a 

 include "hid_codes.inc"
 org 0x1D00

KEYMAP2
	brw	; Add W to PC
   retlw KEY_TAB		;0
	retlw KEY_E
	retlw KEY_W
	retlw KEY_Q
	retlw KEY_R
	retlw KEY_U
	retlw KEY_Y
	retlw KEY_NONE
   retlw KEY_APOSTROPHE		 ;8
	retlw KEY_ENTER
	retlw KEY_INSERT
	retlw KEY_D
	retlw KEY_SPACE
	retlw KEY_S
	retlw KEY_NONE
	retlw KEY_I
   retlw KEY_8			;16
	retlw KEY_9
	retlw KEY_0
	retlw KEY_MINUS
	retlw KEY_EQUAL
	retlw KEY_NONE
	retlw KEY_A
	retlw KEY_BACKSPACE
   retlw KEY_V			;24
	retlw KEY_B
	retlw KEY_N
	retlw KEY_M
	retlw KEY_NONE
	retlw KEY_1
	retlw KEY_F
	retlw KEY_U
   retlw KEY_RIGHTSHIFT		;32
	retlw KEY_NONE
	retlw KEY_ESC
	retlw KEY_NONE
	retlw KEY_COMMA
	retlw KEY_2
	retlw KEY_G
	retlw KEY_O
   retlw KEY_RIGHTALT		;40
	retlw KEY_LEFTSHIFT
	retlw KEY_NONE
	retlw KEY_PAUSE
	retlw KEY_DOT
	retlw KEY_3
	retlw KEY_H
	retlw KEY_P
   retlw KEY_OPEN		;48
	retlw KEY_NONE
	retlw KEY_LEFTCTRL
	retlw KEY_NONE
	retlw KEY_SLASH
	retlw KEY_4
	retlw KEY_J
	retlw KEY_LEFTBRACE
   retlw KEY_NONE		;56
	retlw KEY_DOWN
	retlw KEY_LEFT
	retlw KEY_CAPSLOCK
	retlw KEY_Z
	retlw KEY_5
	retlw KEY_K
	retlw KEY_RIGHTBRACE
   retlw KEY_UP		;64
	retlw KEY_NONE
	retlw KEY_NONE
	retlw KEY_BACKSLASH
	retlw KEY_X
	retlw KEY_6
	retlw KEY_L
	retlw KEY_GRAVE
   retlw KEY_RIGHT	;72
	retlw KEY_NONE
	retlw KEY_NONE
	retlw KEY_SYSRQ
	retlw KEY_C
	retlw KEY_7
	retlw KEY_SEMICOLON
	retlw KEY_BACKSPACE ;7F


