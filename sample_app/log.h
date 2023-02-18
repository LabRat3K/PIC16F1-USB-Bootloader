//; vim:noet:sw=8:ts=8:ai:syn=pic
#include <pic14regs.h>

// Constants
volatile extern unsigned char LOG_HEAD;
volatile extern unsigned char LOG_TAIL;
volatile extern unsigned char LOG_FMT_FLAGS;

#define FMT_EOL   0x20
#define FMT_SPACE 0x40
#define FMT_HEX   0x80

#define isFull()  ((LOG_HEAD+1)&0xFF == LOG_TAIL)
#define isEmpty()  (LOG_HEAD == LOG_TAIL)
void uart_init(void);
void log_init(void);
void log_service(void);
void log_byte(char data);
void mlog_hex(char data);
void log_hex(char data, unsigned char FMT);
void log_char(char data, unsigned char FMT);
void set_multi(unsigned char number,unsigned char FMT);
void set_format(unsigned char FMT);

