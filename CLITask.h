/***************************************************************************
 *
 * XMOD Tiva TM4C1294 Processor Card
 *
 * Copyright (C) 2021, RTZ Microsystems, LLC
 * All Rights Reserved
 *
 ***************************************************************************/

#ifndef __CLITASK_H
#define __CLITASK_H

/*** CONSTANTS AND CONFIGURATION *******************************************/

/* VT100 Escape Sequences */
#define VT100_HOME          "\x01b[1;1;H"       /* home cursor */
#define VT100_CLS           "\x01b[2J"          /* clear screen, home cursor */
#define VT100_BELL          "\x07"              /* bell */
#define VT100_POS           "\x01b[%d;%d;H"     /* pstn cursor (row, col) */
#define VT100_UL_ON         "\x01b#7\x01b[4m"   /* underline on */
#define VT100_UL_OFF        "\x01b[0m"          /* underline off */
#define VT100_INV_ON        "\x01b#7\x01b[7m"   /* inverse on */
#define VT100_INV_OFF       "\x01b[0m"          /* inverse off */
#define VT100_ERASE_EOL     "\x01b[K"           /* erase to end of line */
#define VT100_ERASE_SOL     "\x01b[1K"          /* erase to start of line */
#define VT100_ERASE_LINE    "\x01b[2K"          /* erase entire line */

/* ASCII Codes */
#define SOH     0x01
#define STX     0x02
#define ENQ     0x05
#define ACK     0x06
#define BELL    0x07
#define BKSPC   0x08
#define LF      0x0A
#define CRET    0x0D
#define NAK     0x15
#define SYN     0x16
#define ESC     0x1B
#define CTL_X   0x18
#define CTL_Z   0x1A

/*** FUNCTION PROTOTYPES ***************************************************/

int CLI_init(void);
Bool CLI_startup(void);
int CLI_getc(void);
void CLI_putc(int ch);
void CLI_puts(char* s);
void CLI_crlf(int n);
void CLI_emit(char c, int n);
void CLI_printf(const char *fmt, ...);
void CLI_prompt(void);
void CLI_about(void);
void CLI_home(void);
Void CLITaskFxn(UArg arg0, UArg arg1);

#endif /* __REMOTETASK_H */
