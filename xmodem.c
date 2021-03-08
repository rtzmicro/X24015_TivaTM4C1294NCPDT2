/*
 * XMODEM	Simple XMODEM file transfer driver, implementing several
 *		variations of the X/MODEM protocol.  The code was written
 *		with the 10/14/88 version of Forsberg's specification in
 *		hand.  It is believed to be a correct implementation.
 *
 *		The YMODEM support code (which is used by YMODEM and the
 *		XMODEM-Batch variations of the protocol) is not always
 *		needed, so it can be disabled.
 *
 * TODO:	Test with timeouts, and see if we can automate the choice
 *		of protocol in the receiver.  As Forsberg suggests, we 
 *		can send out C's for a while, and, if that fails, switch
 *		to NAK's for the basic protocol.
 *
 * Version:	@(#)xmodem.c	1.0.1	2007/12/02
 *
 * Author:	Fred N. van Kempen, <fred.van.kempen@microwalt.nl>
 *
 *		Copyright 2007 MicroWalt Corporation.
 *		All Rights Reserved.
 *
 *		This  program  or  documentation  contains  proprietary
 *		confidential information and trade secrets of MicroWalt
 *		Corporation.  Reverse  engineering of  object  code  is
 *		prohibited.  Use of copyright  notice is  precautionary
 *		and does not imply publication.  Any  unauthorized use,
 *		reproduction  or transfer  of this program  is strictly
 *		prohibited.
 *
 *		RESTRICTED RIGHTS NOTICE
 *
 *		Use, duplication, or disclosure  by the U.S. Government
 *		is subject to restrictions as set  forth in subdivision
 *		(b)(3)(ii) of the Rights in Technical Data and Computer
 *		Software clause at 252.227-7013.
 *
 *		MicroWalt Corporation
 *		P O BOX 8
 *		1400AA, BUSSUM, NH
 *		THE NETHERLANDS
 *		PH:  +31 (35) 7503090
 *		FAX: +31 (35) 7503091
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>
#include <xdc/runtime/Memory.h>

#include <ti/sysbios/BIOS.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/mw/fatfs/ff.h>

#include <file.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <time.h>

#include <driverlib/sysctl.h>

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/hal/Seconds.h>

#include "CLITask.h"

/* Feature selection. */
#define USE_YMODEM	1		/* enable YMODEM (batch) support */


/* Supported modes. */
#define XMODEM_BASIC	1		/* standard 128-byte checksum */
#define XMODEM_CRC	    2		/* 128-byte blocks with CRC16 */
#define XMODEM_BATCH	3		/* CRC16 mode with batch info */

/* Various definitions. */
#define PKT_SIZE	128
#define PKT_SIZE_1K	1024

/* ASCII codes used in the protocol. */
#define NUL		0x00
#define SOH		0x01
#define STX		0x02
#define ETX		0x03
#define EOT		0x04
#define ACK		0x06
#define NAK		0x15
#define CAN		0x18
#define SUB		0x1a		/* final packet filler value */
#define CRC		'C'


unsigned char	xmodem_buff[PKT_SIZE_1K];
#if USE_YMODEM
char		xmodem_name[32];
unsigned long	xmodem_size;
#endif


/* Update the CRC16 value for the given byte. */
static unsigned short
xmodem_crc(unsigned short crc, unsigned char c)
{
    register int i;

    crc = crc ^ ((unsigned short)c << 8);

    for (i = 0; i < 8; i++) {
	if (crc & 0x8000)
		crc = (crc << 1) ^ 0x1021;
	  else
		crc <<= 1;
    }

    return(crc);
}


#if USE_YMODEM
/*
 * Grab useful information from the YMODEM info block.
 */
static int
xmodem_info(char *bufp)
{
    char *p = bufp;
    char *q;

    /* Check if we have a NULL file name. */
    if (*p == '\0') return(1);

    /* Grab file name. */
    q = xmodem_name;
    while (*p != '\0')
	*q++ = *p++;
    *q = '\0';
    p++;

    /* Grab file size. */
    q = p;
    while (*p != '\0') p++;
    xmodem_size = atol(q);

    return(0);
}
#endif

    
/*
 * Receive a file using the XMODEM protocol.
 *
 * XMODEM is actually a protocol that comes in many flavors,
 * with many extensions and additions.  We try to implement
 * a number of these, and switch between them automatically.
 */

int xmodem_receive(FILE* fp)
{
    unsigned char c, blk, n;
    unsigned short crc, ncrc;
    unsigned long total;
    char mode, state, can, try;
    int i, size, ptr;
#if USE_YMODEM
    char eot = 0;
#endif

    /*
     * XMODEM starts sending the first block.
     * That said, there is an extension to the protocol
     * called TeLink, which actually starts sending the
     * first block as number 0.  This extra block contains
     * all kinds of information about the file.
     */
    n = 1;

    /* Tell sender we're ready to receive. */
    CLI_printf("XMODEM Receive ready, start sending.\r\n");

    /*
     * We implement the protocol as a state machine
     * handling all the various tasks.  We do need
     * to implement timeouts!
     */
    mode = XMODEM_CRC;
    try = can = 0;
    i = ptr = size = 0;
    crc = blk = 0;
    total = 0;
    state = 1;
    c = 0;
    while (state > 0) {
	/* If we need data, wait for some. */
	if (state > 1) {
		/* Receive a byte of data. */
		i = CLI_getc();
		c = (unsigned char) (i & 0xff);
	}

	/* Now check what we have to do with the byte. */
	switch(state) {
		case 1:		/* send START byte and wait for command */
			if (++try == 5) {
				try = 0;
				mode--;
				if (mode == 0) {
					/* no response, give up. */
					CLI_printf("\r*** TIMEOUT ***\n\n");
					CLI_putc(CAN);
					state = 0;
					break;
				}
			}
			if (mode == XMODEM_BASIC)
				CLI_putc(NAK);
			  else
				CLI_putc(CRC);

			/* Enter "read command" state. */
			state++;
			break;

		case 2:		/* read command byte */
			/* If we got a timeout, see what we must do. */
			if (i < 0) {
				if (size == 0) {
					/* Still negotiating, try again. */
					state = 1;
					break;
				}

				/* Error, handle it. */
				if (++try == 10) {
					/* Too many errors, give up. */
					CLI_printf("\r*** TOO MANY ERRORS\r\n");
					CLI_putc(CAN);
					CLI_putc(CAN);
					CLI_putc(CAN);
					state = 0;
					break;
				}

				/* Send NAK and loop. */
				CLI_putc(NAK);
				break;
			}

			/* Initialize CRC or checksum. */
			crc = 0;

			/* Handle new byte. */
			switch(c) {
				case SOH:	/* standard block */
					size = PKT_SIZE;
					try = 0;
					state++;
					break;

				case STX:	/* 1024-block */
					size = PKT_SIZE_1K;
					try = 0;
					state++;
					break;

				case EOT:	/* end of file */
#if USE_YMODEM
					if (mode == XMODEM_BATCH) {
						if (eot) {
							CLI_putc(ACK);
							CLI_putc(CRC);
						} else {
							CLI_putc(NAK);
						}
						eot = (1 - eot);
					} else {
#endif
						state = 0;
						CLI_putc(ACK);
#if USE_YMODEM
					}
#endif
					break;

				case CAN:	/* user-initiated cancel */
					if (can) {
						state = 0;
						CLI_putc(ACK);
						break;
					}
					can = (1 - can);
					break;

				default:	/* junk, ignore */
					break;
			}
			break;

		case 3:		/* read block number */
			blk = c;
			state++;
			break;

		case 4:		/* read modulo-block number */
			/* The block numbers must be the same. */
			if (c != (255 - blk)) {
				CLI_printf("\r*** BLK ERR %d/%d\r\n", blk, c);
				state = 2;
				CLI_putc(NAK);
				break;
			}

			/*
			 * YMODEM sends a "Block 0" which is really
			 * an extra block containing file info we
			 * may need later.  Just for fun, accept it
			 * and then basically proceed as usual..
			 */
#if USE_YMODEM
			if (blk != 0) {
#endif
				/* Block must be the expected one! */
				if (blk != n) {
					CLI_printf("\r*** UNEXP BLK %d/%d\r\n",
								blk, n);
					state = 2;
					CLI_putc(NAK);
					break;
				}
#if USE_YMODEM
			} else {
				/* Assume YMODEM mode. */
				mode = XMODEM_BATCH;
				n = 0;
			}
#endif

			/* Set up for receiving data. */
			ptr = 0;
			state++;
			break;

		case 5:		/* read data bytes */
			/* Store the data byte in our buffer. */
			xmodem_buff[ptr++] = c;

			/* Update CRC or checksum. */
			if (mode == XMODEM_BASIC)
				crc += c;
			  else
				crc = xmodem_crc(crc, c);

			/* All bytes done? */
			if (ptr == size)
				state++;
			break;

		case 6:		/* read checksum */
			if (mode == XMODEM_BASIC) {
				/* Checksums must match! */
				crc &= 0xff;
				if (crc != c) {
badcrc:
					CLI_printf("\r*** CRC BLK %d ***\r\n", n);
					state = 2;
					CLI_putc(NAK);
					break;
				}

			} else {
				/* CRC modes, read second byte! */
				blk = c;	/* save first byte */

				i = CLI_getc();
				if (i <= 0) {
					CLI_printf("\r*** TIMEOUT ***\r\n\n");
					state = 0;
					CLI_putc(CAN);
					break;
				}
				c = (unsigned char) (i & 0xff);
				ncrc = (blk << 8) | c;
				if (ncrc != crc)
					goto badcrc;
			}

			/* Packet was OK, so write it to the file. */
			CLI_putc(ACK);

			/* FIXME: call the FileWrite(buff, size) func here. */

			/* Request the next block. */
			state = 2;
#if USE_YMODEM
			if (n == 0) {
				/* Process the info block. */
				if (xmodem_info((char *)xmodem_buff) == 0)
					CLI_putc(CRC);	/* accept file */
				  else
					state = 0;	/* all files done */
			} else {
#endif
				total += size;
#if USE_YMODEM
			}
#endif
			n++;
			break;
	}
    }

    /* Some protocol debug info. */
#if DEBUG
    CLI_printf("\r*** COMPLETE, state %d TOTAL %lu\r\n", state, total);
#endif
#if USE_YMODEM
    if (xmodem_name[0] != '\0') CLI_printf("\r*** '%s', size %lu bytes\r\n",
						xmodem_name, xmodem_size);
#endif

    return(state);
}
