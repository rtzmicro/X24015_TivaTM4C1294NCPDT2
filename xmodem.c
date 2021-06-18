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

#include "xmodem.h"

/* Various definitions. */
#define PKT_SIZE    128
#define PKT_SIZE_1K 1024

#define NUM_TRIES   21

/* ASCII codes used in the protocol. */
#define NUL         0x00
#define SOH         0x01
#define STX         0x02
#define ETX         0x03
#define EOT         0x04
#define ACK         0x06
#define NAK         0x15
#define CAN         0x18
#define SUB         0x1a    /* final packet filler value */
#define CRC         'C'

/* XMODEM Packet Buffer */
static uint8_t      xmodem_buff[PKT_SIZE_1K];
#if USE_YMODEM
char                xmodem_name[32];
uint32_t            xmodem_size;
#endif

/******************************************************************************
 * Serial Interface Functions
 ******************************************************************************/

static void uart_putc(UART_Handle handle, uint8_t ch)
{
    UART_write(handle, &ch, 1);
    Task_sleep(100);
}

static void uart_flush(UART_Handle handle)
{
    int ch;
    while (UART_read(handle, &ch, 1) == UART_ERROR);
}

static int uart_getc(UART_Handle handle, int secs)
{
    int ch;

    while (secs)
    {
        if (UART_read(handle, &ch, 1) == 1)
            return ch;

        --secs;
    }

    return -1;
}

/* Update the CRC16 value for the given byte. */
static unsigned short
xmodem_crc(unsigned short crc, unsigned char c)
{
    register int i;

    crc = crc ^ ((unsigned short)c << 8);

    for (i = 0; i < 8; i++)
    {
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
 * Write a block of data to a file stream.
 */
static FRESULT xmodem_write_block(FIL* fp, uint8_t *buf, int32_t size)
{
    uint32_t bytesToWrite = size;
    uint32_t bytesWritten = 0;
    FRESULT res = FR_OK;

    do {

        /* Write a block of data to the destination file */
        res = f_write(fp, xmodem_buff, bytesToWrite, &bytesWritten);

        if (res != FR_OK)
            break;

        bytesToWrite -= bytesWritten;

        buf += bytesWritten;

    } while(bytesToWrite > 0);

    return res;
}
    
/****************************************************************************
 * XMODEM SUPPORT FUNCTIONS
 ***************************************************************************/

/*
 * Receive a file using the XMODEM protocol.
 *
 * XMODEM is actually a protocol that comes in many flavors,
 * with many extensions and additions.  We try to implement
 * a number of these, and switch between them automatically.
 */

int xmodem_receive(UART_Handle handle, FIL* fp)
{
    int i;
    int c;
    int b;
    int try;
    int status = -1;
    bool started = false;
    uint16_t crc_msb, crc_lsb;
    uint16_t crc;
    uint8_t blknum = 1;

    /* Send a 'C' out and look for a SOH reply */

    for (try=0; try < 10; try++)
    {
        /* Send out a C and try to read reply */
        uart_putc(handle, CRC);

        /* Receive a byte of data. */
        c = uart_getc(handle, 3);

        /* Check for user aborting */
        if ((c == EOT) || (c == CAN))
        {
            status = -2;
            break;
        }

        /* Start of header received, start reading packet */
        if (c == SOH)
        {
            status = 0;
            break;
        }
    }

    System_printf("Got SOH!\n");
    System_flush();

    if (status == 0)
    {
        for (try=0; try < 5; try++)
        {
            if (started)
            {
                /* Receive a byte of data. */
                c = uart_getc(handle, 1);

                /* Check for user aborting */
                if (c == EOT)
                {
                    System_printf("EOT!\n");
                    System_flush();

                    status = 1;
                    break;
                }

                if (c == CAN)
                {
                    System_printf("CAN!\n");
                    System_flush();
                    status = -3;
                    break;
                }

                if (c != SOH)
                {
                    System_printf("MISSING SOH!\n");
                    System_flush();

                    /* invalid compliment block number */
                    uart_flush(handle);
                    /* NAK to get sender to send again */
                    uart_putc(handle, NAK);
                    /* Loop and try to synchronize */
                    continue;
                }
            }

            started = true;

            /* Attempt to read block number */
            if ((b = uart_getc(handle, 1)) == -1)
            {
                System_printf("BLKNUM!\n");
                System_flush();

                status = -3;
                goto errout;
            }

            /* Attempt to read inverse block number */
            if ((c = uart_getc(handle, 1)) == -1)
            {
                System_printf("NBLKNUM!\n");
                System_flush();

                status = -4;
                goto errout;
            }

#if 0
            if (b == (255 - c))
            {
                System_printf("CBLKNUM!\n");
                System_flush();

                /* invalid compliment block number */
                uart_flush(handle);
                /* NAK to get sender to send again */
                uart_putc(handle, NAK);
                continue;
            }
#endif

            for (i=0; i < 128; i++)
            {
                if ((c = uart_getc(handle, 1)) == -1)
                {
                    System_printf("SHORT BLK %d!\n", i);
                    System_flush();

                    break;
                }

                xmodem_buff[i] = (uint8_t)c;
            }

            /* Did we get a whole packet? */
            if (i != 128)
            {
                System_printf("CBLKNUM %d!\n", i);
                System_flush();

                /* No, flush, NAK and start over */
                uart_flush(handle);
                /* NAK to get sender to send again */
                uart_putc(handle, NAK);
                continue;
            }

            /* Read CRC high byte word */
            if ((c = uart_getc(handle, 1)) == -1)
            {
                System_printf("MSB!\n");
                System_flush();

                /* invalid compliment block number */
                uart_flush(handle);
                /* NAK to get sender to send again */
                uart_putc(handle, NAK);
                continue;
            }

            crc_msb = (uint16_t)c & 0xFF;

            /* Read CRC low byte word */
            if ((c = uart_getc(handle, 1)) == -1)
            {
                System_printf("LSB!\n");
                System_flush();

                /* invalid compliment block number */
                uart_flush(handle);
                /* NAK to get sender to send again */
                uart_putc(handle, NAK);
                continue;
            }

            crc_lsb = (uint16_t)c & 0xFF;

            crc = (crc_msb << 8) | crc_lsb;

            uart_putc(handle, ACK);

            System_printf("ACK BLOCK %d!\n", blknum);
            System_flush();

            ++blknum;
        }
    }

errout:

    return status;
}



#if 0
int xmodem_receive(UART_Handle handle, FIL* fp)
{
    uint8_t     c, blk, n;
    uint16_t    crc, ncrc;
    uint32_t    total;
    char        mode, state, can, try, first;
    int         i, size, ptr;
#if USE_YMODEM
    char        eot = 0;
#endif

    /*
     * XMODEM starts sending the first block.
     * That said, there is an extension to the protocol
     * called TeLink, which actually starts sending the
     * first block as number 0.  This extra block contains
     * all kinds of information about the file.
     */

    n = 1;

    uart_rxflush(handle);

    /*
     * We implement the protocol as a state machine
     * handling all the various tasks.  We do need
     * to implement timeouts!
     */

    mode = XMODEM_CRC;
    try = can = first = 0;
    i = ptr = size = 0;
    crc = blk = 0;
    total = 0;
    state = 1;
    c = 0;

    while (state > 0)
    {
        /* If we need data, wait for some. */
        if (state > 1)
        {
            /* Receive a byte of data. */
            i = uart_getc(handle, 5);
            c = (unsigned char) (i & 0xff);
        }

        /* Now check what we have to do with the byte. */
        switch (state)
        {
            case 1:     /* send START byte and wait for command */
                if (++try == NUM_TRIES)
                {
                    try = 0;
                    /* no response, give up. */
                    System_printf("NO REPSONSE\n");
                    System_flush();
                    uart_putc(handle, CAN);
                    state = 0;
                }
                else
                {
                    if (mode == XMODEM_BASIC)
                        uart_putc(handle, NAK);
                      else
                        uart_putc(handle, CRC);
                    /* Enter "read command" state. */
                    state++;
                }
                first = 1;
                break;

            case 2:     /* read command byte */
                /* If we got a timeout, see what we must do. */
                if (i < 0)
                {
                    System_printf("TIMEOUT\n");
                    System_flush();

                    if (size == 0)
                    {
                        /* Still negotiating, try again. */
                        state = 1;
                        break;
                    }

                    /* Error, handle it. */
                    if (++try == 10)
                    {
                        System_printf("TOO MANY ERRORS\n");
                        System_flush();
                        uart_putc(handle, CAN);
                        uart_putc(handle, CAN);
                        uart_putc(handle, CAN);
                        state = 0;
                        break;
                    }

                    /* Send NAK and loop. */
                    uart_putc(handle, NAK);
                    break;
                }

                /* Initialize CRC or checksum. */
                crc = 0;

                /* Handle new byte. */
                switch(c)
                {
                    case SOH:   /* standard block */
                        if (first)
                        {
                            first = 0;
                            System_printf("XMODEM\n");
                            System_flush();
                        }
                        size = PKT_SIZE;
                        try = 0;
                        state++;
                        break;

                    case STX:   /* 1024-block */
                        if (first)
                        {
                            first = 0;
                            System_printf("YMODEM\n");
                            System_flush();
                        }
                        size = PKT_SIZE_1K;
                        try = 0;
                        state++;
                        break;

                    case EOT:   /* end of file */
#if USE_YMODEM
                        if (mode == XMODEM_BATCH)
                        {
                            if (eot)
                            {
                                uart_putc(ACK);
                                uart_putc(CRC);
                            }
                            else
                            {
                                uart_putc(NAK);
                            }
                            eot = (1 - eot);
                        }
                        else
                        {
#endif
                            state = 0;
                            uart_putc(handle, ACK);
#if USE_YMODEM
                        }
#endif
                        break;

                    case CAN:   /* user-initiated cancel */
                        if (can)
                        {
                            System_printf("CANCELED\n");
                            System_flush();
                            state = 0;
                            uart_putc(handle, ACK);
                            break;
                        }
                        can = (1 - can);
                        break;

                    default:    /* junk, ignore */
                        break;
                }
                break;

            case 3:     /* read block number */
                blk = c;
                state++;
                break;

            case 4:     /* read modulo-block number */
                /* The block numbers must be the same. */
                if (c != (255 - blk))
                {
                    System_printf("BLK ERR %d\n", c);
                    System_flush();
                    uart_putc(handle, NAK);
                    state = 2;
                    break;
                }

                /*
                 * YMODEM sends a "Block 0" which is really
                 * an extra block containing file info we
                 * may need later.  Just for fun, accept it
                 * and then basically proceed as usual..
                 */
#if USE_YMODEM
                if (blk != 0)
                {
#endif              /* Block must be the expected one! */
                    if (blk != n)
                    {
                        System_printf("UNEXP BLK %d\n", blk);
                        System_flush();
                        uart_putc(handle, NAK);
                        state = 2;
                        break;
                    }
#if USE_YMODEM
                }
                else
                {
                    /* Assume YMODEM mode. */
                    mode = XMODEM_BATCH;
                    n = 0;
                }
#endif
                /* Set up for receiving data. */
                ptr = 0;
                state++;
                break;

            case 5:     /* read data bytes */

                if (i < 0)
                {
                    System_printf("RX TIMEOUT\n");
                    System_flush();
                    state = 0;
                    break;
                }

                /* Store the data byte in our buffer. */
                xmodem_buff[ptr++] = c;

                /* Update CRC or checksum. */
                if (mode == XMODEM_BASIC)
                    crc += c;
                  else
                    crc = xmodem_crc(crc, c);

                /* All bytes done? */
                if (ptr == size)
                {
                    System_printf("RX TIMEOUT\n");
                    System_flush();
                    state++;
                }
                break;

            case 6:     /* read checksum */
                if (mode == XMODEM_BASIC)
                {
                    /* Checksums must match! */
                    crc &= 0xff;
                    if (crc != c)
                    {
badcrc:                 System_printf("CRC BLK %d\n", n);
                        System_flush();
                        state = 2;
                        uart_putc(handle, NAK);
                        break;
                    }

                }
                else
                {
                    /* CRC modes, read second byte! */
                    blk = c;    /* save first byte */

                    i = uart_getc(handle, 5);

                    if (i < 0)
                    {
                        System_printf("TIMEOUT\n");
                        System_flush();
                        state = 0;
                        uart_putc(handle, CAN);
                        break;
                    }

                    c = (unsigned char) (i & 0xff);

                    ncrc = (blk << 8) | c;

                    if (ncrc != crc)
                        goto badcrc;
                }

                /* Packet was OK, so write it to the file. */
                uart_putc(handle, ACK);

                /* FIXME: call the FileWrite(buff, size) func here. */
                xmodem_write_block(fp, xmodem_buff, size);

                /* Request the next block. */
                state = 2;
#if USE_YMODEM
                if (n == 0)
                {
                    /* Process the info block. */
                    if (xmodem_info((char *)xmodem_buff) == 0)
                        uart_putc(CRC);    /* accept file */
                    else
                        state = 0;          /* all files done */
                }
                else
                {
#endif
                    total += size;
#if USE_YMODEM
                }
#endif
                System_printf("received %d bytes\n", total);
                System_flush();
                n++;
                break;
        }
    }

    /* Some protocol debug info. */
#if 0
    if (xmodem_name[0] != '\0') {
        System_printf("'%s', size %lu bytes", xmodem_name, xmodem_size);
        System_flush();
    }
#endif

    return state;
}
#endif


/* End-Of-File */
