/* ============================================================================
 *
 * Copyright (C) RTZ Microsystems, LLC
 * All Rights Reserved
 *
 * ============================================================================
 *
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== tcpEcho.c ========
 *    Contains BSD sockets code.
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* NDK BSD support */
#include <sys/socket.h>

#include <file.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

/* Tivaware Driver files */
#include <inc/hw_memmap.h>
#include <inc/hw_gpio.h>
#include <driverlib/eeprom.h>
#include <driverlib/flash.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/fpu.h>

/* X24015 Board Header file */

#include "X24015.h"
#include "Board.h"
#include "Utils.h"

//*****************************************************************************
// Set default runtime values
//*****************************************************************************

void ConfigInitDefaults(SYSCONFIG* p)
{
    /** Default servo parameters **/
    p->version      = MAKEREV(FIRMWARE_VER, FIRMWARE_REV);
    p->build        = FIRMWARE_BUILD;
    p->debug        = 0;                    /* debug mode 0=off             */

}

//*****************************************************************************
// Write system parameters from our global settings buffer to EEPROM.
//
// Returns:  0 = Success
//          -1 = Error writing EEPROM data
//*****************************************************************************

int ConfigParamsWrite(SYSCONFIG* sp)
{
    int32_t rc = 0;

    /* Initialize the version, build# and magic# */
    sp->version = MAKEREV(FIRMWARE_VER, FIRMWARE_REV);
    sp->build   = FIRMWARE_BUILD;
    sp->magic   = MAGIC;

    /* Store the configuration parameters to EPROM */
    rc = EEPROMProgram((uint32_t *)sp, 0, sizeof(SYSCONFIG));

    System_printf("Writing System Parameters %d\n", rc);
    System_flush();

    return rc;
 }

//*****************************************************************************
// Read system parameters into our global settings buffer from EEPROM.
//
// Returns:  0 = Sucess
//          -1 = Error reading flash
//
//*****************************************************************************

int ConfigParamsRead(SYSCONFIG* sp)
{
    ConfigInitDefaults(sp);

    /* Read the configuration parameters from EPROM */
    EEPROMRead((uint32_t *)sp, 0, sizeof(SYSCONFIG));

    /* Does the magic number match? If not, set defaults and
     * store to initialize the system default parameters.
     */
    if (sp->magic != MAGIC)
    {
        System_printf("ERROR Reading System Parameters - Using Defaults...\n");
        System_flush();

        ConfigInitDefaults(sp);

        ConfigParamsWrite(sp);

        return -1;
    }

    /* If firmware is different version, the reset system defaults
     * and store as system default parameters.
     */
    if (sp->version != MAKEREV(FIRMWARE_VER, FIRMWARE_REV))
    {
        System_printf("WARNING New Firmware Version - Using Defaults...\n");
        System_flush();

        ConfigInitDefaults(sp);

        ConfigParamsWrite(sp);

        return -1;
    }

    /* If stored build number is less that minimum build number required,
     * then reset and store system defaults. This is to avoid loading old
     * configuration parameters store from an earlier build version.
     */
    if (sp->build < FIRMWARE_MIN_BUILD)
    {
        System_printf("WARNING New Firmware BUILD - Resetting Defaults...\n");
        System_flush();

        ConfigInitDefaults(sp);

        ConfigParamsWrite(sp);

        return -1;
    }

    return 0;
}

//*****************************************************************************
// Helper Functions
//*****************************************************************************

int GetHexStr(char* textbuf, uint8_t* databuf, int datalen)
{
    char fmt[8];
    uint32_t i;
    int32_t l;

    const uint32_t wordSize = 4;

    *textbuf = 0;
    strcpy(fmt, "%02X");

    for (i=0; i < datalen; i++)
    {
        l = sprintf(textbuf, fmt, *databuf++);
        textbuf += l;

        if (((i % wordSize) == (wordSize-1)) && (i != (datalen-1)))
        {
            l = sprintf(textbuf, "-");
            textbuf += l;
        }
    }

    return strlen(textbuf);
}

//*****************************************************************************
//
//*****************************************************************************

#if 0
void BurnMACAddress(void)
{
    uint32_t ulUser0, ulUser1;

    /* Get the MAC address */
    FlashUserGet(&ulUser0, &ulUser1);

    /* WARNING - THIS IS A ONE TIME FUSE SET OPERATION! */

    if ((ulUser0 == 0xffffffff) && (ulUser1 == 0xffffffff))
    {
        /* Combine MAC address into two 32-bit words */
        ulUser0 = ((((uint32_t)g_sysData.ui8MAC[0] & 0xff) << 0)) |
                  ((((uint32_t)g_sysData.ui8MAC[1] & 0xff) << 8)) |
                  ((((uint32_t)g_sysData.ui8MAC[2] & 0xff) << 16));

        ulUser1 = ((((uint32_t)g_sysData.ui8MAC[3] & 0xff) << 0)) |
                  ((((uint32_t)g_sysData.ui8MAC[4] & 0xff) << 8)) |
                  ((((uint32_t)g_sysData.ui8MAC[5] & 0xff) << 16));

        System_printf("Updating MAC address in user flash!\n");
        System_flush();

        /* Save the two MAC address words into the special user
         * flash area. There are four words available, but we only
         * need the first two words to store the six byte MAC address.
         */
        if (!FlashUserSet(ulUser0, ulUser1))
        {
            System_printf("FlashUserSet failed updating MAC address!\n");
            System_flush();
        }

        /* NOTE - THIS IS A ONE TIME PERMANENT WRITE OPERATION!!! */
        if (!FlashUserSave())
        {
            System_printf("FlashUserSave failed updating MAC address!\n");
            System_flush();
        }

        System_printf("MAC ADDRESS PERMANENTLY WRITTEN TO USER FLASH!\n");
        System_flush();

        /* REBOOT BY JUMPING TO BOOTLOADER! */
        SysCtlReset();
    }
}
#endif

// End-Of-File
