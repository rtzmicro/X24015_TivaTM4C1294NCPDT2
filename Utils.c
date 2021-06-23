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
#include <ti/mw/fatfs/ff.h>

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
#include "MCP79410.h"

//*****************************************************************************
// FATFS Helpers
//*****************************************************************************

/* Return error string for given FATFS error code */

char* FS_GetErrorStr(int errnum)
{
    static char* FSErrorString[] = {
        "Success",
        "A hard error occurred",
        "Assertion failed",
        "Physical drive error",
        "Could not find the file",
        "Could not find the path",
        "The path name format is invalid",
        "Access denied due to prohibited access or directory full",
        "Access denied due to prohibited access",
        "The file/directory object is invalid",
        "The physical drive is write protected",
        "The logical drive number is invalid",
        "The volume has no work area",
        "There is no valid FAT volume",
        "The f_mkfs() aborted due to any parameter error",
        "Could not get a grant to access the volume within defined period",
        "The operation is rejected according to the file sharing policy",
        "LFN working buffer could not be allocated",
        "Too many open files",
        "Given parameter is invalid"
    };

    if (errno > sizeof(FSErrorString)/sizeof(char*))
        return "Unknown error code returned";

    return FSErrorString[errno];
}

/* Returns the current file system time from the RTC clock
 * as a string in 12-hour format.
 */

void FS_GetTimeStr(uint16_t fstime, char* buf, size_t bufsize)
{
    /* 16-Bit Time Format
     * HHHHHMMMMMMSSSSS
     *
     * bit15:11     Hour (0..23)
     * bit10:5      Minute (0..59)
     * bit4:0       Second / 2 (0..29)
     */

    uint32_t hour = (fstime >> 11) & 0x1F;

    bool pm = false;

    if (hour > 12)
    {
        hour = hour - 12;
        pm = true;
    }

    snprintf(buf, bufsize, "%02u:%02u:%02u %s",
            hour,
            (fstime >> 5) & 0x3F,
            ((fstime >> 0) & 0x1F) >> 1,
            pm ? "PM" : "AM");
}

/* Returns the file system date from the RTC clock
 * as a string in DD/MM/YYYY format.
 */

void FS_GetDateStr(uint16_t fsdate, char* buf, size_t bufsize)
{
    /* 16-Bit Date Bits Format
     * YYYYYYYMMMMDDDDD
     *
     * bit15:9      Year origin from 1980 (0..127)
     * bit8:5       Month (1..12)
     * bit4:0       Day (1..31)
     */

    snprintf(buf, bufsize, "%02u/%02u/%04u",
           (fsdate >> 5) & 0x0F,
           (fsdate >> 0) & 0x1F,
           ((fsdate >> 9) & 0x7F) + 1980);
}

/* This function to return the time is hooked
 * into the FATFS library in the CFG file
 */

uint32_t FS_GetFatTime(void)
{
    uint32_t ftime = 0;
    RTCC_Struct ts;

    /* Current local time shall be returned as bit-fields packed into
     * a DWORD value. The bit fields are as follows:
     *
     *  bit31:25    Year origin from the 1980 (0..127, e.g. 37 for 2017)
     *  bit24:21    Month (1..12)
     *  bit20:16    Day of the month (1..31)
     *  bit15:11    Hour (0..23)
     *  bit10:5     Minute (0..59)
     *  bit4:0      Second / 2 (0..29, e.g. 25 for 50)
     *
     * The get_fattime function shall return any valid time even if the
     * system does not support a real time clock. If a zero is returned,
     * the file will not have a valid timestamp.
     */

    if (!RTC_GetDateTime(&ts))
    {
        ts.sec     = 0;
        ts.min     = 0;
        ts.hour    = 0;
        ts.weekday = 1;
        ts.date    = 1;
        ts.month   = 1;
        ts.year    = 20;
    }

    uint32_t year;

    /* seconds 5-bits (divided by 2) */
    ftime |= ((uint32_t)ts.sec >> 1) & 0x1F;

    /* minutues 6-bits */
    ftime |= ((uint32_t)ts.min & 0x3F) << 5;

    /* hour 5-bits */
    ftime |= ((uint32_t)ts.hour & 0x1F) << 11;

    /* day 5-bits */
    ftime |= ((uint32_t)(ts.date + 1) & 0x1F) << 16;

    /* month 4-bits */
    ftime |= ((uint32_t)(ts.month + 1) & 0x0F) << 21;

    /* MCP7940 base year is 2000, and FAT base year is 1980 */
    year = ts.year + (2000 - 1980);

    /* year 7-bits */
    ftime |= (year & 0x7F) << 25;

    return ftime;
}

//*****************************************************************************
// RTC Interface Functions
//*****************************************************************************

bool RTC_IsRunning(void)
{
    if (!MCP79410_IsRunning(g_sys.handleRTC))
        return false;

    return true;
}

bool RTC_GetDateTime(RTCC_Struct* ts)
{
    /* Default values if read fails */
    ts->sec     = 0;
    ts->min     = 0;
    ts->hour    = 0;
    ts->weekday = 1;
    ts->date    = 1;
    ts->month   = 1;
    ts->year    = 0;

    if (!g_sys.handleRTC)
        return false;

    if (!MCP79410_IsRunning(g_sys.handleRTC))
        return false;

    MCP79410_GetTime(g_sys.handleRTC, ts);

    return true;
}

bool RTC_SetDateTime(RTCC_Struct* ts)
{
    if (!g_sys.handleRTC)
        return false;

    /* Set hour format to military time standard */
    MCP79410_SetHourFormat(g_sys.handleRTC, H24);
    /* Enable battery backup */
    MCP79410_EnableVbat(g_sys.handleRTC);
    /* Set the new time/date  values */
    MCP79410_SetTime(g_sys.handleRTC, ts);
    /* Make sure clock is running by enabling oscillator */
    MCP79410_EnableOscillator(g_sys.handleRTC);

    return true;
}

void RTC_GetTimeStr(RTCC_Struct* ts, char* timestr)
{
    snprintf(timestr, 9, "%d:%02d:%02d", ts->hour, ts->min, ts->sec);
}

void RTC_GetDateStr(RTCC_Struct* ts, char* datestr)
{
    snprintf(datestr, 11, "%d/%d/%d", ts->month+1, ts->date+1, ts->year+2000);
}

bool RTC_IsValidTime(RTCC_Struct* ts)
{
    if ((ts->sec > 59) || (ts->min > 59) || (ts->hour > 23))
        return false;

    return true;
}

bool RTC_IsValidDate(RTCC_Struct* ts)
{
    if ((ts->date > 31) || (ts->month > 11) || (ts->year > 199))
        return false;

    return true;
}

//*****************************************************************************
// Set default runtime values
//*****************************************************************************

void ConfigInitDefaults(SYSCONFIG* p)
{
    /** Default servo parameters **/
    p->version = MAKEREV(FIRMWARE_VER, FIRMWARE_REV);
    p->build   = FIRMWARE_BUILD;
    p->length  = sizeof(SYSCONFIG);
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
