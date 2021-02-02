/***************************************************************************
 *
 * XMOD Tiva TM4C1294 Processor Card
 *
 * Copyright (C) 2020, RTZ Microsystems, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Microsystems, LLC
 *
 ***************************************************************************
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
 *
 ***************************************************************************/

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

#include "X24015.h"
#include "Utils.h"
#include "CLITask.h"
#include "Board.h"
#include "MCP79410.h"

//*****************************************************************************
// Type Definitions
//*****************************************************************************

typedef struct {
    const char* name;
    void (*func)(int, char**);
    const char* doc;
} cmd_t;

#define MK_CMD(x) void cmd_ ## x (int, char**)

//*****************************************************************************
// CLI Function Handle Declarations
//*****************************************************************************

MK_CMD(cls);
MK_CMD(help);
MK_CMD(about);
MK_CMD(ipaddr);
MK_CMD(macaddr);
MK_CMD(sernum);
MK_CMD(time);
MK_CMD(dir);

/* The dispatch table */
#define CMD(func, help) {#func, cmd_ ## func, help}

cmd_t dispatch[] = {
    CMD(cls, "Clear the screen"),
    CMD(help, "Display this help"),
    CMD(about, "About the system"),
    CMD(ipaddr, "Displays IP address"),
    CMD(macaddr, "Displays MAC address"),
    CMD(sernum, "Display serial number"),
    CMD(time, "set current time"),
    CMD(dir, "list directory"),
};

#define NUM_CMDS    (sizeof(dispatch)/sizeof(cmd_t))

//*****************************************************************************
// Static and External Data Items
//*****************************************************************************

#define MAX_CHARS       80

#define MAX_ARGS        8
#define MAX_ARG_LEN     16


/*** Static Data Items ***/
static UART_Handle s_handleUart;
static const char *s_delim = " ://\n";
static char s_cmdbuf[MAX_CHARS+3];
static char s_cmdprev[MAX_CHARS+3];

static int   s_argc = 0;
static char* s_argv[MAX_ARGS];
static char  s_args[MAX_ARGS][MAX_ARG_LEN];

/*** Function Prototypes ***/
static int parse_args(char *buf);
static void parse_cmd(char *buf);

static char *FSErrorString(int errno);

/*** External Data Items ***/
extern SYSDATA g_sys;
extern SYSCONFIG g_cfg;

//*****************************************************************************
//
//*****************************************************************************

int CLI_init(void)
{
    UART_Params uartParams;

    UART_Params_init(&uartParams);

    uartParams.readMode       = UART_MODE_BLOCKING;
    uartParams.writeMode      = UART_MODE_BLOCKING;
    uartParams.readTimeout    = 1000;                   // 1 second read timeout
    uartParams.writeTimeout   = BIOS_WAIT_FOREVER;
    uartParams.readCallback   = NULL;
    uartParams.writeCallback  = NULL;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.writeDataMode  = UART_DATA_TEXT;
    uartParams.readDataMode   = UART_DATA_BINARY;
    uartParams.readEcho       = UART_ECHO_OFF;
    uartParams.baudRate       = 115200;
    uartParams.stopBits       = UART_STOP_ONE;
    uartParams.parityType     = UART_PAR_NONE;

    s_handleUart = UART_open(Board_COM1, &uartParams);

    if (s_handleUart == NULL)
        System_abort("Error initializing UART\n");

    return 1;
}

//*****************************************************************************
//
//*****************************************************************************

Bool CLI_startup(void)
{
    Error_Block eb;
    Task_Params taskParams;

    Error_init(&eb);

    Task_Params_init(&taskParams);

    taskParams.stackSize = 2048;
    taskParams.priority  = 2;
    taskParams.arg0      = 0;
    taskParams.arg1      = 0;

    Task_create((Task_FuncPtr)CLITaskFxn, &taskParams, &eb);

    return TRUE;
}

//*****************************************************************************
//
//*****************************************************************************

void CLI_putc(int ch)
{
    UART_write(s_handleUart, &ch, 1);
}

void CLI_puts(char* s)
{
    int l = strlen(s);
    UART_write(s_handleUart, s, l);
}

void CLI_printf(const char *fmt, ...)
{
    va_list arg;
    static char buf[128];
    va_start(arg, fmt);
    vsnprintf(buf, sizeof(buf)-1, fmt, arg);
    va_end(arg);
    UART_write(s_handleUart, buf, strlen(buf));
}

void CLI_prompt(void)
{
    CLI_putc(CRET);
    CLI_putc(LF);
    CLI_putc('>');
    CLI_putc(' ');
}

//*****************************************************************************
// Return File System Error String
//*****************************************************************************

char *FSErrorString(int errno)
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
        return "???";

    return FSErrorString[errno];
}

//*****************************************************************************
//
//*****************************************************************************

Void CLITaskFxn(UArg arg0, UArg arg1)
{
    uint8_t ch;
    int cnt = 0;

    CLI_printf(VT100_HOME);
    CLI_printf(VT100_CLS);

    CLI_printf("X24015 v%d.%02d.%03d\n\n", FIRMWARE_VER, FIRMWARE_REV, FIRMWARE_BUILD);
    CLI_puts("Enter 'help' to view a list valid commands\n\n> ");

    while (true)
    {
        /* Read a character from the console */
        if (UART_read(s_handleUart, &ch, 1) == 1)
        {
            if (ch == CRET)
            {
                if (cnt)
                {
                    CLI_putc(CRET);
                    CLI_putc(LF);

                    /* save command for previous recall */
                    strcpy(s_cmdprev, s_cmdbuf);

                    /* parse new command and execute */
                    parse_cmd(s_cmdbuf);

                    /* reset the command buffer */
                    s_cmdbuf[0] = 0;
                    cnt = 0;
                }
                CLI_prompt();
            }
            else if (ch == BKSPC)
            {
                if (cnt)
                {
                    s_cmdbuf[--cnt] = 0;

                    CLI_putc(BKSPC);
                    CLI_putc(' ');
                    CLI_putc(BKSPC);
                }
            }
            else if (ch == CTL_Z)
            {
                /* restore previous command */
                strcpy(s_cmdbuf, s_cmdprev);
                cnt = strlen(s_cmdbuf);
                CLI_printf("%s", s_cmdbuf);
            }
            else
            {
                if (cnt < MAX_CHARS)
                {
                    if (isalnum((int)ch) || strchr(s_delim, (int)ch) || (ch == '.'))
                    {
                        s_cmdbuf[cnt++] = tolower(ch);
                        s_cmdbuf[cnt] = 0;
                        CLI_putc((int)ch);
                    }
                }
            }
        }
    }
}

//*****************************************************************************
//
//*****************************************************************************

int parse_args(char *buf)
{
    int argc = 0;

    const char* tok = strtok(NULL, s_delim);

    if (!tok)
        return 0;

    while (tok != NULL)
    {
        s_argv[argc] = strncpy(s_args[argc], tok, MAX_ARG_LEN-1);

        if (++argc >= MAX_ARGS)
            break;

        tok = strtok(NULL, s_delim);
    }

    return argc;
}

void parse_cmd(char *buf)
{
    char* tok = strtok(buf, s_delim);

    if (!tok)
        return;

    /* parse args into array */
    s_argc = parse_args(tok);

    int i = NUM_CMDS;

    while(i--)
    {
        cmd_t cur = dispatch[i];

        if (!strncmp(tok, cur.name, strlen(tok)))
        {
            cur.func(s_argc, s_argv);
            return;
        }
    }

    CLI_puts("Command Not Found\n");
}

//*****************************************************************************
// CLI Command Handlers
//*****************************************************************************

void cmd_help(int argc, char *argv[])
{
    int i = NUM_CMDS;

    CLI_puts("\nAvailable Commands:\n\n");

    while(i--)
    {
        cmd_t cmd = dispatch[i];

        CLI_printf("%10s\t %s\n", cmd.name, cmd.doc);
    }
}

void cmd_about(int argc, char *argv[])
{
    CLI_printf("X24015 v%d.%02d.%03d\n", FIRMWARE_VER, FIRMWARE_REV, FIRMWARE_BUILD);
    CLI_puts("Copyright (C) 2020, RTZ Microsystems, LLC.\n");
}

void cmd_cls(int argc, char *argv[])
{
    CLI_puts(VT100_CLS);
    CLI_puts(VT100_HOME);
}

void cmd_sernum(int argc, char *argv[])
{
    char serialnum[64];
    /*  Format the 64 bit GUID as a string */
    GetHexStr(serialnum, g_sys.ui8SerialNumber, 16);
    CLI_printf("%s\n", serialnum);
}

void cmd_ipaddr(int argc, char *argv[])
{
    CLI_printf("%s\n", g_sys.ipAddr);
}

void cmd_macaddr(int argc, char *argv[])
{
    char mac[32];

    sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X",
            g_sys.ui8MAC[0], g_sys.ui8MAC[1], g_sys.ui8MAC[2],
            g_sys.ui8MAC[3], g_sys.ui8MAC[4], g_sys.ui8MAC[5]);

    CLI_printf("%s\n", mac);
}

void cmd_time(int argc, char *argv[])
{
    char str[32];
    RTCC_Struct ts;

    str[0] = '\0';

    if (argc == 0)
    {
        if (!MCP79410_IsRunning(g_sys.handleRTC))
        {
            CLI_printf("clock not running\n");
        }
        else
        {
            MCP79410_GetTime(g_sys.handleRTC, &ts);

            sprintf(str, "%d:%d:%d %d/%d/%d",
                    ts.hour, ts.min, ts.sec,
                    ts.month, ts.weekday,
                    (ts.year - 1900) + 2000);

            CLI_printf("%s\n", str);
        }
    }
    else if (argc == 7)
    {
        int year = atoi(argv[6]);

        if (strcmp(argv[0], "set") == 0)
        {
            ts.hour    = (uint8_t)atoi(argv[1]);
            ts.min     = (uint8_t)atoi(argv[2]);
            ts.sec     = (uint8_t)atoi(argv[3]);

            ts.month   = (uint8_t)atoi(argv[4]);;
            ts.date    = (uint8_t)atoi(argv[5]);
            ts.weekday = (uint8_t)((ts.date % 7) + 1);
            ts.year    = (uint8_t)((year + 1900) - 2000);

            MCP79410_SetHourFormat(g_sys.handleRTC, H24);                // Set hour format to military time standard
            MCP79410_EnableVbat(g_sys.handleRTC);                        // Enable battery backup
            MCP79410_SetTime(g_sys.handleRTC, &ts);
            MCP79410_EnableOscillator(g_sys.handleRTC);                  // Start clock by enabling oscillator

            CLI_printf("time and date set!\n", str);
        }
        else
        {
            CLI_printf("invalid arguments\n");
        }
    }
    else
    {
        CLI_printf("invalid time parameters\n");
    }
}

void cmd_dir(int argc, char *argv[])
{
    //int files = 0;
    FIL file;
    FRESULT res;
    //static DIR dir;
    //static FILINFO fno; /* File information */


    if (g_sys.i2c2 == NULL)
    {
        CLI_printf("no SD drive open\n");
        return;
    }
    else
    {
        if ((res = f_open(&file, "bootld.bin", FA_READ)) != FR_OK)
        {
            CLI_printf("ERROR(%d): %s\n", res, FSErrorString(res));
        }
        else
        {

            f_close(&file);
        }


#if 0
        if ((res = f_opendir(&dir, ".")) != FR_OK)
        {
            CLI_printf("dir empty\n");
            return;
        }
        else
        {
            res = f_findfirst(&dir, &fno, "", "*.*");

            /* Loop while an item is found */

            while (res == FR_OK && fno.fname[0])
            {
                /* Print the object name */
                CLI_printf("%s\n", fno.fname);

                /* Search for next item */
                res = f_findnext(&dir, &fno);
            }

            f_closedir(&dir);
        }
#endif
    }

    //CLI_printf("%s\n");
}

// End-Of-File
