/*
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
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
//#include <ti/sysbios/fatfs/ff.h>
#include <inc/hw_ints.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* USB Driver files */
#include <usblib/usblib.h>
#include <usblib/usb-ids.h>
#include <usblib/device/usbdevice.h>
#include <usblib/device/usbdbulk.h>

/* NDK BSD support */
#include <sys/socket.h>

#include <driverlib/sysctl.h>

#include <file.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include "Board.h"
#include "X24015.h"
#include "Utils.h"
#include "CLITask.h"
#include "usb_device.h"

// Global System Data
SYSCONFIG g_sysConfig;
SYSDATA g_sysData;

//*****************************************************************************
// Main Entry Point
//*****************************************************************************

int main(void)
{
	Task_Params taskParams;
    //Mailbox_Params mboxParams;
    Error_Block eb;

    /* default GUID & MAC values */
    memset(&g_sysConfig, 0, sizeof(g_sysConfig));
    memset(&g_sysData, 0, sizeof(g_sysData));

    memset(g_sysData.ui8SerialNumber, 0xFF, 16);
    memset(g_sysData.ui8MAC, 0xFF, 6);
    memset(g_sysData.ipAddr, 0, 32);

    ConfigInitDefaults(&g_sysConfig);

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
    Board_initSDSPI();
    Board_initSPI();
    Board_initUART();
    //Board_initUSB(Board_USBDEVICE);
    // Board_initWatchdog();

    /* This enables the DIVSCLK output pin on PQ4
     * and generates a 1.2 Mhz clock signal on the.
     * expansion header and pin 16 of the edge
     * connector if a clock signal is required.
     */
#if (DIV_CLOCK_ENABLED > 0)
    EnableClockDivOutput(100);
#endif

    /* Create task with priority 15 */
    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.stackSize = 2048;
    taskParams.priority  = 15;
    Task_create((Task_FuncPtr)MainTaskFxn, &taskParams, &eb);

    System_printf("Starting X24015 execution.\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

//*****************************************************************************
//
//
//*****************************************************************************

Void MainTaskFxn(UArg arg0, UArg arg1)
{
    int count = 0;

    //Error_Block eb;
	//Task_Params taskParams;

    /* Read any system configuration parameters from EPROM. If config
     * hasn't been initialized, then initialize it with defaults.
     */
    ConfigParamsRead(&g_sysConfig);

    /* Enable the LED's during startup up */
    GPIO_write(Board_LED_ACT, Board_LED_ON);
    GPIO_write(Board_LED_ALM, Board_LED_OFF);

    /* Power up any slot cards listening */
    GPIO_write(Board_PWRUP_BUS_OUT, PIN_HIGH);

    /* Reset any slot cards listening */
    GPIO_write(Board_RESET_BUS_OUT, PIN_LOW);
    Task_sleep(100);
    GPIO_write(Board_RESET_BUS_OUT, PIN_HIGH);

    /* Step 1 - Read the globally unique serial number from EPROM. We are also
     * reading the 6-byte MAC address from the AT24MAC serial EPROM.
     */
    if (!ReadGUIDS(g_sysData.ui8SerialNumber, g_sysData.ui8MAC))
    {
        System_printf("Read Serial Number Failed!\n");
        System_flush();
    }

    /* Step 2 - Don't initialize EMAC layer until after reading MAC address above! */
    Board_initEMAC();

    /* Step 3 - Now allow the NDK task, blocked by NDKStackBeginHook(), to run */
    Semaphore_post(g_semaNDKStartup);

    /* Initialize the USB module for device mode */
    //USB_init();

    /* Initialize and startup command line interface on COM1 */
    CLI_init();
    CLI_startup();

    /* Now begin the main program command task processing loop */

    while (true)
    {
        if (count++ > 10)
        {
            count = 0;
            GPIO_toggle(Board_LED_ALM);
        }
   		GPIO_toggle(Board_LED_ACT);
        Task_sleep(1000);
    }
}

// End-Of-File
