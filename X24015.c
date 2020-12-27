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
#include "AD7799.h"
#include "Utils.h"
#include "CLITask.h"
#include "usb_device.h"

/* Global System Data */
SYSCONFIG g_cfg;
SYSDATA g_sys;

/* Static Function Prototypes */
static bool Init_Peripherals(void);
static bool Init_Devices(void);
static uint32_t ADCReadChannel(AD7799_Handle handle, uint32_t channel);
int ReadGUIDS(I2C_Handle handle, uint8_t ui8SerialNumber[16], uint8_t ui8MAC[6]);;
#if (DIV_CLOCK_ENABLED > 0)
static void EnableClockDivOutput(uint32_t div);
#endif

//*****************************************************************************
// Main Entry Point
//*****************************************************************************

int main(void)
{
	Task_Params taskParams;
    //Mailbox_Params mboxParams;
    Error_Block eb;

    /* default GUID & MAC values */
    memset(&g_cfg, 0, sizeof(g_cfg));
    memset(&g_sys, 0, sizeof(g_sys));

    memset(g_sys.ui8SerialNumber, 0xFF, 16);
    memset(g_sys.ui8MAC, 0xFF, 6);
    memset(g_sys.ipAddr, 0, 32);

    ConfigInitDefaults(&g_cfg);

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
    Board_initSDSPI();
    Board_initSPI();
    Board_initUART();
    //Board_initUSB(Board_USBDEVICE);
    // Board_initWatchdog();

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
// This enables the DIVSCLK output pin on PQ4 and generates a clock signal
// from the main cpu clock divided by 'div' parameter. A value of 100 gives
// a clock of 1.2 Mhz.
//*****************************************************************************

#if (DIV_CLOCK_ENABLED > 0)
void EnableClockDivOutput(uint32_t div)
{
    /* Enable pin PQ4 for DIVSCLK0 DIVSCLK */
    GPIOPinConfigure(GPIO_PQ4_DIVSCLK);

    /* Configure the output pin for the clock output */
    GPIODirModeSet(GPIO_PORTQ_BASE, GPIO_PIN_4, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    /* Enable the clock output */

    if (!div)
        SysCtlClockOutConfig(SYSCTL_CLKOUT_DIS | SYSCTL_CLKOUT_SYSCLK, div);
    else
        SysCtlClockOutConfig(SYSCTL_CLKOUT_EN | SYSCTL_CLKOUT_SYSCLK, div);
}
#endif

//*****************************************************************************
// This function reads the unique 128-serial number and 48-bit MAC address
// via I2C from the AT24MAC402 serial EPROM.
//*****************************************************************************

int ReadGUIDS(I2C_Handle handle, uint8_t ui8SerialNumber[16], uint8_t ui8MAC[6])
{
    bool            ret;
    uint8_t         txByte;
    I2C_Transaction i2cTransaction;

    /* default is all FF's  in case read fails*/
    memset(ui8SerialNumber, 0xFF, 16);
    memset(ui8MAC, 0xFF, 6);

    /* Note the Upper bit of the word address must be set
     * in order to read the serial number. Thus 80H would
     * set the starting address to zero prior to reading
     * this sixteen bytes of serial number data.
     */

    txByte = 0x80;

    i2cTransaction.slaveAddress = AT24MAC_EPROM_EXT_ADDR;
    i2cTransaction.writeBuf     = &txByte;
    i2cTransaction.writeCount   = 1;
    i2cTransaction.readBuf      = ui8SerialNumber;
    i2cTransaction.readCount    = 16;

    ret = I2C_transfer(handle, &i2cTransaction);

    if (!ret)
    {
        System_printf("Unsuccessful I2C transfer\n");
        System_flush();
    }

    /* Now read the 6-byte 48-bit MAC at address 0x9A. The EUI-48 address
     * contains six or eight bytes. The first three bytes of the  UI read-only
     * address field are called the Organizationally Unique Identifier (OUI)
     * and the IEEE Registration Authority has assigned FCC23Dh as the Atmel OUI.
     */

    txByte = 0x9A;

    i2cTransaction.slaveAddress = AT24MAC_EPROM_EXT_ADDR;
    i2cTransaction.writeBuf     = &txByte;
    i2cTransaction.writeCount   = 1;
    i2cTransaction.readBuf      = ui8MAC;
    i2cTransaction.readCount    = 6;

    ret = I2C_transfer(handle, &i2cTransaction);

    if (!ret)
    {
        System_printf("Unsuccessful I2C transfer\n");
        System_flush();
    }

    return ret;
}

//*****************************************************************************
//
//
//*****************************************************************************

uint32_t ADCReadChannel(AD7799_Handle handle, uint32_t channel)
{
    uint32_t i;
    uint32_t data = ADC_ERROR;
    uint8_t status = 0;

    /* Select ADC Channel-1 */
    AD7799_SetChannel(handle, channel);

    /* Set the channel mode to start the single conversion */
    AD7799_SetMode(handle, AD7799_MODE_SEL(AD7799_MODE_SINGLE) | AD7799_MODE_RATE(10));

    for (i=0; i < 20; i++)
    {
        /* Check for ADC conversion complete */
        if (AD7799_IsReady(handle))
        {
            /* Read ADC channel */
            data = AD7799_ReadData(handle);

            /* Read the current ADC status and check for error */
            status = AD7799_ReadStatus(handle);

            if (status & AD7799_STAT_ERR)
                data = ADC_ERROR;

            break;
        }

        Task_sleep(10);
    }

    return data;
}

//*****************************************************************************
//
//
//*****************************************************************************

bool Init_Peripherals(void)
{
    SPI_Params  spiParams;
    I2C_Params  params;

    /* I2C-0 Bus */

    I2C_Params_init(&params);

    params.transferCallbackFxn = NULL;
    params.transferMode        = I2C_MODE_BLOCKING;
    params.bitRate             = I2C_100kHz;

    if ((g_sys.i2c0 = I2C_open(Board_I2C0, &params)) == NULL)
    {
        System_printf("Error: Unable to openI2C3 port\n");
        System_flush();
        return false;
    }

    /* I2C-1 Bus */

    I2C_Params_init(&params);

    params.transferCallbackFxn = NULL;
    params.transferMode        = I2C_MODE_BLOCKING;
    params.bitRate             = I2C_100kHz;

    if ((g_sys.i2c1 = I2C_open(Board_I2C1, &params)) == NULL)
    {
        System_printf("Error: Unable to openI2C1 port\n");
        System_flush();
        return false;
    }

    /* I2C-2 Bus */

    I2C_Params_init(&params);

    params.transferCallbackFxn = NULL;
    params.transferMode        = I2C_MODE_BLOCKING;
    params.bitRate             = I2C_100kHz;

    if ((g_sys.i2c2 = I2C_open(Board_I2C2, &params)) == NULL)
    {
        System_printf("Error: Unable to openI2C2 port\n");
        System_flush();
        return false;
    }

    /* I2C-3 Bus */

    I2C_Params_init(&params);

    params.transferCallbackFxn = NULL;
    params.transferMode        = I2C_MODE_BLOCKING;
    params.bitRate             = I2C_100kHz;

    if ((g_sys.i2c3 = I2C_open(Board_I2C3, &params)) == NULL)
    {
        System_printf("Error: Unable to openI2C3 port\n");
        System_flush();
        return false;
    }

    /* SPI-0 bus */

    SPI_Params_init(&spiParams);

    spiParams.mode            = SPI_MASTER;
    spiParams.transferMode    = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 1000;
    spiParams.frameFormat     = SPI_POL1_PHA1;
    spiParams.bitRate         = 100000;            /* 1 Mhz */
    spiParams.dataSize        = 8;

    if ((g_sys.spi0 = SPI_open(Board_SPI0, &spiParams)) == NULL)
    {
        System_printf("Error: Unable to open SPI2 port\n");
        System_flush();
        return false;
    }

    /* SPI-2 bus */

    SPI_Params_init(&spiParams);

    spiParams.mode            = SPI_MASTER;
    spiParams.transferMode    = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 1000;
    spiParams.frameFormat     = SPI_POL1_PHA1;
    spiParams.bitRate         = 100000;            /* 1 Mhz */
    spiParams.dataSize        = 8;

    if ((g_sys.spi2 = SPI_open(Board_SPI2, &spiParams)) == NULL)
    {
        System_printf("Error: Unable to open SPI2 port\n");
        System_flush();
        return false;
    }

    /* SPI-3 bus */

    SPI_Params_init(&spiParams);

    spiParams.mode            = SPI_MASTER;
    spiParams.transferMode    = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 1000;
    spiParams.frameFormat     = SPI_POL1_PHA1;
    spiParams.bitRate         = 100000;             /* 1 Mhz */
    spiParams.dataSize        = 8;

    if ((g_sys.spi3 = SPI_open(Board_SPI3, &spiParams)) == NULL)
    {
        System_printf("Error: Unable to open SPI3 port\n");
        System_flush();
        return false;
    }

    return true;
}

//*****************************************************************************
//
//
//*****************************************************************************

bool Init_Devices(void)
{
    /* This enables the DIVSCLK output pin on PQ4
     * and generates a 1.2 Mhz clock signal on the.
     * expansion header and pin 16 of the edge
     * connector if a clock signal is required.
     */
#if (DIV_CLOCK_ENABLED > 0)
    EnableClockDivOutput(100);
#endif

    /*
     * Create and initialize the MCP79410 RTC object.
     */

    if ((g_sys.handleRTC = MCP79410_create(g_sys.i2c3, NULL)) == NULL)
    {
        System_abort("MCP79410_create failed\n");
    }

    /*
     * Create and Initialize ADC #1 Channels
     */

    if ((g_sys.AD7799Handle1 = AD7799_create(g_sys.spi3, X24015_GPIO_PN0, NULL)) == NULL)
    {
        System_abort("AD7799_create failed\n");
    }

    AD7799_Reset(g_sys.AD7799Handle1);

    if ((g_sys.adcID = AD7799_Init(g_sys.AD7799Handle1)) == 0)
    {
        System_printf("AD7799_Init(1) failed\n");
    }
    else
    {
        /* Set gain to 1 */
        AD7799_SetGain(g_sys.AD7799Handle1, AD7799_GAIN_1);
        /* Set the reference detect */
        AD7799_SetRefDetect(g_sys.AD7799Handle1, AD7799_REFDET_ENA);
        /* Set for unipolar data reading */
        AD7799_SetUnipolar(g_sys.AD7799Handle1, AD7799_UNIPOLAR_ENA);
    }

    /*
     * Create and Initialize ADC #2 Channels
     */

    if ((g_sys.AD7799Handle2 = AD7799_create(g_sys.spi3, X24015_GPIO_PN1, NULL)) == NULL)
    {
        System_abort("AD7799_create failed\n");
    }

    AD7799_Reset(g_sys.AD7799Handle2);

    if ((g_sys.adcID = AD7799_Init(g_sys.AD7799Handle2)) == 0)
    {
        System_printf("AD7799_Init(2) failed\n");
    }
    else
    {
        /* Set gain to 1 */
        AD7799_SetGain(g_sys.AD7799Handle2, AD7799_GAIN_1);
        /* Set the reference detect */
        AD7799_SetRefDetect(g_sys.AD7799Handle2, AD7799_REFDET_ENA);
        /* Set for unipolar data reading */
        AD7799_SetUnipolar(g_sys.AD7799Handle2, AD7799_UNIPOLAR_ENA);
    }

    /* Enable the LED's during startup up */
    GPIO_write(Board_LED_ACT, Board_LED_ON);
    GPIO_write(Board_LED_ALM, Board_LED_OFF);

    /* Power up any slot cards listening */
    GPIO_write(Board_PWRUP_BUS_OUT, PIN_HIGH);

    /* Reset any slot cards listening */
    GPIO_write(Board_RESET_BUS_OUT, PIN_LOW);
    Task_sleep(100);
    GPIO_write(Board_RESET_BUS_OUT, PIN_HIGH);
    Task_sleep(100);

    /* Prepare to initialize EMAC layer.
     * Step 1 - Read the globally unique serial number from EPROM. We are also
     * reading the 6-byte MAC address from the AT24MAC serial EPROM.
     */
    if (!ReadGUIDS(g_sys.i2c0, g_sys.ui8SerialNumber, g_sys.ui8MAC))
    {
        System_printf("MAC & Serial# Read Failed!\n");
    }

    System_flush();

    /* Step 2 - Don't initialize EMAC layer until after reading MAC address above! */
    Board_initEMAC();

    /* Step 3 - Now allow the NDK task, blocked by NDKStackBeginHook(), to run */
    Semaphore_post(g_semaNDKStartup);

    /* Initialize the USB module for device mode */
    USB_init();

    /* Initialize command line interface on COM1 */
    CLI_init();

    return true;
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
    ConfigParamsRead(&g_cfg);

    /* Open the peripherals we plan to use */
    Init_Peripherals();

    /* Initialize any I/O cards in the slots */
    Init_Devices();

    /* Startup command line interface on COM1 */
    CLI_startup();

    /*
     * Now begin the main program command task processing loop
     */

    while (true)
    {
        if (count++ > 10)
        {
            count = 0;
            GPIO_toggle(Board_LED_ALM);
        }

        /* Read ADC level for CHAN-1 in SLOT-1 */
        g_sys.adcData[0] = ADCReadChannel(g_sys.AD7799Handle1, 0);

        /* Read ADC level for CHAN-2 in SLOT-2 */
        g_sys.adcData[1] = ADCReadChannel(g_sys.AD7799Handle1, 1);

        /* Read ADC level for CHAN-3 in SLOT-3 */
        g_sys.adcData[2] = ADCReadChannel(g_sys.AD7799Handle2, 0);

        /* Read ADC level for CHAN-4 in SLOT-4 */
        g_sys.adcData[3] = ADCReadChannel(g_sys.AD7799Handle2, 1);

        //System_printf("chan1=%x chan2=%x\n", g_sys.dacLevel[0], g_sys.dacLevel[1]);
        //System_flush();

        GPIO_toggle(Board_LED_ACT);

        Task_sleep(500);
    }
}

// End-Of-File
