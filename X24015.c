/* ============================================================================
 *
 * XMOD Data Capture and Telemetry Systems
 *
 * Copyright (C) 2021, RTZ Microsystems, LLC
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
 * ============================================================================ */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Assert.h>
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

#include <ti/mw/fatfs/ff.h>

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
#include <locale.h>

#include "Board.h"
#include "X24015.h"
#include "Utils.h"
#include "CLITask.h"
#include "usb_device.h"

//*****************************************************************************
// Global System and Config Data
//*****************************************************************************

SYSCONFIG   g_cfg;      /* Global configuration data from EEPROM */
SYSDATA     g_sys;      /* Global system variables and data storage */

//*****************************************************************************
// Global ADC Card Data
//*****************************************************************************

/* This table contains the handle to each ADC channel allocated in
 * the system along with the chip select SPI handle to the device.
 */
static ADC_CARD g_adcCard[ADC_MAX_CARDS] = {
    /* CARD #1, channels 01-04 chip select PN0, PL2 */
    {{ NULL, X24015_GPIO_PN0, NULL, X24015_GPIO_PL2 }},
    /* CARD #2, channels 05-08 chip select PN1, PL3 */
    {{ NULL, X24015_GPIO_PN1, NULL, X24015_GPIO_PL3 }},
    /* CARD #3, channels 09-12 chip select PN2, PL4 */
    {{ NULL, X24015_GPIO_PN2, NULL, X24015_GPIO_PL4 }},
    /* CARD #4, channels 13-16 chip select PN3, PL5 */
    {{ NULL, X24015_GPIO_PN3, NULL, X24015_GPIO_PL5 }},
};

//*****************************************************************************
// Global RTD Card Data
//*****************************************************************************

/* This table contains the handle to each RTD card/channels allocated in
 * the system along with the various chip selects to access these.
 */
static RTD_CARD g_rtdCard[RTD_MAX_CARDS] = {
    /* RTD Card #1 */
    {{ NULL, 0x01, NULL, 0x02, NULL, 0x04, NULL, 0x08 }, NULL, 0, X24015_GPIO_PM0 },
    /* RTD Card #2 */
    {{ NULL, 0x01, NULL, 0x02, NULL, 0x04, NULL, 0x08 }, NULL, 0, X24015_GPIO_PM1 },
    /* RTD Card #3 */
    {{ NULL, 0x01, NULL, 0x02, NULL, 0x04, NULL, 0x08 }, NULL, 0, X24015_GPIO_PM2 },
    /* RTD Card #4 */
    {{ NULL, 0x01, NULL, 0x02, NULL, 0x04, NULL, 0x08 }, NULL, 0, X24015_GPIO_PM3 },
};

//*****************************************************************************
// Static Function Prototypes
//*****************************************************************************

static bool Init_Peripherals(void);
static bool Init_Devices(void);

static uint32_t RTD_AllocCards(void);
static uint32_t RTD_ReadChannel(uint32_t channel);
static void MAX31865_ChipSelect_Proc(void* param1, void* param2, bool assert);

static uint32_t ADC_AllocCards(void);
static uint32_t ADC_ReadChannel(uint32_t channel);
static float ADC_to_UVPower(uint32_t adc);

static int ReadGUIDS(I2C_Handle handle, uint8_t ui8SerialNumber[16], uint8_t ui8MAC[6]);

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

    setlocale(LC_ALL,"");

    /* default GUID & MAC values */
    memset(&g_cfg, 0, sizeof(g_cfg));
    memset(&g_sys, 0, sizeof(g_sys));

    memset(g_sys.ui8SerialNumber, 0xFF, 16);
    memset(g_sys.ui8MAC, 0xFF, 6);
    memset(g_sys.ipAddr, 0, 32);

    g_sys.lastError = 0;

    ConfigInitDefaults(&g_cfg);

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
    Board_initSPI();
    Board_initSDSPI();
    Board_initUART();
    Board_initUSB(Board_USBDEVICE);
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
// Global error number interface functions.
//*****************************************************************************

uint32_t GetLastError(void)
{
    return g_sys.lastError;
}

void SetLastError(uint32_t error)
{
    g_sys.lastError = error;
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
// This opens all the I2C and SPI port drivers. These handles are passed
// to other driver objects later that need access to these drivers.
//*****************************************************************************

bool Init_Peripherals(void)
{
    SPI_Params  spiParams;
    SDSPI_Params sdParams;
    I2C_Params  i2cParams;

    /* I2C-0 Bus */

    I2C_Params_init(&i2cParams);

    i2cParams.transferCallbackFxn = NULL;
    i2cParams.transferMode        = I2C_MODE_BLOCKING;
    i2cParams.bitRate             = I2C_100kHz;

    if ((g_sys.i2c0 = I2C_open(Board_I2C0, &i2cParams)) == NULL)
    {
        System_abort("Error: Unable to openI2C3 port\n");
    }

    /* I2C-1 Bus */

    I2C_Params_init(&i2cParams);

    i2cParams.transferCallbackFxn = NULL;
    i2cParams.transferMode        = I2C_MODE_BLOCKING;
    i2cParams.bitRate             = I2C_100kHz;

    if ((g_sys.i2c1 = I2C_open(Board_I2C1, &i2cParams)) == NULL)
    {
        System_abort("Error: Unable to openI2C1 port\n");
    }

    /* I2C-2 Bus */

    I2C_Params_init(&i2cParams);

    i2cParams.transferCallbackFxn = NULL;
    i2cParams.transferMode        = I2C_MODE_BLOCKING;
    i2cParams.bitRate             = I2C_100kHz;

    if ((g_sys.i2c2 = I2C_open(Board_I2C2, &i2cParams)) == NULL)
    {
        System_abort("Error: Unable to openI2C2 port\n");
    }

    /* I2C-3 Bus */

    I2C_Params_init(&i2cParams);

    i2cParams.transferCallbackFxn = NULL;
    i2cParams.transferMode        = I2C_MODE_BLOCKING;
    i2cParams.bitRate             = I2C_100kHz;

    if ((g_sys.i2c3 = I2C_open(Board_I2C3, &i2cParams)) == NULL)
    {
        System_abort("Error: Unable to openI2C3 port\n");
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
        System_abort("Error: Unable to open SPI2 port\n");
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
        System_abort("Error: Unable to open SPI2 port\n");
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
        System_abort("Error: Unable to open SPI3 port\n");
    }

    /* SD SPI bus */

    SDSPI_Params_init(&sdParams);

    sdParams.bitRate = 400000;

    if ((g_sys.spiSD = SDSPI_open(X24015_SDSPI0, SD_DRIVE_NUM, &sdParams)) == NULL)
    {
        System_abort("Error: Unable to open SD SPI port\n");
    }

    return true;
}

//*****************************************************************************
// This function allocates and initializes all the hardware devices in the
// system at startup. We also fire up the Ethernet, USB and other devices.
//*****************************************************************************

bool Init_Devices(void)
{
    /* Enable the LED's during startup up */
    GPIO_write(Board_LED_ACT, Board_LED_ON);
    GPIO_write(Board_LED_ALM, Board_LED_OFF);

    /* Power up any slot cards listening */
    GPIO_write(Board_PWRUP_BUS_OUT, PIN_HIGH);
    Task_sleep(100);

    /* Reset any slot cards listening */
    GPIO_write(Board_RESET_BUS_OUT, PIN_LOW);
    Task_sleep(100);
    GPIO_write(Board_RESET_BUS_OUT, PIN_HIGH);
    Task_sleep(100);

    /* This enables the DIVSCLK output pin on PQ4 and generates a 1.2 Mhz clock
     * signal on the bus if any board needs a clock signal for some reason.
     */
#if (DIV_CLOCK_ENABLED > 0)
    EnableClockDivOutput(100);
#endif

    /* Read the globally unique serial number from EPROM. We are also
     * reading the 6-byte MAC address from the AT24MAC serial EPROM.
     */
    if (!ReadGUIDS(g_sys.i2c0, g_sys.ui8SerialNumber, g_sys.ui8MAC))
    {
        System_printf("MAC & Serial# Read Failed!\n");
        System_flush();

        SetLastError(XSYSERR_GUID_SERMAC);
    }

    /* Create and initialize the MCP79410 RTC object. This
     * is our battery backed system time/date clock.
     */
    g_sys.handleRTC = MCP79410_create(g_sys.i2c3, NULL);

    /* Allocate and probe for any ADC UV-C sensor cards */
    g_sys.adcNumChannels = ADC_AllocCards();

    /* Allocate and probe for any RTD temp sensor cards */
    g_sys.rtdNumChannels = RTD_AllocCards();

    System_printf("24035 ADC channels: %d\n", g_sys.adcNumChannels);
    System_printf("24037 RTD channels: %d\n", g_sys.rtdNumChannels);
    System_flush();

    /* Now initialize the EMAC layer with the MAC address from EPROM */
    Board_initEMAC(g_sys.ui8MAC);

    /* Allow the NDK task, blocked by NDKStackBeginHook(), to run */
    Semaphore_post(g_semaNDKStartup);

    /* Initialize the USB module for device mode */
    USB_init();

    /* Initialize command line interface on COM1 */
    CLI_init();

    return true;
}

//*****************************************************************************
// Create and Initialize ADC Channels. Each card contains two ADC must
// be predefined in the channel table. All of the ADC converters are
// are mapped on SPI-3 with chip selects for each.
//*****************************************************************************

uint32_t ADC_AllocCards(void)
{
    size_t i, n;
    uint8_t adcID;
    uint32_t channels = 0;
    AD7799_Handle handle;

    for (n=0; n < ADC_MAX_CARDS; n++)
    {
        ADC_CARD *card = &g_adcCard[n];

        for (i=0; i < ADC_CONVERTERS_PER_CARD; i++)
        {
            /* Allocate an ADC object connected to SPI-3 for the 24035 ADC
             * cards. These cards are always on SPI-3 with two GPIO lines
             * directly mapped to chip selects for each ADC on each card.
             */
            handle = AD7799_create(g_sys.spi3, card->converter[i].gpiocs, NULL);

            card->converter[i].handle = handle;

            Assert_isTrue((handle != NULL), NULL);

            /* Attempt to reset any ADC chip present */
            AD7799_Reset(handle);

            /* Attempt to probe for the ADC and initialize it */
            if ((adcID = AD7799_Init(handle)) != 0)
            {
                /* Found an ADC, save it's type */
                g_sys.adcID = adcID;

                /* Set gain to 1 */
                AD7799_SetGain(handle, AD7799_GAIN_1);

                /* Set the reference detect */
                AD7799_SetRefDetect(handle, AD7799_REFDET_ENA);

                /* Set for unipolar data reading */
                AD7799_SetUnipolar(handle, AD7799_UNIPOLAR_ENA);

                channels += 2;
            }
        }
    }

    return channels;
}

//*****************************************************************************
// This function takes and ADC value and converts it to UV-C level in
// milliwatts per centimeter squared (mW/cm2)
//*****************************************************************************

float ADC_to_UVPower(uint32_t adc)
{
    float power;

    if (adc < 0xFF)
        adc = 0;

    power = (float)adc / 6323.07f;

    return power;
}

//*****************************************************************************
//
//
//*****************************************************************************

uint32_t ADC_ReadChannel(uint32_t channel)
{
    uint32_t i;
    uint32_t rc = 0;
    uint32_t adc = 0;
    uint8_t status = 0;

    size_t card_index = channel / ADC_CHANNELS_PER_CARD;

    if (card_index >= ADC_MAX_CARDS)
        return ADC_ERROR;

    /* Get a pointer to the card info table */
    ADC_CARD *card = &g_adcCard[card_index];

    /* There are two ADC's per card and each has two channels, so we must
     * select sub-channel 0 or 1 also. The board has sub-channel 1 on
     * the top connector and sub-channel 0 on the second connector.
     */
    uint32_t converter = ((channel % 4) <= 1) ? 0 : 1;

    /* Get handle to ADC converter 0 or 1 on the card */
    AD7799_Handle handle = card->converter[converter].handle;

    if (!handle)
        return ADC_ERROR;

    size_t n = channel % 4;

    /* Determine the sub-channel within the ADC itself */
    uint32_t subchan = ((n == 0) || (n == 2)) ? 1 : 0;

    /* Select ADC channel to 0 or 1 */
    AD7799_SetChannel(handle, subchan);

    /* Set the channel mode to start the single conversion */
    AD7799_SetMode(handle, AD7799_MODE_SEL(AD7799_MODE_SINGLE) | AD7799_MODE_RATE(10));

    /* Poll waiting for the conversion to complete */

    for (i=0; i < 20; i++)
    {
        /* Check for ADC conversion complete */
        if (AD7799_IsReady(handle))
        {
            /* Read ADC channel */
            adc = AD7799_ReadData(handle);

            /* Get current ADC status and check for error */
            status = AD7799_ReadStatus(handle);

            //if (status & AD7799_STAT_ERR)
            //    res = ADC_ERROR;

            {
                g_sys.uvcADC[channel] = adc;
                g_sys.uvcPower[channel] = ADC_to_UVPower(adc);
            }

            (void)status;
            break;
        }

        /* Sleep 10ms and try again */
        Task_sleep(10);
    }

    return rc;
}

//*****************************************************************************
// This function allocates all the RTD context objects for communication
// and initializes the RTD converters for use.
//*****************************************************************************

uint32_t RTD_AllocCards(void)
{
    uint8_t configReg;
    size_t i, n;
    uint32_t channels = 0;

    for (n=0; n < RTD_NUM_CARDS; n++)
    {
        RTD_CARD *card = &g_rtdCard[n];

        /*
         * Create the I/O expander object for this card
         */

        /* Initialize the I/O expander device object parameters */
        MCP23S17_Params paramsIOX;
        MCP23S17_Params_init(&paramsIOX);

        /* Chip select to be used for this card's I/O expander */
        paramsIOX.gpioCSIndex = card->chipselIOX;

        /* Create the I/O expander object on SPI-0 for this card */
        card->handleIOX = MCP23S17_create(g_sys.spi0, &paramsIOX);

        Assert_isTrue((card->handleIOX != NULL), NULL);

        /* Read the RTD card DIP switch settings */
        uint8_t dipsw = 0;
        MCP23S17_read(card->handleIOX, MCP_GPIOB, &dipsw);

        /* Upper 4-bits are dip switch, lower 4-bits are DRDY */
        card->dipSwitch = dipsw >> 4;

        /* If DIP switch 1 is set, then configure for 3-wire mode,
         * otherwise 2 or 4 wire is assumed.
         */
#if 0
        if (card->dipSwitch & 0x01)
            configReg = MAX31865_CFG_3WIRE_RTD(1);
        else
            configReg = 0;
#endif

        /*
         * Create and initialize four RTD channel objects for this card
         */

        uint8_t swbit = 1;

        for (i=0; i < RTD_CHANNELS_PER_CARD; i++)
        {
            RTD_CHANNEL *channel = &card->channels[i];

            /* Initialize the RTD device object parameters */
            MAX31865_Params params;
            MAX31865_Params_init(&params);

            /* Each card has a DIP switch with four positions that indicate
             * if a channel is configured for 3 or 2/4 wire mode. If the DIP
             * switch is enabled, the jumper for 3-wire mode must be changed
             * also to allow for 3-wire operation on any single channel. This
             * scheme allows us to support a mix of 2, 3 or 4 wire modes on
             * each channel.
             */

            if ((card->dipSwitch & swbit) == swbit)
            {
                configReg = MAX31865_CFG_3WIRE_RTD(1);

                System_printf("RTD Card %d configured for 3-WIRE mode\n");
                System_flush();
            }
            else
            {
                configReg = 0;

                System_printf("RTD Card %d configured for 2/4-WIRE mode\n");
                System_flush();
            }

            params.charge_time_delay     = MAX31865_CHARGE_TIME;
            params.conversion_time_delay = MAX31865_CONVERSION_TIME;
            params.rtd                   = 100;
            params.rref                  = 400;
            params.lowFaultThreshold     = 0;
            params.highFaultThreshold    = 0xFFFF;
            params.configReg             = configReg;
            params.chipselect            = channel->csMaskIOX;
            params.chipselect_param1     = card;
            params.chipselect_param2     = channel;
            params.chipselect_proc       = MAX31865_ChipSelect_Proc;

            /* Create the I/O expander object on SPI-2 for this card */
            channel->handleRTD = MAX31865_create(g_sys.spi2, &params);

            Assert_isTrue((channel->handleRTD != NULL), NULL);

            /* Attempt to initialize the card */
            if (MAX31865_init(channel->handleRTD))
            {
                channels += 1;
            }

            /* Shift bit mask to the next DIP switch position we test next */
            swbit <<= 1;
        }
    }

    return channels;
}

/* This function gets called for register read/write operations to a
 * RTD card and sets the chip select output from the MCP23S17 to the
 * RTD devices. Only one chip select can be active at any given time and
 * each RTD card has four MAX31865 RTD converters. Note the MCP23S17 is
 * configured so the logic signals are inverted, thus setting an output
 * register pin high drives the chip select low from the MCP23S17 i/o
 * expander chip to assert the chip select.
 */

void MAX31865_ChipSelect_Proc(void* param1, void* param2, bool assert)
{
    RTD_CARD *card = (RTD_CARD*)param1;

    RTD_CHANNEL *channel = (RTD_CHANNEL*)param2;

    uint8_t mask = 0xFF;

    if (assert)
        mask ^= channel->csMaskIOX;

    /* Write to port-a on the i/o expander */
    MCP23S17_write(card->handleIOX, MCP_GPIOA, mask);
}

//*****************************************************************************
//
//
//*****************************************************************************

uint32_t RTD_ReadChannel(uint32_t channel)
{
    uint8_t status;
    uint16_t adc;
    uint32_t rc = RTD_ERROR;
    float tempC;

    size_t card_index = channel / RTD_CHANNELS_PER_CARD;

    if (card_index >= RTD_CHANNELS_PER_CARD)
        return rc;

    RTD_CARD *card = &g_rtdCard[card_index];

    MAX31865_Handle handle = card->channels[channel % 4].handleRTD;

    if (handle)
    {
        /* Read the raw ADC value from the RTD */
        status = MAX31865_readADC(handle, &adc);

        if (status == MAX31865_ERR_SUCCESS)
        {
            /* Convert ADC to Celcius and store in global channel data table */
            tempC =  MAX31865_ADC_to_Celcius(handle, adc);

            g_sys.rtdADC[channel] = adc;
            g_sys.rtdTempC[channel] = tempC;

            rc = 0;
        }
    }

    return rc;
}

//*****************************************************************************
// Main task processing loop.
//*****************************************************************************

Void MainTaskFxn(UArg arg0, UArg arg1)
{
    size_t i;

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
        /* Turn on ALM LED if system error detected */
        GPIO_write(Board_LED_ALM, (GetLastError() != XSYSERR_SUCCESS) ? PIN_HIGH : PIN_LOW);

        /* If any ADC channels were found, then read the ADC for the
         * channel and store the results in global data buffer.
         */

        for (i=0; i < g_sys.adcNumChannels; i++)
        {
            /* Read the ADC value and check for any error. If no value convert
             * the ADC value to UV level in mW/cm2 and store in the data table.
             */
            if (ADC_ReadChannel(i) != ADC_ERROR)
            {
                //System_printf("UV-C[%d] %f\n", i, g_sys.uvcPower[i]);
                //System_flush();

                GPIO_toggle(Board_LED_ACT);
            }
        }

        /* If any RTD channels were found, then read the ADC for the
         * channel and store the results in global data buffer.
         */

        for (i=0; i < g_sys.rtdNumChannels; i++)
        {
            /* Read the RTD ADC value and check for any error. If no value convert
             * the ADC value to Celcius temperature value in the data table.
             */
            if (RTD_ReadChannel(i) != RTD_ERROR)
            {
                System_printf("Temp[%d] %f\n", i, CELCIUS_TO_FAHRENHEIT(g_sys.rtdTempC[i]));
                System_flush();

                GPIO_toggle(Board_LED_ACT);
            }
        }

        Task_sleep(100);
    }
}


// End-Of-File
