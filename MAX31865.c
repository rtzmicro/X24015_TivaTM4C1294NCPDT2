/* This software is based on the original works of author Edwin Koch.
 * The adaptation here has been heavily modified and adapted for use
 * with TI-RTOS by Robert E Starr, Jr. and RTZ Microsystems, LLC.
 */

/*
MIT License
Copyright (c) 2019 Edwin Koch
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/* TI-RTOS Kernel Header files */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* TI-RTOS Kernel Header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Driver Header files */
#include <ti/drivers/SPI.h>
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/GPIO.h>

#include "Board.h"
#include "MAX31865.h"

/* Default MAX31865DB parameters structure */
const MAX31865_Params MAX31865_defaultParams = {
    .charge_time_delay     = MAX31865_CHARGE_TIME,
    .conversion_time_delay = MAX31865_CONVERSION_TIME,
    .rtd                   = 100,
    .rref                  = 400,
    .lowFaultThreshold     = 0,
    .highFaultThreshold    = 0xFFFF,
    .configReg             = 0,
    .chipselect            = 0,
    .chipselect_proc       = NULL,
    .chipselect_param1     = NULL,
    .chipselect_param2     = NULL
};

/*** Static Function Prototypes ***/

static void MAX31865_destruct(MAX31865_Handle handle);

/* Low level register write */
static bool MAX31865_write(MAX31865_Handle handle,
                           uint8_t regaddr,
                           void* databuf,
                           size_t datalen);

/* Low level register read */
static bool MAX31865_read(MAX31865_Handle handle,
                          uint8_t regaddr,
                          void* databuf,
                          size_t datalen);

static void HandleChipSelect(MAX31865_Handle handle, bool assert);

/******************************************************************************
 * MAX31865_Params_init
 ******************************************************************************/

void MAX31865_Params_init(MAX31865_Params *params)
{
    Assert_isTrue(params != NULL, NULL);

    *params = MAX31865_defaultParams;
}

/******************************************************************************
 * MAX31865_construct
 ******************************************************************************/
 
MAX31865_Handle MAX31865_construct(
        MAX31865_Object *obj,
        SPI_Handle spiHandle,
        MAX31865_Params *params)
{
    /* Initialize object SPI handle */
    obj->spiHandle             = spiHandle;
    /* Initialize chip select callback  nd parameters */
    obj->chipselect            = params->chipselect;
    obj->chipselect_proc       = params->chipselect_proc;
    obj->chipselect_param1     = params->chipselect_param1;
    obj->chipselect_param2     = params->chipselect_param2;
    /* Initialize configuration parameters */
    obj->charge_time_delay     = params->charge_time_delay;
    obj->conversion_time_delay = params->conversion_time_delay;
    obj->rtd                   = params->rtd;
    obj->rref                  = params->rref;
    obj->lowFaultThreshold     = params->lowFaultThreshold;
    obj->highFaultThreshold    = params->highFaultThreshold;
    obj->configReg             = params->configReg;

#if (MAX31865_THREAD_SAFE > 0)
    GateMutex_construct(&(obj->gate), NULL);
#endif
    return (MAX31865_Handle)obj;
}

/******************************************************************************
 * MAX31865_destruct
 ******************************************************************************/

void MAX31865_destruct(MAX31865_Handle handle)
{
    Assert_isTrue((handle != NULL), NULL);

#if (MAX31865_THREAD_SAFE > 0)
    GateMutex_destruct(&(handle->gate));
#endif
}

/******************************************************************************
 * MAX31865_create
 ******************************************************************************/
 
MAX31865_Handle MAX31865_create(
        SPI_Handle spiHandle,
        MAX31865_Params *params)
{
    MAX31865_Handle handle;
    Error_Block eb;

    Error_init(&eb);

    handle = Memory_alloc(NULL, sizeof(MAX31865_Object), NULL, &eb);

    if (handle)
        handle = MAX31865_construct(handle, spiHandle, params);

    return handle;
}

/******************************************************************************
 * MAX31865_delete
 ******************************************************************************/

void MAX31865_delete(MAX31865_Handle handle)
{
    MAX31865_destruct(handle);

    Memory_free(NULL, handle, sizeof(MAX31865_Object));
}

/******************************************************************************
 * Initializes the MAX31865 and checks if the device is present.
 ******************************************************************************/

bool MAX31865_init(MAX31865_Handle handle)
{
    bool success = false;

    struct {
        uint16_t highFault;
        uint16_t lowFault;
    } thresholds;

    thresholds.highFault = handle->highFaultThreshold << 1;
    thresholds.lowFault  = handle->lowFaultThreshold << 1;

    /* Detect if the chip is there or not */
    if (MAX31865_probe(handle))
    {
        uint8_t config = handle->configReg;

        /* Found, setup the configuration register */
        success = MAX31865_write(handle,
                                 MAX31865_REG_CONFIG | MAX31865_WRITE,
                                 &config, 1);
        if (success)
        {
            /* Write the high and low fault threshold register values */
            success = MAX31865_write(handle,
                                     MAX31865_REG_HI_FAULT_MSB | MAX31865_WRITE,
                                     &thresholds, 4);
        }
    }

    return success;
}

/*****************************************************************************
 * Probe the command register to detect for presence of MAX31865 chip.
 *****************************************************************************/

bool MAX31865_probe(MAX31865_Handle handle)
{
    bool success;
    uint8_t config = 0;
    uint8_t regval = 0;

    /* We initialize a known value, then attempt to read it back */
    config = MAX31865_CFG_FAULT_CLR(1) | MAX31865_CFG_VBIAS(1);

    /* Write the configuration register */
    success = MAX31865_write(handle,
                             MAX31865_REG_CONFIG | MAX31865_WRITE,
                             &config, 1);
    if (success)
    {
        /* Attempt to read the config register results back */
        success = MAX31865_read(handle,
                                MAX31865_REG_CONFIG,
                                &regval, 1);
        if (success)
        {
            /* The chip should reset the fault clear bit, so we
             * only look for the vbias bit to still being set.
             * If so, assume the MAX31865 chip is there and alive.
             */
            if (regval != MAX31865_CFG_VBIAS(1))
                success = false;
        }
    }

    /* Done probing, reset the fault bit again just to be safe
     * and reset all other configuration flags bits to zero.
     */

    config = MAX31865_CFG_FAULT_CLR(1);

    MAX31865_write(handle,
                   MAX31865_REG_CONFIG | MAX31865_WRITE,
                   &config, 1);

    return success;
}

/*****************************************************************************
 * This is the default chip select handler called prior to any read or
 * write operations. The application can override this if needed during
 * initialization by setting a new handler in the MAX31865_Params structure.
 *****************************************************************************/

void HandleChipSelect(MAX31865_Handle handle, bool assert)
{
    /* Has the user assigned chip select handler to this object
     * already? If so, then we just need to call the users
     * handler and pass any arguments.
     */
    if (handle->chipselect_proc)
    {
        handle->chipselect_proc(assert,
                                handle->chipselect_param1,
                                handle->chipselect_param2);
    }
    else
    {
        /* No user assigned callback handler has been set, so we
         * just set the chip select pin to the state specified.
         */
        GPIO_write(handle->chipselect, (assert) ? PIN_LOW : PIN_HIGH);
    }
}

/*****************************************************************************
 * Write a register to MAX31865
 *****************************************************************************/

bool MAX31865_write(
        MAX31865_Handle handle,
        uint8_t regaddr,
        void*   databuf,
        size_t  datalen
        )
{
    bool success;
    uint8_t txBuffer[8];
    uint8_t rxBuffer[8];
    SPI_Transaction transaction;

    Assert_isTrue((datalen <= 4), NULL);

    memcpy(&txBuffer[1], databuf, datalen);

    /* Set the register address */
    txBuffer[0] = regaddr;

    /* Initialize opcode transaction structure */
    transaction.count = datalen + 1;
    transaction.txBuf = (Ptr)&txBuffer;
    transaction.rxBuf = (Ptr)&rxBuffer;

    /* Assert the chip select */
    HandleChipSelect(handle, TRUE);

    /* Initiate SPI transfer of opcode */
    success = SPI_transfer(handle->spiHandle, &transaction);

    /* Release the chip select */
    HandleChipSelect(handle, FALSE);

    return success;
}

/*****************************************************************************
 * Read a register from MAX31865
 *****************************************************************************/

bool MAX31865_read(
        MAX31865_Handle handle,
        uint8_t regaddr,
        void*   databuf,
        size_t  datalen
        )
{
    bool success;
    uint8_t txBuffer[8];
    uint8_t rxBuffer[8];
    SPI_Transaction transaction;

    Assert_isTrue((datalen <= 4), NULL);

    memset(txBuffer, 0xFF, sizeof(txBuffer));

    /* Set register address */
    txBuffer[0] = regaddr;

    /* Initialize opcode transaction structure */
    transaction.count = datalen + 1;
    transaction.txBuf = (Ptr)&txBuffer;
    transaction.rxBuf = (Ptr)&rxBuffer;

    /* Assert the chip select */
    HandleChipSelect(handle, TRUE);

    /* Initiate SPI transfer of opcode */
    success = SPI_transfer(handle->spiHandle, &transaction);

    /* Release the chip select */
    HandleChipSelect(handle, FALSE);

    /* Return the register data bytes */
    memcpy(databuf, &rxBuffer[1], datalen);

    return success;
}

/******************************************************************************
 * Read the ADC value of the RTD as 15-bit and check for threshold errors.
 * The least significant bit D0 indicates error if set.
 ******************************************************************************/

uint8_t MAX31865_readADC(MAX31865_Handle handle, uint16_t* data)
{
    uint8_t  reg8;
    uint8_t  buf[2] = { 0, 0 };
    uint16_t adc;
    uint8_t  status = MAX31865_ERR_SUCCESS;

#if (MAX31865_THREAD_SAFE > 0)
    IArg key = GateMutex_enter(GateMutex_handle(&(handle->gate)));
#endif

    /*
     * Step-1: Turn on vbias prior to read and allow settling time.
     */

    reg8 = handle->configReg | MAX31865_CFG_VBIAS(1);

    MAX31865_write(handle, MAX31865_REG_CONFIG | MAX31865_WRITE, &reg8, 1);

    Task_sleep(handle->charge_time_delay);

    /*
     * Step-2: Start a one-shot conversion with vbias enabled.
     */

    reg8 = handle->configReg | MAX31865_CFG_1SHOT(1) | MAX31865_CFG_VBIAS(1);

    MAX31865_write(handle, MAX31865_REG_CONFIG | MAX31865_WRITE, &reg8, 1);

    Task_sleep(handle->conversion_time_delay);

    /*
     * Step-3: Read the RTD 16-bit word with D0 being the error flag bit. Note the
     *         actual ADC value is a 15-bit result and D0 is an error flag bit.
     *         D1 is the LSB, so we shift the lower byte down a bit.
     */

    MAX31865_read(handle, MAX31865_REG_RTD_MSB, &buf, 2);

    //adc = (uint16_t)((buf[0] << 8) | (buf[1] >> 1));

    adc = (uint16_t)(((buf[0] << 8) | buf[1]) >> 1);

    /*
     * Step-4: Now turn vbias back off to save power.
     */

    reg8 = handle->configReg;

    MAX31865_write(handle, MAX31865_REG_CONFIG | MAX31865_WRITE, &reg8, 1);

    /* Check for fault */
    if (buf[1] & 0x01)
    {
        uint8_t fault;

        fault = MAX31865_readFault(handle);

        switch(fault)
        {
        case MAX31865_ERR_RTD_HIGH_THRESHOLD:
        case MAX31865_ERR_RTD_LOW_THRESHOLD:
            status = fault;
            break;

        default:
            status = MAX31865_ERR_UNDEFINED;
            break;
        }

        MAX31865_clearFault(handle);
    }

#if (MAX31865_THREAD_SAFE > 0)
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);
#endif

    /* Return the ADC read results */
    *data = adc;

    return status;
}

/******************************************************************************
 *
 ******************************************************************************/

float MAX31865_ADC_to_Celcius(MAX31865_Handle handle, uint16_t adc)
{
    float celcius;

    /* temperature curve polynomial approximation coefficients */
    static const float _a1 = 2.55865721669;
    static const float _a2 = 0.000967360412;
    static const float _a3 = 0.000000731467;
    static const float _a4 = 0.000000000691;
    static const float _a5 = 7.31888555389e-13;

    /* First calculate the RTD resistance in ohms from the ADC value */
    float ohms = ((float)adc * (float)(handle->rref)) / 32768.0f;

    /* Celsius temperature value is calculated using Horners method
     * as this reduces the multiplications and additions needed.
     */
    float x = (float)handle->rtd - ohms;

    celcius = -(x * (_a1 + x * (_a2 + x * (_a3 + x * (_a4 + x * _a5)))));

    return celcius;
}

/******************************************************************************
 *
 ******************************************************************************/

uint8_t MAX31865_readFault(MAX31865_Handle handle)
{
    uint8_t fault;

#if (MAX31865_THREAD_SAFE > 0)
    IArg key = GateMutex_enter(GateMutex_handle(&(handle->gate)));
#endif

    MAX31865_read(handle, MAX31865_REG_FAULT_STATUS, &fault, 1);

#if (MAX31865_THREAD_SAFE > 0)
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);
#endif

    return fault;
}

/******************************************************************************
 *
 ******************************************************************************/

void MAX31865_clearFault(MAX31865_Handle handle)
{
#if (MAX31865_THREAD_SAFE > 0)
    IArg key = GateMutex_enter(GateMutex_handle(&(handle->gate)));
#endif

    uint8_t config = handle->configReg | MAX31865_CFG_FAULT_CLR(1);

    MAX31865_write(handle, MAX31865_REG_CONFIG, &config, 1);

#if (MAX31865_THREAD_SAFE > 0)
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);
#endif
}

/******************************************************************************
 *
 ******************************************************************************/

void MAX31865_setHighFaultThreshold(MAX31865_Handle handle, uint16_t threshold)
{
    uint8_t buf[2];

#if (MAX31865_THREAD_SAFE > 0)
    IArg key = GateMutex_enter(GateMutex_handle(&(handle->gate)));
#endif

    handle->highFaultThreshold = threshold;

    threshold = threshold << 1;

    buf[0] = (uint8_t)(threshold >> 8);
    buf[1] = (uint8_t)(threshold);

    MAX31865_write(handle, MAX31865_REG_HI_FAULT_MSB | MAX31865_WRITE, buf, 2);

#if (MAX31865_THREAD_SAFE > 0)
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);
#endif
}

/******************************************************************************
 *
 ******************************************************************************/

void MAX31865_setLowFaultThreshold(MAX31865_Handle handle, uint16_t threshold)
{
    uint8_t buf[2];

#if (MAX31865_THREAD_SAFE > 0)
    IArg key = GateMutex_enter(GateMutex_handle(&(handle->gate)));
#endif

    handle->lowFaultThreshold = threshold;

    buf[0] = (uint8_t)(threshold >> 8);
    buf[1] = (uint8_t)(threshold);

    MAX31865_write(handle, MAX31865_REG_LO_FAULT_MSB | MAX31865_WRITE, &buf, 2);

#if (MAX31865_THREAD_SAFE > 0)
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);
#endif
}

/******************************************************************************
 *
 ******************************************************************************/

int MAX31865_IsThresholdFault(MAX31865_Handle handle)
{
    int status = 0;
    uint8_t reg8;

#if (MAX31865_THREAD_SAFE > 0)
    IArg key = GateMutex_enter(GateMutex_handle(&(handle->gate)));
#endif

    MAX31865_read(handle, MAX31865_REG_FAULT_STATUS, &reg8, 1);

    if (reg8 & MAX31865_ERR_RTD_LOW_THRESHOLD)
        status = 1;

    if (reg8 & MAX31865_ERR_RTD_HIGH_THRESHOLD)
        status = -1;

#if (MAX31865_THREAD_SAFE > 0)
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);
#endif

    return status;
}

/* End-Of-File */
