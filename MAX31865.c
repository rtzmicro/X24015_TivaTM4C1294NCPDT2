
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
    .chipselect_proc       = NULL
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
    /* Initialize object members */
    obj->spiHandle             = spiHandle;
    /* Initialize from default parameters */
    obj->chipselect            = params->chipselect;
    obj->chipselect_proc       = params->chipselect_proc;
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

    if (handle == NULL)
        return (NULL);

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
 * @brief Initializes the MAX31865 and checks if the device is present.
 *
 * @param None.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
 ******************************************************************************/

bool MAX31865_init(MAX31865_Handle handle)
{
    bool success;

    struct {
        uint16_t highFault;
        uint16_t lowFault;
    } thresholds;

    thresholds.highFault = handle->highFaultThreshold << 1;
    thresholds.lowFault  = handle->lowFaultThreshold << 1;

    uint8_t config = handle->configReg;

    /* Write the configuration register */
    success = MAX31865_write(handle,
                             MAX31865_REG_CONFIG | MAX31865_WRITE,
                             &config, 1);

    /* Write the high and low fault register values */
    success = MAX31865_write(handle,
                             MAX31865_REG_HI_FAULT_MSB | MAX31865_WRITE,
                             &thresholds, 4);
    return success;
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
    uint8_t txBuffer[5];
    uint8_t rxBuffer[5];
    SPI_Transaction transaction;

    Assert_isTrue((datalen <= 4), NULL);

    memcpy(&txBuffer[1], databuf, datalen);

    /* Set the register address */
    txBuffer[0] = regaddr;

    /* Initialize opcode transaction structure */
    transaction.count = datalen + 1;
    transaction.txBuf = (Ptr)&txBuffer;
    transaction.rxBuf = (Ptr)&rxBuffer;

    /* Hold SPI chip select low */
    if (handle->chipselect_proc == NULL)
        GPIO_write(handle->chipselect, PIN_LOW);
    else
        handle->chipselect_proc(0, TRUE);

    /* Initiate SPI transfer of opcode */
    success = SPI_transfer(handle->spiHandle, &transaction);

    /* Release SPI chip select */
    if (handle->chipselect_proc == NULL)
        GPIO_write(handle->chipselect, PIN_HIGH);
    else
        handle->chipselect_proc(0, FALSE);

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
    uint8_t txBuffer[5];
    uint8_t rxBuffer[5];
    SPI_Transaction transaction;

    Assert_isTrue((datalen <= 4), NULL);

    memset(txBuffer, 0, sizeof(txBuffer));

    /* Set register address */
    txBuffer[0] = regaddr;

    /* Initialize opcode transaction structure */
    transaction.count = datalen + 1;
    transaction.txBuf = (Ptr)&txBuffer;
    transaction.rxBuf = (Ptr)&rxBuffer;

    /* Hold SPI chip select low */
    if (handle->chipselect_proc == NULL)
        GPIO_write(handle->chipselect, PIN_LOW);
    else
        handle->chipselect_proc(0, TRUE);

    /* Initiate SPI transfer of opcode */
    success = SPI_transfer(handle->spiHandle, &transaction);

    /* Release SPI chip select */
    if (handle->chipselect_proc == NULL)
        GPIO_write(handle->chipselect, PIN_HIGH);
    else
        handle->chipselect_proc(0, FALSE);

    /* Return the register data byte */
    memcpy(databuf, &rxBuffer[1], datalen);

    return success;
}

/******************************************************************************
 * Read the ADC value of the RTD and check for threshold errors.
 ******************************************************************************/

uint8_t MAX31865_readADC(MAX31865_Handle handle, uint16_t* data)
{
    uint8_t  reg8;
    uint16_t reg16;
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
     * Step-2: Start a one-shot conversion with vbias.
     */

    reg8 = handle->configReg | MAX31865_CFG_1SHOT(1) | MAX31865_CFG_VBIAS(1);

    MAX31865_write(handle, MAX31865_REG_CONFIG | MAX31865_WRITE, &reg8, 1);

    Task_sleep(handle->conversion_time_delay);

    /*
     * Step-3: Read the RTD 16-bit word with D0 being the error flag bit. Note the
     *         actual ADC value is a 15-bit result and D0 is an error flag bit.
     *         D1 is the LSB, so we shift the lower byte down a bit.
     */

    MAX31865_read(handle, MAX31865_REG_RTD_MSB, &reg16, 2);

    adc = (reg16 & 0xF0) | ((reg16 & 0x0F) >> 1);

    /*
     * Step-4: Now turn vbias back off to save power.
     */

    reg8 = handle->configReg;

    MAX31865_write(handle, MAX31865_REG_CONFIG | MAX31865_WRITE, &reg8, 1);

    /* Check for fault */
    if (reg16 & 0x01)
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

uint8_t MAX31865_readRTD_ohm(MAX31865_Handle handle, float* ohms)
{
    uint16_t adc;
    uint8_t status;

    *ohms = 0.0f;

    if ((status = MAX31865_readADC(handle, &adc)) == MAX31865_ERR_SUCCESS)
    {
        *ohms = ((float)adc * (float)(handle->rref)) / 32768.0f;
    }

    return status;
}

/******************************************************************************
 *
 ******************************************************************************/

uint8_t MAX31865_readCelsius(MAX31865_Handle handle, float* celcius)
{
    float x;
    float ohms;
    uint8_t status;

    /* temperature curve polynomial approximation coefficients */
    static const float _a1 = 2.55865721669;
    static const float _a2 = 0.000967360412;
    static const float _a3 = 0.000000731467;
    static const float _a4 = 0.000000000691;
    static const float _a5 = 7.31888555389e-13;

    *celcius = 0.0f;

    if ((status = MAX31865_readRTD_ohm(handle, &ohms)) == MAX31865_ERR_SUCCESS)
    {
        /* Return Celsius temp calculated using Horners method as
         * this reduces the multiplications and additions needed.
         */
        x = (float)handle->rtd - ohms;

        *celcius = -(x * (_a1 + x * (_a2 + x * (_a3 + x * (_a4 + x * _a5)))));
    }

    return status;
}

/******************************************************************************
 *
 ******************************************************************************/

uint8_t MAX31865_readKelvin(MAX31865_Handle handle, float* kelvin)
{
    uint8_t status;
    float celcius;

    if ((status = MAX31865_readCelsius(handle, &celcius)) == MAX31865_ERR_SUCCESS)
    {
        *kelvin = celcius + 273.15f;
    }

    return status;
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

void MAX31865_setHighFaultThreshold(MAX31865_Handle handle,
                                    uint16_t threshold)
{
#if (MAX31865_THREAD_SAFE > 0)
    IArg key = GateMutex_enter(GateMutex_handle(&(handle->gate)));
#endif

    handle->highFaultThreshold = threshold;

    threshold = threshold << 1;

    MAX31865_write(handle, MAX31865_REG_HI_FAULT_MSB | MAX31865_WRITE, &threshold, 2);

#if (MAX31865_THREAD_SAFE > 0)
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);
#endif
}

/******************************************************************************
 *
 ******************************************************************************/

void MAX31865_setLowFaultThreshold(MAX31865_Handle handle,
                                   uint16_t threshold)
{
#if (MAX31865_THREAD_SAFE > 0)
    IArg key = GateMutex_enter(GateMutex_handle(&(handle->gate)));
#endif

    handle->highFaultThreshold = threshold;

    threshold = threshold << 1;

    MAX31865_write(handle, MAX31865_REG_LO_FAULT_MSB | MAX31865_WRITE, &threshold, 2);

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
