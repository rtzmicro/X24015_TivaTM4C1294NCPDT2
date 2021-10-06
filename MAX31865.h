/** ============================================================================
 *  @file       MAX31865.h
 *
 *  @brief      MAX31865 driver interface
 *
 *  The MAX31865 header file should be included in an application as follows:
 *  @code
 *  #include <MAX31865.h>
 *  @endcode
 *
 *  # Operation #
 *  This example module allows an application to read from/write to a MAX31865
 *  device.
 *
 *  The APIs are thread-safe. Two tasks can write/read to the same device safely.
 *  This is accomplished with a GateMutex in the implementation.

 *  ## Creating an instance #
 *  @code
 *  SPI_Params spiParams;
 *  SPI_Handle spiHandle;
 *  MAX31865_Params ad7799Params;
 *  MAX31865_Object obj;
 *  MAX31865_Handle ad7799Handle;
 *  volatile uint8_t ready;
 *
 *  SPI_Params_init(&spiParams);
 *  spiHandle = SPI_open(Board_SPI0, &spiParams);
 *
 *  MAX31865_Params_init(&ad7799Params);
 *  ad7799Handle = MAX31865_create(spiHandle, Board_CS, &ad7799Params);
 *  if (!handle) {
 *      System_printf("MAX31865_create failed");
 *  }
 *  @endcode
 */

#ifndef __MAX31865_H__
#define __MAX31865_H__

/* Set to one for thread safe use */
#ifndef MAX31865_THREAD_SAFE
#define MAX31865_THREAD_SAFE        0
#endif

#include <stdint.h>
#include <stdbool.h>
#include <ti/drivers/SPI.h>
#if (MAX31865_THREAD_SAFE > 0)
#include <ti/sysbios/gates/GateMutex.h>
#endif

/* Simple macro to convert to Fahrenheit */
#define CELCIUS_TO_FAHRENHEIT(c)    ((float)c * 1.8f + 32.0f)

/******************************************************************************/
/* MAX31865 RTD-to-Digital Converter                                          */
/******************************************************************************/

/* MAX31865 Registers */
#define MAX31865_REG_CONFIG         0x00        /* configuration reg    (R/W) */
#define MAX31865_REG_RTD_MSB        0x01        /* RTD data reg MSB     (R)   */
#define MAX31865_REG_RTD_LSB        0x02        /* RTD data reg LSB     (R)   */
#define MAX31865_REG_HI_FAULT_MSB   0x03        /* High fault threshold (R/W) */
#define MAX31865_REG_HI_FAULT_LSB   0x04        /* High fault threshold (R/W) */
#define MAX31865_REG_LO_FAULT_MSB   0x05        /* Low fault threshold  (R/W) */
#define MAX31865_REG_LO_FAULT_LSB   0x06        /* Low fault threshold  (R/W) */
#define MAX31865_REG_FAULT_STATUS   0x07        /* Fault Status         (R)   */

/* Upper bit sets read/write mode */
#define MAX31865_WRITE              0x80        /* bit-7 set for write mode   */

/* MAX31865_REG_CFG register bits */
#define MAX31865_CFG_50HZ(x)        (((x) & 0x01) << 0)
#define MAX31865_CFG_FAULT_CLR(x)   (((x) & 0x01) << 1)
#define MAX31865_CFG_FAULT_CYCLE(x) (((x) & 0x03) << 2)
#define MAX31865_CFG_3WIRE_RTD(x)   (((x) & 0x01) << 4)
#define MAX31865_CFG_1SHOT(x)       (((x) & 0x01) << 5)
#define MAX31865_CFG_MODE(x)        (((x) & 0x01) << 6)
#define MAX31865_CFG_VBIAS(x)       (((x) & 0x01) << 7)

/* Charge time delay */
#define MAX31865_CHARGE_TIME        100
/* Conversion time delay */
#define MAX31865_CONVERSION_TIME    100

/* MAX31865 Error Codes */
#define MAX31865_ERR_SUCCESS                0x00
#define MAX31865_ERR_VOLTAGE_FAULT          0x04
#define MAX31865_ERR_VRTDIN_TO_LOW          0x08
#define MAX31865_ERR_VREFIN_TO_LOW          0x10
#define MAX31865_ERR_VREFIN_TO_HIGH         0x20
#define MAX31865_ERR_RTD_LOW_THRESHOLD      0x40
#define MAX31865_ERR_RTD_HIGH_THRESHOLD     0x80
#define MAX31865_ERR_UNDEFINED              0xFF

/******************************************************************************/
/* MAX31865 Data Structures                                                   */
/******************************************************************************/

/* Define chip select callback function pointer. If this
 * member is NULL, then gpioCS is used to drive the chip
 * select. Otherwise, the callback function is called
 * and must handle controlling the chip select line.
 */
typedef void (*fptr_chipsel)(void* param1, void* param2, bool assert);

/*!
 *  @brief MAX31865 Object
 *
 *  The application should never directly access the fields in the structure.
 */
typedef struct MAX31865_Object {
    SPI_Handle      spiHandle;              /* SPI handle to MAX31865        */
    uint32_t        chipselect;             /* Single chip CS GPIO index     */
    fptr_chipsel    chipselect_proc;        /* Callback for multiplex CS use */
    void*           chipselect_param1;      /* parameter passed to callback  */
    void*           chipselect_param2;      /* parameter passed to callback  */
#if (MAX31865_THREAD_SAFE > 0)
    GateMutex_Struct gate;                  /* Mutex for thread safe use     */
#endif
    uint32_t        charge_time_delay;      /* Charge time for vbias enable  */
    uint32_t        conversion_time_delay;  /* ADC conversion time           */
    uint16_t        rtd;
    uint16_t        rref;
    uint16_t        lowFaultThreshold;      /* Low fault threshold value      */
    uint16_t        highFaultThreshold;     /* High fault threshold value     */
    uint8_t         configReg;              /* Configuration register value   */
} MAX31865_Object;

/*!
 *  @brief MAX31865 Handle
 *
 *  Used to identify a MAX31865 device in the APIs
 */
typedef MAX31865_Object *MAX31865_Handle;

/*!
 *  @brief MAX31865 Parameters
 *
 *  This is a place-holder structure now since there are no parameters
 *  for the create/construct calls.
 *
 *  @sa         MAX31865_Params_init()
 */
typedef struct MAX31865_Params {
    uint32_t        charge_time_delay;
    uint32_t        conversion_time_delay;
    uint16_t        rtd;
    uint16_t        rref;
    uint16_t        lowFaultThreshold;
    uint16_t        highFaultThreshold;
    uint8_t         configReg;
    uint32_t        chipselect;
    fptr_chipsel    chipselect_proc;
    void*           chipselect_param1;
    void*           chipselect_param2;
} MAX31865_Params;

/*!
 *  @brief  Function to initialize a given MAX31865 object
 *
 *  Function to initialize a given MAX31865 object specified by the
 *  particular SPI handle and GPIO CS index values.
 *
 *  @param  obj           Pointer to a MAX31865_Object structure. It does not
 *                        need to be initialized.
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values. All the fields in this structure are
 *                        RO (read-only).
 *
 *  @return A MAX31865_Handle on success or a NULL on an error.
 *
   @sa     MAX31865_destruct()
 */
MAX31865_Handle MAX31865_construct(MAX31865_Object *obj,
                                   SPI_Handle spiHandle,
                                   MAX31865_Params *params);

/*!
 *  @brief  Function to initialize a given MAX31865 device
 *
 *  Function to create a MAX31865 object specified by the
 *  particular SPI handle and GPIO CS index values.
 *
 *  @param  spiHandle     SPI handle that the MAX31865 is attached to
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values. All the fields in this structure are
 *                        RO (read-only).
 *
 *  @return A MAX31865_Handle on success or a NULL on an error.
 *
   @sa     MAX31865_delete()
 */
MAX31865_Handle MAX31865_create(SPI_Handle spiHandle, MAX31865_Params *params);

/*!
 *  @brief  Function to delete a MAX31865 instance
 *
 *  @pre    MAX31865_create() had to be called first.
 *
 *  @param  handle      A MAX31865_Handle returned from MAX31865_create
 *
 *  @sa     MAX31865_create()
 */
void MAX31865_delete(MAX31865_Handle handle);

/* Initialize default params struct */
void MAX31865_Params_init(MAX31865_Params *params);

/* Initialize MAX31865 and check if the device is present*/
bool MAX31865_init(MAX31865_Handle handle);

uint8_t MAX31865_readADC(MAX31865_Handle handle, uint16_t* data);

uint8_t MAX31865_readRTD_ohm(MAX31865_Handle handle, float* ohms);

uint8_t MAX31865_readCelsius(MAX31865_Handle handle, float* celcius);

uint8_t MAX31865_readKelvin(MAX31865_Handle handle, float* kelvin);

uint8_t MAX31865_readFault(MAX31865_Handle handle);

void MAX31865_clearFault(MAX31865_Handle handle);

void MAX31865_setHighFaultThreshold(MAX31865_Handle handle, uint16_t threshold);

void MAX31865_setLowFaultThreshold(MAX31865_Handle handle, uint16_t threshold);

int MAX31865_IsThresholdFault(MAX31865_Handle handle);

#endif	// _MAX31865_H_
