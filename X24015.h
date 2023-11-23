/* ============================================================================
 *
 * XMOD Data Capture and Telemetry Systems
 *
 * Copyright (C) 2021, RTZ Microsystems, LLC
 * All Rights Reserved
 *
 * ============================================================================ */

#ifndef __XMOD_24015_H
#define __XMOD_24015_H

#include "MCP79410.h"       /* RTC clock/cal */
#include "MCP23S17.h"       /* I/O expander  */
#include "MAX31865.h"       /* RTD converter */
#include "AD7799.h"         /* ADC converter */

//*****************************************************************************
// CONSTANTS AND CONFIGURATION
//*****************************************************************************

/* There's only one SD drive in the system, so this is always zero */
#define SD_DRIVE_NUM        0

/* This enables the DIVSCLK output pin on PQ4 and generates a clock signal
 * from the main cpu clock divided by 'div' parameter. A value of 100 gives
 * a clock of 1.2 Mhz.
 */
#define DIV_CLOCK_ENABLED   0

/* VERSION INFO - The min build specifies the minimum build required
 * that does NOT force a default reset of all the config parameters
 * at run time. For instance, if the config loads build 5 and the minimum
 * is set to 3, then it will reset config for anything less than build 3.
 * Likewise, versions 3 or higher would load and use the config values from
 * eprom as normal. This provides a means to force run time config defaults
 * to be reset or not.
 */
#define FIRMWARE_VER        1           /* firmware version */
#define FIRMWARE_REV        8           /* firmware revision */
#define FIRMWARE_BUILD      1           /* firmware build number */
#define FIRMWARE_MIN_BUILD  1           /* min build req'd to force reset */

#if (FIRMWARE_MIN_BUILD > FIRMWARE_BUILD)
#error "X24015 build option FIRMWARE_MIN_BUILD set incorrectly"
#endif

#define MAGIC               0xCEB0FACE  /* magic number for EEPROM data */
#define MAKEREV(v, r)       ((v << 16) | (r & 0xFFFF))

//*****************************************************************************
// ADC Card Data Structures
//*****************************************************************************

/* This defines the max number of ADC cards per rack, the number
 * of converters per card, and the number of channels in the system.
 */
#define ADC_MAX_CARDS               4
#define ADC_CONVERTERS_PER_CARD     2
#define ADC_CHANNELS_PER_CARD       (2 * ADC_CONVERTERS_PER_CARD)

#define ADC_ERROR                   0xFFFFFFFF

/* Each 24035 ADC card has two AD7798 converters per card,
 * each with it's own chip select. This provides a total of
 * four ADC channels per card.
 */
typedef struct _ADC_CONVERTER {
    AD7799_Handle   handle;         /* handle to the AD7799 driver */
    uint32_t        gpiocs;         /* chip select for the ADC     */
} ADC_CONVERTER;

typedef struct _ADC_CARD {
    ADC_CONVERTER   converter[ADC_CONVERTERS_PER_CARD];
} ADC_CARD;

//*****************************************************************************
// RTD Card Data Structures
//*****************************************************************************

/* This defines the max number of RTD cards per rack, the number of
 * converters per card, and the number of channels in the system.
 */
#define RTD_MAX_CARDS               4
#define RTD_CHANNELS_PER_CARD       4

#define RTD_ERROR                   0xFFFFFFFF

/* Each 24037 RTD card has four RTD converters(channels). Each RTD
 * converter requires it's own chip select. The chip selects are driven
 * and de-multiplexed by an MCP23S17 I/O expander chip on the card. Thus
 * one SPI bus is used for chip selects and another for the RTD devices.
 */
typedef struct _RTD_CHANNEL {
    MAX31865_Handle handleRTD;      /* Handle to RTD object for channel */
    uint8_t         csMaskIOX;      /* The IO expander gpio pin for CS  */
} RTD_CHANNEL;

typedef struct _RTD_CARD {
    MCP23S17_Handle handleIOX;      /* Handle of cards I/O expander */
    uint8_t         dipSwitch;      /* config DIP switch on card    */
    uint32_t        chipselIOX;     /* chip select for I/O expander */
    RTD_CHANNEL     channels[RTD_CHANNELS_PER_CARD];
} RTD_CARD;

//*****************************************************************************
//GLOBAL RUN-TIME DATA
//*****************************************************************************

typedef struct _SYSDATA
{
    /* Global Runtime Data */
    uint8_t         ui8SerialNumber[16];    /* 128-bit serial number      */
    uint8_t         ui8MAC[6];              /* 48-bit MAC from EPROM      */
    char            ipAddr[32];             /* IP address from DHCP       */
    uint32_t        lastError;              /* ALM led blinks when set    */
    /* SPI bus peripherals */
    SPI_Handle      spi0;                   /* SPI-0 bus spans all slots  */
    SPI_Handle      spi2;                   /* SPI-2 bus spans all slots  */
    SPI_Handle      spi3;                   /* SPI-3 bus spans all slots  */
    SDSPI_Handle    spiSD;                  /* SPI-3 bus spans all slots  */
    /* I2C bus peripherals */
    I2C_Handle      i2c0;                   /* I2C0 MAC/Serial# part      */
    I2C_Handle      i2c1;                   /* I2C1 spare                 */
    I2C_Handle      i2c2;                   /* I2C2 spare                 */
    I2C_Handle      i2c3;                   /* I2C3 MCP79410 RTC part     */
    /* Devices connected to peripherals */
    MCP79410_Handle handleRTC;              /* MCP79410 RTC part          */
    /* AD7799 ADC data */
    uint8_t         adcID;                  /* chip ID, 16 or 24 bit type */
    uint32_t        adcNumChannels;         /* num of ADC channels active */
    uint32_t        uvcADC[16];             /* the raw ADC value          */
    float           uvcPower[16];           /* the UV-C value in mW/cm2   */
    /* MAX31865 RTD data */
    uint32_t        rtdNumChannels;         /* num of ADC channels active */
    uint32_t        rtdADC[16];             /* the raw ADC value          */
    float           rtdTempC[16];           /* converted to Celcius value */
} SYSDATA;

/* Global System Error Codes for SYSDATA.lastError */

typedef enum XSYSERR {
    XSYSERR_SUCCESS=0,              /* no system errors detected */
    XSYSERR_ADC_INIT,               /* ADC initialization failed */
    XSYSERR_ADC_READ,               /* an ADC sensor read failed */
    XSYSERR_RTD_INIT,               /* RTD initialization failed */
    XSYSERR_RTD_READ,               /* a RTD sensor read failed  */
    XSYSERR_GUID_SERMAC,            /* error reading MAC address */
    /* max error count */
    XSYSERR_LAST_ERROR
} XSYSERR;

//*****************************************************************************
// SYSTEM CONFIG PARAMETERS STORED IN EPROM
//*****************************************************************************

typedef struct _SYSCONFIG
{
    uint32_t    magic;
    uint32_t    version;
    uint32_t    build;
    uint32_t    length;
} SYSCONFIG;

//*****************************************************************************
// External Data Items
//*****************************************************************************

extern SYSDATA g_sys;
extern SYSCONFIG g_cfg;

/*** Function Prototypes ***/

int main(void);
Void MainTaskFxn(UArg arg0, UArg arg1);

void SetLastError(uint32_t error);
uint32_t GetLastError(void);


#endif /* __XMOD_24015_H */
