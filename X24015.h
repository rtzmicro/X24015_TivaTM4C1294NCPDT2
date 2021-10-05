/*
 * X24015.h : created 4/8/2020
 *
 * Copyright (C) 2020, Robert E. Starr, Jr.
 */

#ifndef __XMOD_24015_H
#define __XMOD_24015_H

#include "MCP79410.h"
#include "AD7799.h"

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
#define FIRMWARE_REV        7           /* firmware revision */
#define FIRMWARE_BUILD      1           /* firmware build number */
#define FIRMWARE_MIN_BUILD  1           /* min build req'd to force reset */

#if (FIRMWARE_MIN_BUILD > FIRMWARE_BUILD)
#error "X24015 build option FIRMWARE_MIN_BUILD set incorrectly"
#endif

#define MAGIC               0xCEB0FACE  /* magic number for EEPROM data */
#define MAKEREV(v, r)       ((v << 16) | (r & 0xFFFF))

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
    //AD7799_Handle   AD7799Handle1;
    //AD7799_Handle   AD7799Handle2;
    /* AD7799 ADC data */
    uint8_t         adcID;                  /* chip ID, 16 or 24 bit type */
    uint32_t        adcChannels;            /* num of ADC channels active */
    uint32_t        adcData[16];
} SYSDATA;

/* Global System Error Codes for SYSDATA.lastError */

typedef enum XSYSERR {
    XSYSERR_SUCCESS=0,              /* no system errors detected */
    XSYSERR_ADC_INIT,               /* an ADC board failed to initialize */
    XSYSERR_GUID_SERMAC,            /* error reading MAC & serial number */
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
//
//*****************************************************************************

/*** External Data Items ***/

extern SYSDATA g_sys;
extern SYSCONFIG g_cfg;

/*** Function Prototypes ***/

int main(void);
Void MainTaskFxn(UArg arg0, UArg arg1);

void SetLastError(uint32_t error);
uint32_t GetLastError(void);


#endif /* __XMOD_24015_H */
