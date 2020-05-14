/*
 * X24015.h : created 4/8/2020
 *
 * Copyright (C) 2020, Robert E. Starr, Jr.
 */

#ifndef __XMOD_24015_H
#define __XMOD_24015_H

//*****************************************************************************
// CONSTANTS AND CONFIGURATION
//*****************************************************************************

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
#define FIRMWARE_REV        2           /* firmware revision */
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
    uint8_t     ui8SerialNumber[16];        /* 128-bit serial number      */
    uint8_t     ui8MAC[6];                  /* 48-bit MAC from EPROM      */
    char        ipAddr[32];                 /* IP address from DHCP       */
} SYSDATA;

//*****************************************************************************
// SYSTEM CONFIG PARAMETERS STORED IN EPROM
//*****************************************************************************

typedef struct _SYSCONFIG
{
    uint32_t    magic;
    uint32_t    version;
    uint32_t    build;
    /*** GLOBAL PARAMETERS ***/
    long        debug;                     	/* debug level */
} SYSCONFIG;

//*****************************************************************************
// Meter Command Message Structure
//*****************************************************************************

int main(void);
Void MainTaskFxn(UArg arg0, UArg arg1);
int ReadGUIDS(uint8_t ui8SerialNumber[16], uint8_t ui8MAC[6]);

#endif /* __XMOD_24015_H */
