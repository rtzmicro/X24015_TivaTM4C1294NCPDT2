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
/** ============================================================================
 *  @file       X24015.h
 *
 *  @brief      X24015 Board Specific APIs
 *
 *  The X24015 header file should be included in an application as follows:
 *  @code
 *  #include <X24015.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef __X24015_TM4C1294NCPDT_H
#define __X24015_TM4C1294NCPDT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/GPIO.h>

/* LEDs on X24015 are active high. */
#define X24015_LED_OFF	( 0)
#define X24015_LED_ON	(~0)

#define PIN_LOW			( 0)
#define PIN_HIGH		(~0)

/* Board specific I2C addresses */
#define AT24MAC_EPROM_ADDR      (0xA0 >> 1)
#define AT24MAC_EPROM_EXT_ADDR  (0xB0 >> 1)

/* Push button switch bits on Port-N */
#define B_BTN_SW1		0x01
#define B_BTN_SW2		0x02
#define B_BTN_SW3		0x04
#define B_BTN_SW4		0x08
#define B_BTN_SW5		0x10
#define B_BTN_SW6		0x20

/*!
 *  @def    X24015_EMACName
 *  @brief  Enum of EMAC names on the X24015 dev board
 */
typedef enum X24015_EMACName {
    X24015_EMAC0 = 0,

    X24015_EMACCOUNT
} X24015_EMACName;

/*!
 *  @def    X24015_GPIOName
 *  @brief  Enum of LED names on the X24015 dev board
 */
typedef enum X24015_GPIOName {
    /* Inputs */
    X24015_GPIO_PF4 = 0,
    X24015_GPIO_PL5,
    X24015_GPIO_PL4,
    X24015_GPIO_PL2,
    X24015_GPIO_PL3,
    X24015_GPIO_PN0,
    X24015_GPIO_PN1,
    X24015_GPIO_PN2,
    X24015_GPIO_PN5,
    X24015_GPIO_PN3,
    X24015_GPIO_PN4,
    /* Outputs */
    X24015_GPIO_PH3,
    X24015_GPIO_PH2,
    X24015_GPIO_PK7,
    X24015_GPIO_PM0,
    X24015_GPIO_PM1,
    X24015_GPIO_PM2,
    X24015_GPIO_PM3,
    X24015_GPIO_PM6,
    X24015_GPIO_PM7,
    X24015_GPIO_PP3,
    X24015_GPIO_PP2,
    X24015_GPIO_PQ4,

    X24015_GPIOCOUNT
} X24015_GPIOName;

/*!
 *  @def    X24015_I2CName
 *  @brief  Enum of I2C names on the X24015 dev board
 */
typedef enum X24015_I2CName {
    X24015_I2C0 = 0,
    X24015_I2C1,
    X24015_I2C2,
    X24015_I2C3,

    X24015_I2CCOUNT
} X24015_I2CName;

/*!
 *  @def    X24015_PWMName
 *  @brief  Enum of PWM names on the X24015 dev board
 */
typedef enum X24015_PWMName {
    X24015_PWM0 = 0,
    X24015_PWM1,
    X24015_PWM2,
    X24015_PWMCOUNT
} X24015_PWMName;

/*!
 *  @def    X24015_SDSPIName
 *  @brief  Enum of SDSPI names on the X24015 dev board
 */
typedef enum X24015_SDSPIName {
    X24015_SDSPI0 = 0,		/* SD Card Socket */

    X24015_SDSPICOUNT
} X24015_SDSPIName;

/*!
 *  @def    X24015_SPIName
 *  @brief  Enum of SPI names on the X24015 dev board
 */
typedef enum X24015_SPIName {
    X24015_SPI0 = 0,
    X24015_SPI2,
    X24015_SPI3,

    X24015_SPICOUNT
} X24015_SPIName;

/*!
 *  @def    X24015_UARTName
 *  @brief  Enum of UARTs on the X24015 dev board
 */
typedef enum X24015_UARTName {
    X24015_UART0 = 0,
    X24015_UART3,
    X24015_UART5,
    X24015_UART7,

    X24015_UARTCOUNT
} X24015_UARTName;

/*!
 *  @def    X24015_USBMode
 *  @brief  Enum of USB setup function on the X24015 dev board
 */
typedef enum X24015_USBMode {
    X24015_USBDEVICE,
    X24015_USBHOST
} X24015_USBMode;

/*!
 *  @def    X24015_USBMSCHFatFsName
 *  @brief  Enum of USBMSCHFatFs names on the X24015 dev board
 */
typedef enum X24015_USBMSCHFatFsName {
    X24015_USBMSCHFatFs0 = 0,

    X24015_USBMSCHFatFsCOUNT
} X24015_USBMSCHFatFsName;

/*
 *  @def    X24015_WatchdogName
 *  @brief  Enum of Watchdogs on the X24015 dev board
 */
typedef enum X24015_WatchdogName {
    X24015_WATCHDOG0 = 0,

    X24015_WATCHDOGCOUNT
} X24015_WatchdogName;

/*!
 *  @def    X24015_WiFiName
 *  @brief  Enum of WiFi names on the X24015 dev board
 */
typedef enum X24015_WiFiName {
    X24015_WIFI = 0,

    X24015_WIFICOUNT
} X24015_WiFiName;

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings. This include
 *     - Enable clock sources for peripherals
 */
extern void X24015_initGeneral(void);

/*!
 *  @brief Initialize board specific EMAC settings
 *
 *  This function initializes the board specific EMAC settings and
 *  then calls the EMAC_init API to initialize the EMAC module.
 *
 *  The EMAC address is programmed as part of this call.
 *
 */
extern void X24015_initEMAC(void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_config
 *  variable.
 */
extern void X24015_initGPIO(void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern void X24015_initI2C(void);

/*!
 *  @brief  Initialize board specific PWM settings
 *
 *  This function initializes the board specific PWM settings and then calls
 *  the PWM_init API to initialize the PWM module.
 *
 *  The PWM peripherals controlled by the PWM module are determined by the
 *  PWM_config variable.
 */
extern void X24015_initPWM(void);

/*!
 *  @brief  Initialize board specific SDSPI settings
 *
 *  This function initializes the board specific SDSPI settings and then calls
 *  the SDSPI_init API to initialize the SDSPI module.
 *
 *  The SDSPI peripherals controlled by the SDSPI module are determined by the
 *  SDSPI_config variable.
 */
extern void X24015_initSDSPI(void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
extern void X24015_initSPI(void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern void X24015_initUART(void);

/*!
 *  @brief  Initialize board specific USB settings
 *
 *  This function initializes the board specific USB settings and pins based on
 *  the USB mode of operation.
 *
 *  @param      usbMode    USB mode of operation
 */
extern void X24015_initUSB(X24015_USBMode usbMode);

/*!
 *  @brief  Initialize board specific USBMSCHFatFs settings
 *
 *  This function initializes the board specific USBMSCHFatFs settings and then
 *  calls the USBMSCHFatFs_init API to initialize the USBMSCHFatFs module.
 *
 *  The USBMSCHFatFs peripherals controlled by the USBMSCHFatFs module are
 *  determined by the USBMSCHFatFs_config variable.
 */
extern void X24015_initUSBMSCHFatFs(void);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
extern void X24015_initWatchdog(void);

/*!
 *  @brief  Initialize board specific WiFi settings
 *
 *  This function initializes the board specific WiFi settings and then calls
 *  the WiFi_init API to initialize the WiFi module.
 *
 *  The hardware resources controlled by the WiFi module are determined by the
 *  WiFi_config variable.
 */
extern void X24015_initWiFi(void);

#ifdef __cplusplus
}
#endif

#endif /* __X24015_TM4C1294NCPDT_H */
