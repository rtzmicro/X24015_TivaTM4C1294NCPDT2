/* ============================================================================
 *
 * XMOD Data Capture and Telemetry Systems
 *
 * Copyright (C) 2021, RTZ Microsystems, LLC
 * All Rights Reserved
 *
 * ============================================================================ */

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <X24015_TM4C1294NCPDT.h>

#define Board_initEMAC              X24015_initEMAC
#define Board_initGeneral           X24015_initGeneral
#define Board_initGPIO              X24015_initGPIO
#define Board_initI2C               X24015_initI2C
#define Board_initPWM               X24015_initPWM
#define Board_initSDSPI             X24015_initSDSPI
#define Board_initSPI               X24015_initSPI
#define Board_initUART              X24015_initUART
#define Board_initUSB               X24015_initUSB
#define Board_initWatchdog          X24015_initWatchdog

#define Board_LED_ON                X24015_LED_ON
#define Board_LED_OFF               X24015_LED_OFF

#define Board_LED_ACT               X24015_GPIO_PP3
#define Board_LED_ALM               X24015_GPIO_PP2

#define Board_PWRUP_BUS_OUT         X24015_GPIO_PM6
#define Board_RESET_BUS_OUT         X24015_GPIO_PM7

#define Board_USBHOST               X24015_USBHOST
#define Board_USBDEVICE             X24015_USBDEVICE

#define Board_I2C0                  X24015_I2C0
#define Board_I2C1                  X24015_I2C1
#define Board_I2C2                  X24015_I2C2
#define Board_I2C3                  X24015_I2C3

#define Board_I2C_AT24MAC402        X24015_I2C0
#define Board_I2C_MCP79410          X24015_I2C3

#define Board_SPI0                  X24015_SPI0
#define Board_SPI2                  X24015_SPI2
#define Board_SPI3                  X24015_SPI3

#define Board_PWM0                  X24015_PWM0
#define Board_PWM1                  X24015_PWM0

#define Board_COM1                  X24015_UART0
#define Board_UART3                 X24015_UART3
#define Board_UART5                 X24015_UART5
#define Board_UART7                 X24015_UART7

/* I/O Option Card Specific Port Defines */

#define Board_WATCHDOG0             X24015_WATCHDOG0

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
