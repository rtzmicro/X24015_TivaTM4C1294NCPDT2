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

#define Board_LED_ACT               X24015_GPIO_PP2
#define Board_LED_ALM               X24015_GPIO_PP3

#define Board_PWRUP_BUS_OUT         X24015_GPIO_PM6
#define Board_RESET_BUS_OUT         X24015_GPIO_PM7

#define Board_I2C_AT24MAC402        X24015_I2C0

#define Board_PWM0                  X24015_PWM0
#define Board_PWM1                  X24015_PWM0


#define Board_USBHOST               X24015_USBHOST
#define Board_USBDEVICE             X24015_USBDEVICE

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
