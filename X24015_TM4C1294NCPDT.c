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

/*
 *  ======== X24015.c ========
 *  This file is responsible for setting up the board specific items for the
 *  X24015 board.
 */

#include <stdint.h>
#include <stdbool.h>

#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>

#include <driverlib/flash.h>
#include <driverlib/gpio.h>
#include <driverlib/i2c.h>
#include <driverlib/pin_map.h>
#include <driverlib/pwm.h>
#include <driverlib/ssi.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/udma.h>
#include <driverlib/eeprom.h>
//#include <X24015.h>
#include <X24015_TM4C1294NCPDT.h>

#ifndef TI_DRIVERS_UART_DMA
#define TI_DRIVERS_UART_DMA 0
#endif

/*
 *  =============================== DMA ===============================
 */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#endif
static tDMAControlTable dmaControlTable[32];
static bool dmaInitialized = false;

/* Hwi_Struct used in the initDMA Hwi_construct call */
static Hwi_Struct dmaHwiStruct;

/* Hwi_Struct used in the usbBusFault Hwi_construct call */
static Hwi_Struct usbBusFaultHwiStruct;

/*
 *  ======== dmaErrorHwi ========
 */
static Void dmaErrorHwi(UArg arg)
{
    System_printf("DMA error code: %d\n", uDMAErrorStatusGet());
    uDMAErrorStatusClear();
    System_abort("DMA error!!");
}

/*
 *  ======== X24015_usbBusFaultHwi ========
 */
static Void X24015_usbBusFaultHwi(UArg arg)
{
    /*
     *  This function should be modified to appropriately manage handle
     *  a USB bus fault.
    */
    System_printf("USB bus fault detected.");
    Hwi_clearInterrupt(INT_GPIOQ4);
    System_abort("USB error!!");
}

/*
 *  ======== X24015_initDMA ========
 */
void X24015_initDMA(void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    if (!dmaInitialized) {
        Error_init(&eb);
        Hwi_Params_init(&hwiParams);
        Hwi_construct(&(dmaHwiStruct), INT_UDMAERR, dmaErrorHwi,
                      &hwiParams, &eb);
        if (Error_check(&eb)) {
            System_abort("Couldn't construct DMA error hwi");
        }

        SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
        uDMAEnable();
        uDMAControlBaseSet(dmaControlTable);

        dmaInitialized = true;
    }
}

/*
 *  =============================== General ===============================
 */
/*
 *  ======== X24015_initGeneral ========
 */
void X24015_initGeneral(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);

    // Initialize the EEPROM so we can access it later

    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

    if (EEPROMInit() != EEPROM_INIT_OK)
        System_abort("EEPROMInit() failed!\n");

    uint32_t size = EEPROMSizeGet();
}

/*
 *  =============================== EMAC ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(EMAC_config, ".const:EMAC_config")
#pragma DATA_SECTION(emacHWAttrs, ".const:emacHWAttrs")
#pragma DATA_SECTION(NIMUDeviceTable, ".data:NIMUDeviceTable")
#endif

#include <ti/drivers/EMAC.h>
#include <ti/drivers/emac/EMACSnow.h>

/*
 *  Required by the Networking Stack (NDK). This array must be NULL terminated.
 *  This can be removed if NDK is not used.
 *  Double curly braces are needed to avoid GCC bug #944572
 *  https://bugs.launchpad.net/gcc-linaro/+bug/944572
 */
NIMU_DEVICE_TABLE_ENTRY NIMUDeviceTable[2] = {
    {
#if TI_EXAMPLES_PPP
        /* Use PPP driver for PPP example only */
        .init = USBSerialPPP_NIMUInit
#else
        /* Default: use Ethernet driver */
        .init = EMACSnow_NIMUInit
#endif
    },
    {NULL}
};

EMACSnow_Object emacObjects[X24015_EMACCOUNT];

/*
 *  EMAC configuration structure
 *  Set user/company specific MAC octates. The following sets the address
 *  to ff-ff-ff-ff-ff-ff. Users need to change this to make the label on
 *  their boards.
 */
unsigned char macAddress[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

const EMACSnow_HWAttrs emacHWAttrs[X24015_EMACCOUNT] = {
    {
        .baseAddr    = EMAC0_BASE,
        .intNum      = INT_EMAC0,
        .intPriority = (~0),
        .macAddress  = macAddress
    }
};

const EMAC_Config EMAC_config[] = {
    {
        .fxnTablePtr = &EMACSnow_fxnTable,
        .object      = &emacObjects[0],
        .hwAttrs     = &emacHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== X24015_initEMAC ========
 */

/* Don't call Board_initEMAC() in main(). Instead call it in a task after you get the MAC address.
 *
 * You have a couple options to delay the starting of the NDK stack:
 * 1. Do not graphically generate it. Instead supply the thread yourself and start it after calling Board_initEMAC().
 * 2. Add a startup hook function into the stack via the .cfg file. Have that function block on a semaphore.
 * Post the semaphore after you call Board_initEMAC(). The hook is called very early in the stack thread,
 * well before the MAC address is referenced.
 *
 * .cfg file
 *  Global.stackBeginHook = "&myNDKStackBeginHook";
 *
 * .c file
 *  void myNDKStackBeginHook()
 *  {
 *      Semaphore_pend(mySem, BIOS_WAIT_FOREVER);
 *  }
 *
 */

void X24015_initEMAC(unsigned char* mac)
{
    /* We use the MAC address from Atmel AT24 ID EPROM */
    memcpy(macAddress, mac, 6);

#if 0
    uint32_t ulUser0, ulUser1;

    /* Get the MAC address */
    FlashUserGet(&ulUser0, &ulUser1);
    if ((ulUser0 != 0xffffffff) && (ulUser1 != 0xffffffff)) {
        System_printf("Using MAC address in flash\n");
        /*
         *  Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
         *  address needed to program the hardware registers, then program the MAC
         *  address into the Ethernet Controller registers.
         */
        macAddress[0] = ((ulUser0 >>  0) & 0xff);
        macAddress[1] = ((ulUser0 >>  8) & 0xff);
        macAddress[2] = ((ulUser0 >> 16) & 0xff);
        macAddress[3] = ((ulUser1 >>  0) & 0xff);
        macAddress[4] = ((ulUser1 >>  8) & 0xff);
        macAddress[5] = ((ulUser1 >> 16) & 0xff);
    }
    else if (macAddress[0] == 0xff && macAddress[1] == 0xff &&
             macAddress[2] == 0xff && macAddress[3] == 0xff &&
             macAddress[4] == 0xff && macAddress[5] == 0xff) {
        System_abort("Change the macAddress variable to match your boards MAC sticker");
    }
#endif
    // Enable peripheral EPHY0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);

    GPIOPinConfigure(GPIO_PF0_EN0LED0);  /* X24015_J5 YEL */
    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0);

    GPIOPinConfigure(GPIO_PK6_EN0LED1);  /* X24015_J5_GRN */
    GPIOPinTypeEthernetLED(GPIO_PORTK_BASE, GPIO_PIN_6);

    /* Once EMAC_init is called, EMAC_config cannot be changed */
    EMAC_init();
}

/*
 *  =============================== GPIO ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(GPIOTiva_config, ".const:GPIOTiva_config")
#endif

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOTiva.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in X24015_TM4C1294NCPDT.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array.  Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /**** Input Interrupt pins ****/

    /**** Output pins ****/
    /* X24015_GPIO_PL2 */
    GPIOTiva_PL_2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PL3 */
    GPIOTiva_PL_3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PL4 */
    GPIOTiva_PL_4 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PL5 */
    GPIOTiva_PL_5 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PN0 */
    GPIOTiva_PN_0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PN1 */
    GPIOTiva_PN_1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PN2 */
    GPIOTiva_PN_2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PN3 */
    GPIOTiva_PN_3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PN4 */
    GPIOTiva_PN_4 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PN5 */
    GPIOTiva_PN_5 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PH2 */
    GPIOTiva_PH_2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PH3 */
    GPIOTiva_PH_3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PM0 */
    GPIOTiva_PM_0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    /* X24015_GPIO_PM1 */
    GPIOTiva_PM_1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    /* X24015_GPIO_PM2 */
    GPIOTiva_PM_2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    /* X24015_GPIO_PM3 */
    GPIOTiva_PM_3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    /* X24015_GPIO_PM6 (PWRUP_BUS_OUT) */
    GPIOTiva_PM_6 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    /* X24015_GPIO_PM7 (RESET_BUS_OUT) */
    GPIOTiva_PM_7 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    /* X24015_GPIO_PP2 */
    GPIOTiva_PP_2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    /* X24015_GPIO_PP3 */
    GPIOTiva_PP_3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    /* X24015_GPIO_PQ4 */
    GPIOTiva_PQ_4 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in X24015_TM4C1294NCPDT.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,   /* X24015_GPIO_xxx */
    NULL,   /* X24015_GPIO_xxx */
    NULL,   /* X24015_GPIO_xxx */
    NULL,   /* X24015_GPIO_xxx */
    NULL,   /* X24015_GPIO_xxx */
    NULL,   /* X24015_GPIO_xxx */
    NULL,   /* X24015_GPIO_xxx */
    NULL,   /* X24015_GPIO_xxx */
    NULL,   /* X24015_GPIO_xxx */
    NULL,   /* X24015_GPIO_xxx */
    NULL,   /* X24015_GPIO_xxx */
};

/* The device-specific GPIO_config structure */
const GPIOTiva_Config GPIOTiva_config = {
    .pinConfigs         = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks          = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
    .numberOfCallbacks  = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority        = (~0)
};

/*
 *  ======== X24015_initGPIO ========
 */
void X24015_initGPIO(void)
{
    // Enable pin PL2 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_2);
    // Enable pin PL3 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);
    // Enable pin PL4 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4);
    // Enable pin PL5 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_5);

    // Enable pin PN0 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    // Enable pin PN1 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);
    // Enable pin PN2 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);
    // Enable pin PN3 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3);
    // Enable pin PN4 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4);
    // Enable pin PN5 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_5);

    // Enable pin PH2 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_2);
    // Enable pin PH3 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_3);
    // Enable pin PM0 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);
    // Enable pin PM1 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_1);
    // Enable pin PM2 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_2);
    // Enable pin PM3 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_3);
    // Enable pin PM6 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_6);
    // Enable pin PM7 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_7);

    // Enable pin PP2 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_2);
    // Enable pin PP3 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_3);
    // Enable pin PQ4 for GPIOOutput
    GPIOPinTypeGPIOOutput(GPIO_PORTQ_BASE, GPIO_PIN_4);

    /* Once GPIO_init is called, GPIO_config cannot be changed */
    GPIO_init();
}

/*
 *  =============================== I2C ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(I2C_config, ".const:I2C_config")
#pragma DATA_SECTION(i2cTivaHWAttrs, ".const:i2cTivaHWAttrs")
#endif

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CTiva.h>

I2CTiva_Object i2cTivaObjects[X24015_I2CCOUNT];

const I2CTiva_HWAttrs i2cTivaHWAttrs[X24015_I2CCOUNT] = {
    {
        .baseAddr    = I2C0_BASE,
        .intNum      = INT_I2C0,
        .intPriority = (~0)
    },
    {
        .baseAddr    = I2C1_BASE,
        .intNum      = INT_I2C1,
        .intPriority = (~0)
    },
    {
        .baseAddr    = I2C2_BASE,
        .intNum      = INT_I2C2,
        .intPriority = (~0)
    },
    {
        .baseAddr    = I2C3_BASE,
        .intNum      = INT_I2C3,
        .intPriority = (~0)
    }
};

const I2C_Config I2C_config[] = {
    {
        .fxnTablePtr = &I2CTiva_fxnTable,
        .object      = &i2cTivaObjects[0],
        .hwAttrs     = &i2cTivaHWAttrs[0]
    },
    {
        .fxnTablePtr = &I2CTiva_fxnTable,
        .object      = &i2cTivaObjects[1],
        .hwAttrs     = &i2cTivaHWAttrs[1]
    },
    {
        .fxnTablePtr = &I2CTiva_fxnTable,
        .object      = &i2cTivaObjects[2],
        .hwAttrs     = &i2cTivaHWAttrs[2]
    },
    {
        .fxnTablePtr = &I2CTiva_fxnTable,
        .object      = &i2cTivaObjects[3],
        .hwAttrs     = &i2cTivaHWAttrs[3]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== X24015_initI2C ========
 */
void X24015_initI2C(void)
{
    /* Enable the peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);

    /* I2C0 Init */
    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    /* I2C1 Init */
    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PG0_I2C1SCL);
    GPIOPinConfigure(GPIO_PG1_I2C1SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_1);

    /* I2C2 Init */
    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PL1_I2C2SCL);
    GPIOPinConfigure(GPIO_PL0_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTL_BASE, GPIO_PIN_1);
    GPIOPinTypeI2C(GPIO_PORTL_BASE, GPIO_PIN_0);

    /* I2C3 Init */
    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PK4_I2C3SCL);
    GPIOPinConfigure(GPIO_PK5_I2C3SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTK_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTK_BASE, GPIO_PIN_5);

    I2C_init();
}

/*
 *  =============================== PWM ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(PWM_config, ".const:PWM_config")
#pragma DATA_SECTION(pwmTivaHWAttrs, ".const:pwmTivaHWAttrs")
#endif

#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTiva.h>
#include <driverlib/pwm.h>

PWMTiva_Object pwmTivaObjects[X24015_PWMCOUNT];

const PWMTiva_HWAttrs pwmTivaHWAttrs[X24015_PWMCOUNT] = {
    {
        .baseAddr   = PWM0_BASE,
        .pwmOutput  = PWM_OUT_0,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN
    },
    {
        .baseAddr   = PWM0_BASE,
        .pwmOutput  = PWM_OUT_1,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN
    },
    {
        .baseAddr   = PWM0_BASE,
        .pwmOutput  = PWM_OUT_2,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN
    }
};

const PWM_Config PWM_config[] = {
    {
        .fxnTablePtr = &PWMTiva_fxnTable,
        .object      = &pwmTivaObjects[0],
        .hwAttrs     = &pwmTivaHWAttrs[0]
    },
    {
        .fxnTablePtr = &PWMTiva_fxnTable,
        .object      = &pwmTivaObjects[1],
        .hwAttrs     = &pwmTivaHWAttrs[1]
    },
    {
        .fxnTablePtr = &PWMTiva_fxnTable,
        .object      = &pwmTivaObjects[2],
        .hwAttrs     = &pwmTivaHWAttrs[2]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== X24015_initPWM ========
 */
void X24015_initPWM(void)
{
    /* Enable PWM peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // Enable pin PF3 for PWM0 M0PWM3
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

    // Enable pin PF1 for PWM0 M0PWM1
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

    // Enable pin PF2 for PWM0 M0PWM2
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    PWM_init();
}

/*
 *  =============================== SDSPI ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SDSPI_config, ".const:SDSPI_config")
#pragma DATA_SECTION(sdspiTivaHWattrs, ".const:sdspiTivaHWattrs")
#endif

#include <ti/drivers/SDSPI.h>
#include <ti/drivers/sdspi/SDSPITiva.h>

/* SDSPI objects */
SDSPITiva_Object sdspiTivaObjects[X24015_SDSPICOUNT];

const SDSPITiva_HWAttrs sdspiTivaHWattrs[X24015_SDSPICOUNT] = {
    {
         /* SD Card Socket (J1) */
        .baseAddr = SSI1_BASE,          /* SPI base address */
        .portSCK  = GPIO_PORTB_BASE,    /* SPI SCK PORT */
        .pinSCK   = GPIO_PIN_5,         /* SCK PIN (PB5) */
        .portMISO = GPIO_PORTE_BASE,    /* SPI MISO PORT */
        .pinMISO  = GPIO_PIN_5,         /* MISO PIN (PE5) */
        .portMOSI = GPIO_PORTE_BASE,    /* SPI MOSI PORT */
        .pinMOSI  = GPIO_PIN_4,         /* MOSI PIN (PE4) */
        .portCS   = GPIO_PORTK_BASE,    /* GPIO CS PORT */
        .pinCS    = GPIO_PIN_7          /* CS PIN (PK7) */
    }
};

const SDSPI_Config SDSPI_config[] = {
    {
        .fxnTablePtr = &SDSPITiva_fxnTable,
        .object      = &sdspiTivaObjects[0],
        .hwAttrs     = &sdspiTivaHWattrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== X24015_initSDSPI ========
 */
void X24015_initSDSPI(void)
{
    /* Enable SD SSI peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    /* SSI-1 Configure Pins */

    // Enable pin PE5 for SSI1 SSI1XDAT1
    GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);
    GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_5);

    // Enable pin PE4 for SSI1 SSI1XDAT0
    GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);
    GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_4);

    // Enable pin PB5 for SSI1 SSI1CLK
    GPIOPinConfigure(GPIO_PB5_SSI1CLK);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5);

    // Enable pin PB4 for SSI1 SSI1FSS
    //GPIOPinConfigure(GPIO_PB4_SSI1FSS);
    //GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4);

    // Enable pin PK7 for GPIOOutput (SSI1FSS_SD)
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_7);

    /* Configure pad settings */

    /* SCK (PB5) */
    GPIOPadConfigSet(GPIO_PORTB_BASE,
                     GPIO_PIN_5,
                     GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    /* MOSI (PE4) */
    GPIOPadConfigSet(GPIO_PORTE_BASE,
                     GPIO_PIN_4,
                     GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    /* MISO (PE5) */
    GPIOPadConfigSet(GPIO_PORTE_BASE,
                     GPIO_PIN_5,
                     GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
    /* CS (PK7) */
    GPIOPadConfigSet(GPIO_PORTK_BASE,
                     GPIO_PIN_7,
                     GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    SDSPI_init();
}

/*
 *  =============================== SPI ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SPI_config, ".const:SPI_config")
#pragma DATA_SECTION(spiTivaDMAHWAttrs, ".const:spiTivaDMAHWAttrs")
#endif

#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPITivaDMA.h>

SPITivaDMA_Object spiTivaDMAObjects[X24015_SPICOUNT];

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(spiTivaDMAscratchBuf, 32)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=32
#elif defined(__GNUC__)
__attribute__ ((aligned (32)))
#endif
uint32_t spiTivaDMAscratchBuf[X24015_SPICOUNT];

const SPITivaDMA_HWAttrs spiTivaDMAHWAttrs[X24015_SPICOUNT] = {
    {
        .baseAddr               = SSI0_BASE,
        .intNum                 = INT_SSI0,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[0],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_CHANNEL_SSI0RX,
        .txChannelIndex         = UDMA_CHANNEL_SSI0TX,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH10_SSI0RX,
        .txChannelMappingFxnArg = UDMA_CH11_SSI0TX
    },
    {
        .baseAddr               = SSI2_BASE,
        .intNum                 = INT_SSI2,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[1],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_SEC_CHANNEL_UART2RX_12,
        .txChannelIndex         = UDMA_SEC_CHANNEL_UART2TX_13,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH12_SSI2RX,
        .txChannelMappingFxnArg = UDMA_CH13_SSI2TX
    },
    {
        .baseAddr               = SSI3_BASE,
        .intNum                 = INT_SSI3,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[2],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_SEC_CHANNEL_TMR2A_14,
        .txChannelIndex         = UDMA_SEC_CHANNEL_TMR2B_15,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH14_SSI3RX,
        .txChannelMappingFxnArg = UDMA_CH15_SSI3TX
    }
};

const SPI_Config SPI_config[] = {
    {
        .fxnTablePtr = &SPITivaDMA_fxnTable,
        .object      = &spiTivaDMAObjects[0],
        .hwAttrs     = &spiTivaDMAHWAttrs[0]
    },
    {
        .fxnTablePtr = &SPITivaDMA_fxnTable,
        .object      = &spiTivaDMAObjects[1],
        .hwAttrs     = &spiTivaDMAHWAttrs[1]
    },
    {
        .fxnTablePtr = &SPITivaDMA_fxnTable,
        .object      = &spiTivaDMAObjects[2],
        .hwAttrs     = &spiTivaDMAHWAttrs[2]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== X24015_initSPI ========
 */
void X24015_initSPI(void)
{
	/* Enable SSI2 and SSI3 peripheral devices */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

    /*
     * SSI0 - NORMAL SPEED SPI
     */

    // Enable pin PA4 for SSI0 SSI0XDAT0
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_4);
    // Enable pin PA3 for SSI0 SSI0FSS
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_3);
    // Enable pin PA5 for SSI0 SSI0XDAT1
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5);
    // Enable pin PA2 for SSI0 SSI0CLK
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2);

	/*
     * SSI2 - SINGLE OR QUAD SPEED SPI
     */
    // Enable pin PD2 for SSI2 SSI2FSS
    GPIOPinConfigure(GPIO_PD2_SSI2FSS);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_2);

	// Enable pin PD3 for SSI2 SSI2CLK
	GPIOPinConfigure(GPIO_PD3_SSI2CLK);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3);
#if QUAD_SPEED_SSI2
	/* THESE ARE FOR QUAD SPEED SPI */
	// Enable pin PD6 for SSI2 SSI2XDAT3
	GPIOPinConfigure(GPIO_PD6_SSI2XDAT3);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_6);

	// Enable pin PD7 for SSI2 SSI2XDAT2
	// First open the lock and select the bits we want to modify in the GPIO commit register.
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;

	// Now modify the configuration of the pins that we unlocked.
	GPIOPinConfigure(GPIO_PD7_SSI2XDAT2);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_7);
#endif
	// Enable pin PD1 for SSI2 SSI2XDAT0
	GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_1);
	// Enable pin PD0 for SSI2 SSI2XDAT1
	GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0);

    /*
     * SSI3 - SINGLE OR QUAD SPEED SPI
     */
	// Enable pin PQ1 for SSI3 SSI3FSS
	GPIOPinConfigure(GPIO_PQ1_SSI3FSS);
	GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_1);
    // Enable pin PQ3 for SSI3 SSI3XDAT1
    GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
    GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_3);
    // Enable pin PQ2 for SSI3 SSI3XDAT0
    GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_2);
#if QUAD_SPEED_SSI3
	/* THESE ARE FOR QUAD SPEED SPI */
    // Enable pin PP0 for SSI3 SSI3XDAT2
    GPIOPinConfigure(GPIO_PP0_SSI3XDAT2);
    GPIOPinTypeSSI(GPIO_PORTP_BASE, GPIO_PIN_0);
    // Enable pin PP1 for SSI3 SSI3XDAT3
    GPIOPinConfigure(GPIO_PP1_SSI3XDAT3);
    GPIOPinTypeSSI(GPIO_PORTP_BASE, GPIO_PIN_1);
#endif
    // Enable pin PQ0 for SSI3 SSI3CLK
    GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_0);

    X24015_initDMA();

    SPI_init();
}

/*
 *  =============================== UART ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UART_config, ".const:UART_config")
#pragma DATA_SECTION(uartTivaHWAttrs, ".const:uartTivaHWAttrs")
#endif

#include <ti/drivers/UART.h>
#if TI_DRIVERS_UART_DMA
#include <ti/drivers/uart/UARTTivaDMA.h>

UARTTivaDMA_Object uartTivaObjects[X24015_UARTCOUNT];

const UARTTivaDMA_HWAttrs uartTivaHWAttrs[X24015_UARTCOUNT] = {
    {
        .baseAddr       = UART0_BASE,
        .intNum         = INT_UART0,
        .intPriority    = (~0),
        .rxChannelIndex = UDMA_CH8_UART0RX,
        .txChannelIndex = UDMA_CH9_UART0TX,
    },
    {
        .baseAddr       = UART3_BASE,
        .intNum         = INT_UART3,
        .intPriority    = (~0),
        .rxChannelIndex = UDMA_CH16_UART3RX,
        .txChannelIndex = UDMA_CH16_UART3TX,
    },
    {
        .baseAddr       = UART5_BASE,
        .intNum         = INT_UART5,
        .intPriority    = (~0),
        .rxChannelIndex = UDMA_CH6_UART5RX,
        .txChannelIndex = UDMA_CH7_UART5TX,
    },
    {
        .baseAddr       = UART7_BASE,
        .intNum         = INT_UART7,
        .intPriority    = (~0),
        .rxChannelIndex = UDMA_CH20_UART7RX,
        .txChannelIndex = UDMA_CH21_UART7TX,
    }
};

const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTTivaDMA_fxnTable,
        .object      = &uartTivaObjects[0],
        .hwAttrs     = &uartTivaHWAttrs[0]
    },
    {
        .fxnTablePtr = &UARTTivaDMA_fxnTable,
        .object      = &uartTivaObjects[1],
        .hwAttrs     = &uartTivaHWAttrs[1]
    },
    {
        .fxnTablePtr = &UARTTivaDMA_fxnTable,
        .object      = &uartTivaObjects[2],
        .hwAttrs     = &uartTivaHWAttrs[2]
    },
    {
        .fxnTablePtr = &UARTTivaDMA_fxnTable,
        .object      = &uartTivaObjects[3],
        .hwAttrs     = &uartTivaHWAttrs[3]
    },
    {NULL, NULL, NULL}
};
#else
#include <ti/drivers/uart/UARTTiva.h>

UARTTiva_Object uartTivaObjects[X24015_UARTCOUNT];
unsigned char uartTivaRingBuffer[X24015_UARTCOUNT][32];

/* UART configuration structure */
const UARTTiva_HWAttrs uartTivaHWAttrs[X24015_UARTCOUNT] = {
    {
        .baseAddr    = UART0_BASE,
        .intNum      = INT_UART0,
        .intPriority = (~0),
        .flowControl = UART_FLOWCONTROL_TX|UART_FLOWCONTROL_RX,
        .ringBufPtr  = uartTivaRingBuffer[0],
        .ringBufSize = sizeof(uartTivaRingBuffer[0])
    },
    {
        .baseAddr    = UART3_BASE,
        .intNum      = INT_UART3,
        .intPriority = (~0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer[1],
        .ringBufSize = sizeof(uartTivaRingBuffer[1])
    },
    {
        .baseAddr    = UART5_BASE,
        .intNum      = INT_UART5,
        .intPriority = (~0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer[2],
        .ringBufSize = sizeof(uartTivaRingBuffer[2])
    },
    {
        .baseAddr    = UART7_BASE,
        .intNum      = INT_UART7,
        .intPriority = (~0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer[3],
        .ringBufSize = sizeof(uartTivaRingBuffer[3])
    }
};

const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTTiva_fxnTable,
        .object      = &uartTivaObjects[0],
        .hwAttrs     = &uartTivaHWAttrs[0]
    },
    {
        .fxnTablePtr = &UARTTiva_fxnTable,
        .object      = &uartTivaObjects[1],
        .hwAttrs     = &uartTivaHWAttrs[1]
    },
    {
        .fxnTablePtr = &UARTTiva_fxnTable,
        .object      = &uartTivaObjects[2],
        .hwAttrs     = &uartTivaHWAttrs[2]
    },
    {
        .fxnTablePtr = &UARTTiva_fxnTable,
        .object      = &uartTivaObjects[3],
        .hwAttrs     = &uartTivaHWAttrs[3]
    },
    {NULL, NULL, NULL}
};
#endif /* TI_DRIVERS_UART_DMA */

/*
 *  ======== X24015_initUART ========
 */
void X24015_initUART(void)
{
	/* Enable UART Peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);

    // Enable pin PA0 for UART0 U0RX
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);
    // Enable pin PA1 for UART0 U0TX
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);
    // Enable pin PH0 for UART0 U0RTS
    GPIOPinConfigure(GPIO_PH0_U0RTS);
    GPIOPinTypeUART(GPIO_PORTH_BASE, GPIO_PIN_0);
    // Enable pin PH1 for UART0 U0CTS
    GPIOPinConfigure(GPIO_PH1_U0CTS);
    GPIOPinTypeUART(GPIO_PORTH_BASE, GPIO_PIN_1);

    // Enable pin PJ1 for UART3 U3TX
    GPIOPinConfigure(GPIO_PJ1_U3TX);
    GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_1);
    // Enable pin PP5 for UART3 U3CTS
    GPIOPinConfigure(GPIO_PP5_U3CTS);
    GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_5);
    // Enable pin PJ0 for UART3 U3RX
    GPIOPinConfigure(GPIO_PJ0_U3RX);
    GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_0);
    // Enable pin PP4 for UART3 U3RTS
    GPIOPinConfigure(GPIO_PP4_U3RTS);
    GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_4);

    // Enable pin PC6 for UART5 U5RX
    GPIOPinConfigure(GPIO_PC6_U5RX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6);
    // Enable pin PC7 for UART5 U5TX
    GPIOPinConfigure(GPIO_PC7_U5TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_7);

    // Enable pin PC5 for UART7 U7TX
    GPIOPinConfigure(GPIO_PC5_U7TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_5);
    // Enable pin PC4 for UART7 U7RX
    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4);

    /* Initialize the UART driver */
#if TI_DRIVERS_UART_DMA
    X24015_initDMA();
#endif
    UART_init();
}

/*
 *  =============================== USB ===============================
 */
/*
 *  ======== X24015_initUSB ========
 *  This function just turns on the USB
 */
void X24015_initUSB(X24015_USBMode usbMode)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    /* Enable the USB peripheral and PLL */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
    SysCtlUSBPLLEnable();

    /* Setup pins for USB operation */
    GPIOPinTypeUSBAnalog(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    /* Additional configurations for Host mode */
    if (usbMode == X24015_USBHOST) {
        /* Configure the pins needed */
        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0xff;
        GPIOPinConfigure(GPIO_PD6_USB0EPEN);
        GPIOPinTypeUSBDigital(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

        /*
         *  USB bus fault USB0 USB0PFLT is routed to pin PA7.  We create a Hwi
         *  to allow us to detect power faults and recover gracefully or terminate
         *  the program. PA7 is active low; set the pin as input with a weak pull-up.
         */
        GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
        GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_FALLING_EDGE);
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);

        /* Create a Hwi for PA7 pin. */
        Error_init(&eb);
        Hwi_Params_init(&hwiParams);
        Hwi_construct(&(usbBusFaultHwiStruct), INT_GPIOQ4,
                      X24015_usbBusFaultHwi, &hwiParams, &eb);
        if (Error_check(&eb)) {
            System_abort("Couldn't construct USB bus fault hwi");
        }
    }
}

/*
 *  =============================== USBMSCHFatFs ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(USBMSCHFatFs_config, ".const:USBMSCHFatFs_config")
#pragma DATA_SECTION(usbmschfatfstivaHWAttrs, ".const:usbmschfatfstivaHWAttrs")
#endif

#include <ti/drivers/USBMSCHFatFs.h>
#include <ti/drivers/usbmschfatfs/USBMSCHFatFsTiva.h>

USBMSCHFatFsTiva_Object usbmschfatfstivaObjects[X24015_USBMSCHFatFsCOUNT];

const USBMSCHFatFsTiva_HWAttrs usbmschfatfstivaHWAttrs[X24015_USBMSCHFatFsCOUNT] = {
    {
        .intNum      = INT_USB0,
        .intPriority = (~0)
    }
};

const USBMSCHFatFs_Config USBMSCHFatFs_config[] = {
    {
        .fxnTablePtr = &USBMSCHFatFsTiva_fxnTable,
        .object      = &usbmschfatfstivaObjects[0],
        .hwAttrs     = &usbmschfatfstivaHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== X24015_initUSBMSCHFatFs ========
 */
void X24015_initUSBMSCHFatFs(void)
{
    /* Initialize the DMA control table */
    X24015_initDMA();

    /* Call the USB initialization function for the USB Reference modules */
    X24015_initUSB(X24015_USBHOST);
    USBMSCHFatFs_init();
}

/*
 *  =============================== Watchdog ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(Watchdog_config, ".const:Watchdog_config")
#pragma DATA_SECTION(watchdogTivaHWAttrs, ".const:watchdogTivaHWAttrs")
#endif

#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogTiva.h>

WatchdogTiva_Object watchdogTivaObjects[X24015_WATCHDOGCOUNT];

const WatchdogTiva_HWAttrs watchdogTivaHWAttrs[X24015_WATCHDOGCOUNT] = {
    {
        .baseAddr    = WATCHDOG0_BASE,
        .intNum      = INT_WATCHDOG,
        .intPriority = (~0),
        .reloadValue = 80000000 // 1 second period at default CPU clock freq
    },
};

const Watchdog_Config Watchdog_config[] = {
    {
        .fxnTablePtr = &WatchdogTiva_fxnTable,
        .object      = &watchdogTivaObjects[0],
        .hwAttrs     = &watchdogTivaHWAttrs[0]
    },
    {NULL, NULL, NULL},
};

/*
 *  ======== X24015_initWatchdog ========
 *
 * NOTE: To use the other watchdog timer with base address WATCHDOG1_BASE,
 *       an additional function call may need be made to enable PIOSC. Enabling
 *       WDOG1 does not do this. Enabling another peripheral that uses PIOSC
 *       such as ADC0 or SSI0, however, will do so. Example:
 *
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);
 *
 *       See the following forum post for more information:
 *       http://e2e.ti.com/support/microcontrollers/stellaris_arm_cortex-m3_microcontroller/f/471/p/176487/654390.aspx#654390
 */
void X24015_initWatchdog(void)
{
    /* Enable peripherals used by Watchdog */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    /* Initialize the Watchdog driver */
    Watchdog_init();
}

/*
 *  =============================== ADC ===============================
 */

void X24015_initADC(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable pin PE0 for ADC AIN3
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
    // Enable pin PK2 for ADC AIN18
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_2);
    // Enable pin PK0 for ADC AIN16
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0);
    // Enable pin PK1 for ADC AIN17
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);
    // Enable pin PE3 for ADC AIN0
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    // Enable pin PE1 for ADC AIN2
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
    // Enable pin PE2 for ADC AIN1
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    // Enable pin PK3 for ADC AIN19
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);
}

/*
 *  ============================== TIMER ==============================
 */

void X24015_initTimers(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);

    // Enable pin PM4 for TIMER4 T4CCP0
    GPIOPinConfigure(GPIO_PM4_T4CCP0);
    GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_4);

    // Enable pin PM5 for TIMER4 T4CCP1
    GPIOPinConfigure(GPIO_PM5_T4CCP1);
    GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_5);

    // Enable pin PM7 for TIMER5 T5CCP1
    GPIOPinConfigure(GPIO_PM7_T5CCP1);
    GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_7);

    // Enable pin PM6 for TIMER5 T5CCP0
    GPIOPinConfigure(GPIO_PM6_T5CCP0);
    GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_6);
}
