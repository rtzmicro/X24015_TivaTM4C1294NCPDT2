/* ============================================================================
 *
 * XMOD Data Capture and Telemetry Systems
 *
 * Copyright (C) 2021, RTZ Microsystems, LLC
 * All Rights Reserved
 *
 * ============================================================================
 *
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
 * ============================================================================ */

/** ============================================================================
 *  @file       AD7799.h
 *
 *  @brief      AD7799 driver interface
 *
 *  The AD7799 header file should be included in an application as follows:
 *  @code
 *  #include <AD7799.h>
 *  @endcode
 *
 *  # Operation #
 *  This example module allows an application to read from/write to a AD7799
 *  device. The full functionality of the device has not be implemented (please
 *  refer to Enhancements below).
 *
 *  The APIs are thread-safe. Two tasks can write/read to the same device safely.
 *  This is accomplished with a GateMutex in the implementation.

 *  ## Creating an instance #
 *  @code
 *  SPI_Params spiParams;
 *  SPI_Handle spiHandle;
 *  AD7799_Params ad7799Params;
 *  AD7799_Object obj;
 *  AD7799_Handle ad7799Handle;
 *  volatile uint8_t ready;
 *
 *  SPI_Params_init(&spiParams);
 *  spiHandle = SPI_open(Board_SPI0, &spiParams);
 *
 *  AD7799_Params_init(&ad7799Params);
 *  ad7799Handle = AD7799_create(spiHandle, Board_CS, &ad7799Params);
 *  if (!handle) {
 *      System_printf("AD7799_create failed");
 *  }
 *  @endcode
 */

/***************************************************************************//**
 *   @file   AD7799.h
 *   @brief  Header file of AD7799 Driver.
 *   @author Bancisor Mihai
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 577
*******************************************************************************/

#ifndef __AD7799_H__
#define __AD7799_H__

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/SPI.h>
#include <ti/sysbios/gates/GateMutex.h>

/******************************************************************************/
/* AD7799                                                                   */
/******************************************************************************/

/*AD7799 Registers*/
#define AD7799_REG_COMM		0   /* Communications Register(WO, 8-bit) */
#define AD7799_REG_STAT	    0   /* Status Register	      (RO, 8-bit) */
#define AD7799_REG_MODE	    1   /* Mode Register	      (RW, 16-bit */
#define AD7799_REG_CONF	    2   /* Configuration Register (RW, 16-bit)*/
#define AD7799_REG_DATA	    3   /* Data Register	      (RO, 16-/24-bit) */
#define AD7799_REG_ID	    4   /* ID Register	     	  (RO, 8-bit) */
#define AD7799_REG_IO	    5   /* IO Register	     	  (RO, 8-bit) */
#define AD7799_REG_OFFSET   6   /* Offset Register	      (RW, 24-bit */
#define AD7799_REG_FULLSALE	7   /* Full-Scale Register	  (RW, 24-bit */

/* Communications Register Bit Designations (AD7799_REG_COMM) */
#define AD7799_COMM_WEN		(1 << 7) 			/* Write Enable */
#define AD7799_COMM_WRITE	(0 << 6) 			/* Write Operation */
#define AD7799_COMM_READ    (1 << 6) 			/* Read Operation */
#define AD7799_COMM_ADDR(x)	(((x) & 0x7) << 3)	/* Register Address */
#define AD7799_COMM_CREAD	(1 << 2) 			/* Continuous Read of Data Register */

/* Status Register Bit Designations (AD7799_REG_STAT) */
#define AD7799_STAT_RDY		(1 << 7)            /* Ready */
#define AD7799_STAT_ERR		(1 << 6)            /* Error (Overrange, Underrange) */
#define AD7799_STAT_CH3		(1 << 2)            /* Channel 3 */
#define AD7799_STAT_CH2		(1 << 1)            /* Channel 2 */
#define AD7799_STAT_CH1		(1 << 0)            /* Channel 1 */

/* Mode Register Bit Designations (AD7799_REG_MODE) */
#define AD7799_MODE_SEL(x)		(((x) & 0x7) << 13)	/* Operation Mode Select */
#define AD7799_MODE_PSW(x)		(1 << 12)			/* Power Switch Control Bit */	
#define AD7799_MODE_RATE(x)		((x) & 0xF) 		/* Filter Update Rate Select */

/* AD7799_MODE_SEL(x) options */
#define AD7799_MODE_CONT		 0  /* Continuous Conversion Mode */
#define AD7799_MODE_SINGLE		 1  /* Single Conversion Mode */
#define AD7799_MODE_IDLE		 2  /* Idle Mode */
#define AD7799_MODE_PWRDN		 3  /* Power-Down Mode */
#define AD7799_MODE_CAL_INT_ZERO 4  /* Internal Zero-Scale Calibration */
#define AD7799_MODE_CAL_INT_FULL 5  /* Internal Full-Scale Calibration */
#define AD7799_MODE_CAL_SYS_ZERO 6  /* System Zero-Scale Calibration */
#define AD7799_MODE_CAL_SYS_FULL 7  /* System Full-Scale Calibration */

/* Configuration Register Bit Designations (AD7799_REG_CONF) */
#define AD7799_CONF_BO_EN	    (1 << 13) 			/* Burnout Current Enable */
#define AD7799_CONF_UNIPOLAR(x) (((x) & 0x1) << 12) /* Unipolar/Bipolar Enable */
#define AD7799_CONF_GAIN(x)	    (((x) & 0x7) << 8) 	/* Gain Select */
#define AD7799_CONF_REFDET(x)   (((x) & 0x1) << 5) 	/* Reference detect function */
#define AD7799_CONF_BUF(x)		(((x) & 0x1)  << 4) /* Buffered Mode Enable */
#define AD7799_CONF_CHAN(x)	    ((x) & 0x7) 		/* Channel select */

/* AD7799_CONF_GAIN(x) options */
#define AD7799_GAIN_1       0
#define AD7799_GAIN_2       1
#define AD7799_GAIN_4       2
#define AD7799_GAIN_8       3
#define AD7799_GAIN_16      4
#define AD7799_GAIN_32      5
#define AD7799_GAIN_64      6
#define AD7799_GAIN_128     7

/* AD7799_CONF_REFDET(x) options */
#define AD7799_REFDET_ENA   1	
#define AD7799_REFDET_DIS   0

#define AD7799_UNIPOLAR_ENA 1
#define AD7799_UNIPOLAR_DIS 0

/* AD7799_CONF_CHAN(x) options */
#define AD7799_CH_AIN1P_AIN1M	0   /* AIN1(+) - AIN1(-) */
#define AD7799_CH_AIN2P_AIN2M	1   /* AIN2(+) - AIN2(-) */
#define AD7799_CH_AIN3P_AIN3M	2   /* AIN3(+) - AIN3(-) */
#define AD7799_CH_AIN1M_AIN1M	3   /* AIN1(-) - AIN1(-) */
#define AD7799_CH_AVDD_MONITOR	7   /* AVDD Monitor */

/* ID Register Bit Designations (AD7799_REG_ID) */
#define AD7799_ID			0x9
#define AD7798_ID           0x8
#define AD7799_ID_MASK		0xF

#define AD7798_WORDSIZE     2       /* 2-bytes for 16-bit */
#define AD7799_WORDSIZE     3       /* 3-bytes for 24-bit */

#define AD7798_FULLSCALE    0xFFFF
#define AD7799_FULLSCALE    0xFFFFFF

/* IO (Excitation Current Sources) Register Bit Designations (AD7799_REG_IO) */
#define AD7799_IOEN			(1 << 6)
#define AD7799_IO1(x)		(((x) & 0x1) << 4)
#define AD7799_IO2(x)		(((x) & 0x1) << 5)

/*!
 *  @brief AD7799 Parameters
 *
 *  This is a place-holder structure now since there are no parameters
 *  for the create/construct calls.
 *
 *  @sa         AD7799_Params_init()
 */
typedef struct AD7799_Params {
    uint8_t dummy;
} AD7799_Params;

/*!
 *  @brief AD7799 Object
 *
 *  The application should never directly access the fields in the structure.
 */
typedef struct AD7799_Object {
    SPI_Handle  spiHandle;      /* SPI handle   */
    uint32_t    gpioCS;         /* chip select  */
    uint8_t     adcWordSize;    /* 16 or 24 bit */
    uint8_t     adcID;          /* 8 or 9 */
} AD7799_Object;

/*!
 *  @brief AD7799 Handle
 *
 *  Used to identify a AD45DB device in the APIs
 */
typedef AD7799_Object *AD7799_Handle;

/*!
 *  @brief  Function to initialize a given AD7799 object
 *
 *  Function to initialize a given AD7799 object specified by the
 *  particular SPI handle and GPIO CS index values.
 *
 *  @param  obj           Pointer to a AD7799_Object structure. It does not
 *                        need to be initialized.
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values. All the fields in this structure are
 *                        RO (read-only).
 *
 *  @return A AD7799_Handle on success or a NULL on an error.
 *
   @sa     AD7799_destruct()
 */
AD7799_Handle AD7799_construct(AD7799_Object *obj,
                               SPI_Handle spiHandle,
                               uint32_t gpioCSIndex,
                               AD7799_Params *params);

/*!
 *  @brief  Function to initialize a given AD7799 device
 *
 *  Function to create a AD7799 object specified by the
 *  particular SPI handle and GPIO CS index values.
 *
 *  @param  spiHandle     SPI handle that the AD7799 is attached to
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values. All the fields in this structure are
 *                        RO (read-only).
 *
 *  @return A AD7799_Handle on success or a NULL on an error.
 *
   @sa     AD7799_delete()
 */
AD7799_Handle AD7799_create(SPI_Handle spiHandle, uint32_t gpioCSIndex, AD7799_Params *params);

/*!
 *  @brief  Function to delete a AD7799 instance
 *
 *  @pre    AD7799_create() had to be called first.
 *
 *  @param  handle      A AD7799_Handle returned from AD7799_create
 *
 *  @sa     AD7799_create()
 */
void AD7799_delete(AD7799_Handle handle);

Void AD7799_Params_init(AD7799_Params *params);

/* Initialize AD7799 and check if the device is present*/
uint8_t AD7799_Init(AD7799_Handle handle);

/* Low level register read */
uint32_t AD7799_GetRegisterValue(AD7799_Handle handle, uint8_t regAddress, uint8_t size);

/* Low level register write */
void AD7799_SetRegisterValue(AD7799_Handle handle, uint8_t regAddress, uint32_t regValue, uint8_t size);

/* Reads /RDY bit of Status register. */
uint8_t AD7799_IsReady(AD7799_Handle handle);

/* Sends 32 consecutive 1's on SPI in order to reset the part. */
void AD7799_Reset(AD7799_Handle handle);

/* Sets the gain of the In-Amp. */
void AD7799_SetGain(AD7799_Handle handle, uint32_t gain);

/* Enables or disables the reference detect function. */
void AD7799_SetRefDetect(AD7799_Handle handle, uint8_t state);

void AD7799_SetUnipolar(AD7799_Handle handle, uint8_t state);

void AD7799_SetBuffer(AD7799_Handle handle, uint8_t state);

/* Selects the channel of AD7799. */
void AD7799_SetChannel(AD7799_Handle handle, uint32_t channel);

/* Sets the operating mode of AD7799. */
void AD7799_SetMode(AD7799_Handle handle, uint32_t mode);

/* Read the 24-bit data register */
uint32_t AD7799_ReadData(AD7799_Handle handle);

uint8_t AD7799_ReadStatus(AD7799_Handle handle);

#endif	// _AD7799_H_
