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

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
//#include <ti/sysbios/fatfs/ff.h>
#include <inc/hw_ints.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* USB Driver files */
#include <usblib/usblib.h>
#include <usblib/usb-ids.h>
#include <usblib/device/usbdevice.h>
#include <usblib/device/usbdbulk.h>

/* NDK BSD support */
#include <sys/socket.h>

#include <driverlib/sysctl.h>

#include <file.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include "Board.h"
#include "X24015.h"
#include "Utils.h"
#include "usb_device.h"
#include "usb_bulk_structs.h"

// External Global Data Items
extern SYSCONFIG g_sysConfig;
extern SYSDATA g_sysData;

// Local Constants
#define COMMAND_PACKET_RECEIVED     0x00000001
#define COMMAND_STATUS_UPDATE       0x00000002

// Global flag indicating that a USB configuration has been set.
volatile bool g_bUSBConfigured = false;
volatile uint32_t g_ui32TxCount = 0;
volatile uint32_t g_ui32RxCount = 0;
volatile uint32_t g_ui32Flags = 0;
char *g_pcStatus;

static uint32_t ui32RxCount = 0;
static uint32_t ui32TxCount = 0;

static Hwi_Struct usbHwiStruct;

/* Function Prototypes */

uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);
uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);

//*****************************************************************************
// Initialize the USB for device mode
//*****************************************************************************

void USB_init(void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    // Tell the user what we are up to.
    System_printf("Configuring USB\n");

    // Initialize the transmit and receive buffers.
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);

    /* Setup USB0 HWI interrupt handler */
    Error_init(&eb);
    Hwi_Params_init(&hwiParams);
    Hwi_construct(&(usbHwiStruct), INT_USB0, (ti_sysbios_interfaces_IHwi_FuncPtr)USB0DeviceIntHandler, &hwiParams, &eb);
    if (Error_check(&eb)) {
        System_abort("Couldn't construct USB error hwi");
    }

    // Initialize the USB stack for device mode.
    //USBStackModeSet(0, eUSBModeDevice, 0);
    // Forcing device mode so that the VBUS and ID pins are not used or monitored by the USB controller.
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    // Pass our device information to the USB library and place the device
    // on the bus.
    USBDBulkInit(0, &g_sBulkDevice);
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************

uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    ++ui32RxCount;

    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;
            g_pcStatus = "Host connected.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            break;
        }

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
            g_pcStatus = "Host disconn.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            break;
        }

        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            //tUSBDBulkDevice *psDevice;

            //
            // Get a pointer to our instance data from the callback data
            // parameter.
            //
            //psDevice = (tUSBDBulkDevice *)pvCBData;

            //
            // Read the new packet and echo it back to the host.
            //
            //return(EchoNewDataToHost(psDevice, pvMsgData, ui32MsgValue));
            break;
        }

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // Ignore all other events and return 0.
        //
        default:
            break;
    }

    return 0;
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************

uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    ++ui32TxCount;

    //
    // We are not required to do anything in response to any transmit event
    // in this example. All we do is update our transmit counter.
    //
    if(ui32Event == USB_EVENT_TX_COMPLETE)
    {
        g_ui32TxCount += ui32MsgValue;
    }

    //
    // Dump a debug message.
    //
    //DEBUG_PRINT("TX complete %d\n", ui32MsgValue);

    return 0;
}

// End-Of-File
