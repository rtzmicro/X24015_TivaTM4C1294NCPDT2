//*****************************************************************************
//
// usb_bulk_structs.h - Data structures defining this bulk USB device.
//
// Copyright (c) 2008-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-LM4F232 Firmware Package.
//
//*****************************************************************************

#ifndef _USB_DEVICE_H_
#define _USB_DEVICE_H_

//*****************************************************************************
// Our PIX sub-license for PMX42 allocated to us by TI
// VID: 0x1CBE PID: 0x02e9
//*****************************************************************************

#define USB_PID_X24015      0x02ea

void USB_init(void);

#endif
