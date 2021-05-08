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
 *    ======== tcpEchoHooks.c ========
 *    Contains non-BSD sockets code (NDK Network Open Hook)
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>

/* NDK BSD support */
#include <sys/socket.h>

#include <file.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* X24015 Board Header file */
#include "X24015.h"
#include "Board.h"
#include "Utils.h"
#include "XMODTCP.h"

#define TCPPACKETSIZE   256
#define MAX_WORKERS     3

#define TCPPORT         1000

#ifdef CYASSL_TIRTOS
#define TCPHANDLERSTACK 8704
#else
#define TCPHANDLERSTACK 1024
#endif

/*** External Data Items ***/

extern SYSDATA g_sys;
extern SYSCONFIG g_cfg;

/*** Prototypes ***/

void netOpenHook(void);

static Void tcpHandler(UArg arg0, UArg arg1);
static Void tcpWorker(UArg arg0, UArg arg1);
static int TcpRecv(int fd, void *pbuf, int size, int flags);
static int TcpSend(int fd, void *pbuf, int size, int flags);

/*** External Function Prototypes ***/
extern void NtIPN2Str(uint32_t IPAddr, char *str);
extern Void tcpModbusHandler(UArg arg0, UArg arg1);

//*****************************************************************************
// This is a hook into the NDK stack to allow delaying execution of the NDK
// stack task until after we load the MAC address from the AT24MAC serial
// EPROM part. This hook blocks on a semaphore until after we're able to call
// Board_initEMAC() in the CommandTaskFxn() below. This mechanism allows us
// to delay execution until we load the MAC from EPROM.
//*****************************************************************************

void netStackBeginHook(void)
{
    Semaphore_pend(g_semaNDKStartup, BIOS_WAIT_FOREVER);
}

//*****************************************************************************
// This handler is called when the DHCP client is assigned an
// address from a DHCP server. We store this in our runtime data
// structure for use later.
//*****************************************************************************

void netIPUpdate(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
    if (fAdd)
        NtIPN2Str(IPAddr, g_sys.ipAddr);
    else
        NtIPN2Str(0, g_sys.ipAddr);

    System_printf("netIPUpdate() dhcp->%s\n", g_sys.ipAddr);
    System_flush();
}


/* This function performs a blocked read for 'size' number of bytes. It will
 * continue to read until all bytes are read, or return if an error occurs.
 */

int TcpRecv(int fd, void *pbuf, int size, int flags)
{
    int bytesRcvd = 0;
    int bytesToRecv = size;

    uint8_t* buf = (uint8_t*)pbuf;

    do {

        if ((bytesRcvd = recv(fd, buf, bytesToRecv, flags)) <= 0)
        {
            System_printf("Error: TCP recv failed %d.\n", bytesRcvd);
            break;
        }

        bytesToRecv -= bytesRcvd;

        buf += bytesRcvd;

    } while(bytesToRecv > 0);

    return bytesRcvd;
}

/* This function performs a blocked write for 'size' number of bytes. It will
 * continue to write until all bytes are sent, or return if an error occurs.
 */

int TcpSend(int fd, void *pbuf, int size, int flags)
{
    int bytesSent = 0;
    int bytesToSend = size;

    uint8_t* buf = (uint8_t*)pbuf;

    do {

        if ((bytesSent = send(fd, buf, bytesToSend, flags)) <= 0)
        {
            System_printf("Error: TCP send failed %d.\n", bytesSent);
            break;
        }

        bytesToSend -= bytesSent;

        buf += bytesSent;

    } while (bytesToSend > 0);

    return bytesSent;
}

//*****************************************************************************
// This handler is called when the network is opened. Start up the
// TCP listener task on the port number passed in the arg list.
//*****************************************************************************

void netOpenHook(void)
{
    Task_Handle taskHandle;
    Task_Params taskParams;
    Error_Block eb;

    /* Make sure Error_Block is initialized */
    Error_init(&eb);

    /* Create the task that listens for incoming TCP connections
     * to handle streaming transport state info. The parameter arg0
     * will be the port that this task listens on.
     */

    Task_Params_init(&taskParams);

    taskParams.stackSize = TCPHANDLERSTACK;
    taskParams.priority  = 1;
    taskParams.arg0      = XMOD_TCP_PORT;

    taskHandle = Task_create((Task_FuncPtr)tcpHandler, &taskParams, &eb);

    //taskHandle = Task_create((Task_FuncPtr)tcpModbusHandler, &taskParams, &eb);

    if (taskHandle == NULL)
        System_printf("netOpenHook: Failed to create tcpStateHandler Task\n");

    System_flush();
}

//*****************************************************************************
// Creates new Task to handle new TCP connections.
//*****************************************************************************

Void tcpHandler(UArg arg0, UArg arg1)
{
    int                status;
    int                clientfd;
    int                server;
    struct sockaddr_in localAddr;
    struct sockaddr_in clientAddr;
    int                optval;
    int                optlen = sizeof(optval);
    socklen_t          addrlen = sizeof(clientAddr);
    Task_Handle        taskHandle;
    Task_Params        taskParams;
    Error_Block        eb;

    Task_Handle hSelf = Task_self();
    fdOpenSession(hSelf);

    server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (server == -1) {
        System_printf("Error: socket not created.\n");
        goto shutdown;
    }

    memset(&localAddr, 0, sizeof(localAddr));

    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddr.sin_port = htons(arg0);

    status = bind(server, (struct sockaddr *)&localAddr, sizeof(localAddr));

    if (status == -1) {
        System_printf("Error: bind failed.\n");
        goto shutdown;
    }

    status = listen(server, MAX_WORKERS);

    if (status == -1) {
        System_printf("Error: listen failed.\n");
        goto shutdown;
    }

    if (setsockopt(server, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0) {
        System_printf("Error: setsockopt failed\n");
        goto shutdown;
    }

    while ((clientfd = accept(server, (struct sockaddr *)&clientAddr, &addrlen)) != -1) {

        System_printf("tcpHandler: Creating thread clientfd = %d\n", clientfd);

        /* Init the Error_Block */
        Error_init(&eb);

        /* Initialize the defaults and set the parameters. */
        Task_Params_init(&taskParams);
        taskParams.arg0 = (UArg)clientfd;
        taskParams.stackSize = 1280;

        taskHandle = Task_create((Task_FuncPtr)tcpWorker, &taskParams, &eb);

        if (taskHandle == NULL) {
            System_printf("Error: Failed to create new Task\n");
            close(clientfd);
        }

        /* addrlen is a value-result param, must reset for next accept call */
        addrlen = sizeof(clientAddr);
    }

    System_printf("Error: accept failed.\n");

shutdown:
    if (server > 0) {
        close(server);
    }

    fdClose(hSelf);
}

//*****************************************************************************
// Task to handle TCP connection. Can be multiple Tasks running this function.
//*****************************************************************************

static int cmd_adc_get_config(XMOD_CMD_HDR* hdr, XMOD_ADC_GET_CONFIG* msg);
static int cmd_adc_read_data(XMOD_CMD_HDR* hdr, XMOD_ADC_READ_DATA* msg);


Void tcpWorker(UArg arg0, UArg arg1)
{
    int  fd = (int)arg0;
    int  bytesRcvd;
    int  bytesSent;
    int  bytesToSend;

    struct packet {
        XMOD_CMD_HDR hdr;
        uint8_t      data[TCPPACKETSIZE];
    } buf;

    System_printf("tcpWorker: start fd = 0x%x\n", fd);

    while (1)
    {
        /* Read the command message header */
        bytesRcvd = TcpRecv(fd, &buf.hdr, sizeof(XMOD_CMD_HDR), 0);

        if (bytesRcvd <= 0)
            break;

        /* Validate the command message header length */
        if (buf.hdr.hdrlen != sizeof(XMOD_CMD_HDR))
        {
            System_printf("tcpWorker: invalid header size %x\n", buf.hdr.hdrlen);
            break;
        }

        /* If any trailing request data after header, read it to */
        if (buf.hdr.datalen)
        {
            if (buf.hdr.datalen > TCPPACKETSIZE)
            {
                System_printf("tcpWorker: packet size overflow! %x\n", buf.hdr.datalen);
                break;
            }

            bytesRcvd = TcpRecv(fd, &buf.data, buf.hdr.datalen, 0);

            if (bytesRcvd <= 0)
                break;
        }

        /* Now validate and process the command request */

        switch(buf.hdr.command)
        {
        case XMOD_CMD_ADC_GET_CONFIG:
            bytesToSend = cmd_adc_get_config(&buf.hdr, (XMOD_ADC_GET_CONFIG*)&buf.data);
            break;

        case XMOD_CMD_ADC_READ_DATA:
            bytesToSend = cmd_adc_read_data(&buf.hdr, (XMOD_ADC_READ_DATA*)&buf.data);
            break;

        default:
            buf.hdr.status = (uint16_t)-1;
            bytesToSend = 0;
            break;
        }

        /* The reply message data size */
        buf.hdr.datalen = bytesToSend;
        buf.hdr.hdrlen  = sizeof(XMOD_CMD_HDR);

        /* Send header and any reply data */
        bytesToSend += sizeof(XMOD_CMD_HDR);

        /* Send the message header and reply data to the client */
        bytesSent = TcpSend(fd, &buf, bytesToSend, 0);

        if (bytesSent < 0 || bytesSent != bytesToSend)
        {
            System_printf("Error: send failed.\n");
            break;
        }
    }

    System_printf("tcpWorker stop fd = 0x%x\n", fd);

    close(fd);
}

//*****************************************************************************
// Client command request handlers
//*****************************************************************************

int cmd_adc_get_config(XMOD_CMD_HDR* hdr, XMOD_ADC_GET_CONFIG* msg)
{
    msg->adc_channels = (uint8_t)g_sys.adcChannels;
    msg->adc_id       = (uint8_t)g_sys.adcID;

    hdr->status = 0;

    return sizeof(XMOD_ADC_GET_CONFIG);
}

int cmd_adc_read_data(XMOD_CMD_HDR* hdr, XMOD_ADC_READ_DATA* msg)
{
    int i;

    for (i=0; i < XMOD_ADC_CHANNELS; i++)
    {
        if (i > g_sys.adcChannels)
            break;

        msg->adc_data[i] = (uint8_t)g_sys.adcData[i];
    }

    hdr->status = 0;

    return sizeof(XMOD_ADC_READ_DATA);
}
