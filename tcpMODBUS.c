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

#define MAX_WORKERS     4
#define PACKET_SIZE     256

static Void tcpModbusWorker(UArg arg0, UArg arg1);

int WriteReply(int fd, uint16_t tid, uint16_t pid, uint8_t uid, uint8_t*data, uint16_t len);

static int ReadData(int fd, void *pbuf, int size, int flags);
static int WriteData(int fd, void *pbuf, int size, int flags);

//*****************************************************************************
// Creates new Task to handle new TCP connections.
//*****************************************************************************

Void tcpModbusHandler(UArg arg0, UArg arg1)
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

    //Task_Handle hSelf = Task_self();
    //fdOpenSession(hSelf);

    if ((server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1)
    {
        System_printf("Error: socket not created.\n");
        goto shutdown;
    }

    memset(&localAddr, 0, sizeof(localAddr));

    localAddr.sin_family      = AF_INET;
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddr.sin_port        = htons(arg0);

    if ((status = bind(server, (struct sockaddr *)&localAddr, sizeof(localAddr))) == -1)
    {
        System_printf("Error: bind failed.\n");
        goto shutdown;
    }

    if ((status = listen(server, MAX_WORKERS)) == -1)
    {
        System_printf("Error: listen failed.\n");
        goto shutdown;
    }

    if (setsockopt(server, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0)
    {
        System_printf("Error: setsockopt failed\n");
        goto shutdown;
    }

    while ((clientfd = accept(server, (struct sockaddr*)&clientAddr, &addrlen)) != -1)
    {
        System_printf("tcpHandler: Creating thread clientfd = %d\n", clientfd);

        /* Init the Error_Block */
        Error_init(&eb);

        /* Initialize the defaults and set the parameters. */
        Task_Params_init(&taskParams);

        taskParams.arg0      = (UArg)clientfd;
        taskParams.priority  = 10;
        taskParams.stackSize = 1280;

        if ((taskHandle = Task_create((Task_FuncPtr)tcpModbusWorker, &taskParams, &eb)) == NULL)
        {
            System_printf("Error: Failed to create new Task\n");
            close(clientfd);
        }

        /* addrlen is a value-result param, must reset for next accept call */
        addrlen = sizeof(clientAddr);
    }

    System_printf("Error: accept failed.\n");

shutdown:
    if (server > 0)
        close(server);

    (void)status;
    (void)taskHandle;

    //fdClose(hSelf);
}

//*****************************************************************************
// Task to handle TCP connection. Can be multiple Tasks running this function.
//*****************************************************************************

Void tcpModbusWorker(UArg arg0, UArg arg1)
{
    int         clientfd = (int)arg0;
    int         count;
    int         bytes;
    uint16_t    data;
    uint16_t    tid;
    uint16_t    pid;
    uint16_t    len;
    uint8_t     uid;

    static uint8_t buffer[PACKET_SIZE];

    /* ----------------------- MBAP Header --------------------------------------*/
    /*
     *
     * <------------------------ MODBUS TCP/IP ADU(1) ------------------------->
     *              <----------- MODBUS PDU (1') ---------------->
     *  +-----------+---------------+------------------------------------------+
     *  | TID | PID | Length | UID  |Code | Data                               |
     *  +-----------+---------------+------------------------------------------+
     *  |     |     |        |      |
     * (2)   (3)   (4)      (5)    (6)
     *
     * (2)  ... MB_TCP_TID          = 0 (Transaction Identifier - 2 Byte)
     * (3)  ... MB_TCP_PID          = 2 (Protocol Identifier - 2 Byte)
     * (4)  ... MB_TCP_LEN          = 4 (Number of bytes - 2 Byte)
     * (5)  ... MB_TCP_UID          = 6 (Unit Identifier - 1 Byte)
     * (6)  ... MB_TCP_FUNC         = 7 (Modbus Function Code)
     *
     * (1)  ... Modbus TCP/IP Application Data Unit
     * (1') ... Modbus Protocol Data Unit
     */

    System_printf("tcpModbusWorker: start clientfd = 0x%x\n", clientfd);
    System_flush();

    while(TRUE)
    {
        /* Read the TID (2-bytes) */
        if ((count = recv(clientfd, (void*)&data, 2, 0)) <= 0)
        {
            System_printf("Rx TID failed\n");
            break;
        }

        tid = ntohs(data);

        /* Read the PID (2 bytes) */
        if ((count = recv(clientfd, (void*)&data, 2, 0)) <= 0)
        {
            System_printf("Rx PID failed\n");
            break;
        }

        pid = ntohs(data);

        /* Read the LENGTH (2 bytes) */
        if ((count = recv(clientfd, (void*)&data, 2, 0)) <= 0)
        {
            System_printf("Rx LEN failed\n");
            break;
        }

        len = ntohs(data);

        /* Read the UID (1 byte) */
        if ((count = recv(clientfd, (void*)&uid, 1, 0)) <= 0)
        {
            System_printf("Rx UID failed\n");
            break;
        }

        if (len < 1)
        {
            System_printf("Bad LEN %d\n", len);
            break;
        }

        /* Calculate the data portion of the modbus packet */

        bytes = len - 1;

        if (bytes > sizeof(buffer))
        {
            System_printf("Bad LEN %d\n", len);
            break;
        }

        /* Read the remaining data portion of the modbus packet */

        if ((count = ReadData(clientfd, buffer, bytes, 0)) <= 0)
        {
            System_printf("Read Data Failed %d\n", len);
            break;
        }

        /* Write the reply response packet */

        if (!WriteReply(clientfd, tid, pid, uid, buffer, (uint16_t)bytes))
        {
            System_printf("Write Reply Data Failed\n");
            break;
        }
    }

    (void)tid;
    (void)pid;
    (void)count;

    System_printf("tcpModbusWorker close clientfd = 0x%x\n", clientfd);
    System_flush();

    close(clientfd);
}

/*
 *
 * <------------------------ MODBUS TCP/IP ADU(1) ------------------------->
 *              <----------- MODBUS PDU (1') ---------------->
 *  +-----------+---------------+------------------------------------------+
 *  | TID | PID | Length | UID  |Code | Data                               |
 *  +-----------+---------------+------------------------------------------+
 *  |     |     |        |      |
 * (2)   (3)   (4)      (5)    (6)
 *
 * (2)  ... MB_TCP_TID          = 0 (Transaction Identifier - 2 Byte)
 * (3)  ... MB_TCP_PID          = 2 (Protocol Identifier - 2 Byte)
 * (4)  ... MB_TCP_LEN          = 4 (Number of bytes - 2 Byte)
 * (5)  ... MB_TCP_UID          = 6 (Unit Identifier - 1 Byte)
 * (6)  ... MB_TCP_FUNC         = 7 (Modbus Function Code)
 *
 * (1)  ... Modbus TCP/IP Application Data Unit
 * (1') ... Modbus Protocol Data Unit
 */

int WriteReply(int fd, uint16_t tid, uint16_t pid, uint8_t uid, uint8_t*data, uint16_t len)
{
    /* Write the TID word */
    if (!WriteData(fd, &tid, 2, 0))
        return 0;

    /* Write the PID word */
    if (!WriteData(fd, &pid, 2, 0))
        return 0;

    /* Write the LEN word */
    if (!WriteData(fd, &len, 2, 0))
        return 0;

    /* Write the UID byte */
    if (!WriteData(fd, &uid, 1, 0))
        return 0;

    /* Write the reply packet data */
    if (!WriteData(fd, data, (int)len, 0))
        return 0;

    return 1;
}


/* This function performs a blocked read for 'size' number of bytes. It will
 * continue to read until all bytes are read, or return if an error occurs.
 */

int ReadData(int fd, void *pbuf, int size, int flags)
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

int WriteData(int fd, void *pbuf, int size, int flags)
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

