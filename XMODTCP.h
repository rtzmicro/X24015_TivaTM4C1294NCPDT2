// =========================================================================
// XMODTCP.h v1.01 04/22/2021
// XMOD Client/Server Network Packet Definitions
// Developed by Robert E. Starr, Jr.
// Copyright (C) 2021, RTZ Microsystems, LLC
// All Rights Reserved
// =========================================================================

#ifndef _XMODTCP_H_
#define _XMODTCP_H_

#pragma once
#pragma pack(push, 8)

#include <stdint.h>

// =========================================================================
// TCP/IP Port Numbers for XMOD TCP server
// =========================================================================

#define XMOD_TCP_PORT       5240    /* xmod tcp port number */

#define XMOD_ADC_CHANNELS   8       /* max ADC channels */

// =========================================================================
// XMOD COMMAND/RESPONSE Messages
// =========================================================================

 /* Command Message Types for 'XMOD_CMD_HDR.command' */

typedef enum XMOD_COMMAND_ID {
    XMOD_CMD_ADC_GET_CONFIG,
    XMOD_CMD_ADC_READ_DATA,
} XMOD_COMMAND_ID;

/* Command Message Header Prefix */

typedef struct _XMOD_CMD_HDR {
    uint16_t        hdrlen;             /* size of this msg structure */
    uint16_t        command;            /* the command ID to execute  */
    uint16_t        status;             /* return status/error code   */
    uint16_t        datalen;            /* trailing payload data len  */
} XMOD_CMD_HDR;

/* XMOD_CMD_ADC_GET_CONFIG - Read the ADC configuration */
typedef struct _XMOD_ADC_GET_CONFIG {
    uint8_t         adc_id;             /* ADC ID 16 or 24 bit        */
    uint8_t         adc_channels;       /* num of ADC channels active */
} XMOD_ADC_GET_CONFIG;

/* XMOD_CMD_ADC_READ_DATA - Read all channels of ADC Data */
typedef struct _XMOD_ADC_READ_DATA {
    uint32_t        adc_data[XMOD_ADC_CHANNELS];
} XMOD_ADC_READ_DATA;

#pragma pack(pop)

#endif /* _XMODTCP_H_ */
