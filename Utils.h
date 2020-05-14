/***************************************************************************
 *
 * XMOD Tiva TM4C1294 Processor Card
 *
 * Copyright (C) RTZ Microsystems, LLC
 * All Rights Reserved
 *
 ***************************************************************************/

#ifndef __UTILS_H
#define __UTILS_H

//*****************************************************************************
// Function Prototypes
//*****************************************************************************

int ReadGUIDS(uint8_t ui8SerialNumber[16], uint8_t ui8MAC[6]);

void ConfigInitDefaults(SYSCONFIG* p);
int ConfigParamsRead(SYSCONFIG* sp);
int ConfigParamsWrite(SYSCONFIG* sp);

int GetHexStr(char* textbuf, uint8_t* databuf, int datalen);

#endif /* __UTILS_H */
