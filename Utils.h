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

char* FS_GetErrorStr(int errnum);
void FS_GetDateStr(uint16_t fsdate, char* buf, size_t bufsize);
void FS_GetTimeStr(uint16_t fstime, char* buf, size_t bufsize);
uint32_t FS_GetFatTime(void);

bool RTC_IsRunning(void);
bool RTC_GetDateTime(RTCC_Struct* ts);
bool RTC_SetDateTime(RTCC_Struct* ts);
void RTC_GetTimeStr(RTCC_Struct* ts, char *timestr);
void RTC_GetDateStr(RTCC_Struct* ts, char *datestr);
bool RTC_IsValidTime(RTCC_Struct* ts);
bool RTC_IsValidDate(RTCC_Struct* ts);

void ConfigInitDefaults(SYSCONFIG* p);
int ConfigParamsRead(SYSCONFIG* sp);
int ConfigParamsWrite(SYSCONFIG* sp);

int GetMACAddrStr(char* buf, uint8_t* mac);
int GetSerialNumStr(char* buf, uint8_t* sn);

int ReadGUIDS(I2C_Handle handle, uint8_t ui8SerialNumber[16], uint8_t ui8MAC[6]);

#if (DIV_CLOCK_ENABLED > 0)
void EnableClockDivOutput(uint32_t div);
#endif

#endif /* __UTILS_H */
