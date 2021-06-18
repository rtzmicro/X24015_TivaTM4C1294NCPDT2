#ifndef _XMODEM_H_
#define _XMODEM_H_

/* Feature selection. */
#define USE_YMODEM      0       /* enable YMODEM support */

/* Supported modes. */
#define XMODEM_BASIC    1       /* standard 128-byte checksum */
#define XMODEM_CRC      2       /* 128-byte blocks with CRC16 */
#define XMODEM_BATCH    3       /* CRC16 mode with batch info */

/* Interface Functions */

int xmodem_receive(UART_Handle handle, FIL* fp);

#endif /* _XMODEM_H_ */
