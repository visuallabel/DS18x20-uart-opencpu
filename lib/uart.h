/**
 * 
 * Based on:
 * 
 * https://github.com/dword1511/onewire-over-uart
 * 
 * Modified for OpenCPU/bc66
 */
#ifndef __UART_H__
#define __UART_H__

#include "ql_uart.h"

#define UART_SUCCESS 0

/* Settings */
/* NOTE: baud rate lower than 9600 / 15200 might not work */
#ifndef BAUD_LOW
#define BAUD_LOW 9600
#endif
#ifndef BAUD_HIGH
#define BAUD_HIGH 115200
#endif

/* UART implemention for different platforms */
int uart_init(/*char *dev_path*/ Enum_SerialPort port);
void uart_finit(void);
void uart_setb(uint32_t baud);
void uart_putc(unsigned char c);
unsigned char uart_getc(void);

#endif /* __UART_H__ */
