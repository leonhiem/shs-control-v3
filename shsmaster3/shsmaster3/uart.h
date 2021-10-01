#ifndef _UART_H
#define _UART_H

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define MSG_INFO  1
#define MSG_DEBUG 2

void uart_init(int8_t onoff);
void uart_putc(char c);
void uart_puts(char *s);
void uart_puts_info(char *s);
void uart_print(const uint8_t ch, const char *fmt, ...);
void uart_set_print_info(bool b);
bool uart_get_print_info(void);

#endif // _UART_H
