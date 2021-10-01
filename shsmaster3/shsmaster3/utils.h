/*
 * Kamworks Codeswitch firmware
 *
 * Author: Leon Hiemstra <l.hiemstra@gmail.com>
 */

#ifndef _UTILS_H
#define _UTILS_H

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

void delay_ms(unsigned long ms);
void delay_us(unsigned long us);
void delay_1us(void);
char is_digit(char c);
int decode_rcode(char *code_str, uint8_t *seq, uint32_t *id, uint16_t *days);

#endif // _UTILS_H
