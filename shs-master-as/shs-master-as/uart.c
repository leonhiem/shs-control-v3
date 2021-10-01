/*
 * SIM Switch
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   Fri May 10 22:47:24 CEST 2013
 */

#include <stdint.h>
#include <stdio.h>

#include <avr/io.h>

#define F_OSC F_CPU

#include "delay.h"

#include "io.h"
#include "uart.h"



/*
 * Send character c down the UART Tx, wait until tx holding register
 * is empty.
 */
int
uart_putchar(char c, FILE *stream)
{
  if (c == '\n')
    uart_putchar('\r', stream);
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;

  return 0;
}


