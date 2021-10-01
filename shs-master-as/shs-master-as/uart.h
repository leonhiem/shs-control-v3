/*
 * SIM Switch
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   Fri May 10 22:47:24 CEST 2013
 */

#ifndef _UART_H
#define _UART_H

/* UART baud rate */
#define UART_BAUD  9600

#define BRR9600  47
#define U2X9600  0

#define BRR19200  23
#define U2X19200  0
  
#define BRR115200 3
#define U2X115200 0
  
#define BRR230400 1
#define U2X230400 0
  
#define BRR500000 0
#define U2X500000 1


#define INIT_UARTN_8N1_PASTE(n_lit,baud,dblspeed) \
    UBRR ##n_lit## H = (unsigned char)(baud >> 8); \
    UBRR ##n_lit## L = (unsigned char)(baud & 0xFF); \
    UCSR ##n_lit## A = (dblspeed)?(1<<U2X ##n_lit):(0x00); \
    UCSR ##n_lit## B = (1<<TXEN ##n_lit) | (1<<RXEN ##n_lit); \
    UCSR ##n_lit## C = (1<<UCSZ ##n_lit## 1) | (1<<UCSZ ##n_lit## 0);
#define INIT_UARTN_8N1(n,baud,dblspeed) INIT_UARTN_8N1_PASTE(n,baud,dblspeed) //Expand macro args before pasting is done.

  //  UCSR ##n_lit## B = (1<<TXEN ##n_lit) | (1<<RXEN ##n_lit) | (1<<RXCIE ##n_lit) | (1<<TXCIE ##n_lit); 


/*
 * Send one character to the UART.
 */
int uart_putchar(char c, FILE *stream);
unsigned char Poll_TERM(void);
unsigned char Poll_SIM(void);
unsigned char Receive_SIM_Byte(void);

#endif //_UART_H
