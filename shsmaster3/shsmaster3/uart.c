#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "hal_gpio.h"
#include "utils.h"
#include "uart.h"


extern uint8_t serial_rxbuf;
extern bool serial_rxflag;
static bool print_info;
static bool print_debug;


void SERCOM1_Handler(void)
{
    uint8_t c;
    if (SERCOM1->USART.INTFLAG.bit.RXC) {
        c = SERCOM1->USART.DATA.bit.DATA;
        serial_rxbuf = c;
        serial_rxflag = true;
    }
}


void uart_init(int8_t onoff)
{
    uint32_t baud = 115200;
    uint64_t br = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU;
  
    HAL_GPIO_UART_TX_out();
    HAL_GPIO_UART_TX_pmuxen(PORT_PMUX_PMUXE_D_Val);
    HAL_GPIO_UART_RX_in();
    HAL_GPIO_UART_RX_pullup();
    HAL_GPIO_UART_RX_pmuxen(PORT_PMUX_PMUXE_D_Val);
    
    if(onoff==0){
        PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM1;
    } else {    
        PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1;
  
        GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM1_GCLK_ID_CORE) |
                            GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);
  
        SERCOM1->USART.CTRLA.reg =
            SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
            SERCOM_USART_CTRLA_RXPO_PAD3 | SERCOM_USART_CTRLA_TXPO_PAD2;
  
        SERCOM1->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
            SERCOM_USART_CTRLB_CHSIZE(0/*8 bits*/);
  
        SERCOM1->USART.BAUD.reg = (uint16_t)br+1;
  
        // set RX interrupt:
        SERCOM1->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
  
        SERCOM1->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
        NVIC_EnableIRQ(SERCOM1_IRQn);
#if (TESTING==1)
        print_info=true;
        print_debug=true;
#else
        print_info=false;
        print_debug=false;
#endif
    }    
}

//-----------------------------------------------------------------------------
void uart_putc(char c)
{
    while (!(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE));
    SERCOM1->USART.DATA.reg = c;
}

//-----------------------------------------------------------------------------
void uart_puts(char *s)
{
    while (*s) uart_putc(*s++);
}


void uart_puts_info(char *s)
{
    if(print_info) { while (*s) uart_putc(*s++); }
}

void uart_print(const uint8_t ch, const char *fmt, ...)
{
    char buf[200];
    if(ch==MSG_INFO && print_info==false) return;
    if(ch==MSG_DEBUG && print_debug==false) return;
      
    va_list args;
    va_start(args,fmt);
    vsprintf(buf,fmt,args);
    uart_puts(buf);
    va_end(args);
}

bool uart_get_print_info(void)
{
    return print_info;
}

void uart_set_print_info(bool b)
{
    print_info=b;
}