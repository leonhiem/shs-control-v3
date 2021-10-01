#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "hal_gpio.h"
#include "utils.h"
#include "uart.h"


extern uint8_t i2c_timeout;
#define I2C_TIMEOUT 3 //  3/10 second


void i2c_init(int8_t onoff)
{
    
    if(onoff==0) {
        uart_print(0,"i2c: stop\n\r");
        /* send stop condition */
        //SERCOM2->I2CM.CTRLB.bit.CMD = 0x3;
        //while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY);

        uart_print(0,"i2c: disable\n\r");
        /* SERCOM2 peripheral enabled by setting the ENABLE bit as 1*/ 
        SERCOM2->I2CM.CTRLA.reg &= ~SERCOM_I2CM_CTRLA_ENABLE; 
        /* SERCOM Enable synchronization busy */ 
        while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY);

        PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM2;
        HAL_GPIO_SCC_SDA_in();
        HAL_GPIO_SCC_SDA_pullup();
        HAL_GPIO_SCC_SCL_out();
        uart_print(0,"i2c: off\n\r");
    } else {    
        uart_print(0,"i2c: on\n\r");
        // I2C on SERCOM2    
        HAL_GPIO_SCC_SDA_out();
        HAL_GPIO_SCC_SDA_pmuxen(PORT_PMUX_PMUXE_D_Val); // SDA is PA08 -> pad0
        HAL_GPIO_SCC_SCL_out();
        HAL_GPIO_SCC_SCL_pmuxen(PORT_PMUX_PMUXE_D_Val); // SCL is PA09 -> pad1

        PM->APBCMASK.reg |= PM_APBCMASK_SERCOM2;
        
        GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM2_GCLK_ID_CORE) |
                            GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);    
    
        /* By setting the SPEED bit field as 0x01, I2C Master runs at Fast mode + -  1MHz,   
           By setting the SDAHOLD bit field as 0x02, SDA hold time is configured for 300-600ns,    
           By setting the RUNSTDBY bit as 0x01,Generic clock is enabled in all sleep modes,any interrupt can wake up the device,    
           SERCOM2 is configured as an I2C Master by writing the MODE bitfield as 0x5 
         */
        SERCOM2->I2CM.CTRLA.reg = //SERCOM_I2CM_CTRLA_LOWTOUT |
                                 // SERCOM_I2CM_CTRLA_INACTOUT(0x3) |
                                  SERCOM_I2CM_CTRLA_SDAHOLD(3) | 
                                 // SERCOM_I2CM_CTRLA_RUNSTDBY  | 
                                  SERCOM_I2CM_CTRLA_MODE_I2C_MASTER;

        /* smart mode enabled by setting the bit SMEN as 1 */ 
        SERCOM2->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN; 
        /* synchronization busy */ 
        while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY); 
    
        /* BAUDLOW is non-zero, and baud register is loaded */
        //SERCOM2->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(127) | SERCOM_I2CM_BAUD_BAUDLOW(255);//  ratio 1:2 // 31kHz
        //SERCOM2->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(100) | SERCOM_I2CM_BAUD_BAUDLOW(50);//  15kHz
        //SERCOM2->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(10) | SERCOM_I2CM_BAUD_BAUDLOW(5);


        // according to page 440 of datasheet ATSAMD20:
        // (GCLK=8e6 Hz)
        // assumed Trise=1e-6 s
        //
        // >> 8e6/(10+255+255+8e6*1e-6)
        //ans =    1.5152e+04 

        SERCOM2->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(255) | SERCOM_I2CM_BAUD_BAUDLOW(255);//  15kHz
        
        /* synchronization busy */ 
        while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY); 
    
        /* SERCOM2 peripheral enabled by setting the ENABLE bit as 1*/ 
        SERCOM2->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE; 
        /* SERCOM Enable synchronization busy */ 
        while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY);
    
        /* bus state is forced into idle state */ 
        SERCOM2->I2CM.STATUS.bit.BUSSTATE = 0x1; 
        /* synchronization busy */ 
        while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY); 
    
        /* Both master on bus and slave on bus interrupt is enabled */ 
        SERCOM2->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB; 
        /* SERCOM2 handler enabled */ 
        NVIC_EnableIRQ(SERCOM2_IRQn);
    }    
} 

void i2c_reset(void)
{
    int8_t i;
    uint8_t sda;
    uart_print(0,"i2c: reset\n\r");
    i2c_init(0);

    HAL_GPIO_SCC_SCL_set();
    delay_ms(1);
    sda = HAL_GPIO_SCC_SDA_read();
    if(sda==0) {
        uart_print(0,"i2c: try release SDA: pulse SCL "); // assuming SDA is input
        for(i=0;i<9;i++) {
            HAL_GPIO_SCC_SCL_clr();
            delay_ms(2);
            HAL_GPIO_SCC_SCL_set();
            delay_ms(1);
            sda = HAL_GPIO_SCC_SDA_read();
            delay_ms(1);
            if(sda) break; // slave released SDA
        }
        uart_print(0,"%dx\n\r",i);
    }
    // Send STOP:
    uart_print(0,"i2c: STOP\n\r");
    HAL_GPIO_SCC_SDA_out();
    HAL_GPIO_SCC_SCL_clr();
    HAL_GPIO_SCC_SDA_clr();
    delay_ms(1);
    HAL_GPIO_SCC_SCL_set();
    delay_ms(1);
    HAL_GPIO_SCC_SDA_set();

    i2c_init(1);
}

#define I2C_BUF_SIZE 32
uint8_t i2c_tx_buf[I2C_BUF_SIZE]; 
uint8_t i2c_rx_buf[I2C_BUF_SIZE];
uint8_t i2c_idx;
uint8_t i2c_nof_tx, i2c_nof_rx;
volatile bool i2c_tx_done = false, i2c_rx_done = false; 

void SERCOM2_Handler(void) 
{ 
    /* Master on bus interrupt checking */ 
    if (SERCOM2->I2CM.INTFLAG.bit.MB) {
        if (i2c_idx == i2c_nof_tx) {   
            /* After transferring the last byte stop condition will be sent */
            SERCOM2->I2CM.CTRLB.bit.CMD = 0x3; 
            while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY); 
            i2c_tx_done = true; 
            i2c_idx = 0; 
        } else {
            /* placing the data from transmitting buffer to DATA register*/ 
            SERCOM2->I2CM.DATA.reg = i2c_tx_buf[i2c_idx++]; 
            while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY);
        }
    }
    /* Slave on bus interrupt checking */ 
    if (SERCOM2->I2CM.INTFLAG.bit.SB) {
        if (i2c_idx == (i2c_nof_rx-1)) { 
            /* NACK should be sent before reading the last byte */
            SERCOM2->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT;
            while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY); 
            
            SERCOM2->I2CM.CTRLB.bit.CMD = 0x3;
            while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY); 
            
            i2c_rx_buf[i2c_idx++] = SERCOM2->I2CM.DATA.reg; 
            while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY); 
            i2c_rx_done = true;
        } else {
            SERCOM2->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT; 
            while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY); 
            
            i2c_rx_buf[i2c_idx++] = SERCOM2->I2CM.DATA.reg; 
            while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY); 
            
            /* sending ACK after reading each byte */ 
            SERCOM2->I2CM.CTRLB.bit.CMD = 0x2; 
            while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY); 
        }
    }                   
}
    
int i2c_transact(const uint8_t i2c_slave, const uint8_t *cmd, const uint8_t cmd_len, uint8_t *data, const uint8_t data_len)
{
    i2c_idx = 0;
    i2c_tx_done = false;
    i2c_rx_done = false;
    int ret=data_len;
        
    if(cmd_len > I2C_BUF_SIZE) return -1;
    
    memcpy((void *)i2c_tx_buf,(void *)cmd,cmd_len);
    i2c_nof_tx = cmd_len;
    i2c_nof_rx = data_len;
    
    /* Both master on bus and slave on bus interrupt is enabled */
    SERCOM2->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB;
    
    /* Acknowledge section is set as ACK signal by writing 0 in ACKACT bit */
    SERCOM2->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;
    while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY);
    
    if(cmd_len>0) {          
       
        /* slave address with Write(0) */
        i2c_timeout = I2C_TIMEOUT;
        SERCOM2->I2CM.ADDR.reg = (i2c_slave << 1) | 0;    
        while(!i2c_tx_done) {
            delay_ms(1);
            if(i2c_timeout==0) {            
                ret=-1;
                //uart_print(0,"i2c tx timeout\n\r");
                /* send stop condition */
                SERCOM2->I2CM.CTRLB.bit.CMD = 0x3;
                while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY);
            
                break;
            }        
        }
        if(ret==-1) {        
            /*interrupts are cleared */
            SERCOM2->I2CM.INTENCLR.reg = SERCOM_I2CM_INTENCLR_MB | SERCOM_I2CM_INTENCLR_SB;
            return ret;
        }
    }    
    if(data_len == 0) {
        return ret; // response is not requested
    }

    i2c_idx = 0;
    /* Acknowledge section is set as ACK signal by writing 0 in ACKACT bit */
    SERCOM2->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;
    while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY);
    
    /* slave address with read (1) */
    i2c_timeout = I2C_TIMEOUT;
    SERCOM2->I2CM.ADDR.reg = (i2c_slave << 1) | 1;
    while(!i2c_rx_done) {
        delay_ms(1);
        if(i2c_timeout==0) {
            ret=-1;
            //uart_print(0,"i2c rx timeout\n\r");
            /* send stop condition */
            SERCOM2->I2CM.CTRLB.bit.CMD = 0x3;
            while(SERCOM2->I2CM.STATUS.bit.SYNCBUSY);
            break;
        }
    }        
    /*interrupts are cleared */
    SERCOM2->I2CM.INTENCLR.reg = SERCOM_I2CM_INTENCLR_MB | SERCOM_I2CM_INTENCLR_SB;
    memcpy((void *)data,(void *)i2c_rx_buf,data_len);
    return ret;
}
