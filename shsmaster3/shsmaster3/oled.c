#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "oled.h"
#include "utils.h"
#include "hal_gpio.h"
#include "u8g2.h"
#include "shsctrl.h"
#include "uart.h"


extern sysVals_t sysval;
extern sysTime_t sysTime;
extern uint8_t oled_i2c_timeout;

// Matlab: 95-round(cos(0:(pi/(40-1)):pi)*30)
static char xload[40]={65,65,65,66,67,67,68,70,71,73,74,76,78,80,82,84,87,89,91,94,96,99,101,
                       103,106,108,110,112,114,116,117,119,120,122,123,123,124,125,125,125};
// Matlab: 55-round(sin(0:(pi/(40-1)):pi)*30)
static char yload[40]={55,53,50,48,45,43,41,39,37,35,33,32,30,29,28,27,26,26,25,25,25,25,26,26,27,28,29,30,
                       32,33,35,37,39,41,43,45,48,50,53,55};
  
// Matlab: 95-round(cos(0:(pi/(20-1)):pi)*30)
//static char xload[20]={65,65,67,69,71,75,79,83,88,93,97,102,107,111,115,119,121,123,125,125};
// Matlab: 55-round(sin(0:(pi/(20-1)):pi)*30)
//static char yload[20]={55,50,45,41,37,33,30,28,26,25,25,26,28,30,33,37,41,45,50,55};

static u8g2_t u8g2; // SH1106
static uint8_t oled_update_idx;
static int16_t oled_credit_value;
static uint8_t oled_credit_value_blinking;
static uint8_t oled_soc;
static uint8_t oled_load;
static uint8_t oled_draw_errorsign;
static uint8_t oled_draw_credit_value;
static uint8_t oled_draw_battery;
static uint8_t oled_draw_load;

#define icon_error_25x25_width 25
#define icon_error_25x25_height 25
static unsigned char icon_error_25x25_bits[] = {
	0x00, 0x10, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
	0x00, 0x7c, 0x00, 0x00, 0x00, 0x6c, 0x00, 0x00, 0x00, 0xee, 0x00, 0x00,
	0x00, 0xc6, 0x00, 0x00, 0x00, 0xc7, 0x01, 0x00, 0x00, 0x93, 0x01, 0x00,
	0x80, 0xbb, 0x03, 0x00, 0x80, 0x39, 0x03, 0x00, 0xc0, 0x39, 0x07, 0x00,
	0xe0, 0x38, 0x0e, 0x00, 0xe0, 0x38, 0x0e, 0x00, 0x70, 0x10, 0x1c, 0x00,
	0x70, 0x10, 0x1c, 0x00, 0x38, 0x10, 0x38, 0x00, 0x38, 0x00, 0x38, 0x00,
	0x1c, 0x38, 0x70, 0x00, 0x1c, 0x38, 0x70, 0x00, 0x0e, 0x38, 0xe0, 0x00,
	0x0e, 0x00, 0xe0, 0x00, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0x01,
0xff, 0xff, 0xff, 0x01 };

#define icon_alarm_11x14_width 11
#define icon_alarm_11x14_height 14
static unsigned char icon_alarm_11x14_bits[] = {
	0x20, 0x00, 0x26, 0x03, 0xfb, 0x06, 0x05, 0x05, 0x22, 0x02, 0x21, 0x04,
	0x21, 0x04, 0x21, 0x04, 0x11, 0x04, 0x09, 0x04, 0x02, 0x02, 0x04, 0x01,
0xfc, 0x01, 0x8e, 0x03 };

#define icon_charge_6x12_width 6
#define icon_charge_6x12_height 12
static unsigned char icon_charge_6x12_bits[] = {
0x01, 0x01, 0x01, 0x09, 0x0d, 0x0b, 0x09, 0x08, 0x08, 0x2a, 0x1c, 0x08 };

	

uint8_t u8g2_gpio_and_delay_samd20(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
    switch(msg) {
       	case U8X8_MSG_GPIO_AND_DELAY_INIT: // do this in oled_init()           
       	    break;
       	case U8X8_MSG_DELAY_NANO:
       	    /* not required for SW I2C */
       	    break;
       	case U8X8_MSG_DELAY_10MICRO:
       	    /* not used at the moment */
            break;
       	case U8X8_MSG_DELAY_100NANO:
       	    /* not used at the moment */
       	    break;
       	case U8X8_MSG_DELAY_MILLI:           
       	    delay_ms(arg_int);
       	    break;
       	case U8X8_MSG_DELAY_I2C:  // no need anymore           
       	    break;
       	case U8X8_MSG_GPIO_I2C_CLOCK: // do this in hw callback
       	    break;
       	case U8X8_MSG_GPIO_I2C_DATA: // do this in hw callback
       	    break;
       	default:
       	    u8x8_SetGPIOResult(u8x8, 1);
       	    break;
    }
    return 1;
}

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#define OLED_I2C_TIMEOUT 3 //  3/10 second
#define OLED_I2C_BUF_SIZE 32
uint8_t oled_i2c_tx_buf[OLED_I2C_BUF_SIZE]; 
uint8_t oled_i2c_rx_buf[OLED_I2C_BUF_SIZE];
uint8_t oled_i2c_idx;
uint8_t oled_i2c_nof_tx, oled_i2c_nof_rx;
uint8_t oled_i2c_concat_idx;
volatile bool oled_i2c_tx_done = false, oled_i2c_rx_done = false; 

void SERCOM3_Handler(void) 
{ 
    /* Master on bus interrupt checking */ 
    if (SERCOM3->I2CM.INTFLAG.bit.MB) {
        if (oled_i2c_idx == oled_i2c_nof_tx) {   
            /* After transferring the last byte stop condition will be sent */
            SERCOM3->I2CM.CTRLB.bit.CMD = 0x3; 
            while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY); 
            oled_i2c_tx_done = true; 
            oled_i2c_idx = 0; 
        } else {
            /* placing the data from transmitting buffer to DATA register*/ 
            SERCOM3->I2CM.DATA.reg = oled_i2c_tx_buf[oled_i2c_idx++]; 
            while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY);
        }
    }
    /* Slave on bus interrupt checking */ 
    if (SERCOM3->I2CM.INTFLAG.bit.SB) {
        if (oled_i2c_idx == (oled_i2c_nof_rx-1)) { 
            /* NACK should be sent before reading the last byte */
            SERCOM3->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT;
            while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY); 
            
            SERCOM3->I2CM.CTRLB.bit.CMD = 0x3;
            while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY); 
            
            oled_i2c_rx_buf[oled_i2c_idx++] = SERCOM3->I2CM.DATA.reg; 
            while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY); 
            oled_i2c_rx_done = true;
        } else {
            SERCOM3->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT; 
            while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY); 
            
            oled_i2c_rx_buf[oled_i2c_idx++] = SERCOM3->I2CM.DATA.reg; 
            while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY); 
            
            /* sending ACK after reading each byte */ 
            SERCOM3->I2CM.CTRLB.bit.CMD = 0x2; 
            while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY); 
        }
    }                   
}

void oled_init(int8_t onoff)
{
    HAL_GPIO_OLED_SDA_in();
    HAL_GPIO_OLED_SDA_pullup();
    HAL_GPIO_OLED_SDA_pmuxen(PORT_PMUX_PMUXE_C_Val); // SDA is PA22 -> pad0
    HAL_GPIO_OLED_SCL_in();
    HAL_GPIO_OLED_SCL_pullup();
    HAL_GPIO_OLED_SCL_pmuxen(PORT_PMUX_PMUXE_C_Val); // SCL is PA23 -> pad1

    if(onoff==0) {
        u8g2_SetPowerSave(&u8g2,1);
        PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM3;
    } else {    
        PM->APBCMASK.reg |= PM_APBCMASK_SERCOM3;
        
        GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM3_GCLK_ID_CORE) |
                            GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);    
    
        /* By setting the SPEED bit field as 0x01, I2C Master runs at Fast mode + -  1MHz,   
           By setting the SDAHOLD bit field as 0x02, SDA hold time is configured for 300-600ns,    
           By setting the RUNSTDBY bit as 0x01,Generic clock is enabled in all sleep modes,any interrupt can wake up the device,    
           SERCOM3 is configured as an I2C Master by writing the MODE bitfield as 0x5 
         */
        SERCOM3->I2CM.CTRLA.reg =// SERCOM_I2CM_CTRLA_LOWTOUT |
                                 // SERCOM_I2CM_CTRLA_INACTOUT(0x3) |
                                 // SERCOM_I2CM_CTRLA_SDAHOLD(0x2) | 
                                  SERCOM_I2CM_CTRLA_RUNSTDBY  | 
                                  SERCOM_I2CM_CTRLA_MODE_I2C_MASTER;
        /* smart mode enabled by setting the bit SMEN as 1 */ 
        SERCOM3->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN; 
        /* synchronization busy */ 
        while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY); 
    
        /* BAUDLOW is non-zero, and baud register is loaded */ 
        SERCOM3->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(11) | SERCOM_I2CM_BAUD_BAUDLOW(22); // ratio 1:2 //178kHz (measured)
    
        /* synchronization busy */ 
        while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY); 
    
        /* SERCOM3 peripheral enabled by setting the ENABLE bit as 1*/ 
        SERCOM3->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE; 
        /* SERCOM Enable synchronization busy */ 
        while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY);
    
        /* bus state is forced into idle state */ 
        SERCOM3->I2CM.STATUS.bit.BUSSTATE = 0x1; 
        /* synchronization busy */ 
        while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY); 
    
        /* Both master on bus and slave on bus interrupt is enabled */ 
        SERCOM3->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB; 
        /* SERCOM3 handler enabled */ 
        NVIC_EnableIRQ(SERCOM3_IRQn);

#if (OLED_ROTATE==1)
#warning "OLED 180 degrees rotate"
        u8g2_Setup_sh1106_i2c_128x64_noname_2(&u8g2,U8G2_R2,u8x8_byte_hw_i2c,u8g2_gpio_and_delay_samd20); // 180 degrees rotation
#else
        u8g2_Setup_sh1106_i2c_128x64_noname_2(&u8g2,U8G2_R0,u8x8_byte_hw_i2c,u8g2_gpio_and_delay_samd20); // 0 degrees rotation
#endif

        u8g2_SetI2CAddress(&u8g2,0x78);
        u8g2_InitDisplay(&u8g2);
        u8g2_SetPowerSave(&u8g2,0);
    
        oled_credit_value=0;
        oled_credit_value_blinking=0;
        oled_soc=0;
        oled_load=0;
        oled_update_idx=0;
        oled_draw_battery=0;
        oled_draw_load=0;
        oled_draw_errorsign=0;
        oled_draw_credit_value=0;
    }    
}

void oled_write_number_int(const int16_t value, const uint8_t blinking)
{
    if(value < 0) {
        oled_credit_value=0;
    } else {    
        oled_credit_value=value;
    }    
    oled_credit_value_blinking=blinking;
}

void oled_update(void)
{
    char buf[10];    
    int8_t blink_toggle,i,nof_mods;
    int16_t iload_tmp=0;

    __disable_irq();
    blink_toggle = sysTime.ms>500;
    __enable_irq();
    
    if(oled_update_idx==0) {
        u8g2_FirstPage(&u8g2);
    	oled_update_idx++;
        oled_soc = sysval.SoC/2;

        oled_draw_load=1;
#if(OLED_LOAD_ACGAIN > 1)
#warning "OLED Applying DCloadcurr + ACloadcurr*OLED_LOAD_ACGAIN"
        iload_tmp = sysval.I_load_ac * OLED_LOAD_ACGAIN;
#endif
        iload_tmp += sysval.I_load;
        if(iload_tmp > OLED_MAX_LOAD_CURR_cA) {
            oled_draw_load=blink_toggle;
            oled_load = 39;
        } else {
            oled_load = (uint8_t)(iload_tmp/(OLED_MAX_LOAD_CURR_cA/40));
        }

        if(oled_load > 39) oled_load=39;

#if (SYSTEM_CCNEO_SCC==1)
        nof_mods=sysval.scc_nof_scc;
#else
        nof_mods=4;
#endif
        oled_draw_battery=1;        
        // some battery empty?
        for(i=0;i<nof_mods;i++) {
            uint8_t tmp_battstate = (sysval.batt_state>>(i*2))&0x03;
            uint8_t tmp_sysstate = (sysval.sys_state>>(i*2))&0x03;
            uint8_t tmp_loadstate = (sysval.load_state>>(i*2))&0x03;
            //uart_print(0,"%x %x %x\n\r",tmp_battstate,tmp_sysstate,tmp_loadstate);
            if( (tmp_battstate==0x02 && tmp_sysstate!=0x03) || sysval.SoC<10 || 
                (tmp_loadstate==0x03 && tmp_sysstate!=0x03 && sysval.Vbatt < 1080) ) {
                oled_draw_battery=blink_toggle; break;
            }
        }     
        
        // some SCC initializing?
        for(i=0;i<nof_mods;i++) {            
            uint8_t tmp_sysstate = (sysval.sys_state>>(i*2))&0x03;          
            
            if( tmp_sysstate==0x00 ) {
                oled_draw_battery=0; break;
            }
        }   
        
        oled_draw_errorsign=0;        
        // some SCC has fault state?
        for(i=0;i<nof_mods;i++) {
            uint8_t tmp_sysstate = (sysval.sys_state>>(i*2))&0x03;
            uint8_t tmp_loadstate = (sysval.load_state>>(i*2))&0x03;
            
            if( tmp_loadstate==0x03 && tmp_sysstate!=0x03 && sysval.Vbatt >= 1080) {
                oled_draw_errorsign=blink_toggle;
            }
        }
                
        oled_draw_credit_value=1;                    
        if(oled_credit_value_blinking) {
            oled_draw_credit_value=blink_toggle;            
        }           
        
    } else {
        u8g2_SetFont(&u8g2,u8g2_font_9x15_mn);
        		
        // error sign
        if(oled_draw_errorsign) {
            u8g2_DrawXBM(&u8g2,100,0,icon_error_25x25_width,icon_error_25x25_height,icon_error_25x25_bits);
        }

        if(sysval.I_charge > 10) { // 0.1 A
            u8g2_DrawXBM(&u8g2,12,0,icon_charge_6x12_width,icon_charge_6x12_height,icon_charge_6x12_bits); // charging indicate
        }
        // draw battery
        if(oled_draw_battery) {                       
            u8g2_DrawBox  (&u8g2,10,13,8,3);      // solid pole of battery // was 10,13,10,3
            u8g2_DrawFrame(&u8g2,0, 16, 28,63-16); // frame for battery	// was 0, 16, 30,63-16
            u8g2_DrawBox  (&u8g2,0, 16+(50-oled_soc),28,63-(50-oled_soc)); // solid box representing State Of Charge // was 0, 16+(50-oled_soc),30,63-(50-oled_soc)
        }
        		
        // draw load usage        
        u8g2_DrawCircle(&u8g2,95,55,24,U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
        u8g2_DrawCircle(&u8g2,95,55,25,U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
        u8g2_DrawDisc(&u8g2,95,55,3,U8G2_DRAW_ALL);
        	
        u8g2_DrawLine(&u8g2,95,28,95,33);
        u8g2_DrawLine(&u8g2,95-27,55,95-22,55);
        u8g2_DrawLine(&u8g2,95+27,55,95+22,55);       
        
        
        if(oled_draw_load) {
            u8g2_DrawLine(&u8g2,95,55,xload[oled_load],yload[oled_load]); // load indicator
        }
                		
        // clock picture
        u8g2_DrawXBM(&u8g2,45,0,icon_alarm_11x14_width,icon_alarm_11x14_height,icon_alarm_11x14_bits);
                
        if(oled_draw_credit_value) {		
            sprintf(buf, "%d",oled_credit_value);
            u8g2_DrawStr(&u8g2,60,12,buf); // nr of days
        }
#if (SYSTEM_CCNEO_SCC==1)
        sprintf(buf, "%d",sysval.scc_nof_scc);
        u8g2_DrawStr(&u8g2,30,63,buf); // nr of SCCs
#endif        
        if(u8g2_NextPage(&u8g2)) { oled_update_idx++; } else { oled_update_idx=0; }		
    }
}


uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  //uint8_t *data;
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND: {
        oled_i2c_idx = 0;
        oled_i2c_tx_done = false;
        oled_i2c_rx_done = false;
        if((oled_i2c_concat_idx+arg_int) >= OLED_I2C_BUF_SIZE) {
            uart_print(0,"oled I2C message too long!!\n\r");
            break;
        }
        memcpy((void *)&oled_i2c_tx_buf[oled_i2c_concat_idx],(void *)arg_ptr,arg_int);
        oled_i2c_concat_idx+=arg_int;
        oled_i2c_nof_tx = oled_i2c_concat_idx;
        oled_i2c_nof_rx = 0;
      }
      break;

    case U8X8_MSG_BYTE_INIT:
      // already initialized in oled_init()
      //i2c_hw_init(u8x8);
      break;
    case U8X8_MSG_BYTE_SET_DC:
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      oled_i2c_concat_idx=0;  
      u8x8->i2c_started = 1;
      //i2c_hw_start(u8x8);
      //i2c_hw_write_byte(u8x8, u8x8_GetI2CAddress(u8x8));      
      break;
    case U8X8_MSG_BYTE_END_TRANSFER: { // do complete I2C transaction with collected buffer
                                       // buffer was prepared in case U8X8_MSG_BYTE_SEND
        int ret=0;       
        
        /* Both master on bus and slave on bus interrupt is enabled */
        SERCOM3->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB;
        
        /* Acknowledge section is set as ACK signal by writing 0 in ACKACT bit */
        SERCOM3->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;
        while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY);
        
        /* slave address with Write(0) */
        oled_i2c_timeout = OLED_I2C_TIMEOUT;
        SERCOM3->I2CM.ADDR.reg = (u8x8_GetI2CAddress(u8x8)) | 0;//(u8x8_GetI2CAddress(u8x8) << 1) | 0;
        while(!oled_i2c_tx_done) {
            delay_ms(1);
            if(oled_i2c_timeout==0) {
                ret=-1;
                //uart_print(0,"i2c tx timeout\n\r");
                /* send stop condition */
                SERCOM3->I2CM.CTRLB.bit.CMD = 0x3;
                while(SERCOM3->I2CM.STATUS.bit.SYNCBUSY);
                
                break;
            }
        }
        if(ret==-1) {
            /*interrupts are cleared */
            SERCOM3->I2CM.INTENCLR.reg = SERCOM_I2CM_INTENCLR_MB | SERCOM_I2CM_INTENCLR_SB;
        }
      u8x8->i2c_started = 0;      
      }      
      break;
    default:
      return 0;
  }
  return 1;
}

