/*
 * SIM Switch
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "sim.h"
#include "uart.h"
#include "utils.h"
#include "hal_gpio.h"
#include "shsctrl.h"
#include "flash.h"
#include "bt.h"
#include "cmd.h"
#include "ir_receiver.h"

extern sysVals_t sysval;
extern sim_task_queue_t sim_task_queue;
extern uint8_t sim_timeout;
static Eeprom_sim eeprom_sim;


/* UART buffer defines */
#define UART_SIM_RX_BUFFER_SIZE 256 /* power of 2 */
#define UART_SIM_TX_BUFFER_SIZE 512
#define UART_SIM_RX_BUFFER_MASK (UART_SIM_RX_BUFFER_SIZE - 1)

#define UART_SIM_TX_BUFFER_MASK (UART_SIM_TX_BUFFER_SIZE - 1)

#define UART_TERM_RX_BUFFER_SIZE 256 /* power of 2 */
#define UART_TERM_RX_BUFFER_MASK (UART_TERM_RX_BUFFER_SIZE - 1)

/* Static Variables */
static char UART_SIM_RxBuf[UART_SIM_RX_BUFFER_SIZE];
static volatile char UART_SIM_RxHead;
static volatile char UART_SIM_RxTail;
static char UART_SIM_TxBuf[UART_SIM_TX_BUFFER_SIZE];
static volatile char UART_SIM_TxHead;
static volatile char UART_SIM_TxTail;

unsigned char Receive_SIM_Byte(void);
void Transmit_SIM_Byte(unsigned char data);



unsigned char Receive_SIM_Byte(void)
{
    unsigned char tmptail;
    /* Wait for incoming data */
    while (UART_SIM_RxHead == UART_SIM_RxTail);
    /* Calculate buffer index */
    tmptail = (UART_SIM_RxTail + 1) & UART_SIM_RX_BUFFER_MASK;
    /* Store new index */
    UART_SIM_RxTail = tmptail;
    /* Return data */
    return UART_SIM_RxBuf[tmptail];
}

unsigned char Poll_SIM(void)
{
    return (UART_SIM_RxHead != UART_SIM_RxTail);
}

/**
 * UART Data Received Interrupt - Update receive buffer.
 * SIM900
 */
void SERCOM0_Handler(void)
{
   unsigned char data;
   unsigned char tmphead;
   if (SERCOM0->USART.INTFLAG.bit.RXC) {
       data = SERCOM0->USART.DATA.bit.DATA;
       /* Calculate buffer index */ 
       tmphead = (UART_SIM_RxHead + 1) & UART_SIM_RX_BUFFER_MASK;
       /* Store new index */
       UART_SIM_RxHead = tmphead;
       if (tmphead == UART_SIM_RxTail) {
               /* ERROR! Receive buffer overflow */
       }
       /* Store received data in buffer */
       UART_SIM_RxBuf[tmphead] = data;
   }
}

void EIC_Handler(void)
{
    if((EIC->INTFLAG.reg & 5) != 0) { // EXTINT0 | EXTINT2
        EIC->INTFLAG.reg=5;           // clear pin interrupts EXTINT0, EXTINT2
    
#if (MASTER_PCB_v32==0)
        //HAL_GPIO_LED_RED_toggle();        
#endif

        // On (MASTER_PCB_v32==0): RI is on EXTINT[0]
        // On (MASTER_PCB_v32==1): RI is on EXTINT[2]

#if (MASTER_PCB_v32==1)
// FIXME: on v32 it does not seem to work in pin-interrupt (moved to tasks.c)
/*
        // RI line going low:
        if(sim_is_on() && HAL_GPIO_SIM_RI_PIN_read()==0) {
            // URC or SMS message coming in
            sysval.ringing=1;
        }
*/
#endif

        // RC_PIN is on EXTINT00
        if(HAL_GPIO_RC_PIN_read()==0 && irrecv_get_irparams_rcvstate()==STATE_IDLE) {
            TC1->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
            NVIC_EnableIRQ(TC1_IRQn);
#if (MASTER_PCB_v32==0)
            HAL_GPIO_LED_RED_set();
#endif
        }
    }
}

//-----------------------------------------------------------------------------
static void sim_uart_init(int8_t onoff)
{
    uint32_t baud=115200;
    uint64_t br = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU;
  
    HAL_GPIO_SIM_UART_TX_out();
    HAL_GPIO_SIM_UART_TX_pmuxen(PORT_PMUX_PMUXE_C_Val);
    HAL_GPIO_SIM_UART_RX_in();
    HAL_GPIO_SIM_UART_RX_pullup();
    HAL_GPIO_SIM_UART_RX_pmuxen(PORT_PMUX_PMUXE_C_Val);
    
    if(onoff==0) {
        PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM0;
    } else {    
        PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;
  
        GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_CORE) |
                            GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);
  
        SERCOM0->USART.CTRLA.reg =
            SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
            SERCOM_USART_CTRLA_RXPO_PAD3 | SERCOM_USART_CTRLA_TXPO_PAD2;
  
        SERCOM0->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
            SERCOM_USART_CTRLB_CHSIZE(0/*8 bits*/);
  
        SERCOM0->USART.BAUD.reg = (uint16_t)br+1;
  
        // set RX interrupt:
        SERCOM0->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
  
        SERCOM0->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
        NVIC_EnableIRQ(SERCOM0_IRQn);
    }    
}

void sim_ri_init(int8_t onoff)
{
    // setup External pin interrupt on RI line
    HAL_GPIO_RC_PIN_pmuxen(PORT_PMUX_PMUXE_A_Val);
#if (MASTER_PCB_v32==1)
#warning "MASTER_PCB_v32==1"
    HAL_GPIO_SIM_RI_PIN_pmuxen(PORT_PMUX_PMUXE_A_Val);
#endif
    
    HAL_GPIO_SIM_RI_PIN_in();
    HAL_GPIO_SIM_RI_PIN_pullup();
    
    if(onoff==0) {
        PM->APBAMASK.reg &= ~PM_APBAMASK_EIC;
    } else {    
        PM->APBAMASK.reg |= PM_APBAMASK_EIC;
        GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC) |
                            GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);
  
        EIC->CTRL.bit.SWRST=1;
        while (EIC->CTRL.bit.SWRST | EIC->STATUS.bit.SYNCBUSY) { }
        // RI line on (PA00, EXTINT0)
        EIC->EVCTRL.reg = EIC_EVCTRL_EXTINTEO0;
#if (MASTER_PCB_v32==1)
        EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO2;
#endif
        EIC->INTENSET.bit.EXTINT0 = 1;
        EIC->WAKEUP.bit.WAKEUPEN0 = 1;
        EIC->CONFIG[0].reg = 0x8 | 0x2; // filter | interrupt on falling edge (page 296)
  
        EIC->CTRL.bit.ENABLE=1;
        NVIC_EnableIRQ(EIC_IRQn);  
    }      
}

char sim_rxflush(void)
{
    char ch=0;
#ifdef SIM_DEBUG
    uart_print(0,"\n\rrxflush:");
#endif
    sim_timeout=3;
    while(sim_timeout>0) {
        delay_ms(10);
        if(Poll_SIM()) {            
            ch=Receive_SIM_Byte();
#ifdef SIM_DEBUG
            uart_print(0,"[%x]",ch);
#endif
        }
    }
#ifdef SIM_DEBUG
    uart_print(0,"\n\r");
#endif
    return ch;
}

/*
 * returns:
 *  0 if len=0 or 'expect' not found
 * >0 if len>0
 */
uint16_t sim_read(char *sim_bufptr, const char *expect, int timeout_centisec, uint16_t maxlen)
{
    uint16_t len=0;
    char ch=0, found=0;

#ifdef SIM_DEBUG
    uart_print(0,"\n\rsim_read:");
#endif
    sim_timeout=timeout_centisec;
    while(sim_timeout>0) {
        if(len>=(maxlen-1)) break;	 
        if(Poll_SIM()) {           
           ch = Receive_SIM_Byte();			
#ifdef SIM_DEBUG
           if(ch==0) uart_print(0,"\\0");
           else if(ch=='\n') uart_print(0,"\\n");
           else if(ch=='\r') uart_print(0,"\\r");
           else uart_print(0,"%c",ch);
#endif
           sim_bufptr[len]=ch;
           len++;
           sim_bufptr[len]=0;
           if(strstr(sim_bufptr,expect)!=NULL) { found=1; break; }
        } else {
            delay_ms(10);
        }
    }
    if(sim_timeout == 0) {
       uart_print(0,"sim timeout\n\r");
       len=0;
    }
    sim_bufptr[len]=0;
    len=strlen(sim_bufptr);

    /*
     * Response looks like:
     * echo\r\nresponse\r\n\r\nOK\r\n
     *
     * echo=echo of given command
     * response=response of that command
     * OK=status, but can also be something else.
     */
#ifdef SIM_DEBUG
    uart_print(0,"\n\rlen=%d\n\r",len);	
#endif
    if(found) {  
        return len;
    } else {
        return 0;
    }
}

/*
 * returns:
 *  NULL if none in 'expect' is found
 *  *expect string if that one is found
 */
char * sim_read_multi(char *sim_bufptr, const char *expect[], const int nof_expect, int timeout_centisec, uint16_t maxlen)
{
    uint16_t len=0;
    int i;
    char ch=0, found=0;
    char *found_ptr=NULL;

#ifdef SIM_DEBUG
    uart_print(0,"\n\rsim_read_multi:");
#endif
    sim_timeout=timeout_centisec;
    while(sim_timeout>0) {
        if(len>=(maxlen-1)) break;	 
        if(Poll_SIM()) {
            ch = Receive_SIM_Byte();			
#ifdef SIM_DEBUG
            if(ch==0) uart_print(0,"\\0");
            else if(ch=='\n') uart_print(0,"\\n");
            else if(ch=='\r') uart_print(0,"\\r");
            else uart_print(0,"%c",ch);
#endif
            sim_bufptr[len]=ch;
            len++;
            sim_bufptr[len]=0;
            for(i=0;i<nof_expect;i++) {
                if((found_ptr=strstr(sim_bufptr,expect[i]))!=NULL) { found=1; break; }
            }
            if(found) break;
        } else {
            delay_ms(10);
        }
    }			 
    if(sim_timeout == 0) {
       uart_print(0,"sim timeout\n\r");
       found_ptr=NULL;
    }
    return found_ptr;
}

void sim_tx_char(char c)
{
    uint16_t timeout;
    
    timeout=2000;    
    while (!(SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE)) {
        delay_us(10);
        timeout--;
        if(timeout==0) break;
    }      
    
    timeout=2000;           
    while (HAL_GPIO_SIM_CTS_PIN_read()==1) {
       delay_us(10);
       timeout--;
       if(timeout==0) break;
    }  

    SERCOM0->USART.DATA.reg = c;
#ifdef SIM_DEBUG
    if(c==0) uart_print(0,"\\0");
	else if(c=='\n') uart_print(0,"\\n");
	else if(c=='\r') uart_print(0,"\\r");
	else uart_print(0,"%c",c);
#endif
}

void sim_tx(const char *buf, int len)
{
    char c;
    int i;
#ifdef SIM_DEBUG
	uart_print(0,"\n\rsim_write:");
#endif
    for(i=0;i<len;i++) {
        c=buf[i];
        sim_tx_char(c);
    }
}

int phonenr_ok(const char *nr)
{
    int i;
    if(strlen(nr) < SIM_MIN_PHONE_NR_LENGTH) return 0;
    if(strlen(nr) > SIM_MAX_PHONE_NR_LENGTH) return 0;
    //if(nr[0] != '+') return 0;
    for(i=1;i<strlen(nr);i++) {
        if(!isdigit(nr[i])) return 0;
    }
    return 1;
}

void sim_send_sms(sim_task_t *task)
{
    uint8_t next_state=task->state;
    char buf[SIM_MAX_PHONE_NR_LENGTH];
    char sim_rxbuf[256];
    uint8_t len;

    if(!sim_is_on()) {
        task->retries = 5;
        task->state = SIM_TASK_STATE_FINISHED;
    }
    
    if(task->retries > 5) {
        task->state = SIM_TASK_STATE_FINISHED;
        task->retries = 0;
    }
    switch(task->state) {
        case SIM_TASK_STATE_START:
          uart_print(0,"SIM_TASK_STATE_START\n\r");
          uart_print(0,"size=%d len=%d\n\r",task->arg1,strlen(UART_SIM_TxBuf));
          uart_print(0,"sendsms:%s\n\r",UART_SIM_TxBuf);

          if(!phonenr_ok(sysval.sms_phonenr)) {
              task->retries = 5;
              task->state = SIM_TASK_STATE_FINISHED;
              break;
          }
          sim_tx("AT+CMGS=\"",9);
          len=sprintf(buf,"%s\"\r",sysval.sms_phonenr);
          sim_tx(buf,len);

          if(sim_read(sim_rxbuf,">",5,sizeof(sim_rxbuf))==0) {
              uart_print(0,"abort\n\r");
              len=sprintf(buf,"%c",0x1b); // abort by send ESC
              sim_tx(buf,len);
              sim_rxflush();
              task->retries++;
              break;
          }
          sim_tx(UART_SIM_TxBuf,task->arg1); // send prepared sms message
          len=sprintf(buf,"%c",0x1a); // end SMS
          sim_tx(buf,len);
          //sim_rxflush();

          next_state = SIM_TASK_STATE_SENDSMS_WAIT1;
          task->retries = 0;
          break;
        case SIM_TASK_STATE_SENDSMS_WAIT1:
          next_state = SIM_TASK_STATE_SENDSMS_WAIT2;
          break;
        case SIM_TASK_STATE_SENDSMS_WAIT2:
          next_state = SIM_TASK_STATE_SENDSMS_RESPONSE;
          break;
        case SIM_TASK_STATE_SENDSMS_RESPONSE:
          uart_print(0,"SIM_TASK_STATE_SENDSMS_RESPONSE\n\r");
          if(sim_read(sim_rxbuf,"+CMGS",20,sizeof(sim_rxbuf))==0) {
              next_state = SIM_TASK_STATE_SENDSMS_WAIT1;
              task->retries++;
              break;
          }
          strcpy(sysval.sms_phonenr," "); // reset
          task->retries = 0;
          next_state = SIM_TASK_STATE_FINISHED;
          break;
        default:
          next_state = SIM_TASK_STATE_FINISHED;
          uart_print(0,"->SIM_TASK_STATE_FINISHED\n\r");
          break;
    }
    task->state = next_state;
    return;
}


void sim_receive_sms(sim_task_t *task)
{
    uint8_t next_state=task->state;
    char sim_rxbuf[200];
    char *phonenr_ptr=NULL;
    char *sim_bufptr=NULL;
    char *ptr;
    char c;
    uint8_t match;
    int i;

    if(!sim_is_on()) {
        task->retries = 1;
        task->state = SIM_TASK_STATE_FINISHED;
    }
    if(task->retries > 1) {
        task->state = SIM_TASK_STATE_REC_SMS_DEL;
    }
    
    switch(task->state) {
        case SIM_TASK_STATE_START:
          uart_print(0,"SIM_TASK_STATE_START\n\r"); 
          sim_rxflush(); // dump the "SMS Ready" URC message         
          sim_tx("AT+CMGL\r",8);
          next_state = SIM_TASK_STATE_REC_SMS_CHK;
          break;
        case SIM_TASK_STATE_REC_SMS_CHK:
          if(sim_read(sim_rxbuf,"REC UNREAD",7,sizeof(sim_rxbuf))==0) {
              next_state = SIM_TASK_STATE_START;
              task->retries++;
              break;
          }
          if(sim_read(sim_rxbuf,"\r\nOK\r\n",7,sizeof(sim_rxbuf))==0) {
              next_state = SIM_TASK_STATE_START;
              task->retries++;
              break;
          }
          // message response comes in like:
          // first pass:  AT+CMGL\r\r\n+CMGL: 1,"REC UNREAD
          // second pass: ","+31612622133","Leon","13/05/10,13:16:52+08"\r\nSOC?\r\n\r\nOK\r\n
          sim_bufptr=sim_rxbuf;
          match=0;
          while((c=*sim_bufptr) != 0) {
              if(c=='\"') match++;
              sim_bufptr++;
              if(match==2) {
                  match++;
                  uart_print(0,"phonenr: ");
                  phonenr_ptr=sim_bufptr;
              }
              if(match==4) {
                  *(sim_bufptr-1)=0; // eos for phone number
                  sim_bufptr++;
                  uart_print(0,"%s\n\r",phonenr_ptr);
                  break;
              }
          }
          sim_bufptr=strstr(sim_bufptr,"\r\n");
          if(sim_bufptr==NULL) { 
              uart_print(0,"nothing\n\r"); 
              next_state = SIM_TASK_STATE_REC_SMS_DEL;
              break;
          }
          sim_bufptr+=2; // point to message
          // remove "\r\n\r\nOK\r\n" after message:
          ptr=sim_bufptr;
          while((c=*ptr) != 0) {
              if(c=='\r') {
                  *ptr=0;
                  break;
              }
              ptr++;
          }
          uart_print(0,"sms:%s\n\r",sim_bufptr);

          // verify if sender phone number is ok:
          if(phonenr_ok(phonenr_ptr)) {
              uart_print(0,"phone number OK\n\r");
              strcpy(sysval.sms_phonenr,phonenr_ptr);   // Leon's SMART nr        
              /* Process SMS command */
              cmd_task_add(CMD_TASK_SMS,sim_bufptr,0);
          }
          next_state = SIM_TASK_STATE_REC_SMS_DEL;
          break;
        case SIM_TASK_STATE_REC_SMS_DEL:
          sim_delete_sms();
          next_state = SIM_TASK_STATE_FINISHED;
          break;
        default:
          next_state = SIM_TASK_STATE_FINISHED;
          uart_print(0,"->SIM_TASK_STATE_FINISHED\n\r");
          break;
    }
    task->state = next_state;
    return;
}

uint8_t sim_delete_sms(void)
{
    char sim_rxbuf[32];
    uint16_t len;
    sim_tx("AT+CMGD=1\r",10);
    sim_read(sim_rxbuf,"AT+CMGD=1\r",7,sizeof(sim_rxbuf)); // read echo
    len=sim_read(sim_rxbuf,"OK\r\n",7,sizeof(sim_rxbuf));	
    return (uint8_t)len;
}

uint8_t sim_delete_all_sms(void)
{
    char sim_rxbuf[32];
    uint16_t len;
    char *cmd="AT+CMGDA=\"DEL ALL\"\r";
    sim_tx(cmd,strlen(cmd));
    sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
    len=sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf));	
    return (uint8_t)len;
}

char * sim_read_IMEI(char *sim_rxbuf, uint16_t maxlen)
{
    char *ptr_imei,*ptr,c;
    uint8_t retry=5;
    char *cmd="AT+GSN\r";
	
    while(retry) {
        sim_tx(cmd,strlen(cmd));
        sim_read(sim_rxbuf,cmd,5,maxlen); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",5,maxlen)==0) {
            retry--;
            delay_ms(50);
            uart_print(0,"bad imei");
            sim_rxflush();
            continue;
        }

        // skip heading \r\n
        ptr_imei=sim_rxbuf;
        while((c=*ptr_imei) != 0) {
        	if(c=='\r' || c=='\n') {
        		ptr_imei++;
        		continue;
        	} else break;
        }
        // remove "\r\n\r\nOK\r\n" after message:
        ptr=ptr_imei;
        while((c=*ptr) != 0) {
        	if(c=='\r') {
        		*ptr=0;
        		break;
        	}
        	ptr++;
        }
		
        if(strlen(ptr_imei)>10) break;
        else {
            retry--;
            delay_ms(50);
            sim_rxflush();
            continue;
        }
    }
    if(retry==0) return "NO-IMEI";
    else return ptr_imei;
}

int sim_read_signalstrength(void)
{
    char sim_rxbuf[32];
    char *ptr;
    int found,signal,ber;
    char *cmd="AT+CSQ\r";
    
    sim_rxflush();
    sim_tx(cmd,strlen(cmd));
    sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
    if(sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf))==0)
        return -3;

    // response looks like: AT+CSQ\r\r\n+CSQ: 28,0
    ptr=sim_rxbuf;
    found=sscanf(ptr,"\r\n+CSQ: %d,%d",&signal,&ber);    
    if(found==2) {
#ifdef SIM_DEBUG
        uart_print(0,"CSQ:%d\n\r",signal);
#endif
        return signal;
    } else return -4;
}

void sim_check_creg(sim_task_t *task)
{
    uint8_t next_state=task->state;
    int creg=0;

    if(!sim_is_on()) {
        task->retries = 5;
        task->state = SIM_TASK_STATE_FINISHED;
    }
    if(task->retries > 5) {
        task->state = SIM_TASK_STATE_FINISHED;        
    }
    switch(task->state) {
        case SIM_TASK_STATE_START:
          uart_print(0,"SIM_TASK_STATE_START\n\r");
          creg = sim_read_CREG();
          uart_print(0,"creg=%d\n\r",creg);
          if(creg!=1 && creg!=5) { // 1 or 5 is registered
              task->retries = 5;
              next_state = SIM_TASK_STATE_FINISHED;
              break;
          }
          sysval.signalstrength=sim_read_signalstrength();
          next_state = SIM_TASK_STATE_FINISHED;
          break;
        default:
          next_state = SIM_TASK_STATE_FINISHED;
          uart_print(0,"->SIM_TASK_STATE_FINISHED\n\r");
          break;
    }
    task->state = next_state;
    return;
}

int sim_read_location(void)
{
    char sim_rxbuf[80];
    char *ptr;
    int found,stat;
    char *cmd="AT+CLBS=4,1\r";

    uart_print(0,"Location:\n\r");
    sim_tx(cmd,strlen(cmd));
    sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
    if(sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf))==0)
        return -1;

    // response looks like: ... \r\nOK
    uart_print(0,"%s\n\r",sim_rxbuf);
    return 1;
}

char * sim_2g_state(char *sim_rxbuf, uint16_t maxlen)
{
    char *cmd="AT+CIPSTATUS\r";
    sim_tx(cmd,strlen(cmd));    
    sim_read(sim_rxbuf,cmd,20,maxlen); // read echo
    // response looks like: AT+CIPSTATUS\r\r\nOK\r\n\r\nSTATE: IP INITIAL\r\n
    if(sim_read(sim_rxbuf,"STATE: ",20,maxlen)==0)
        return NULL;
    if(sim_read(sim_rxbuf,"\r\n",20,maxlen)==0)
        return NULL;
    else 
        return sim_rxbuf;
}

void sim_start_2g(sim_task_t *task)
{
    uint8_t next_state=task->state;
    char sim_rxbuf[256];
    char buf[LOGIN_LEN+4];
    uint8_t len;
    char *sim_state;
    char *cmd1="AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r";
    char *cmd2="AT+SAPBR=3,1,\"APN\",\"";
    char *cmd3="AT+SAPBR=3,1,\"USER\",\"";
    char *cmd4="AT+SAPBR=3,1,\"PWD\",\"";
    char *cmd5="AT+CSTT=\"";
    char *cmd6="AT+CIICR\r";
    char *cmd7="AT+CIFSR\r";
    char *cmd8="AT+SAPBR=1,1\r";
    char *cmd9_shut="AT+CIPSHUT\r";

    if(!sim_is_on()) {
        task->retries = 5;
        task->state = SIM_TASK_STATE_FINISHED;
    }
    if(task->retries > 5) {
        task->state = SIM_TASK_STATE_FINISHED;        
    }
    switch(task->state) {
        case SIM_TASK_STATE_START:
          uart_print(0,"SIM_TASK_STATE_START\n\r");
          sim_tx(cmd1,strlen(cmd1));
          sim_read(sim_rxbuf,"AT+SAPBR=3,1,",2,sizeof(sim_rxbuf)); // read echo
          if(sim_read(sim_rxbuf,"OK\r\n",2,sizeof(sim_rxbuf))==0) {
              task->retries++;
              break;
          }
          next_state = SIM_TASK_STATE_START2G_GET_SIMSTATE;
          task->retries = 0;
          break;
        case SIM_TASK_STATE_START2G_GET_SIMSTATE:
          uart_print(0,"SIM_TASK_STATE_START2G_GET_SIMSTATE\n\r");
          // find out current state:
          memset(sim_rxbuf,0,sizeof(sim_rxbuf));
          sim_state=sim_2g_state(sim_rxbuf,sizeof(sim_rxbuf));
          uart_print(0,"2g:state=%s\n\r",sim_state);
          if(strncmp(sim_state,"IP INITIAL",10)==0) {
              next_state = SIM_TASK_STATE_START2G_INIT;
          } else if(strncmp(sim_state,"IP START",8)==0) {
              next_state = SIM_TASK_STATE_START2G_START;
          } else if(strncmp(sim_state,"IP GPRSACT",10)==0) {
              next_state = SIM_TASK_STATE_START2G_GPRSACT;
          } else if(strncmp(sim_state,"IP STATUS",9)==0) {
              next_state = SIM_TASK_STATE_START2G_STATUS;
          } else if(strncmp(sim_state,"PDP DEACT",9)==0) {
              next_state = SIM_TASK_STATE_START2G_DEACT;
          } else {
              next_state = SIM_TASK_STATE_FINISHED;
          }
          break;
        case SIM_TASK_STATE_START2G_INIT:
          uart_print(0,"SIM_TASK_STATE_START2G_INIT\n\r");
          sim_tx(cmd2,strlen(cmd2));
          len=sprintf(buf,"%s\"\r",eeprom_sim.apn);
          sim_tx(buf,len);
          sim_read(sim_rxbuf,"AT+SAPBR=3,1,",2,sizeof(sim_rxbuf)); // read echo
          if(sim_read(sim_rxbuf,"OK\r\n",2,sizeof(sim_rxbuf))==0) {
              task->retries++;
              break;
          }

          sim_tx(cmd3,strlen(cmd3));
          len=sprintf(buf,"%s\"\r",eeprom_sim.user);
          sim_tx(buf,len);
          sim_read(sim_rxbuf,"AT+SAPBR=3,1,",2,sizeof(sim_rxbuf)); // read echo
          if(sim_read(sim_rxbuf,"OK\r\n",2,sizeof(sim_rxbuf))==0) {
              task->retries++;
              break;
          }

          sim_tx(cmd4,strlen(cmd4));
          len=sprintf(buf,"%s\"\r",eeprom_sim.pwd);
          sim_tx(buf,len);
          sim_read(sim_rxbuf,"AT+SAPBR=3,1,",2,sizeof(sim_rxbuf)); // read echo
          if(sim_read(sim_rxbuf,"OK\r\n",2,sizeof(sim_rxbuf))==0) {
              task->retries++;
              break;
          }

          sim_tx(cmd5,strlen(cmd5));
          len=sprintf(buf,"%s\",\"",eeprom_sim.apn);
          sim_tx(buf,len);

          len=sprintf(buf,"%s\",\"",eeprom_sim.user);
          sim_tx(buf,len);

          len=sprintf(buf,"%s\"\r",eeprom_sim.pwd);
          sim_tx(buf,len);
          sim_read(sim_rxbuf,"AT+CSTT=",15,sizeof(sim_rxbuf)); // read echo
          if(sim_read(sim_rxbuf,"OK\r\n",15,sizeof(sim_rxbuf))==0) {
              task->retries++;
              break;
          }

          next_state = SIM_TASK_STATE_START2G_GET_SIMSTATE;
          task->retries = 0;
          break;
        case SIM_TASK_STATE_START2G_START:
          uart_print(0,"SIM_TASK_STATE_START2G_START\n\r");
          // bring up wireless connection with GPRS
          sim_tx(cmd6,strlen(cmd6));
          task->arg2=20; // seconds wait
          next_state = SIM_TASK_STATE_START2G_START_WAIT1;
          break;
        case SIM_TASK_STATE_START2G_START_WAIT1:
          uart_print(0,"SIM_TASK_STATE_START2G_START_WAIT1\n\r");
          task->arg2--;
          if(task->arg2 == 0) {
              next_state = SIM_TASK_STATE_START2G_START2;
          }
          break;          
        case SIM_TASK_STATE_START2G_START2:
          uart_print(0,"SIM_TASK_STATE_START2G_START2\n\r");
          //sim_read(sim_rxbuf,cmd6,20,sizeof(sim_rxbuf)); // read echo
          if(sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf))==0) {
              task->retries++;
              next_state = SIM_TASK_STATE_START2G_START;              
              break;
          }
          next_state = SIM_TASK_STATE_START2G_GET_SIMSTATE;
          task->retries = 0;
          break;
        case SIM_TASK_STATE_START2G_GPRSACT:
          uart_print(0,"SIM_TASK_STATE_START2G_GPRSACT\n\r");
          // get an IP address
          sim_tx(cmd7,strlen(cmd7));
          sim_read(sim_rxbuf,cmd7,15,sizeof(sim_rxbuf)); // read echo
          if(sim_read(sim_rxbuf,".",15,sizeof(sim_rxbuf))==0) { // response is the ip address
              task->retries++;
              break;
          }
          sim_rxflush();
          next_state = SIM_TASK_STATE_START2G_GET_SIMSTATE;
          task->retries = 0;
          break;
        case SIM_TASK_STATE_START2G_STATUS:
          uart_print(0,"SIM_TASK_STATE_START2G_STATUS\n\r");
          // open bearer
          sim_tx(cmd8,strlen(cmd8));
          sim_read(sim_rxbuf,cmd8,20,sizeof(sim_rxbuf)); // read echo
          next_state = SIM_TASK_STATE_START2G_STATUS2;
          break;
        case SIM_TASK_STATE_START2G_STATUS2:
          uart_print(0,"SIM_TASK_STATE_START2G_STATUS2\n\r");
          if(sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf))==0) {
              task->retries++;
              next_state = SIM_TASK_STATE_START2G_STATUS;
              break;
          }
          next_state = SIM_TASK_STATE_FINISHED;
          task->retries = 0;
          break;
        case SIM_TASK_STATE_START2G_DEACT:
          uart_print(0,"SIM_TASK_STATE_START2G_DEACT\n\r");
          sim_tx(cmd9_shut,strlen(cmd9_shut));
          sim_read(sim_rxbuf,cmd9_shut,20,sizeof(sim_rxbuf)); // read echo
          delay_ms(200);
          sim_read(sim_rxbuf,"SHUT OK\r\n",40,sizeof(sim_rxbuf));
          task->retries = 5; // does not make sense to send2g message
          next_state = SIM_TASK_STATE_FINISHED;
          break;
        default:
          next_state = SIM_TASK_STATE_FINISHED;
          uart_print(0,"->SIM_TASK_STATE_FINISHED\n\r");
          break;
    }
    task->state = next_state;
    return;
}

void sim_send_2g_msg(sim_task_t *task)
{
    uint8_t next_state=task->state;
    char sim_rxbuf[384];
    char *cmd1_init     ="AT+HTTPINIT\r";
    char *cmd2_term     ="AT+HTTPTERM\r";
    char *cmd3_para_cid ="AT+HTTPPARA=\"CID\",1\r";
    char *cmd4_para_url ="AT+HTTPPARA=\"URL\",\"";
    char *cmd5_action   ="AT+HTTPACTION=0\r";
    char *cmd6_status   ="AT+HTTPSTATUS?\r";
    char *cmd7_httpread ="AT+HTTPREAD=0,255\r";	
 
    if(!sim_is_on()) {
        task->retries = 5;
        task->state = SIM_TASK_STATE_FINISHED;
    }
    
    if(task->retries > 5) {
        task->state = SIM_TASK_STATE_SEND2G_TERM;
        task->retries = 0;
    }
    switch(task->state) {
        case SIM_TASK_STATE_START:
          uart_print(0,"SIM_TASK_STATE_START\n\r");
          sim_tx(cmd1_init,strlen(cmd1_init));
          sim_read(sim_rxbuf,cmd1_init,10,sizeof(sim_rxbuf)); // read echo
          if(sim_read(sim_rxbuf,"OK\r\n",15,sizeof(sim_rxbuf))==0) {
              // reason of failure could be unterminated HTTP. Terminate it:
              sim_rxflush();
              sim_tx(cmd2_term,strlen(cmd2_term));
              sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf));
              task->retries++;
              break;
          }
          //sim_read_location();
          next_state = SIM_TASK_STATE_SEND2G_HTTPPARA_CID;
          task->retries = 0;
          break;
        case SIM_TASK_STATE_SEND2G_HTTPPARA_CID:
          uart_print(0,"SIM_TASK_STATE_SEND2G_HTTPPARA_CID\n\r");
          sim_tx(cmd3_para_cid,strlen(cmd3_para_cid));
          sim_read(sim_rxbuf,"AT+HTTPPARA=",10,sizeof(sim_rxbuf)); // read echo
          if(sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf))==0) {
              task->retries++;
              break;
          }
          next_state = SIM_TASK_STATE_SEND2G_HTTPPARA_URL;
          task->retries = 0;
          break;
        case SIM_TASK_STATE_SEND2G_HTTPPARA_URL:
          uart_print(0,"SIM_TASK_STATE_SEND2G_HTTPPARA_URL\n\r");
          uart_print(0,"send2g:%s\n\r",UART_SIM_TxBuf);
          sim_tx(cmd4_para_url,strlen(cmd4_para_url));
          sim_tx(UART_SIM_TxBuf,task->arg1); // send prepared command+msg
          sim_tx("\"\r",2); // end of command
          uart_print(0,"send2g:sent!\n\r");
          sim_read(sim_rxbuf,"AT+HTTPPARA=",20,sizeof(sim_rxbuf)); // read echo
          uart_print(0,"echo rec\n\r");
          if(sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf))==0) {
              task->retries++;
              break;
          }
          next_state = SIM_TASK_STATE_SEND2G_HTTPACTION;
          task->retries = 0;
          break;
        case SIM_TASK_STATE_SEND2G_HTTPACTION:
          uart_print(0,"SIM_TASK_STATE_SEND2G_HTTPACTION\n\r");
          sim_tx(cmd5_action,strlen(cmd5_action));
          task->arg2=20; // seconds wait  
          next_state = SIM_TASK_STATE_SEND2G_HTTPACTION_WAIT1;
          break;
        case SIM_TASK_STATE_SEND2G_HTTPACTION_WAIT1:
          uart_print(0,"SIM_TASK_STATE_SEND2G_HTTPACTION_WAIT1\n\r");
          task->arg2--;
          if(task->arg2 == 0) {
              next_state = SIM_TASK_STATE_SEND2G_HTTPACTION2;
          }         
          break;       
        case SIM_TASK_STATE_SEND2G_HTTPACTION2:       
          uart_print(0,"SIM_TASK_STATE_SEND2G_HTTPACTION2\n\r");
          //sim_read(sim_rxbuf,cmd5_action,20,sizeof(sim_rxbuf)); // read echo
          if(sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf))==0) {
              task->retries++;
              next_state = SIM_TASK_STATE_SEND2G_HTTPACTION;
              break;
          }       
          next_state = SIM_TASK_STATE_SEND2G_HTTPREAD;
          task->retries = 0;
          break;          
        case SIM_TASK_STATE_SEND2G_HTTPREAD:
          uart_print(0,"SIM_TASK_STATE_SEND2G_HTTPREAD\n\r");
          sim_tx(cmd7_httpread,strlen(cmd7_httpread));
          sim_read(sim_rxbuf,cmd7_httpread,20,sizeof(sim_rxbuf)); // read echo
          sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf));
          sim_rxflush();
          next_state = SIM_TASK_STATE_SEND2G_HTTPSTATUS;
          task->retries = 0;
          break;                
        case SIM_TASK_STATE_SEND2G_HTTPSTATUS:
          uart_print(0,"SIM_TASK_STATE_SEND2G_HTTPSTATUS\n\r");
          sim_tx(cmd6_status,strlen(cmd6_status));
          next_state = SIM_TASK_STATE_SEND2G_HTTPSTATUS_WAIT1;
          break;
        case SIM_TASK_STATE_SEND2G_HTTPSTATUS_WAIT1:
          uart_print(0,"SIM_TASK_STATE_SEND2G_HTTPSTATUS_WAIT1\n\r");
          next_state = SIM_TASK_STATE_SEND2G_HTTPSTATUS_WAIT2;
          break;  
        case SIM_TASK_STATE_SEND2G_HTTPSTATUS_WAIT2:
          uart_print(0,"SIM_TASK_STATE_SEND2G_HTTPSTATUS_WAIT2\n\r");          
          next_state = SIM_TASK_STATE_SEND2G_HTTPSTATUS2;
          break;
        case SIM_TASK_STATE_SEND2G_HTTPSTATUS2:
          uart_print(0,"SIM_TASK_STATE_SEND2G_HTTPSTATUS2\n\r");
          // response is: AT+HTTPSTATUS?\r\r\n+HTTPSTATUS: GET,0,0,0\r\n\r\nOK\r\n
          sim_read(sim_rxbuf,cmd6_status,20,sizeof(sim_rxbuf)); // read echo
          if(sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf))==0) {
              task->retries++;
              next_state = SIM_TASK_STATE_SEND2G_HTTPSTATUS;
              break;
          }
          if(strstr(sim_rxbuf,"GET,0,0,0")==NULL) { // response should become 0,0,0
              task->retries++;
              next_state = SIM_TASK_STATE_SEND2G_HTTPSTATUS;
              break;
          }
          next_state = SIM_TASK_STATE_SEND2G_TERM;
          task->retries = 0;
          break;
        case SIM_TASK_STATE_SEND2G_TERM:
          uart_print(0,"SIM_TASK_STATE_SEND2G_TERM\n\r");
          sim_rxflush();
          sim_tx(cmd2_term,strlen(cmd2_term));
          sim_read(sim_rxbuf,cmd2_term,10,sizeof(sim_rxbuf)); // read echo
          sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf));
          next_state = SIM_TASK_STATE_FINISHED;
          break;
        default:
          next_state = SIM_TASK_STATE_FINISHED;
          uart_print(0,"->SIM_TASK_STATE_FINISHED\n\r");
          break;
    }
    task->state = next_state;
    return;
}

void sim_wait(sim_task_t *task)
{
    uint8_t next_state=task->state;
    switch(task->state) {
        case SIM_TASK_STATE_START:
          uart_print(0,"SIM_TASK_STATE_START\n\r");
          task->arg1--;
          if(task->arg1 == 0) {
              next_state = SIM_TASK_STATE_FINISHED;
          } else {
              uart_print(0,"wait %d more seconds\n\r",task->arg1);
          }
          break;
        default:
          next_state = SIM_TASK_STATE_FINISHED;
          uart_print(0,"->SIM_TASK_STATE_FINISHED\n\r");
          break;
    }
    task->state = next_state;
    return;
}

void sim_stop_2g(sim_task_t *task)
{
    uint8_t next_state=task->state;
    char sim_rxbuf[64];
    char *cmd1_sapbr="AT+SAPBR=0,1\r";
    char *cmd2_shut="AT+CIPSHUT\r";

    if(!sim_is_on()) {
        task->state = SIM_TASK_STATE_FINISHED;
    }
    if(task->retries > 5) {
        task->state = SIM_TASK_STATE_STOP2G_TERM;
        task->retries = 0;
    }
    switch(task->state) {
        case SIM_TASK_STATE_START:
          uart_print(0,"SIM_TASK_STATE_START\n\r");
          sim_tx(cmd1_sapbr,strlen(cmd1_sapbr));
          sim_read(sim_rxbuf,cmd1_sapbr,10,sizeof(sim_rxbuf)); // read echo
          next_state = SIM_TASK_STATE_STOP2G_WAIT;
          break;
        case SIM_TASK_STATE_STOP2G_WAIT:
          uart_print(0,"SIM_TASK_STATE_STOP2G_WAIT\n\r");
          next_state = SIM_TASK_STATE_STOP2G_SAPBR_RESPONSE;
          break;
        case SIM_TASK_STATE_STOP2G_SAPBR_RESPONSE:
          uart_print(0,"SIM_TASK_STATE_STOP2G_SAPBR_RESPONSE\n\r");
          if(sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf)) == 0) {
              task->retries++;
              next_state = SIM_TASK_STATE_START;
              break;
          }
          next_state = SIM_TASK_STATE_STOP2G_TERM;
          task->retries = 0;
          break;
        case SIM_TASK_STATE_STOP2G_TERM:
          uart_print(0,"SIM_TASK_STATE_STOP2G_TERM\n\r");
          sim_tx(cmd2_shut,strlen(cmd2_shut));
          sim_read(sim_rxbuf,cmd2_shut,10,sizeof(sim_rxbuf)); // read echo
          next_state = SIM_TASK_STATE_STOP2G_TERM_WAIT;
          break;
        case SIM_TASK_STATE_STOP2G_TERM_WAIT:
          uart_print(0,"SIM_TASK_STATE_STOP2G_TERM_WAIT\n\r");
          sim_read(sim_rxbuf,"SHUT OK\r\n",10,sizeof(sim_rxbuf));
          next_state = SIM_TASK_STATE_FINISHED;
          break;
        default:
          next_state = SIM_TASK_STATE_FINISHED;
          uart_print(0,"->SIM_TASK_STATE_FINISHED\n\r");
          break;
    }
    task->state = next_state;
    return;
}

int sim_read_CREG(void)
{
    char sim_rxbuf[64];
    char *ptr;
    int found,stat;
    char *cmd="AT+CREG?\r";

    sim_tx(cmd,strlen(cmd));
    sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
    if(sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf))==0)
        return -1;

    // response looks like: +CREG: 0,1\r\nOK
    ptr=sim_rxbuf;
    found=sscanf(ptr,"\r\n+CREG: 0,%d",&stat);
    if(found==1) {
        uart_print(0,"CREG:%d\n\r",stat);
        return stat;
    } else return -1;
}

uint8_t sim_set_flowcontrol(void)
{
    char sim_rxbuf[32];
    char *cmd1="AT+IFC=2,2\r";
    char *cmd2="AT+CSCLK=0\r";
    //char *cmd3="AT&D0\r";
	
    sim_tx(cmd1,strlen(cmd1)); // set hardware flow control
    sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf));

    sim_tx(cmd2,strlen(cmd2)); // do not go to sleep
    return (uint8_t)sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf));

    //sim_tx(cmd3,strlen(cmd3)); // ignore DTR status
    //return (uint8_t)sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf));
}

/*
 * Returns:
 *  0 if error or not found "OK"
 * >0 if found "OK": means success
 */
uint8_t sim_set_textmode(void)
{
    char sim_rxbuf[32];
    char *cmd="AT+CMGF=1\r"; 
    sim_tx(cmd,strlen(cmd)); // select text mode
    return (uint8_t)sim_read(sim_rxbuf,"OK\r\n",5,sizeof(sim_rxbuf));
}

/*
 * Returns:
 *  0 if error or not found "READY"
 * >0 if found "READY": means pincode check OK; no pending pincode
 */
uint8_t sim_pincode_check(void)
{
    char sim_rxbuf[32];
    uint16_t len;
    char *cmd="AT+CPIN?\r";
	
    sim_tx(cmd,strlen(cmd));
    delay_ms(200);
    len=sim_read(sim_rxbuf,"READY",30,sizeof(sim_rxbuf));
    sim_rxflush();
    return (uint8_t)len;
}


uint8_t sim_prepare_data_message(uint8_t sms_only, const uint32_t system_id)
{
    //char sim_rxbuf[64];
    uint8_t len=0;
    
    if(!sms_only) {
        len+=sprintf(&UART_SIM_TxBuf[len],"http://");    
        strcpy(&UART_SIM_TxBuf[len],eeprom_sim.hostname);    
        len=strlen(UART_SIM_TxBuf);

        len+=sprintf(&UART_SIM_TxBuf[len],"/analytics/datapoint_backend/?data=");
    }    
    
    //sprintf(&UART_SIM_TxBuf[len],"%s;",sim_read_IMEI(sim_rxbuf,sizeof(sim_rxbuf)));
    sprintf(&UART_SIM_TxBuf[len],"%09ld;",system_id);    
    len=strlen(UART_SIM_TxBuf);
    
    return len;
}

uint8_t sim_addto_data_message(uint8_t offset, char *str)
{
    strcpy(&UART_SIM_TxBuf[offset],str);
    return strlen(UART_SIM_TxBuf);
}


uint8_t sim_available(void)
{
#if (MASTER_PCB_v32==0)
    return (HAL_GPIO_SIM_AVAIL_PIN_read()==0);
#elif (MASTER_PCB_v32==1)
    uint8_t ret=0;
    HAL_GPIO_SIM_POWERKEY_PIN_in();
    HAL_GPIO_SIM_POWERKEY_PIN_pullup();
    ret = (HAL_GPIO_SIM_POWERKEY_PIN_read()==0);
    HAL_GPIO_SIM_POWERKEY_PIN_out();
    HAL_GPIO_SIM_POWERKEY_PIN_clr();
    return ret;
#endif
}

void sim_power(sim_task_t *task)
{
    uint8_t next_state=task->state;
    char sim_rxbuf[64];
    char *ptr=NULL;

    if(task->retries > 5) {
        task->state = SIM_TASK_STATE_FINISHED;
        return;
    }

    switch(task->state) {
        case SIM_TASK_STATE_START:
          uart_print(0,"SIM_TASK_STATE_START\n\r");
          if(task->arg2 == 0) {
              // turn off powersupply SIM module:
              HAL_GPIO_SIM_ON_PIN_clr();
              next_state = SIM_TASK_STATE_FINISHED;
          } else {
              // turn on powersupply SIM module:
              HAL_GPIO_SIM_ON_PIN_set();
              next_state = SIM_TASK_STATE_POWER_PULSEON;
          }
          break;
        case SIM_TASK_STATE_POWER_PULSEON:
          uart_print(0,"SIM_TASK_STATE_POWER_PULSEON\n\r");
          HAL_GPIO_SIM_POWERKEY_PIN_set(); // press ON
          next_state = SIM_TASK_STATE_POWER_PULSEON_WAIT1;
          break;
        case SIM_TASK_STATE_POWER_PULSEON_WAIT1:
          uart_print(0,"SIM_TASK_STATE_POWER_PULSEON_WAIT\n\r");
          next_state = SIM_TASK_STATE_POWER_PULSEON_WAIT2;
          break;
        case SIM_TASK_STATE_POWER_PULSEON_WAIT2:
          next_state = SIM_TASK_STATE_POWER_PULSEON_WAIT3;
          break;
        case SIM_TASK_STATE_POWER_PULSEON_WAIT3:
          next_state = SIM_TASK_STATE_POWER_PULSEON_WAIT4;
          break;
        case SIM_TASK_STATE_POWER_PULSEON_WAIT4:
          next_state = SIM_TASK_STATE_POWER_PULSEON_WAIT5;
          break;
        case SIM_TASK_STATE_POWER_PULSEON_WAIT5:
          uart_print(0,"SIM_TASK_STATE_POWER_PULSEON_WAIT5\n\r");
          HAL_GPIO_SIM_POWERKEY_PIN_clr(); // release ON
          if(sim_is_on()) {
              sim_tx("AT\r",3); // help sim with autobauding
              task->retries=0;
              next_state = SIM_TASK_STATE_POWER_AUTOBAUD;
          } else {
              task->retries++;
              next_state = SIM_TASK_STATE_POWER_PULSEON;
          }
          break;
        case SIM_TASK_STATE_POWER_AUTOBAUD:
          uart_print(0,"SIM_TASK_STATE_POWER_AUTOBAUD\n\r");
          sim_tx("AT\r",3); // help sim with autobauding
          sim_rxflush();
          next_state = SIM_TASK_STATE_POWER_SET_TEXTMODE;
          break;
        case SIM_TASK_STATE_POWER_SET_TEXTMODE:
          uart_print(0,"SIM_TASK_STATE_POWER_SET_TEXTMODE\n\r");
          sim_set_textmode();
          if(task->arg1 != 0) {
              next_state = SIM_TASK_STATE_POWER_DEL_MSGS;
          } else {
              next_state = SIM_TASK_STATE_POWER_SET_FLOWCTRL;
          }
          break;
        case SIM_TASK_STATE_POWER_DEL_MSGS:
          uart_print(0,"SIM_TASK_STATE_POWER_DEL_MSGS\n\r");
          sim_delete_all_sms();
          next_state = SIM_TASK_STATE_POWER_SET_FLOWCTRL;
          break;
        case SIM_TASK_STATE_POWER_SET_FLOWCTRL:
          uart_print(0,"SIM_TASK_STATE_POWER_SET_FLOWCTRL\n\r");
          sim_set_flowcontrol();
          next_state = SIM_TASK_STATE_POWER_READ_IMEI;
          break;
        case SIM_TASK_STATE_POWER_READ_IMEI:
          uart_print(0,"SIM_TASK_STATE_POWER_READ_IMEI\n\r");
          ptr = sim_read_IMEI(sim_rxbuf, sizeof(sim_rxbuf));
          uart_print(0,"IMEI=%s\n\r",ptr);
          uart_print(0,"Pincheck=%s\n\r",(sim_pincode_check() ? "OK" : "FAIL"));
          if(ptr!=NULL) {
              next_state = SIM_TASK_STATE_FINISHED;
          } else {
              task->retries++;
          }
          break;
        default:
          next_state = SIM_TASK_STATE_FINISHED;
          uart_print(0,"->SIM_TASK_STATE_FINISHED\n\r");
          break;
    }
/*
    if(sim_has_bluetooth()) {
        //sysval.has_bluetooth=1;
        sysval.has_bluetooth=0; // FIXME temporary disable
    }
*/
    task->state = next_state;
    return;
}

uint8_t sim_is_on(void)
{
    return (HAL_GPIO_SIM_STATUS_PIN_read());
}

int8_t sim_task_add(const uint8_t task, const uint16_t arg1, const uint8_t arg2)
{
  int8_t i;
  for(i=0;i<SIM_TASK_QUEUE_LEN;i++) {
    if(sim_task_queue.task[((sim_task_queue.current+i)&SIM_TASK_QUEUE_MASK)].id == SIM_TASK_IDLE) {
      // queue entry is available
      sim_task_queue.task[((sim_task_queue.current+i)&SIM_TASK_QUEUE_MASK)].id = task;
      sim_task_queue.task[((sim_task_queue.current+i)&SIM_TASK_QUEUE_MASK)].state = SIM_TASK_STATE_START;
      sim_task_queue.task[((sim_task_queue.current+i)&SIM_TASK_QUEUE_MASK)].arg1 = arg1;
      sim_task_queue.task[((sim_task_queue.current+i)&SIM_TASK_QUEUE_MASK)].arg2 = arg2;
      sim_task_queue.task[((sim_task_queue.current+i)&SIM_TASK_QUEUE_MASK)].retries = 0;
      uart_print(0,"sim_task_add[%d](%d)\n\r",i,task);
      return 1; // ok
    }
  }
  uart_print(0,"sim_task_add(%d) FAIL\n\r",task);
  return 0; // fail
}

uint8_t sim_task_exist(const uint8_t task)
{
    int8_t i;
    for(i=0;i<SIM_TASK_QUEUE_LEN;i++) {
        if(sim_task_queue.task[i].id == task) {
            uart_print(0,"sim_task_exist=1\n\r");           
            return 1; // ok
        }
    }
    uart_print(0,"sim_task_exist=0\n\r");
    return 0;
}

void sim_task_done(void)
{
    //uart_print(0,"sim_task_done %d\n\r",sim_task_queue.task[sim_task_queue.current].id);
    sim_task_queue.task[sim_task_queue.current].id = SIM_TASK_IDLE;
    sim_task_queue.task[sim_task_queue.current].state = SIM_TASK_STATE_FINISHED;
    sim_task_queue.task[sim_task_queue.current].arg1 = 0;
    sim_task_queue.task[sim_task_queue.current].arg2 = 0;
    sim_task_queue.task[sim_task_queue.current].retries = 0;
    sim_task_queue.current = (sim_task_queue.current+1)&SIM_TASK_QUEUE_MASK; // point to next
}

void sim_tasks(void)
{
    uint8_t tasklist;    
    __disable_irq();
    tasklist=sysval.sim_tasklist;
    sysval.sim_tasklist&=0x7f;
    __enable_irq();

    if((tasklist&0x80)==0) return; // not yet allowed to run
    
    if(sysval.ringing) {
        uart_print(0,"RING\n\r");
        if(!sim_task_exist(SIM_TASK_READ_SMS)) {
            sim_task_add(SIM_TASK_READ_SMS,0,0);
        }
        sysval.ringing=0;
    }
    
    // task allowed to run
    uart_print(0,"sim_task[%d]:\n\r",sim_task_queue.task[sim_task_queue.current].id);

    switch(sim_task_queue.task[sim_task_queue.current].id) {
        case SIM_TASK_POWER:
          uart_print(0,"SIM_TASK_POWER\n\r");
          sim_power(&sim_task_queue.task[sim_task_queue.current]);
          break;
        case SIM_TASK_START_2G:
          uart_print(0,"SIM_TASK_START_2G\n\r");
          sim_start_2g(&sim_task_queue.task[sim_task_queue.current]);
          break;
        case SIM_TASK_STOP_2G:
          uart_print(0,"SIM_TASK_STOP_2G\n\r");
          sim_stop_2g(&sim_task_queue.task[sim_task_queue.current]);
          break;
        case SIM_TASK_SEND_2G:
          uart_print(0,"SIM_TASK_SEND_2G\n\r");
          sim_send_2g_msg(&sim_task_queue.task[sim_task_queue.current]);
          break;
        case SIM_TASK_SEND_SMS:
          uart_print(0,"SIM_TASK_SEND_SMS\n\r");
          sim_send_sms(&sim_task_queue.task[sim_task_queue.current]);
          break;
        case SIM_TASK_WAIT:
          uart_print(0,"SIM_TASK_WAIT\n\r");
          sim_wait(&sim_task_queue.task[sim_task_queue.current]);
          break;
        case SIM_TASK_READ_SMS:
          uart_print(0,"SIM_TASK_READ_SMS\n\r");
          sim_receive_sms(&sim_task_queue.task[sim_task_queue.current]);
          break;
        case SIM_TASK_CREG:
          uart_print(0,"SIM_TASK_CREG\n\r");
          sim_check_creg(&sim_task_queue.task[sim_task_queue.current]);
          break;
    }
    if(sim_task_queue.task[sim_task_queue.current].state == SIM_TASK_STATE_FINISHED) {
        switch(sim_task_queue.task[sim_task_queue.current].id) {
          case SIM_TASK_START_2G:
            if(sim_task_queue.task[sim_task_queue.current].retries < 5) {
                sim_task_add(SIM_TASK_SEND_2G,
                             // pass arg1 (size) from previous task
                             sim_task_queue.task[sim_task_queue.current].arg1,0);
                sim_task_add(SIM_TASK_STOP_2G,0,0);
            } else {
                sim_task_add(SIM_TASK_STOP_2G,0,0);
            }
            break;          
          case SIM_TASK_CREG:
            if(sim_task_queue.task[sim_task_queue.current].retries >= 5) {
                sim_task_add(SIM_TASK_POWER,0,0); // turn off SIM module
                sim_task_add(SIM_TASK_WAIT,30,0); // wait 30 seconds
                sim_task_add(SIM_TASK_POWER,0,1); // then turn on
            }
            break;          
          default:
            break;
        }
        sim_task_done();
    }
}

void sim_init(int8_t onoff)
{
    uint8_t x=0;
    memset((void *)&sim_task_queue,0,sizeof(sim_task_queue_t));
    UART_SIM_RxTail = x;
    UART_SIM_RxHead = x;
    UART_SIM_TxTail = x;
    UART_SIM_TxHead = x;
    
    HAL_GPIO_SIM_POWERKEY_PIN_out();
    HAL_GPIO_SIM_POWERKEY_PIN_clr();
    HAL_GPIO_SIM_ON_PIN_out();
    HAL_GPIO_SIM_ON_PIN_clr();
#if (MASTER_PCB_v32==0)
    HAL_GPIO_SIM_AVAIL_PIN_in();
    HAL_GPIO_SIM_AVAIL_PIN_pullup();
#endif
    HAL_GPIO_SIM_CTS_PIN_in();
    HAL_GPIO_SIM_CTS_PIN_pullup();
    HAL_GPIO_SIM_STATUS_PIN_in();
    HAL_GPIO_SIM_STATUS_PIN_pulldown();
    
    sim_uart_init(onoff);
    sim_ri_init(onoff);
    if(onoff==0) return;
    
    flash_read_eeprom_sim(&eeprom_sim);
    if (eeprom_sim.valid == false) {

#if (TESTING==1)
#warning "Testing on hanuman-stag.kamworks.com"      
        sim_eeprom_set_HOST("hanuman-stag.kamworks.com");
#else
        sim_eeprom_set_HOST("hanuman.kamworks.com");
#endif

#if (SIM_PROVIDER==0) // Cambodia: Cellcard
#warning "SIM_PROVIDER=cellcard Cambodia"
        sim_eeprom_set_APN("cellcard");
        sim_eeprom_set_USER("mobitel");
        sim_eeprom_set_PWD("mobitel"); 
#elif (SIM_PROVIDER==1)  // China
#warning "SIM_PROVIDER=CMNET China"
        sim_eeprom_set_APN("CMNET");
        sim_eeprom_set_USER("");
        sim_eeprom_set_PWD("");
#elif (SIM_PROVIDER==2)  // Holland: KPN
#warning "SIM_PROVIDER=KPN Nederland"
        sim_eeprom_set_APN("portalmmm.nl");
        sim_eeprom_set_USER("");
        sim_eeprom_set_PWD("");
#elif (SIM_PROVIDER==3)  // Holland: AH
#warning "SIM_PROVIDER=AH Nederland"
        sim_eeprom_set_APN("internet");
        sim_eeprom_set_USER("");
        sim_eeprom_set_PWD("");
#elif (SIM_PROVIDER==4)  // Glo 3G
#warning "SIM_PROVIDER=Glo 3G"
        sim_eeprom_set_APN("glo3gvideo");
        sim_eeprom_set_USER("wap");
        sim_eeprom_set_PWD("wap");
#elif (SIM_PROVIDER==5)  // Glo Flat
#warning "SIM_PROVIDER=Glo Flat"
        sim_eeprom_set_APN("gloflat");
        sim_eeprom_set_USER("flat");
        sim_eeprom_set_PWD("flat");
#endif
        eeprom_sim.valid = true;        
        flash_write_eeprom_sim(&eeprom_sim);
    }
}

void sim_eeprom_set_APN(const char *apn)
{
    strcpy(eeprom_sim.apn,apn);    
    flash_write_eeprom_sim(&eeprom_sim);
}
void sim_eeprom_set_USER(const char *user)
{
    strcpy(eeprom_sim.user,user);    
    flash_write_eeprom_sim(&eeprom_sim);
}
void sim_eeprom_set_PWD(const char *pwd)
{
    strcpy(eeprom_sim.pwd,pwd);    
    flash_write_eeprom_sim(&eeprom_sim);
}
void sim_eeprom_set_HOST(const char *host)
{
    strcpy(eeprom_sim.hostname,host);    
    flash_write_eeprom_sim(&eeprom_sim);
}

// --------------------
// Bluetooth functions:
// --------------------
uint8_t sim_has_bluetooth(void)
{
	char sim_rxbuf[60];
	char *ptr;
	char *cmd="AT+CGMR\r"; // read firmware version. ends with "_BT" if has bluetooth
	
	sim_tx(cmd,strlen(cmd));
	delay_ms(200);
	if(sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf))==0)
	    return 0;
	
	ptr=sim_rxbuf;
	if(strstr(ptr,"BT")!=NULL) {
		return 1;
	} else {
		return 0;
	}
}

