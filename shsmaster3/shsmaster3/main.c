/*
 * Kamworks shsmaster v3 firmware
 *
 * Author: Leon Hiemstra <l.hiemstra@gmail.com>
 *
 * Compiler tool: Atmel Studio 7.0.1417
 *
 * Microcontroller: ATSAMD20
 * Required fuse settings: none (keep default)
 */

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "hal_gpio.h"
#include "utils.h"
#include "rtc.h"
#include "uart.h"
#include "tone.h"
#include "flash.h"
#include "oled.h"
#include "ir_receiver.h"
#include "sim.h"
#include "tasks.h"
#include "scc.h"
#include "shsctrl.h"
#include "code.h"
#include "i2c.h"
#include "cmd.h"
#include "mod.h"
#include "wdt.h"
#include "adc.h"

#ifndef SYSTEM_CCNEO_SCC
#  define SYSTEM_CCNEO_SCC 1
#endif

#ifndef SYSTEM_MODULAR_DC
#  define SYSTEM_MODULAR_DC 0
#endif

#ifndef SYSTEM_MODULAR_AC
#  define SYSTEM_MODULAR_AC 0
#endif

#ifndef SYSTEM_MODULAR_PV
#  define SYSTEM_MODULAR_PV 0
#endif

#ifndef SYSTEM_NOF_MOD_PV
#  define SYSTEM_NOF_MOD_PV 1
#endif

#ifndef SYSTEM_NOF_MOD_AC
#  define SYSTEM_NOF_MOD_AC 1
#endif

#ifndef SYSTEM_NOF_MOD_DC
#  define SYSTEM_NOF_MOD_DC 1
#endif

#if (SYSTEM_CCNEO_SCC==1 && (SYSTEM_MODULAR_DC==1 || SYSTEM_MODULAR_AC==1 || SYSTEM_MODULAR_PV==1))
#  error "No support for SCC and MODULAR together in 1 system"
#endif

#if (SYSTEM_CCNEO_SCC==0 && (SYSTEM_MODULAR_DC==1 || SYSTEM_MODULAR_AC==1 || SYSTEM_MODULAR_PV==1))
#  warning "Using System MODULAR"
#endif

#if (SYSTEM_CCNEO_SCC==1 && SYSTEM_MODULAR_DC==0 && SYSTEM_MODULAR_AC==0 && SYSTEM_MODULAR_PV==0) 
#  warning "Using System CCNEO SCC"
#endif

#ifndef MAX_NOF_SCC
#  define MAX_NOF_SCC 1
#endif

#ifndef MASTER_PCB_v32
#  define MASTER_PCB_v32 0 // disabled
#endif

#ifndef TESTING
#  define TESTING 0 // disabled
#endif

#ifndef SIM_PROVIDER
#  define SIM_PROVIDER 0 // Cambodia cellcard
#endif

#ifndef OLED_MAX_LOAD_CURR_cA
#  define OLED_MAX_LOAD_CURR_cA 1000
#endif

#ifndef OLED_LOAD_ACGAIN
#  define OLED_LOAD_ACGAIN 1
#endif

#ifndef OLED_ROTATE
#  define OLED_ROTATE 0
#endif

#ifndef MOD_VBATT_FULL_cV
#  define MOD_VBATT_FULL_cV 1280
#endif

#ifndef MOD_VBATT_EMPTY_cV
#  define MOD_VBATT_EMPTY_cV 1120
#endif

volatile bool alarmTriggered = false;
volatile bool usart_rx_ready=false;
volatile bool usart_tx_ready=true;
#define RXBUF_SIZE 100
char rxbuf[RXBUF_SIZE];
int data_receive_idx=0;

volatile sysVals_t sysval;
volatile sysTime_t sysTime;
volatile uint8_t serial_rxbuf;
volatile bool serial_rxflag=false;
volatile sim_task_queue_t sim_task_queue;
volatile cmd_task_queue_t cmd_task_queue;
volatile task_queue_t task_queue;
volatile uint8_t bod33_tripped,wdt_wakeup;
uint8_t rcause;

void all_init(int8_t onoff);

void SYSCTRL_Handler(void)
{
    if(SYSCTRL->INTFLAG.bit.BOD33DET) {
        bod33_tripped=1;//=SYSCTRL->INTFLAG.reg;
        //bod33_det=SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_BOD33DET;
        SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33DET; // clear interrupt  
        //HAL_GPIO_KEYPAD6_toggle();
    }    
}

//-----------------------------------------------------------------------------
static void sys_init(uint8_t onoff)
{
    //if(onoff==0) {
    //    SYSCTRL->OSC8M.bit.PRESC = 0;
    //    PM->APBAMASK.reg &= ~PM_APBAMASK_GCLK; 
    //} else {
    PM->APBAMASK.reg |= PM_APBAMASK_GCLK;
    // Switch to 8MHz clock (disable prescaler)
    SYSCTRL->OSC8M.bit.PRESC = 0;
    SYSCTRL->BOD33.reg = 0; // disable to avoid spurious interrupts
    SYSCTRL->BOD33.reg = SYSCTRL_BOD33_LEVEL(48) | SYSCTRL_BOD33_CEN | SYSCTRL_BOD33_MODE |
                         SYSCTRL_BOD33_RUNSTDBY | SYSCTRL_BOD33_ACTION(2) | SYSCTRL_BOD33_HYST;
    bod33_tripped=0;
    wdt_wakeup=0;
    SYSCTRL->BOD33.reg |= SYSCTRL_BOD33_ENABLE;
    while((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_B33SRDY) == 0);
    SYSCTRL->INTENSET.reg = SYSCTRL_INTENSET_BOD33DET;
    NVIC_EnableIRQ(SYSCTRL_IRQn);
    
    // Enable interrupts
    asm volatile ("cpsie i");
    //}
}

void check_bod(void)
{
    uint8_t bod33_trip;
    __disable_irq();
    bod33_trip=bod33_tripped;
    bod33_tripped=0;   
    __enable_irq();
    if(bod33_trip) {
        uart_print(0,">>> bod33_trip=%d SLEEP! <<<\n\r",bod33_trip);
        all_init(0);
        //NVMCTRL->CTRLB.bit.SLEEPPRM=3; // Errata 13140
        while(1) {
            wdt_wakeup=0;            
            wdt_sleep(2000);            
            if((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_BOD33DET) == 0) break;
        }           
        //NVMCTRL->CTRLB.bit.SLEEPPRM=0; // Errata 13140
        all_init(1);
        uart_print(0,">>> bod33 ok wakeup <<<\n\r");   
    }      
}

void print_prompt(void)
{
    uart_puts("\n\rCMD> ");	
}

char addchar(char c)
{
	char buf[2]={0,0};
	buf[0]=c;
	
	if(buf[0]=='*') {
		data_receive_idx=0;
	} else if(buf[0]=='#' && rxbuf[0]=='*') {
	    usart_rx_ready=true;
	} else if(!is_digit(buf[0])) {
	    play_song_error();
	    //data_receive_idx=0;
	    return ' ';
	}
	
	if(data_receive_idx>80) {
	    data_receive_idx=0;
	    return ' ';
	}
	
	play_toneC();	
	
	strcpy(&rxbuf[data_receive_idx],buf);
	data_receive_idx++;
	return c;
}

void unused_init(void)
{
  HAL_GPIO_DOORSW_PIN_in();
  HAL_GPIO_DOORSW_PIN_pullup();
  HAL_GPIO_KEYPAD1_out();
  HAL_GPIO_KEYPAD1_clr();
  HAL_GPIO_KEYPAD2_out();
  HAL_GPIO_KEYPAD2_clr();
  HAL_GPIO_KEYPAD3_out();
  HAL_GPIO_KEYPAD3_clr();
  HAL_GPIO_KEYPAD4_out();
  HAL_GPIO_KEYPAD4_clr();
  HAL_GPIO_KEYPAD5_out();
  HAL_GPIO_KEYPAD5_clr();
  HAL_GPIO_KEYPAD6_out();
  HAL_GPIO_KEYPAD6_clr();
  
  HAL_GPIO_EXT_UART_TX_out();
  HAL_GPIO_EXT_UART_TX_clr();
  HAL_GPIO_EXT_UART_RX_in();
  HAL_GPIO_EXT_UART_RX_pullup();    
}

void show_intro_text(void)
{    
    uart_print(0,"Kamworks SHS Master\n\rVersion=%d.%02d\n\r",VERSION,SUBVERSION);
    uart_print(0,"RCAUSE=0x%x\n\r",rcause);
#if (TESTING==1)
    uart_print(0,"TESTING\n\r");
#endif

    uart_print(0,"\n\rChipID:");
    print_myid();
  
    uart_print(0,"\nHelp: *code#\n\rcode=1: read id\n\rcode=2: remaining days\n\rcode=5: read RTC\n\r");
    uart_print(0,"code=10: info OFF, code=11: info ON\n\r");
    uart_print(0,"code=12: display subversion\n\r");
#if (TESTING==1)
    uart_print(0,"code=3: mosfet ON\n\rcode=4: mosfet OFF\n\r");
    uart_print(0,"code=5yy: set Year\n\rcode=6mm: set Month\n\rcode=7dd: set Day\n\r");
    uart_print(0,"code=8hh: set Hour\n\rcode=9mm: set Minute\n\r");
    uart_print(0,"code=*123456789# : 3 minutes credit\r\n");
#endif
    uart_print(0,"else code is days credit (14 digits)\n\n\r");    
}

void all_init(int8_t onoff)
{
    bod33_tripped=0;
    //rtc_init(onoff);
    if(onoff) {
        __disable_irq();
        memset((void *)&sysval,0,sizeof(sysVals_t));        
        memset((void *)&sysTime,0,sizeof(sysTime_t));
        __enable_irq();        
        wdt_enable(5000,false);
    }
    tone_init(onoff);
    uart_init(onoff);
    i2c_init(onoff);
    tasks_init(onoff);
#if (SYSTEM_MODULAR_DC==1 || SYSTEM_MODULAR_AC==1 || SYSTEM_MODULAR_PV==1)
    mod_init(onoff);
#endif
#if (SYSTEM_CCNEO_SCC==1) 
    scc_init(onoff);
#endif
    code_init(onoff);
    irrecv_init(onoff);
    sim_init(onoff);
    cmd_init(onoff);
    oled_init(onoff);
    
    if(onoff) {
        show_intro_text();
    }
    if(onoff==0) {
        wdt_disable();
    }
}

//-----------------------------------------------------------------------------
int main(void)
{   
  decode_results results; // IR receiver results
  char ch='.'; 
  char log_rtc[1]; 
  rcause=PM->RCAUSE.reg;

  delay_ms(100);
  
  // fixed pheripherals
  sys_init(1);
  rtc_initialize(); 
  wdt_init();
  unused_init();  
  flash_init();
  
  // dynamic pheripherals
  all_init(1);
  
  if(sim_available()) {
      uart_print(0,"SIM available\n\r");
      sim_task_add(SIM_TASK_WAIT,10,0); // wait 10 seconds
      sim_task_add(SIM_TASK_POWER,1,1); // power on
  }

  read_print_rtc(true,log_rtc,0);
  print_prompt();
  
  while (1) {    
    wdt_reset();

    __disable_irq();
    if(serial_rxflag) {
        serial_rxflag=false;	
        ch=addchar(serial_rxbuf);
        __enable_irq();
        uart_putc(ch);
    } else __enable_irq();
	//uart_print(0,"IRstate=%d\r\n",irrecv_get_irparams_rcvstate()); 
    if (irrecv_get_irparams_rcvstate() == STATE_STOP) {
        if (irrecv_decode(&results) ) {
            ch = irrecv_convert(results.value);
            if(ch&0x80) { ch &= 0x7f; addchar(ch); }				
            uart_putc(ch);
            uart_print(0," [IR:%0lx:%c]",results.value,ch);
            print_prompt();
        }       
    }

    if(usart_rx_ready) {
        int cmdstat=0;
        int16_t creditleft;	
        uart_puts_info("\n\r");		
        if(strcmp(rxbuf,"*1#")==0) {
            print_myid();		
        } else if(strcmp(rxbuf,"*2#")==0) {
            creditleft=read_print_rtc(true,log_rtc,0);
            if(creditleft<=0) {
                play_song_expired();
            } else {
                play_nof_tones(creditleft);
            }
#if (TESTING==1)
        } else if(strcmp(rxbuf,"*3#")==0) {
            task_set_load(1);
        } else if(strcmp(rxbuf,"*4#")==0) {
            task_set_load(0);
#endif
        } else if(strcmp(rxbuf,"*5#")==0) {
            if(!uart_get_print_info()) {
                uart_set_print_info(true);
                read_print_rtc(true,log_rtc,0);
                uart_set_print_info(false);
            } else {
                read_print_rtc(true,log_rtc,0);
            }
        } else if(strcmp(rxbuf,"*10#")==0) {
            uart_set_print_info(false);
        } else if(strcmp(rxbuf,"*11#")==0) {
            uart_set_print_info(true);				
#if (TESTING==1)		
        } else if(strlen(rxbuf) == 5) { // *123#  (set date, time) // disable this command after debug!
            unsigned long givencode;
            char opcode = rxbuf[1];
            rxbuf[0]=' '; // strip '*'
            rxbuf[4]=0;   // strip '#'
            rxbuf[1]=' '; // strip opcode
            givencode = strtoul(rxbuf,NULL,10);		
            if(opcode == '5') { // set year			
            	setYear((uint8_t)givencode);
            } else if(opcode == '6') { // set month
                setMonth((uint8_t)givencode);
            } else if(opcode == '7') { // set day
                setDay((uint8_t)givencode);
            } else if(opcode == '8') { // set hour
                setHours((uint8_t)givencode);
            } else if(opcode == '9') { // set minute
                setMinutes((uint8_t)givencode);
            } 
            read_print_rtc(true,log_rtc,0);
            uart_print(0,"Enter new credit code to write flash!\n\r");
        } else if(strlen(rxbuf) == 11) { // *123456789# // for demo
            code_demo(rxbuf,log_rtc,0);
#endif
        } else if(strlen(rxbuf) == 16) { // *12345678901234#
            cmdstat = code_real(rxbuf,log_rtc,0);
        } else cmdstat=-1;

        if(cmdstat < 0) play_song_error();

        memset(rxbuf,0,RXBUF_SIZE);
        usart_rx_ready=false;		
        print_prompt();
    }
    if (alarmTriggered) {   // If the alarm has been triggered
        alarmTriggered=false;
        uart_puts_info("Alarm! mosfet OFF!\n\r");
        code_alarm_triggered();
        task_set_load(0);
    }

    oled_update();
      check_bod();
    run_tasks();
      check_bod();
      
#if (SYSTEM_CCNEO_SCC==1) 
    scc_tasks();
#endif

      check_bod();
    oled_update();
      check_bod();
      
#if (SYSTEM_MODULAR_DC==1 || SYSTEM_MODULAR_AC==1 || SYSTEM_MODULAR_PV==1)
    mod_tasks();
#endif

    sim_tasks();
      check_bod();
    cmd_tasks();
      check_bod();
  }
  return 0;
}

// get rid of linker error when using linker flag --specs=nano.specs
void _sbrk(void) {}

