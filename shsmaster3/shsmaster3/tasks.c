#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "hal_gpio.h"
#include "utils.h"
#include "shsctrl.h"
#include "tasks.h"
#include "sim.h"
#include "bt.h"
#include "flash.h"
#include "scc.h"
#include "i2c.h"
#include "ir_receiver.h"
#include "code.h"
#include "uart.h"
#include "cmd.h"
#include "mod.h"


extern sysTime_t sysTime;
extern sysVals_t sysval;
extern task_queue_t task_queue;
static Eeprom_shs eeprom_shs;
volatile uint8_t i2c_timeout;
volatile uint8_t sim_timeout;
volatile uint8_t oled_i2c_timeout;


char sysstate2str(uint8_t si)
{
        uint8_t ss = (sysval.sys_state>>(si*2))&3;
        switch(ss) {
                case 0:  return 'I'; // INIT
                case 1:  return 'W'; // WAIT
                case 2:  return 'C'; // CHARGE
                default: return 'e'; // ERROR
        }
}
char solarstate2str(uint8_t si)
{
        uint8_t ss = (sysval.solar_state>>(si*2))&3;
        switch(ss) {
                case 0:  return 'O'; // OK
                case 1:  return 'L'; // LOW
                case 2:  return 'H'; // TOO HIGH
                default: return 'x'; // NOT AVAIL
        }
}

char loadstate2str(uint8_t si)
{
        // fo fo fo fo (status: f=1=FAULT o=1=ON)

        uint8_t ss = (sysval.load_state>>(si*2))&3;
        switch(ss) {
                case 0:  return '0'; // OFF
                case 1:  return '1'; // ON
                case 2:  return 'f'; // FAULT, OFF
                default: return 'F'; // FAULT, ON
        }
}

char daystate2str(uint8_t si)
{
        // dp dp dp dp (status: d=1=DAY p=1=PM)

        uint8_t ss = (sysval.day_state>>(si*2))&3;
        switch(ss) {
                case 0:  return 'z'; // NIGHT
                case 1:  return 'z'; // NIGHT
                case 2:  return 'A'; // DAY, AM
                default: return 'P'; // DAY, PM
        }
}
uint16_t task_soc_dump(char *output_buf)
{
        uint16_t len=0;
        len=sprintf(output_buf,   "\r\nSoC  Vpv  Icharge  Vbatt  Iload temp  Pin Pout  Ws_in  Ws_out  uptime\r\n");
        len+=sprintf(&output_buf[len],"%3d ",sysval.SoC);
        len+=sprintf(&output_buf[len]," %4d ",sysval.Vpv);
        len+=sprintf(&output_buf[len]," %4d ",sysval.I_charge);
        len+=sprintf(&output_buf[len],"    %4d ",sysval.Vbatt);
        len+=sprintf(&output_buf[len]," %4d ",sysval.I_load);
        len+=sprintf(&output_buf[len],"   %2d ",sysval.temp);
        len+=sprintf(&output_buf[len]," %3d ",sysval.P_in);
        len+=sprintf(&output_buf[len]," %3d ",sysval.P_out);
        len+=sprintf(&output_buf[len]," %5ld ",sysval.Ws_in);
        len+=sprintf(&output_buf[len]," %5ld ",sysval.Ws_out);
        len+=sprintf(&output_buf[len]," %6ld ",sysval.seconds);
        len+=sprintf(&output_buf[len],"\r\n");
        return len;
}

void task_set_load(uint8_t onoff)
{
#if (SYSTEM_CCNEO_SCC==1) 
    scc_set_load(onoff);
#endif
#if (SYSTEM_MODULAR_AC==1) 
    mod_ac_set_relay_all(onoff);
#endif
#if (SYSTEM_MODULAR_DC==1) 
    mod_dc_set_relay_all(onoff);
#endif
}

void task_cal_temp(uint8_t temp)
{
#if (SYSTEM_CCNEO_SCC==1) 
    scc_cal_temp(temp);
#endif
}

void task_cal_batt(int8_t offset)
{
#if (SYSTEM_CCNEO_SCC==1) 
    scc_cal_batt(offset);
#endif
}

void task_set_ah_batt(int8_t ah)
{
#if (SYSTEM_CCNEO_SCC==1) 
    scc_set_ah_batt(ah);
#endif
}

void task_start_equalize(void)
{
#if (SYSTEM_CCNEO_SCC==1) 
    scc_start_equalize();
#endif
}

void task_every_second(void)
{
}

void task_every_5second(void)
{   
}

void task_every_minute(void)
{
    code_run_update();
#if (SYSTEM_CCNEO_SCC==1) 
    scc_read_batteries();
#endif
}

void task_hourly(void)
{
}

void task_daily(void)
{
    eeprom_shs.interval2G   = sysval.interval2G;
    eeprom_shs.secondsMeter = sysval.seconds;
    eeprom_shs.Ws_in        = sysval.Ws_in;
    eeprom_shs.Ws_out       = sysval.Ws_out;
    eeprom_shs.Ws_out_ac    = sysval.Ws_out_ac;
    
    flash_write_eeprom_shs(&eeprom_shs);
    code_daily_flash();

    if(sim_is_on()) {
        if(sysval.has_bluetooth) { bt_stop_bluetooth(); }
        sim_task_add(SIM_TASK_POWER,0,0); // turn off SIM module
        sim_task_add(SIM_TASK_WAIT,60,0); // wait 60 seconds
        sim_task_add(SIM_TASK_POWER,0,1); // then turn on
    }
}

uint8_t addto_data_message(uint8_t offset)
{
    char buf[64];
    uint8_t tmp_ledstate=0;

    sprintf(buf, DATAPACKET_VERSION ";%d.%d;%c%c%c%c;%c%c%c%c;%lu;%c%c%c%c;",
        VERSION,SUBVERSION,
        daystate2str(3),daystate2str(2),daystate2str(1),daystate2str(0),
        sysstate2str(3),sysstate2str(2),sysstate2str(1),sysstate2str(0), sysval.seconds,
        loadstate2str(3),loadstate2str(2),loadstate2str(1),loadstate2str(0));
    offset = sim_addto_data_message(offset,buf);

    sprintf(buf,"%d;%d;%d;%lu;%lu;%lu;%c%c%c%c;",
        sysval.Door_open,sysval.SoC,sysval.SoCC,
        sysval.Ah, sysval.Ws_in/3600, sysval.Ws_out/3600,
        solarstate2str(3),solarstate2str(2),solarstate2str(1),solarstate2str(0));
    offset = sim_addto_data_message(offset,buf);

    tmp_ledstate=0;
    /*
    if(bit_is_set(PORTC,LED_SHORTFAULT) == 0) tmp_ledstate|=0x08;
    if(bit_is_set(PORTB,LED_CHARGING)   == 0) tmp_ledstate|=0x10;
    if(bit_is_set(PORTB,LED_100PERCENT) == 0) tmp_ledstate|=0x20;
    if(bit_is_set(PORTB,LED_60PERCENT)  == 0) tmp_ledstate|=0x40;
    if(bit_is_set(PORTB,LED_30PERCENT)  == 0) tmp_ledstate|=0x80;
    */
    sprintf(buf,"%d;%d;%d;%d;%d;%d;%d;",sysval.Vbatt,sysval.Vpv,sysval.I_load,sysval.I_charge,
                                        sysval.P_out,sysval.P_in,sysval.temp);
                                        
    offset = sim_addto_data_message(offset,buf);
        
    sprintf(buf,"%d;%d;%d;%d;%d;%d;%d;",
            sysval.ydayl,
            ((tmp_ledstate&0x10)==0x10), // LED_CHARGING
            ((tmp_ledstate&0x20)==0x20), // LED_100PERCENT
            ((tmp_ledstate&0x40)==0x40), // LED_60PERCENT
            ((tmp_ledstate&0x80)==0x80), // LED_30PERCENT
            ((tmp_ledstate&0x08)==0x08), // LED_SHORTFAULT (FIXME: what if it is blinking?)
            sysval.payled_mode);
    
    offset = sim_addto_data_message(offset,buf);
   
    sprintf(buf,"%d;%d;%d;%d;0;0;",sysval.signalstrength,
                                   0,//sysval.scc_icharge[0],
                                   0,//sysval.scc_icharge[1],
                                   0//sysval.scc_icharge[2]
                                   );

    offset = sim_addto_data_message(offset,buf);

    sprintf(buf,"%d;%d;%d;%lu;",sysval.V_load_ac,
                                sysval.I_load_ac,
                                sysval.P_out_ac,
                                sysval.Ws_out_ac/3600
                                );
    offset = sim_addto_data_message(offset,buf);
    return offset;
}

void task_sms(task_t *task)
{
    uint8_t next_state=task->state;
    switch(task->state) {
        case TASK_STATE_START:
          uart_print(0,"TASK_STATE_START\n\r");
          if(sim_is_on()) {
              uint8_t message_len;
              if(sysval.has_bluetooth) { bt_stop_bluetooth(); }

              message_len = sim_prepare_data_message(1,get_myid()); // prepare msg with id;
              if(task->buf[0] != 0) {
                  // add msg
                  message_len = sim_addto_data_message(message_len, task->buf);
              } else {
                  // not add msg but full SoC string
                  message_len = addto_data_message(message_len);        
              }
              sim_task_add(SIM_TASK_SEND_SMS,message_len,task->arg1);
              next_state = TASK_STATE_FINISHED;
          }
          break;
        default:
          next_state = TASK_STATE_FINISHED;
          uart_print(0,"->TASK_STATE_FINISHED\n\r");
          break;
    }
    task->state = next_state;
    return;
}

int8_t task_add(const uint8_t task, const char *buf, const uint8_t arg1)
{
  int8_t i;
  for(i=0;i<TASK_QUEUE_LEN;i++) {
    if(task_queue.task[((task_queue.current+i)&TASK_QUEUE_MASK)].id == TASK_IDLE) {
      // queue entry is available
      task_queue.task[((task_queue.current+i)&TASK_QUEUE_MASK)].id = task;
      task_queue.task[((task_queue.current+i)&TASK_QUEUE_MASK)].state = TASK_STATE_START;
      strncpy(task_queue.task[((task_queue.current+i)&TASK_QUEUE_MASK)].buf,buf,
              MAX_COMMAND_LEN);
      task_queue.task[((task_queue.current+i)&TASK_QUEUE_MASK)].buf[MAX_COMMAND_LEN-1]=0;
      task_queue.task[((task_queue.current+i)&TASK_QUEUE_MASK)].arg1 = arg1;
      task_queue.task[((task_queue.current+i)&TASK_QUEUE_MASK)].retries = 0;
      uart_print(0,"task_add[%d](%d)\n\r",i,task);
      return 1; // ok
    }
  }
  uart_print(0,"task_add(%d) FAIL\n\r",task);
  return 0; // fail
}

void task_done(void)
{
    //uart_print(0,"task_done %d\n\r",task_queue.task[task_queue.current].id);
    task_queue.task[task_queue.current].id = TASK_IDLE;
    task_queue.task[task_queue.current].state = TASK_STATE_FINISHED;
    task_queue.task[task_queue.current].buf[0]=0;
    task_queue.task[task_queue.current].arg1 = 0;
    task_queue.task[task_queue.current].retries = 0;
    task_queue.current = (task_queue.current+1)&TASK_QUEUE_MASK; // point to next
}


void run_tasks(void)
{
    uint16_t tasklist;
    __disable_irq();
    tasklist=sysval.tasklist;
    sysval.tasklist=0;
    __enable_irq();  


    if(tasklist & (1<<TASK_SECOND)) {
        
        /* Do all pending tasks first: */
        uart_print(0,"task[%d]:\n\r",task_queue.task[task_queue.current].id);

        switch(task_queue.task[task_queue.current].id) {
            case TASK_SMS:
            uart_print(0,"TASK_SMS\n\r");
            task_sms(&task_queue.task[task_queue.current]);
            break;
        }
        if(task_queue.task[task_queue.current].state == TASK_STATE_FINISHED) {
            task_done();
        }
        
                //uart_print(0,"---TASK_SECOND---\n\r");
                task_every_second();

                if(sysval.has_bluetooth) {
                        if(sysval.bluetooth_connected) {
                                char btbuf[32];
                                char *btbuf_ptr=bt_bluetooth_read(btbuf, sizeof(btbuf));
                                if(btbuf_ptr != NULL) {
                                        uint16_t bt_len=0;
                                        uart_print(0,"Bluetooth message received:%s\n\r",btbuf_ptr);
                                        if(strncasecmp(btbuf_ptr,"SCC?",4)==0) {
                                            //bt_len=scc_monitor_dump(UART_SIM_TxBuf,0);
                                        } else if(strncasecmp(btbuf_ptr,"SCCD?",5)==0) {
                                            //bt_len=scc_monitor_dump(UART_SIM_TxBuf,1);
                                        } else if(strncasecmp(btbuf_ptr,"SOC?",4)==0) {
                                            //bt_len=task_soc_dump(UART_SIM_TxBuf);
                                        } else if(strncasecmp(btbuf_ptr,"LOAD=",5)==0) { // expect: 0 or 1
                                            char *ptr=&btbuf_ptr[5];
                                            btbuf_ptr[6]=0;
                                            task_set_load(atoi(ptr));
                                            //bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"setting load\r\n");
                                        } else if(strncasecmp(btbuf_ptr,"TEMP=",5)==0) { // expect: number
                                            char *ptr=&btbuf_ptr[5];
                                            btbuf_ptr[7]=0;
                                            task_cal_temp(atoi(ptr));
                                            //bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"calibrate temp\r\n");
                                        } else if(strncasecmp(btbuf_ptr,"BATO=",5)==0) { // expect: number
                                            char *ptr=&btbuf_ptr[5];
                                            btbuf_ptr[7]=0;
                                            task_cal_batt(atoi(ptr));
                                            //bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"calibrate batt Ah offset\r\n");
                                        } else if(strncasecmp(btbuf_ptr,"AH=",3)==0) { // expect: number
                                            char *ptr=&btbuf_ptr[3];
                                            btbuf_ptr[6]=0;
                                            task_set_ah_batt(atoi(ptr));
                                            //bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"set batt Ah\r\n");
                                        } else if(strncasecmp(btbuf_ptr,"RUNEQ",5)==0) {
                                            task_start_equalize();
                                            //bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"Start equalizing\r\n");
                                        } else if(strncasecmp(btbuf_ptr,"RESET",5)==0) {
                                            //RESTART_MICROCONTROLLER();
                                        } else {/*
                                                bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"------ Help: -----------------\r\n");
                                                bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"SCC?   read SCCs sensors\r\n");
                                                bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"SCCD?  read SCCs other\r\n");
                                                bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"SOC?   read SoC of full system\r\n");
                                                bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"LOAD=x set load to 0 or 1\r\n");
                                                bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"TEMP=x calibrate tempsensors\r\n");
                                                bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"BATO=x set selfcurrent offset\r\n");
                                                bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"AH=x   set batteries to xAh\r\n");
                                                bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"RUNEQ  force equalization\r\n");
                                                bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"RESET  reset master\r\n");
                                                */
                                        }
                                        //sim_bluetooth_write(UART_SIM_TxBuf,bt_len);
                                }
                        }
                }
    }

    if(tasklist & (1<<TASK_2SECOND)) {
    }
    if(tasklist & (1<<TASK_3SECOND)) {
        //uart_print(0,"---TASK_3SEC---\n\r");
        if(sysval.has_bluetooth) {
                if(sysval.bluetooth_connected==0) {
                        bt_start_bluetooth();
                        if(sysval.bluetooth_status == 25) {
                                bt_bluetooth_connect();
                                sysval.bluetooth_connected = 1;
                                uart_print(0,"connect!\n\r");
                        }
                }
        }
    }
    if(tasklist & (1<<TASK_5SECOND)) {
        //uart_print(0,"---TASK_5SEC---\n\r");
        task_every_5second();
    }

    if(tasklist & (1<<TASK_MINUTE)) {
        uart_print(0,"---TASK_MIN---\n\r");
        task_every_minute();       
    }    
    
    /*
     * periodic 2G data sending:
     */
    if( (tasklist & (1<<TASK_2G)) && sysval.interval2G != 0 ) {
        uint16_t message_len;
        
        /* First check if the SIM module is on and how good the network connection is: */    
        uart_print(0,"---TASK_CREG---\n\r");        
        if(sim_available()) {
            if(!sim_is_on()) {
                uart_print(0,"SIM available but not on\n\r");
                sim_task_add(SIM_TASK_POWER,0,0); // turn off SIM module
                sim_task_add(SIM_TASK_WAIT,10,0); // wait 10 seconds
                sim_task_add(SIM_TASK_POWER,0,1); // then turn on
            } else {
                if(sysval.has_bluetooth) { bt_stop_bluetooth(); }         
                sim_task_add(SIM_TASK_CREG,0,0); 
                uart_print(0,"---TASK_2G---\n\r");
                message_len = sim_prepare_data_message(0,get_myid());
                message_len = addto_data_message(message_len);        
                sim_task_add(SIM_TASK_START_2G,message_len,0);
            }        
        }
    }

    if(tasklist & (1<<TASK_HOUR)) {
        uart_print(0,"---TASK_HOUR---\n\r");
        task_hourly();
    }

    if(tasklist & (1<<TASK_DAY)) {
        uart_print(0,"---TASK_DAY---\n\r");
        task_daily();
    }
}

//-----------------------------------------------------------------------------
static void timer3_init(int8_t onoff)
{
    if(onoff==0) {
        PM->APBCMASK.reg &= ~PM_APBCMASK_TC3;
    } else {    
        PM->APBCMASK.reg |= PM_APBCMASK_TC3;
        GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC3_GCLK_ID) |
                            GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);
  
        TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
                                 TC_CTRLA_PRESCALER_DIV64 | TC_CTRLA_PRESCSYNC_RESYNC;
  
        TC3->COUNT16.COUNT.reg = 0;
  
        TC3->COUNT16.CC[0].reg = 125; // 1kHz
        TC3->COUNT16.COUNT.reg = 0;
  
        TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
        TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);
        NVIC_EnableIRQ(TC3_IRQn);
    }    
}

void TC3_Handler(void)
{
    if(TC3->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1)) {
        //Update MS Tick
        sysTime.ms++;

//#if (MASTER_PCB_v32==0)
// FIXME: on v32 it does not seem to work in pin-interrupt (sim.c)
            // RI line going low:
            if(sim_is_on() && HAL_GPIO_SIM_RI_PIN_read()==0) {
                //HAL_GPIO_LED_RED_toggle();
                // URC or SMS message coming in
                sysval.ringing=1;
            }
//#endif
    
        if(!(sysTime.ms%100)){ // 1/10 second
            if(i2c_timeout>0) i2c_timeout--;
            if(sim_timeout>0) sim_timeout--;
            if(oled_i2c_timeout>0) oled_i2c_timeout--;
            
/*
//#if (MASTER_PCB_v32==0)
            // RI line going low:
            if(sim_is_on() && HAL_GPIO_SIM_RI_PIN_read()==0) {
                // URC or SMS message coming in
                sysval.ringing=1;
            }
//#endif
*/
        }
    
        //Update Sec Tick
        if(sysTime.ms >= 1000) {
            sysTime.ms=0;
            sysTime.sec++;
            sysval.seconds++;
            sysval.tasklist |= (1<<TASK_SECOND);
    
            if((sysTime.sec % 5)==0) {
                sysval.tasklist |= (1<<TASK_5SECOND);
                sysval.watchdog++;
            }
            if((sysTime.sec % 2)==0) {
                sysval.tasklist |= (1<<TASK_2SECOND);
            }
            if((sysTime.sec % 3)==0) {
                sysval.tasklist |= (1<<TASK_3SECOND);
            }           
            
            sysval.scc_tasklist |= 0x80;
            sysval.sim_tasklist |= 0x80;
            sysval.cmd_tasklist |= 0x80;
            sysval.mod_tasklist |= 0x80;
        }

        //Update minutes
        if(sysTime.sec>=60){
            sysTime.sec=0;
            sysval.minutes++;
            sysval.tasklist |= (1<<TASK_MINUTE);
    
            if((sysval.minutes % 60)==0) {
                sysval.tasklist |= (1<<TASK_HOUR);
            }
            if((sysval.minutes % sysval.interval2G)==0) {
                sysval.tasklist |= (1<<TASK_2G);
            }
            if(sysval.minutes >= MINUTES_PER_DAY) {
                sysval.minutes=0;
                sysval.tasklist |= (1<<TASK_DAY);
            }
            // let the SIM poll every minute anyways in case the ring signal is missed
            if(sim_is_on() ) {
                sysval.ringing=1;
            }
        }
    }
    TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
}

void tasks_init(int8_t onoff)
{
    i2c_timeout=0;
    sim_timeout=0;

    memset((void *)&task_queue,0,sizeof(task_queue_t));

    timer3_init(onoff);
#if (MASTER_PCB_v32==0)
    HAL_GPIO_LED_RED_out();
#endif
    flash_read_eeprom_shs(&eeprom_shs);
    if (eeprom_shs.valid == false) {
        eeprom_shs.interval2G   = 20;
        eeprom_shs.secondsMeter = 0UL;
        eeprom_shs.Ws_in        = 0UL;
        eeprom_shs.Ws_out       = 0UL;
        eeprom_shs.Ws_out_ac    = 0UL;
        eeprom_shs.valid = true;        
        flash_write_eeprom_shs(&eeprom_shs);
        
    }
    sysval.seconds     = eeprom_shs.secondsMeter;
    sysval.Ws_in       = eeprom_shs.Ws_in;
    sysval.Ws_out      = eeprom_shs.Ws_out;
    sysval.Ws_out_ac   = eeprom_shs.Ws_out_ac;
    sysval.Door_open   = 0;
    sysval.interval2G  = eeprom_shs.interval2G;
}

void tasks_eeprom_set_interval2G(uint16_t t)
{
    __disable_irq();
    if(t>MINUTES_PER_DAY) {
        sysval.interval2G=MINUTES_PER_DAY;
    } else {
        sysval.interval2G=t;
    }
    __enable_irq();
    eeprom_shs.interval2G = sysval.interval2G;    
    flash_write_eeprom_shs(&eeprom_shs);
}

