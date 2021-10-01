/*
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "samd20.h"
#include "cmd.h"
#include "uart.h"
#include "utils.h"
#include "shsctrl.h"
#include "tasks.h"
#include "code.h"
#include "sim.h"

extern sysVals_t sysval;
extern cmd_task_queue_t cmd_task_queue;

void cmd_sms(cmd_task_t *task)
{
    char response_msg[CMD_MAX_COMMAND_LEN];
    uint8_t next_state=task->state;

    response_msg[0]=0;

    switch(task->state) {
        case CMD_TASK_STATE_START:
          uart_print(0,"CMD_TASK_STATE_START\n\r");

          if(strncasecmp(task->cmd,"SOC?",4)==0) {
              task->cmd[0]=0; // no message, just the full SoC string
              task_add(TASK_SMS,task->cmd,task->arg1);
              
          } else if(strlen(task->cmd) == 11 && task->cmd[0]=='*') { // *123456789# for demo
              code_demo(task->cmd, response_msg, CMD_MAX_COMMAND_LEN);
              task_add(TASK_SMS,response_msg,task->arg1);
              
          } else if(strlen(task->cmd) == 16 && task->cmd[0]=='*') { // *12345678901234# for real
              code_real(task->cmd, response_msg, CMD_MAX_COMMAND_LEN);
              task_add(TASK_SMS,response_msg,task->arg1);
              
          } else if(strncasecmp(task->cmd,"*1#",3)==0) {                  
              task_add(TASK_SMS," ",task->arg1); // id is already in the msg
              
          } else if(strncasecmp(task->cmd,"*2#",3)==0) {
              read_print_rtc(false, response_msg, CMD_MAX_COMMAND_LEN);
              task_add(TASK_SMS,response_msg,task->arg1);
              
#if (TESTING==1)
          } else if(strncasecmp(task->cmd,"LOAD=",5)==0) { // expect: 0 or 1
              char *bufptr=&task->cmd[5];
              int state;
              task->cmd[6]=0;
              state=atoi(bufptr);
              task_set_load(state);
              task->cmd[0]=0; // no message, just the full SoC string
              task_add(TASK_SMS,task->cmd,task->arg1);
          } else if(strncasecmp(task->cmd,"apn=",4)==0) { // expect: APN
              char c;
              char *ptr=&task->cmd[4];
              while((c=*ptr) != 0) {
                  if(isspace(c)) { *ptr=0; break; }
                  ptr++;
              }
              sim_eeprom_set_APN(&task->cmd[4]);
              task_add(TASK_SMS,"OK",task->arg1);
          } else if(strncasecmp(task->cmd,"user=",5)==0) { // expect: user
              char c;
              char *ptr=&task->cmd[5];
              while((c=*ptr) != 0) {
                  if(isspace(c)) { *ptr=0; break; }
                  ptr++;
              }
              sim_eeprom_set_USER(&task->cmd[5]);
              task_add(TASK_SMS,"OK",task->arg1);
          } else if(strncasecmp(task->cmd,"pwd=",4)==0) { // expect: pwd
              char c;
              char *ptr=&task->cmd[4];
              while((c=*ptr) != 0) {
                  if(isspace(c)) { *ptr=0; break; }
                  ptr++;
              }
              sim_eeprom_set_PWD(&task->cmd[4]);
              task_add(TASK_SMS,"OK",task->arg1);
          } else if(strncasecmp(task->cmd,"host=",5)==0) { // expect: hostname
              char c;
              char *ptr=&task->cmd[5];
              while((c=*ptr) != 0) {
                  if(isspace(c)) { *ptr=0; break; }
                  ptr++;
              }
              sim_eeprom_set_HOST(&task->cmd[5]);
              task_add(TASK_SMS,"OK",task->arg1);

          } else if(strncasecmp(task->cmd,"T_2G=",5)==0) { // expect: minutes
              char *bufptr=&task->cmd[5];
              int i;
              for(i=5;i<strlen(task->cmd);i++) {
                    if(!isdigit(task->cmd[i])) { task->cmd[i]=0; break; }
              }
              tasks_eeprom_set_interval2G((uint16_t)atol(bufptr));
              task_add(TASK_SMS,"OK",task->arg1);
          }
#endif // TESTING
          next_state = CMD_TASK_STATE_FINISHED;
          break;
        default:
          next_state = CMD_TASK_STATE_FINISHED;
          uart_print(0,"->CMD_TASK_STATE_FINISHED\n\r");
          break;
    }
    task->state = next_state;
    return;
}

int8_t cmd_task_add(const uint8_t task, const char *cmd, const uint8_t arg1)
{
  int8_t i;
  for(i=0;i<CMD_TASK_QUEUE_LEN;i++) {
    if(cmd_task_queue.task[((cmd_task_queue.current+i)&CMD_TASK_QUEUE_MASK)].id == CMD_TASK_IDLE) {
      // queue entry is available
      cmd_task_queue.task[((cmd_task_queue.current+i)&CMD_TASK_QUEUE_MASK)].id = task;
      cmd_task_queue.task[((cmd_task_queue.current+i)&CMD_TASK_QUEUE_MASK)].state = CMD_TASK_STATE_START;
      strncpy(cmd_task_queue.task[((cmd_task_queue.current+i)&CMD_TASK_QUEUE_MASK)].cmd,cmd,
              CMD_MAX_COMMAND_LEN);
      cmd_task_queue.task[((cmd_task_queue.current+i)&CMD_TASK_QUEUE_MASK)].cmd[CMD_MAX_COMMAND_LEN-1]=0;
      cmd_task_queue.task[((cmd_task_queue.current+i)&CMD_TASK_QUEUE_MASK)].arg1 = arg1;
      cmd_task_queue.task[((cmd_task_queue.current+i)&CMD_TASK_QUEUE_MASK)].retries = 0;
      uart_print(0,"cmd_task_add[%d](%d)\n\r",i,task);
      return 1; // ok
    }
  }
  uart_print(0,"cmd_task_add(%d) FAIL\n\r",task);
  return 0; // fail
}

void cmd_task_done(void)
{
    //uart_print(0,"cmd_task_done %d\n\r",cmd_task_queue.task[cmd_task_queue.current].id);
    cmd_task_queue.task[cmd_task_queue.current].id = CMD_TASK_IDLE;
    cmd_task_queue.task[cmd_task_queue.current].state = CMD_TASK_STATE_FINISHED;
    cmd_task_queue.task[cmd_task_queue.current].cmd[0]=0;
    cmd_task_queue.task[cmd_task_queue.current].arg1 = 0;
    cmd_task_queue.task[cmd_task_queue.current].retries = 0;
    cmd_task_queue.current = (cmd_task_queue.current+1)&CMD_TASK_QUEUE_MASK; // point to next
}

void cmd_tasks(void)
{
    uint8_t tasklist;    
    __disable_irq();
    tasklist=sysval.cmd_tasklist;
    sysval.cmd_tasklist&=0x7f;
    __enable_irq();

    if((tasklist&0x80)==0) return; // not yet allowed to run
    
    // task allowed to run
    uart_print(0,"cmd_task[%d]:\n\r",cmd_task_queue.task[cmd_task_queue.current].id);

    switch(cmd_task_queue.task[cmd_task_queue.current].id) {
        case CMD_TASK_SMS:
          uart_print(0,"CMD_TASK_SMS\n\r");
          cmd_sms(&cmd_task_queue.task[cmd_task_queue.current]);
          break;
    }
    if(cmd_task_queue.task[cmd_task_queue.current].state == CMD_TASK_STATE_FINISHED) {
        cmd_task_done();
    }
}

void cmd_init(int8_t onoff)
{
    memset((void *)&cmd_task_queue,0,sizeof(cmd_task_queue_t));
}

