/*
 * SIM Switch
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#ifndef _SIM_H
#define _SIM_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define SIM_TASK_QUEUE_LEN  8
#define SIM_TASK_QUEUE_MASK (SIM_TASK_QUEUE_LEN-1)

#define SIM_TASK_IDLE     0
#define SIM_TASK_POWER    1
#define SIM_TASK_START_2G 2
#define SIM_TASK_STOP_2G  3
#define SIM_TASK_SEND_2G  4
#define SIM_TASK_SEND_SMS 5
#define SIM_TASK_READ_SMS 6
#define SIM_TASK_WAIT     7
#define SIM_TASK_CREG     8

#define SIM_TASK_STATE_FINISHED  0
#define SIM_TASK_STATE_START     1

#define SIM_TASK_STATE_POWER_PULSEON       2
#define SIM_TASK_STATE_POWER_PULSEON_WAIT1 3
#define SIM_TASK_STATE_POWER_PULSEON_WAIT2 4
#define SIM_TASK_STATE_POWER_PULSEON_WAIT3 5
#define SIM_TASK_STATE_POWER_PULSEON_WAIT4 6
#define SIM_TASK_STATE_POWER_PULSEON_WAIT5 7
#define SIM_TASK_STATE_POWER_AUTOBAUD      8
#define SIM_TASK_STATE_POWER_SET_TEXTMODE  9
#define SIM_TASK_STATE_POWER_DEL_MSGS      10
#define SIM_TASK_STATE_POWER_SET_FLOWCTRL  11
#define SIM_TASK_STATE_POWER_READ_IMEI     12

#define SIM_TASK_STATE_START2G_GET_SIMSTATE 2
#define SIM_TASK_STATE_START2G_INIT         3
#define SIM_TASK_STATE_START2G_START        4
#define SIM_TASK_STATE_START2G_START_WAIT1  5
#define SIM_TASK_STATE_START2G_START2       7
#define SIM_TASK_STATE_START2G_GPRSACT      8
#define SIM_TASK_STATE_START2G_STATUS       9
#define SIM_TASK_STATE_START2G_STATUS2      10
#define SIM_TASK_STATE_START2G_DEACT        11

#define SIM_TASK_STATE_SEND2G_HTTPPARA_CID     2
#define SIM_TASK_STATE_SEND2G_HTTPPARA_URL     3
#define SIM_TASK_STATE_SEND2G_HTTPACTION       4
#define SIM_TASK_STATE_SEND2G_HTTPACTION_WAIT1 5
#define SIM_TASK_STATE_SEND2G_HTTPACTION2      6
#define SIM_TASK_STATE_SEND2G_HTTPREAD         7
#define SIM_TASK_STATE_SEND2G_HTTPSTATUS       8
#define SIM_TASK_STATE_SEND2G_HTTPSTATUS_WAIT1 9
#define SIM_TASK_STATE_SEND2G_HTTPSTATUS_WAIT2 10
#define SIM_TASK_STATE_SEND2G_HTTPSTATUS2      11
#define SIM_TASK_STATE_SEND2G_TERM             12

#define SIM_TASK_STATE_STOP2G_WAIT             2
#define SIM_TASK_STATE_STOP2G_SAPBR_RESPONSE   3
#define SIM_TASK_STATE_STOP2G_TERM             4
#define SIM_TASK_STATE_STOP2G_TERM_WAIT        5

#define SIM_TASK_STATE_REC_SMS_CHK             2
#define SIM_TASK_STATE_REC_SMS_DEL             3

#define SIM_TASK_STATE_SENDSMS_WAIT1           2
#define SIM_TASK_STATE_SENDSMS_WAIT2           3
#define SIM_TASK_STATE_SENDSMS_RESPONSE        4



typedef struct {
  uint8_t id;
  uint16_t arg1;
  uint8_t  arg2;
  uint8_t state;
  uint8_t retries;
} sim_task_t;

typedef struct {
  sim_task_t task[SIM_TASK_QUEUE_LEN];
  uint8_t current;
} sim_task_queue_t;


void sim_tx(const char *buf, int len);
char sim_rxflush(void);
uint16_t sim_read(char *sim_bufptr, const char *expect, int timeout_centisec, uint16_t maxlen);
char * sim_read_multi(char *sim_bufptr, const char *expect[], const int nof_expect, int timeout_centisec, uint16_t maxlen);

void sim_init(int8_t onoff);
uint8_t sim_available(void);
void sim_tasks(void);
int8_t sim_task_add(const uint8_t task, const uint16_t arg1, const uint8_t arg2);
uint16_t sim_read(char *sim_bufptr, const char *expect, int timeout_centisec, uint16_t maxlen);
uint8_t sim_is_on(void);
uint8_t sim_delete_sms(void);
uint8_t sim_delete_all_sms(void);
char * sim_read_IMEI(char *sim_rxbuf, uint16_t maxlen);
int sim_read_signalstrength(void);
int sim_read_CREG(void);
uint8_t sim_set_flowcontrol(void);
uint8_t sim_pincode_check(void);
uint8_t sim_set_textmode(void);
uint8_t sim_prepare_data_message(uint8_t sms_only, const uint32_t system_id);
uint8_t sim_addto_data_message(uint8_t offset, char *str);
uint8_t sim_has_bluetooth(void);

void sim_eeprom_set_APN(const char *apn);
void sim_eeprom_set_USER(const char *user);
void sim_eeprom_set_PWD(const char *pwd);
void sim_eeprom_set_HOST(const char *host);
int sim_read_location(void);

#endif // _SIM_H
