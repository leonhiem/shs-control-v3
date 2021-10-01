/*
 *
 * Author: Leon Hiemstra <l.hiemstra@gmail.com>
 */

#ifndef _TASKS_H
#define _TASKS_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "shsctrl.h"


#define TASK_QUEUE_LEN  4
#define TASK_QUEUE_MASK (TASK_QUEUE_LEN-1)

#define TASK_IDLE     0
#define TASK_SMS      7

#define TASK_STATE_FINISHED  0
#define TASK_STATE_START     1


typedef struct {
  uint8_t id;
  char    buf[MAX_COMMAND_LEN];
  uint8_t arg1;
  uint8_t state;
  uint8_t retries;
} task_t;

typedef struct {
  task_t task[TASK_QUEUE_LEN];
  uint8_t current;
} task_queue_t;


int8_t task_add(const uint8_t task, const char *buf, const uint8_t arg1);
void tasks_init(int8_t onoff);
void run_tasks(void);

void tasks_eeprom_set_interval2G(uint16_t t);
void tasks_eeprom_set_intervalSMS(uint16_t t);

void task_set_load(uint8_t onoff);
void task_cal_temp(uint8_t temp);
void task_cal_batt(int8_t offset);
void task_set_ah_batt(int8_t ah);
void task_start_equalize(void);


#endif // _TASKS_H
