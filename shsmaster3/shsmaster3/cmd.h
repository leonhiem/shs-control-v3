/*
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#ifndef _CMD_H
#define _CMD_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "shsctrl.h"

#define CMD_TASK_QUEUE_LEN  4
#define CMD_TASK_QUEUE_MASK (CMD_TASK_QUEUE_LEN-1)

#define CMD_MAX_COMMAND_LEN MAX_COMMAND_LEN

#define CMD_TASK_IDLE     0
#define CMD_TASK_SMS      1

#define CMD_TASK_STATE_FINISHED  0
#define CMD_TASK_STATE_START     1


typedef struct {
  uint8_t id;
  char    cmd[CMD_MAX_COMMAND_LEN];
  uint8_t arg1;
  uint8_t state;
  uint8_t retries;
} cmd_task_t;

typedef struct {
  cmd_task_t task[CMD_TASK_QUEUE_LEN];
  uint8_t current;
} cmd_task_queue_t;


void cmd_init(int8_t onoff);
void cmd_tasks(void);
int8_t cmd_task_add(const uint8_t task, const char *cmd, const uint8_t arg1);


#endif // _CMD_H
