#ifndef _CODE_H
#define _CODE_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "flash.h"
#include "rtc.h"


void code_init(int8_t onoff);
void code_run_update(void);
void code_daily_flash(void);

void code_demo(char *rxbuf, char *log, uint8_t allowed_log_len);
int code_real(char *rxbuf, char *log, uint8_t allowed_log_len);
void code_sms(char *rxbuf);

int16_t read_print_rtc(bool do_print, char *log, uint8_t allowed_log_len);
void generate_newalarm(Time *newalarm, Time *now, Time *credit);
void code_alarm_triggered(void);


uint32_t get_myid(void);
void print_myid(void);



#endif  /* _CODE_H */

