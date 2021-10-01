/*
 * SHS Control
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#ifndef _TASKS_H
#define _TASKS_H

void task_every_second(void);
void task_every_5second(void);
void task_every_minute(void);
void task_hourly(void);
void task_daily(void);
void task_set_load(uint8_t onoff);
void task_cal_temp(uint8_t temp);
void task_cal_batt(int8_t offset);
void task_start_equalize(void);
void task_set_ah_batt(int8_t ah);
uint16_t task_soc_dump(char *output_buf);
char solarstate2str(uint8_t si);
char sysstate2str(uint8_t si);
char loadstate2str(uint8_t si);
char daystate2str(uint8_t si);

#endif // _TASKS_H
