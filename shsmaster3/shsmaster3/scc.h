/*
 * SHS data acquisition
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#ifndef _SCC_H
#define _SCC_H

#if (SYSTEM_CCNEO_SCC==1)

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define I2C_SCC_BASE_ADDR 0x2c

#define I2C_CMD_READADC      0xa5
#define I2C_CMD_READSTAT     0xa6
#define I2C_CMD_STAT_WDT     0xa7
#define I2C_CMD_RESET        0xaa
#define I2C_CMD_CALTEMP      0xab
#define I2C_CMD_READBATT     0xac
#define I2C_CMD_EQUALIZE     0xad
#define I2C_CMD_CALBATT      0xae
#define I2C_CMD_SETAHBATT    0xaf

void scc_init(uint8_t onoff);
void scc_probe(void);
void scc_tasks(void);
uint8_t scc_monitor(uint8_t s);
uint16_t scc_monitor_dump(char *output_buf, int part);
void scc_set_load(uint8_t onoff);
void scc_cal_temp(uint8_t temp);
void scc_cal_batt(int8_t offset);
uint8_t scc_read_battery(uint8_t s);
void scc_start_equalize(void);
void scc_set_ah_batt(int8_t ah);
void scc_update_sysval(void);
void scc_read_batteries(void);
void scc_send_cmd(uint8_t cmd, uint8_t val, uint8_t write_val);
#endif // SYSTEM_CCNEO_SCC==1
#endif // _SCC_H
