/*
 * SHS data acquisition
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#ifndef _SCC_H
#define _SCC_H

#define I2C_CMD_READADC      0xa5
#define I2C_CMD_READSTAT     0xa6
#define I2C_CMD_READSTAT_WDT 0xa7
#define I2C_CMD_LOAD_ON      0xa8
#define I2C_CMD_LOAD_OFF     0xa9
#define I2C_CMD_RESET        0xaa
#define I2C_CMD_CALTEMP      0xab
#define I2C_CMD_READBATT     0xac
#define I2C_CMD_EQUALIZE     0xad
#define I2C_CMD_CALBATT      0xae
#define I2C_CMD_SETAHBATT    0xaf

void scc_monitor(void);
uint16_t scc_monitor_dump(char *output_buf, int part);
void scc_set_load(uint8_t onoff);
void scc_cal_temp(uint8_t temp);
void scc_cal_batt(int8_t offset);
void scc_read_battery(void);
void scc_start_equalize(void);
void scc_set_ah_batt(int8_t ah);

#endif // _SCC_H
