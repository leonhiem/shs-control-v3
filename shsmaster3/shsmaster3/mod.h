/*
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#ifndef _MOD_H
#define _MOD_H

#if (SYSTEM_MODULAR_DC==1 || SYSTEM_MODULAR_AC==1 || SYSTEM_MODULAR_PV==1)

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define MOD_AC_I2C_slaveAddress 0x18
#define MOD_DC_I2C_slaveAddress 0x14
#define MOD_PV_I2C_slaveAddress 0x10

#define MOD_I2C_CMD_FREEZE_VOLT_CURR   0x20
#define MOD_I2C_CMD_READ_ADC_CURR_LSB  0x30
#define MOD_I2C_CMD_READ_ADC_CURR_MSB  0x31
#define MOD_I2C_CMD_READ_ADC_VOLT_LSB  0x40
#define MOD_I2C_CMD_READ_ADC_VOLT_MSB  0x41
#define MOD_I2C_CMD_READ_ADC_VOLTB_LSB 0x42
#define MOD_I2C_CMD_READ_ADC_VOLTB_MSB 0x43
#define MOD_I2C_CMD_SET_RELAY_OFF      0x50
#define MOD_I2C_CMD_SET_RELAY_ON       0x51


void mod_init(int8_t onoff);
void mod_tasks(void);

void mod_dc_set_relay(uint8_t s, uint8_t onoff);
void mod_ac_set_relay(uint8_t s, uint8_t onoff);
void mod_ac_set_relay_all(uint8_t onoff);
void mod_dc_set_relay_all(uint8_t onoff);

void mod_ac_monitor(uint8_t s);
void mod_dc_monitor(uint8_t s);
void mod_pv_monitor(uint8_t s);

void mod_update_sysval(void);
uint16_t mod_monitor_dump(char *output_buf, int part);

#endif // (SYSTEM_MODULAR_DC==1 || SYSTEM_MODULAR_AC==1 || SYSTEM_MODULAR_PV==1)
#endif // _MOD_H
