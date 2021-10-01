/*
 * SIM Switch
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#ifndef _BT_H
#define _BT_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Bluetooth functions:
uint8_t bt_set_bluetooth(void);
void bt_start_bluetooth(void);
void bt_stop_bluetooth(void);
int bt_read_bluetooth_status(void);
int bt_bluetooth_connect(void);
char * bt_bluetooth_read(char *sim_rxbuf, uint16_t maxlen);
uint8_t bt_bluetooth_write(const char *msg, uint16_t size);

#endif // _BT_H
