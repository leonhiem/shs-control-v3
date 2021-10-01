/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Written by Cristian Maglie

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
   modified from C++ --> C     by LeonH
*/


#ifndef _FLASH_H
#define _FLASH_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "rtc.h"
#include "shsctrl.h"

typedef struct {
   uint8_t valid;
   uint8_t days, minutes, seq;
   Time now, alarm; 
} Eeprom_time;

typedef struct {
    uint8_t  valid;
    char     hostname[HOSTNAME_LEN];
    char     apn[APN_LEN];
    char     user[USER_LEN];
    char     pwd[PWD_LEN];
} Eeprom_sim;

typedef struct {
    uint8_t  valid;
    uint8_t  onoff;
    uint16_t interval2G;
    uint32_t secondsMeter;
    uint32_t Ws_in;
    uint32_t Ws_out;
    uint32_t Ws_out_ac;
} Eeprom_shs;

typedef struct {	      
    Eeprom_time eeprom_time;
    Eeprom_shs  eeprom_shs;
    Eeprom_sim  eeprom_sim;
} Eeprom;


// Concatenate after macro expansion
#define PPCAT_NX(A, B) A ## B
#define PPCAT(A, B) PPCAT_NX(A, B)

#define FlashStorage(name, T) \
  __attribute__((__aligned__(256))) \
  static const uint8_t PPCAT(_data,name)[(sizeof(T)+255)/256*256] = { }; \
  flash_address = PPCAT(_data,name);
  

void flash_config(void);
void flash_write(const uint8_t *data);
void flash_erase_sized(void);
void flash_read(void *);
void flash_init(void);
void flash_read_eeprom_time(Eeprom_time *data);
void flash_read_eeprom_shs(Eeprom_shs *data);
void flash_read_eeprom_sim(Eeprom_sim *data);
void flash_write_eeprom_time(const Eeprom_time *data);
void flash_write_eeprom_shs(const Eeprom_shs *data);
void flash_write_eeprom_sim(const Eeprom_sim *data);

#endif // _FLASH_H
