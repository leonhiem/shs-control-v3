/*
 * SHS Control
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "shsctrl.h"

uint8_t quota_to_percent(uint16_t quota, uint16_t max_quota)
{
    uint32_t percent;
    percent=quota*100;
    percent=percent/max_quota;
    return (uint8_t)percent;
}

