#ifndef _OLED_H
#define _OLED_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"

void oled_init(int8_t onoff);
void oled_demo(void);
void oled_update(void);
void oled_write_number_int(const int16_t value, const uint8_t blinking);




#endif  /* _OLED_H */

