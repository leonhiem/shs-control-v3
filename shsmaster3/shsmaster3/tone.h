/*
 * Derived from Arduino Tone class
 * Modified from C++ to C    by LeonH
 */

#ifndef _TONE_H
#define _TONE_H

typedef struct {
	int tone,duration;
} Melody;

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"


#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);
void resetTC2(void);
void noTone(void);
void tone (uint32_t frequency, uint32_t duration);
void play_song_success(void);
void play_song_error(void);
void play_song_start(void);
void play_song_expired(void);
void play_nof_tones(int16_t nof_tones);
void play_toneC(void);
void tone_init(int8_t onoff);

#endif // _TONE_H
