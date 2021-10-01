/*
 *
 * Author: Leon Hiemstra 
 * Date:   
 */

#ifndef _RTC_H
#define _RTC_H


/*
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"

typedef struct {
	int8_t year, month, day, hour, minute;
} Time;


void irq_handler_rtc(void);

/*
 * Get Functions
 */
uint8_t getSeconds();
uint8_t getMinutes();
uint8_t getHours();
uint8_t getDay();
uint8_t getMonth();
uint8_t getYear();
uint8_t getAlarmSeconds();
uint8_t getAlarmMinutes();
uint8_t getAlarmHours();
uint8_t getAlarmDay();
uint8_t getAlarmMonth();
uint8_t getAlarmYear();

int16_t Days_left(Time *a, Time *b, uint8_t want_hours_left, uint8_t want_minutes_left);
void Time_add(Time *sum, Time *now, Time *credit);
void getTimeNow(Time *now);
void getAlarmTimeNow(Time *date);



/*
 * Set Functions
 */
void setSeconds(uint8_t seconds);
void setMinutes(uint8_t minutes);
void setHours(uint8_t hours);
void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
void setDay(uint8_t day);
void setMonth(uint8_t month);
void setYear(uint8_t year);
void setDate(uint8_t day, uint8_t month, uint8_t year);

void setAlarmSeconds(uint8_t seconds);
void setAlarmMinutes(uint8_t minutes);
void setAlarmHours(uint8_t hours);
void setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
void setAlarmDay(uint8_t day);
void setAlarmMonth(uint8_t month);
void setAlarmYear(uint8_t year);
void setAlarmDate(uint8_t day, uint8_t month, uint8_t year);

/*
  enum Alarm_Match: uint8_t // Should we have this enum or just use the identifiers from /component/rtc.h ?
  {
	  MATCH_OFF          = RTC_MODE2_MASK_SEL_OFF_Val,          // Never
	  MATCH_SS           = RTC_MODE2_MASK_SEL_SS_Val,           // Every Minute
	  MATCH_MMSS         = RTC_MODE2_MASK_SEL_MMSS_Val,         // Every Hour
	  MATCH_HHMMSS       = RTC_MODE2_MASK_SEL_HHMMSS_Val,       // Every Day
	  MATCH_DHHMMSS      = RTC_MODE2_MASK_SEL_DDHHMMSS_Val,     // Every Month
	  MATCH_MMDDHHMMSS   = RTC_MODE2_MASK_SEL_MMDDHHMMSS_Val,   // Every Year
	  MATCH_YYMMDDHHMMSS = RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val  // Once, on a specific date and a specific time
  };
  */
void enableAlarm(uint8_t match);
void disableAlarm();
void RTCdisable();
void RTCenable();
void RTCreset();
void RTCresetRemove();
void rtc_init(uint8_t onoff);
void rtc_initialize(void);

#endif // _RTC_H
