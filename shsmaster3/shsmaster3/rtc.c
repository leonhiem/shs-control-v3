/*
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "rtc.h"

extern volatile bool alarmTriggered;

void RTC_Handler(void)
{
    alarmTriggered = true;
    RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0; // must clear flag at end
}


/* Wait for sync in write operations */
inline bool RTCisSyncing()
{
    return (RTC->MODE2.STATUS.bit.SYNCBUSY);
}

/* Synchronise the CLOCK register for reading*/
inline void RTCreadRequest() {
    RTC->MODE2.READREQ.reg = RTC_READREQ_RREQ;
    while (RTCisSyncing());
}


/*
 * Get Functions
 */

void getTimeNow(Time *now)
{
    RTCreadRequest();
    
    now->year   = (int8_t)RTC->MODE2.CLOCK.bit.YEAR;
    now->month  = (int8_t)RTC->MODE2.CLOCK.bit.MONTH;
    now->day    = (int8_t)RTC->MODE2.CLOCK.bit.DAY;
    now->hour   = (int8_t)RTC->MODE2.CLOCK.bit.HOUR;
    now->minute = (int8_t)RTC->MODE2.CLOCK.bit.MINUTE;
}

uint8_t getSeconds()
{
    RTCreadRequest();
    return RTC->MODE2.CLOCK.bit.SECOND;
}

uint8_t getMinutes()
{
    RTCreadRequest();
    return RTC->MODE2.CLOCK.bit.MINUTE;
}

uint8_t getHours()
{
    RTCreadRequest();
    return RTC->MODE2.CLOCK.bit.HOUR;
}

uint8_t getDay()
{
    RTCreadRequest();
    return RTC->MODE2.CLOCK.bit.DAY;
}

uint8_t getMonth()
{
    RTCreadRequest();
    return RTC->MODE2.CLOCK.bit.MONTH;
}

uint8_t getYear()
{
    RTCreadRequest();
    return RTC->MODE2.CLOCK.bit.YEAR;
}

uint8_t getAlarmSeconds()
{
    return RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND;
}

uint8_t getAlarmMinutes()
{
    return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE;
}

uint8_t getAlarmHours()
{
    return RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR;
}

uint8_t getAlarmDay()
{
    return RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY;
}

uint8_t getAlarmMonth()
{
    return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH;
}

uint8_t getAlarmYear()
{
    return RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR;
}

void getAlarmTimeNow(Time *date)
{    
    date->year   = (int8_t)RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR;
    date->month  = (int8_t)RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH;
    date->day    = (int8_t)RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY;
    date->hour   = (int8_t)RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR;
    date->minute = (int8_t)RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE;
}

int8_t get_days_in_month(Time *d)
{
    if((d->year&0x3) == 0) { // leap year
        if(d->month==2) return 29; // february
        } else {
        if(d->month==2) return 28; // february
    }
    switch(d->month) {
        case 1: // january
        case 3: // march
        case 5: // may
        case 7: // july
        case 8: // august
        case 10: // october
        case 12: // december
        return 31;
    }
    return 30;
}

int16_t get_days_in_year(Time *d)
{
    if((d->year&0x3) == 0) { // leap year
        return 366;
        } else {
        return 365;
    }
}

int16_t Days_left(Time *a, Time *b, uint8_t want_hours_left, uint8_t want_minutes_left)
{
    int8_t i;
    int32_t d,minutes_a=0;
    int32_t minutes_b=0;
    int32_t days_year_a=0;
    int32_t days_year_b=0;
    int32_t minutes_delta;
    Time tmp;

    tmp.year = a->year;
    for(i=2;i <= a->month;i++) {
        tmp.month = i-1;
        d = get_days_in_month(&tmp);        
        minutes_a += (d * 1440);
    }
    minutes_a += (a->day * 1440);
    minutes_a += (a->hour * 60);
    minutes_a += a->minute;

    tmp.year  = b->year;
    for(i=2;i <= b->month;i++) {
        tmp.month = i-1;
        d = get_days_in_month(&tmp);        
        minutes_b += (d * 1440);
    }
    minutes_b += (b->day * 1440);
    minutes_b += (b->hour * 60);
    minutes_b += b->minute;

    minutes_delta = minutes_a - minutes_b;    

    for(i=2;i <= a->year; i++) {
        tmp.year = i-1;
        days_year_a += get_days_in_year(&tmp);
    }
    for(i=2;i <= b->year; i++) {
        tmp.year = i-1;
        days_year_b += get_days_in_year(&tmp);
    }    
    minutes_delta += (days_year_a-days_year_b)*1440;    
    if(minutes_delta < 0) return -1;    

    if(want_hours_left) {
        return (int16_t)(minutes_delta/60); // return hours
    } else if(want_minutes_left) {
        if(minutes_delta > 0x7fff) {
            return 0x7fff;
        } else {
            return (int16_t)minutes_delta; // return minutes
        }
    } else {
        return (int16_t)(minutes_delta/1440); // return days
    }
}


void Time_add(Time *sum, Time *now, Time *credit)
{
    int8_t days_in_month;
    sum->minute = 0;
    sum->hour = 0;
    sum->day = 0;

    sum->year  = now->year  + credit->year;
    sum->month = now->month + credit->month;
    while(sum->month > 12) {
        sum->year++; sum->month-=12;        
    }
    sum->minute += now->minute + credit->minute;
    while(sum->minute > 59) {
        sum->hour++; sum->minute-=60;        
    }
    sum->hour   += now->hour   + credit->hour;
    while(sum->hour > 23) {
        sum->day++; sum->hour-=24;        
    }
    days_in_month = get_days_in_month(sum);    

    sum->day    += now->day    + credit->day;    

    while(sum->day > days_in_month) {
        sum->day-=days_in_month;
        sum->month++; if(sum->month > 12) { sum->year++; sum->month-=12; }
        days_in_month = get_days_in_month(sum);        
    }
}

/*
 * Set Functions
 */
void setSeconds(uint8_t seconds)
{
    RTC->MODE2.CLOCK.bit.SECOND = seconds;
    while (RTCisSyncing());
}

void setMinutes(uint8_t minutes)
{
    RTC->MODE2.CLOCK.bit.MINUTE = minutes;
    while (RTCisSyncing());
}

void setHours(uint8_t hours)
{
    RTC->MODE2.CLOCK.bit.HOUR = hours;
    while (RTCisSyncing());
}

void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    setSeconds(seconds);
    setMinutes(minutes);
    setHours(hours);
}

void setDay(uint8_t day)
{
    RTC->MODE2.CLOCK.bit.DAY = day;
    while (RTCisSyncing());
}

void setMonth(uint8_t month)
{
    RTC->MODE2.CLOCK.bit.MONTH = month;
    while (RTCisSyncing());
}

void setYear(uint8_t year)
{
    RTC->MODE2.CLOCK.bit.YEAR = year;
    while (RTCisSyncing());
}

void setDate(uint8_t day, uint8_t month, uint8_t year)
{
    setDay(day);
    setMonth(month);
    setYear(year);
}

void setAlarmSeconds(uint8_t seconds)
{
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND = seconds;
    while (RTCisSyncing());
}

void setAlarmMinutes(uint8_t minutes)
{
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE = minutes;
    while (RTCisSyncing());
}

void setAlarmHours(uint8_t hours)
{
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR = hours;
    while (RTCisSyncing());
}

void setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    setAlarmSeconds(seconds);
    setAlarmMinutes(minutes);
    setAlarmHours(hours);
}

void setAlarmDay(uint8_t day)
{
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY = day;
    while (RTCisSyncing());
}

void setAlarmMonth(uint8_t month)
{
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH = month;
    while (RTCisSyncing());
}

void setAlarmYear(uint8_t year)
{
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR = year;
    while (RTCisSyncing());
}

void setAlarmDate(uint8_t day, uint8_t month, uint8_t year)
{
    setAlarmDay(day);
    setAlarmMonth(month);
    setAlarmYear(year);
}

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

void enableAlarm(uint8_t match)
{
    RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = match;
    while (RTCisSyncing());
}

void disableAlarm()
{
    RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = 0x00;
    while (RTCisSyncing());
}

void RTCdisable()
{
    RTC->MODE2.CTRL.reg &= ~RTC_MODE2_CTRL_ENABLE; // disable RTC
    while (RTCisSyncing());
}

void RTCenable()
{
    RTC->MODE2.CTRL.reg |= RTC_MODE2_CTRL_ENABLE; // enable RTC
    while (RTCisSyncing());
}

void RTCreset()
{
    RTC->MODE2.CTRL.reg |= RTC_MODE2_CTRL_SWRST; // software reset
    while (RTCisSyncing());
}
void RTCresetRemove()
{
    RTC->MODE2.CTRL.reg &= ~RTC_MODE2_CTRL_SWRST; // software reset remove
    while (RTCisSyncing());
}


void rtc_init(uint8_t onoff)
{
#if (MASTER_PCB_v32==1)
    if(onoff==0) {
        // use internal XTAL
        GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSC32K | GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_DIVSEL );
    } else {
        // use external XTAL
        GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_DIVSEL );
    }
#endif      
}

void rtc_initialize(void)
{
    uint16_t tmp_reg = 0;
    PM->APBAMASK.reg |= PM_APBAMASK_RTC; // turn on digital interface clock
    
#if (MASTER_PCB_v32==1)
    SYSCTRL->XOSC32K.reg = //SYSCTRL_XOSC32K_ONDEMAND |
        SYSCTRL_XOSC32K_RUNSTDBY |
        SYSCTRL_XOSC32K_EN32K |
        SYSCTRL_XOSC32K_XTALEN |
        SYSCTRL_XOSC32K_STARTUP(6);
    SYSCTRL->XOSC32K.bit.ENABLE = 1;    
    //while((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0);
    
#elif (MASTER_PCB_v32==0)

    SYSCTRL->OSC32K.reg = //SYSCTRL_OSC32K_ONDEMAND |
        SYSCTRL_OSC32K_RUNSTDBY |
        SYSCTRL_OSC32K_EN32K |    
        SYSCTRL_OSC32K_STARTUP(6);
    SYSCTRL->OSC32K.bit.ENABLE = 1;
    //while((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC32KRDY) == 0);
#endif      
   
    /* Attach peripheral clock to 32k oscillator */
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(2)|GCLK_GENDIV_DIV(4);// divided by: 2^(GENDIV.DIV+1)=32
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
    
#if (MASTER_PCB_v32==1)
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_DIVSEL );
#elif (MASTER_PCB_v32==0)
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSC32K | GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_DIVSEL );
#endif
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
    GCLK->CLKCTRL.reg = (uint32_t)((GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2 | (RTC_GCLK_ID << GCLK_CLKCTRL_ID_Pos)));
    while (GCLK->STATUS.bit.SYNCBUSY);       
    
    RTCdisable();
    RTCreset();
    tmp_reg |= RTC_MODE2_CTRL_MODE_CLOCK; // set clock operating mode
    tmp_reg |= RTC_MODE2_CTRL_PRESCALER_DIV1024; // set prescaler to 1024 for MODE2
    tmp_reg &= ~RTC_MODE2_CTRL_MATCHCLR; // disable clear on match    
    //According to the datasheet RTC_MODE2_CTRL_CLKREP = 0 for 24h
    tmp_reg &= ~RTC_MODE2_CTRL_CLKREP; // 24h time representation
    RTC->MODE2.READREQ.reg &= ~RTC_READREQ_RCONT; // disable continuously mode
    RTC->MODE2.CTRL.reg = tmp_reg;
    while (RTCisSyncing());
    NVIC_EnableIRQ(RTC_IRQn); // enable RTC interrupt 
    //NVIC_SetPriority(RTC_IRQn, 0x00);
    RTC->MODE2.INTENSET.reg |= RTC_MODE2_INTENSET_ALARM0; // enable alarm interrupt
    RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = RTC_MODE2_MASK_SEL_OFF_Val; // default alarm match is off (disabled)    
    while (RTCisSyncing());
    RTCenable();
    RTCresetRemove();
}

