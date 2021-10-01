#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "code.h"
#include "utils.h"
#include "rtc.h"
#include "flash.h"
#include "tone.h"
#include "oled.h"
#include "uart.h"
#include "cmd.h"
#include "tasks.h"

static Eeprom_time eeprom_time;
static uint8_t rtc_hour_old=0;
static uint8_t rtc_minutes_old=0;

void code_init(int8_t onoff)
{
    char log[10];
    rtc_hour_old=0;
    rtc_minutes_old=0;

    if(onoff==0) return;
    
    flash_read_eeprom_time(&eeprom_time);
    
    // If this is the first run the "valid" value should be "false"
    if (eeprom_time.valid == false) {
        uart_print(0,"eeprom empty: mosfet off\n\r");
        task_set_load(0);
        setTime(0,0,0);// set time (hour 0-23, minute 0-59, second 0-59)
        setDate(1,1,1);// set date (day 1-31, month 1-12, year 0-99)
        eeprom_time.seq=99;
        play_song_expired();
    } else {
        int16_t creditleft=-1;
        uart_print(0,"eeprom ok: read values\n\r");
          
        // restore time, date, alarm:
        /*
        setTime(eeprom_time.now.hour,eeprom_time.now.minute,0);
        setDate(eeprom_time.now.day,eeprom_time.now.month,eeprom_time.now.year);
        setAlarmTime(eeprom_time.alarm.hour,eeprom_time.alarm.minute,0);
        setAlarmDate(eeprom_time.alarm.day,eeprom_time.alarm.month,eeprom_time.alarm.year);
        */
        uart_print(0,"(days=%d) (seq=%d)\n\r",eeprom_time.days,eeprom_time.seq);
          
        if(eeprom_time.days==0 && eeprom_time.minutes==0) {
            uart_print(0,"free mode, no alarm: mosfet on\n\r");
            task_set_load(1);
            play_song_success();
        } else {
            creditleft=read_print_rtc(true,log,0);
            if(creditleft <= 0) { // passed the alarm time
                uart_print(0,"already expired, mosfet off\n\r");
                task_set_load(0);
                play_song_expired();
            } else { // not yet passed the alarm time
                uart_print(0,"enable alarm, mosfet on\n\r");
                enableAlarm(RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val);
                task_set_load(1);
                play_song_success();
            }
        }
    }
}


// returns -1 if passed alarm time already
// returns  >=0 if have still days credit left
int16_t read_print_rtc(bool do_print, char *log, uint8_t allowed_log_len)
{
    char buf[50];
    Time now,alarm;
    int16_t credit;
    uint8_t show_hours_left=0; // default show days left
    getTimeNow(&now);
    getAlarmTimeNow(&alarm);
    
    credit = Days_left(&alarm,&now,show_hours_left,0);
    if(credit<=0) {
        show_hours_left=1;
        credit = Days_left(&alarm,&now,show_hours_left,0);
    }
    
    if(do_print) {
        sprintf(buf,"\n\r      YY-MM-DD hh:mm\n\r");
        uart_puts_info(buf);
        sprintf(buf," time:%02d-%02d-%02d %02d:%02d\n\r",now.year,now.month,now.day,now.hour,now.minute);
        uart_puts_info(buf);
        sprintf(buf,"alarm:%02d-%02d-%02d %02d:%02d\n\r",alarm.year,alarm.month,alarm.day,alarm.hour,alarm.minute);
        uart_puts_info(buf);
        sprintf(buf,"credit=%d %s left\n\r",credit,(show_hours_left ? "hours" : "days"));
        uart_puts_info(buf);

        oled_write_number_int(credit,show_hours_left);
    }
    if(allowed_log_len>=9) {
        if(show_hours_left) {
            sprintf(log,"[%dhrs]",credit);
        } else {
            sprintf(log,"[%dday]",credit);
        }
    }
    if(credit<=0) {
        credit = Days_left(&alarm,&now,0,1); // get credit for minutes left
    }
    return credit;
}

void generate_newalarm(Time *newalarm, Time *now, Time *credit)
{
    Time alarm;
    int16_t oldcredit;
    getAlarmTimeNow(&alarm);
    oldcredit = Days_left(&alarm,now,0,0); // are there days left?
    if(oldcredit<=0) {
        oldcredit = Days_left(&alarm,now,0,1); // maybe some minutes left?
    }
    
    if(oldcredit < 0) {
        // credit is expired; add credit to now
        uart_puts_info("add to now\n\r");
        Time_add(newalarm, now, credit);
        } else {
        // credit not yet expired; add credit; move alarm forward
        uart_puts_info("add to alarm\n\r");
        Time_add(newalarm, &alarm, credit);
    }
}

void code_alarm_triggered(void)
{
    play_song_expired();
    getTimeNow(&eeprom_time.now);    
    flash_write_eeprom_time(&eeprom_time);
}

void code_daily_flash(void)
{
    getTimeNow(&eeprom_time.now);        
    flash_write_eeprom_time(&eeprom_time);
}

void code_run_update(void)
{
    char log[10];
    if(read_print_rtc(true,log,0) == 0) {         // 0 credit left? (last day)
    }
}

uint32_t get_myid(void)
{
    // reserved 30 bits for id.
    //
    // truncate 128bit id.
    
    // ATSAMD21 replies: 0x67718634504a5230352e3120ff03171c
    // ATSAMD11 replies: 0xe1c74e77514d4b5331202020ff0b4631
    
    // most significant side seems to be more unique.
    
    volatile uint32_t *chip_ptr = (volatile uint32_t *)0x0080A00C; // word0 address
    return (*chip_ptr & 0x3fffffff); // mask 30bits
}
void print_myid(void)
{
    uint32_t chipval = get_myid();
    uart_print(0,"\n\r%09ld\n\r",chipval);
}

// *123456789# // for 3min demo
// *987654321# // for 5day demo
void code_demo(char *rxbuf, char *log, uint8_t allowed_log_len) 
{
	unsigned long givencode;
	uint8_t givenminutes=0;
	uint8_t givendays=0;
	rxbuf[0]=' '; // strip '*'
	rxbuf[10]=0;  // strip '#'
	
    if(strncasecmp(&rxbuf[1],"123456789",9)==0) {
	    givenminutes=3; // 3 minutes for demo
    } else if(strncasecmp(&rxbuf[1],"987654321",9)==0) {
	    givendays=5; // 5 days for demo
    } else {
        play_song_error();
        return;
    }    
    			
	givencode = strtoul(rxbuf,NULL,10);
	uart_print(0," code=%lu (0x%lx):",givencode,givencode);
	// done decoding the code
	Time now, credit, newalarm;
    uint8_t seconds_now;
	uart_puts_info("[setting alarm]");
				
	credit.year=0;
	credit.month=0;
	credit.day=givendays;
	credit.hour=0;
	credit.minute=givenminutes;
				
	getTimeNow(&now);
    seconds_now=getSeconds();
	generate_newalarm(&newalarm,&now,&credit);
	setAlarmTime(newalarm.hour,newalarm.minute,seconds_now);
	setAlarmDate(newalarm.day,newalarm.month,newalarm.year);
	enableAlarm(RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val);
				
	eeprom_time.alarm.hour   = newalarm.hour;
	eeprom_time.alarm.minute = newalarm.minute;
	eeprom_time.alarm.day    = newalarm.day;
	eeprom_time.alarm.month  = newalarm.month;
	eeprom_time.alarm.year   = newalarm.year;
	eeprom_time.now.hour   = now.hour;
	eeprom_time.now.minute = now.minute;
	eeprom_time.now.day    = now.day;
	eeprom_time.now.month  = now.month;
	eeprom_time.now.year   = now.year;
				
	eeprom_time.valid = true;
	eeprom_time.seq   = 1;
	eeprom_time.days  = givendays;
	eeprom_time.minutes = givenminutes;	
	flash_write_eeprom_time(&eeprom_time);
	read_print_rtc(true,log,allowed_log_len);
	play_song_success();
    task_set_load(1);
}

int code_real(char *rxbuf, char *log, uint8_t allowed_log_len) // *12345678901234#
{
    int cmdstat=0;
	unsigned long givenid;
	uint8_t givenseq;
	uint16_t givendays,givendays_i;
	int8_t givendays_int8;
	//uint64_t givencode;
	char rxbuf_cpy[32];
	rxbuf[16]=0;  // strip '#'
	strcpy(rxbuf_cpy,&rxbuf[1]); // skip '*'
				
	// decode the code:
	if(decode_rcode(rxbuf_cpy,&givenseq,&givenid,&givendays) > 0) {
    	uart_print(0," [seq=%d days=%d id=%ld]",givenseq,givendays,givenid);
        uart_print(0,"[eeprom seq=%d]",eeprom_time.seq);
    	if(givenid == get_myid()) {
        	uart_puts_info("[id match]");
        	if(givenseq > eeprom_time.seq || (givenseq==0 && eeprom_time.seq>95)) {
            	if(givendays > 0 && givendays < 1000) {
                	Time now, credit, newalarm;
                				
                	givendays_i=givendays;
                	while(givendays_i > 0) {
                    	getTimeNow(&now);
                    	uart_puts_info("[setting alarm]");
                    	credit.year=0;
                    	credit.month=0;
                    	// days is an int8_t, so max 127.  127-31=96 is the max increment
                    	if(givendays_i > 96) {givendays_int8=96;} else {givendays_int8=(int8_t)givendays_i;}
                    	givendays_i -= givendays_int8;
                    	credit.day=givendays_int8;
                    	credit.hour=0;
                    	credit.minute=0;
                    	generate_newalarm(&newalarm,&now,&credit);
                    	setAlarmTime(newalarm.hour,newalarm.minute,0);
                    	setAlarmDate(newalarm.day,newalarm.month,newalarm.year);
                    	enableAlarm(RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val);
                    	read_print_rtc(true,log,0);
                	}
                				
                	eeprom_time.alarm.hour   = newalarm.hour;
                	eeprom_time.alarm.minute = newalarm.minute;
                	eeprom_time.alarm.day    = newalarm.day;
                	eeprom_time.alarm.month  = newalarm.month;
                	eeprom_time.alarm.year   = newalarm.year;
                	eeprom_time.now.hour   = now.hour;
                	eeprom_time.now.minute = now.minute;
                	eeprom_time.now.day    = now.day;
                	eeprom_time.now.month  = now.month;
                	eeprom_time.now.year   = now.year;
                    
                } else {  // if givendays==0 stay on forever.
                	uart_puts_info("[disable alarm]");
                	disableAlarm();
            	}
            	eeprom_time.valid = true;
            	eeprom_time.seq   = givenseq;
            	eeprom_time.days  = givendays;
            	eeprom_time.minutes = 0;            	
            	flash_write_eeprom_time(&eeprom_time);
            	read_print_rtc(true,log,allowed_log_len);
            	play_song_success();
                task_set_load(1);
            } else {
            	uart_puts_info("[already used]");
                if(allowed_log_len>15) {                    
                    sprintf(log,"[used,myseq=%d]",eeprom_time.seq);
                }                
            	cmdstat=-1;
        	}
        } else {
        	uart_puts_info("[id not match]");
                if(allowed_log_len>0) strcat(log,"[!id]");
        	cmdstat=-1;
    	}
    } else {
    	uart_puts_info("[decode error]");
        if(allowed_log_len>0) strcat(log,"[!decode]");
    	cmdstat=-1;
	}
    return cmdstat;
}

// *1234# // for send SOC sms to operator 0
void code_sms(char *rxbuf)
{    
    char msg[CMD_MAX_COMMAND_LEN];
    if(strncasecmp(rxbuf,"*1234#",6)!=0) {
        play_song_error();
        return;
    }
    msg[0]=0; // no message, just the full SoC string
    task_add(TASK_SMS,msg,0); // to operator 0
}
