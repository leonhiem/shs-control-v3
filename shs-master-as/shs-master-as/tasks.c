#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "io.h"
#include "uart.h"
#include "sim900.h"
#include "shsctrl.h"
#include "delay.h"
#include "scc.h"
#include "quota.h"
#include "twi.h"


extern sysVals_t sysval;
extern uint32_t EEMEM EEPROM_ADDR_Ws_quota;
extern uint32_t EEMEM EEPROM_ADDR_secondsMeter;
extern uint32_t EEMEM EEPROM_ADDR_Ws_in;
extern uint32_t EEMEM EEPROM_ADDR_Ws_out;



/* 
 * Task for every second:
 */
void task_every_second(void)
{
}

/*
 * Task for every 5 seconds:
 */
void task_every_5second(void)
{
    int s,nof_scc=0;
    int16_t vpv=0, icharge=0, vbatt=0, iload=0, soc=0, socc=0, temp=0;
	uint16_t ydayl=0;
    int32_t pin=0,pout=0, Ah=0;
    uint16_t dt;

    scc_monitor();
    for(s=0;s<MAX_NOF_SCC;s++) {
		sysval.load_state &= ~(3<<(s*2)); // reset bit pair
		sysval.solar_state &= ~(3<<(s*2)); // reset bit pair
		sysval.batt_state &= ~(3<<(s*2)); // reset bit pair
		sysval.sys_state &= ~(3<<(s*2)); // reset bit pair
		sysval.day_state &= ~(3<<(s*2)); // reset bit pair
		
        if(sysval.scc_sysstate[s] & 0x80) { // valid SCC?
            nof_scc++;
            soc     += (int16_t)sysval.scc_soc[s];
			socc    += (int16_t)sysval.scc_socc[s];
            temp    += (int16_t)sysval.scc_temp[s];
            vpv     += sysval.scc_vpv[s];
            icharge += sysval.scc_icharge[s];
            vbatt   += sysval.scc_vbatt[s];
            iload   += sysval.scc_iload[s];
            pin     += ((int32_t)sysval.scc_vbatt[s] * (int32_t)sysval.scc_icharge[s]);
            pout    += ((int32_t)sysval.scc_vbatt[s] * (int32_t)sysval.scc_iload[s]);
            Ah      += (int32_t)sysval.scc_Ah[s];
            sysval.Door_open += sysval.scc_door[s];
			ydayl   += sysval.scc_ydayl[s];
        } else {
			// fill SCC values with errors or zeros:
			sysval.scc_loadstate[s] = 0x2;
			sysval.scc_solarstate[s] = 0x3;
			sysval.scc_battstate[s]  = 0x3;
			sysval.scc_sysstate[s]   = 0x3;
			sysval.scc_daystate[s]   = 0x0;
			sysval.scc_soc[s]=0;
			sysval.scc_socc[s]=0;
			sysval.scc_temp[s]=0;
			sysval.scc_vpv[s]=0;
			sysval.scc_icharge[s]=0;
			sysval.scc_vbatt[s]=0;
			sysval.scc_iload[s]=0;
			sysval.scc_ydayl[s]=0;
		}
		// SCC LOAD STATUS byte:
		// 76 54 32 10 (bit number)
		// 03 02 01 00 (SCC number)
		// fo fo fo fo (status: f=1=FAULT o=1=ON)
		sysval.load_state |= ((sysval.scc_loadstate[s]&0x3)<<(s*2)); // fill in bit pair

		// SCC SOLAR STATUS byte:
		sysval.solar_state |= ((sysval.scc_solarstate[s]&0x3)<<(s*2)); // fill in bit pair
		
		// SCC BATT STATUS byte:
		sysval.batt_state |= ((sysval.scc_battstate[s]&0x3)<<(s*2)); // fill in bit pair
		//
		// SCC SYS STATUS byte:
		sysval.sys_state |= ((sysval.scc_sysstate[s]&0x3)<<(s*2)); // fill in bit pair
		
		// SCC DAY STATUS byte:
		sysval.day_state |= ((sysval.scc_daystate[s]&0x3)<<(s*2)); // fill in bit pair
    }
    sysval.SoC      = (uint8_t)(soc / nof_scc);  // average
	sysval.SoCC     = (uint8_t)(socc / nof_scc);  // average
    sysval.temp     = (uint8_t)(temp / nof_scc); // average
    sysval.Vpv      = vpv   / nof_scc; // average
    sysval.Vbatt    = vbatt / nof_scc; // average
	sysval.ydayl    = ydayl / nof_scc; // average
	sysval.Ah       = Ah;      // sum
    sysval.I_charge = icharge; // sum
    sysval.I_load   = iload;   // sum
    pin  += (int32_t)5000;     // integer roundoff +0.5
    pout += (int32_t)5000;     // integer roundoff +0.5
    sysval.P_in     = pin/(int32_t)10000;  // to Watt
    sysval.P_out    = pout/(int32_t)10000; // to Watt	

    if(sysval.I_charge > 10) { // 0.1 A
        LED_CHARGING_ON();
    }
    if(sysval.I_charge < 5) { // 0.05 A
        LED_CHARGING_OFF();
    }

    if(sysval.seconds_old==0) {
        dt=0;
    } else {
        dt=(uint16_t)(sysval.seconds - sysval.seconds_old);
    }
    sysval.seconds_old=sysval.seconds;
    sysval.Ws_in+=(uint32_t)(sysval.P_in * dt);
    sysval.Ws_out+=(uint32_t)(sysval.P_out * dt);

    if(sysval.Ws_quota_enabled) {
        sysval.Ws_quota-=(int32_t)(sysval.P_out * dt);
        if(sysval.Ws_quota <= 0) { sysval.Ws_quota=0; }
    }
	scc_monitor_dump(NULL,0);
}

char sysstate2str(uint8_t si)
{
	uint8_t ss = (sysval.sys_state>>(si*2))&3;
	switch(ss) {
		case 0:  return 'I'; // INIT
		case 1:  return 'W'; // WAIT
		case 2:  return 'C'; // CHARGE
		default: return 'e'; // ERROR
	}
}
char solarstate2str(uint8_t si)
{
	uint8_t ss = (sysval.solar_state>>(si*2))&3;
	switch(ss) {
		case 0:  return 'O'; // OK
		case 1:  return 'L'; // LOW
		case 2:  return 'H'; // TOO HIGH
		default: return 'x'; // NOT AVAIL
	}
}

char loadstate2str(uint8_t si)
{
	// fo fo fo fo (status: f=1=FAULT o=1=ON)
	
	uint8_t ss = (sysval.load_state>>(si*2))&3;
	switch(ss) {
		case 0:  return '0'; // OFF
		case 1:  return '1'; // ON
		case 2:  return 'f'; // FAULT, OFF
		default: return 'F'; // FAULT, ON
	}
}

char daystate2str(uint8_t si)
{
	// dp dp dp dp (status: d=1=DAY p=1=PM)
	
	uint8_t ss = (sysval.day_state>>(si*2))&3;
	switch(ss) {
		case 0:  return 'z'; // NIGHT
		case 1:  return 'z'; // NIGHT
		case 2:  return 'A'; // DAY, AM
		default: return 'P'; // DAY, PM
	}
}

uint16_t task_soc_dump(char *output_buf)
{
	uint16_t len=0;
	len=sprintf(output_buf,   "\r\nSoC  Vpv  Icharge  Vbatt  Iload temp  Pin Pout  Ws_in  Ws_out  uptime\r\n");
	len+=sprintf(&output_buf[len],"%3d ",sysval.SoC);
	len+=sprintf(&output_buf[len]," %4d ",sysval.Vpv);
	len+=sprintf(&output_buf[len]," %4d ",sysval.I_charge);
	len+=sprintf(&output_buf[len],"    %4d ",sysval.Vbatt);
	len+=sprintf(&output_buf[len]," %4d ",sysval.I_load);
	len+=sprintf(&output_buf[len],"   %2d ",sysval.temp);
	len+=sprintf(&output_buf[len]," %3d ",sysval.P_in);
	len+=sprintf(&output_buf[len]," %3d ",sysval.P_out);
	len+=sprintf(&output_buf[len]," %5ld ",sysval.Ws_in);
	len+=sprintf(&output_buf[len]," %5ld ",sysval.Ws_out);	
	len+=sprintf(&output_buf[len]," %6ld ",sysval.seconds);
	len+=sprintf(&output_buf[len],"\r\n");
	return len;
}

void task_set_load(uint8_t onoff)
{
    scc_set_load(onoff);
}

void task_cal_temp(uint8_t temp)
{
	scc_cal_temp(temp);
}

void task_cal_batt(int8_t offset)
{
    scc_cal_batt(offset);
}

void task_set_ah_batt(int8_t ah)
{
    scc_set_ah_batt(ah);
}

void task_start_equalize(void)
{
	scc_start_equalize();
}

/*
 * Task for every minute:
 */
void task_every_minute(void)
{
	scc_read_battery();
	
    /*
     * Display Quota if enabled:
     */
    if(sysval.Ws_quota_enabled) {
        // display Quota
        uint8_t tmp8 = quota_to_percent((uint16_t)(sysval.Ws_quota/3600),
               (uint16_t)(eeprom_read_dword(&EEPROM_ADDR_Ws_quota)/3600));
        if(tmp8 < 20) {
            LED_100PERCENT_OFF();
            LED_60PERCENT_OFF();
            LED_30PERCENT_ON();
        } else if(tmp8 <= 80) {
            LED_100PERCENT_OFF();
            LED_60PERCENT_ON();
            LED_30PERCENT_OFF();
        } else {
            LED_100PERCENT_ON();
            LED_60PERCENT_OFF();
            LED_30PERCENT_OFF();
        }
    }
}


/*
 * Task for every hour:
*/
void task_hourly(void)
{
    uint8_t sreg;
    sreg=SREG; cli();
    eeprom_update_dword(&EEPROM_ADDR_secondsMeter,sysval.seconds);
    SREG=sreg;

    sreg=SREG; cli();
    eeprom_update_dword(&EEPROM_ADDR_Ws_in,sysval.Ws_in);
    SREG=sreg;

    sreg=SREG; cli();
    eeprom_update_dword(&EEPROM_ADDR_Ws_out,sysval.Ws_out);
    SREG=sreg;
}


/*
 * Task for a new day:
 */
void task_daily(void)
{
    // new quota for the new day
    sysval.Ws_quota  = (int32_t)eeprom_read_dword(&EEPROM_ADDR_Ws_quota);

/*
    if(sim_is_on()) {
        sim_onoff_pulse(); // just turn off SIM900 (next minute back on)
    }
*/
}

