/*
 * DAQ 
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

#include "shsctrl.h"
#include "scc.h"
#include "io.h"
#include "twi.h"
#include "delay.h"



extern sysVals_t sysval;

char * battstate_to_str(unsigned char battstate)
{
    switch(battstate) {
      case 0:
        return(" NORMAL");
      case 1:
        return(" FULL  ");
      case 2: 
        return(" EMPTY!");
      case 3: 
        return(" GONE! ");
      default: 
        return(" ??????");
    }
}
char * sysstate_status_to_str(unsigned char sysstate)
{
	if(sysstate&0x80) return("OK "); else return("XX ");
}
char * sysstate_to_str(unsigned char sysstate)
{
    switch(sysstate&0xf) {
      case 0:
        return("INIT  ");
      case 1:
        return("WAIT  ");
      case 2:
        return("CHARGE");
      case 3: 
        return("ERROR ");
      default: 
        return("??????");
    }
}
char * solarstate_to_str(unsigned char solarstate)
{
    switch(solarstate) {
      case 0:
        return("  OK  ");
      case 1:
        return("  LOW ");
      case 2:
        return("  HIGH");
      default: 
        return("  ????");
    }
}
char * loadstate_to_str(unsigned char loadstate)
{
    switch(loadstate) {
      case 0:
        return(" OFF OK");
      case 1:
        return(" ON  OK");
      case 2:
        return(" OFF :(");
      case 3:
        return(" ON  :(");
      default:
        return(" ??????");
    }
}
short vsolar_to_volt(unsigned short vsolar)
{
    uint32_t v=(uint32_t)vsolar;
    v *= 13255;
    v /= 1023;
    return (short)v;
}
short isense_to_amps(unsigned short isense)
{
    // 0.4 @ 4A
    uint32_t i=(uint32_t)isense;
    i *= 3300;
    i /= 1023;
    return (short)i;
}
short vbatt_to_volt(unsigned short vbatt)
{
    uint32_t v=(uint32_t)vbatt;
    v *= 1650;
    v /= 1023;
    return (short)v;
}


void scc_monitor(void)
{
    uint8_t twi_sla;
    uint8_t twi_data[16];
    unsigned char s,checksum,checksum_chk;
    int8_t step;
    unsigned short tmp_adc;	
	unsigned short day_meter,minute_meter;

    wdt_reset();
	sysval.watchdog=0;

    for(s=0;s<MAX_NOF_SCC;s++) {
        twi_sla=I2C_SCC_BASE_ADDR+s;
        delay_ms(50);
        memset(twi_data,0,16);
        sysval.scc_sysstate[s]=0;
        if(twi_write_one(twi_sla,I2C_CMD_READSTAT_WDT) >= 0) {
            delay_ms(5);
            if(twi_read_small(16,twi_data,twi_sla) >= 0) {
                int i;
                checksum=twi_data[15];
                twi_data[15]=0;
                checksum_chk=0;
                for(i=0;i<16;i++) {
                    checksum_chk+=twi_data[i];
                }
                if(checksum_chk==checksum) {
					i=1;
					day_meter =((unsigned short)twi_data[i++])<<8;
					day_meter|=((unsigned short)twi_data[i++])&0xff;
					sysval.scc_days[s]       = day_meter;
					
					minute_meter =((unsigned short)twi_data[i++])<<8;
					minute_meter|=((unsigned short)twi_data[i++])&0xff;
					//printf("[%d] dayl=%d min=%d\n",s,day_meter,minute_meter);					
					
                    i=5;
                    sysval.scc_sysstate[s]   = twi_data[i++];
                    sysval.scc_battstate[s]  = twi_data[i++];
                    sysval.scc_solarstate[s] = twi_data[i++];
                    sysval.scc_loadstate[s]  = twi_data[i++];
                    sysval.scc_soc[s]        = twi_data[i++];
					
                    step                     = twi_data[i++]; //i++; // mppt_stepsize
                    printf("[%d] MPPT=%d:",s,step);
                    if(step<-1)     printf(0,"[<<<|   ]\n\r");
                    else if(step<0) printf(0,"[  <|   ]\n\r");
                    else if(step>1) printf(0,"[   |>>>]\n\r");
                    else if(step>0) printf(0,"[   |>  ]\n\r");
                    else            printf(0,"[   |   ]\n\r");

                    sysval.scc_door[s]       = twi_data[i++];
					sysval.scc_daystate[s]   = twi_data[i++];
					
					day_meter =((unsigned short)twi_data[i++])<<8;
					day_meter|=((unsigned short)twi_data[i++])&0xff;
					sysval.scc_ydayl[s]      = day_meter;
					
                } else sysval.scc_sysstate[s] |= 0x40; //communication was ERROR

                delay_ms(10);
                memset(twi_data,0,16);
                if(twi_write_one(twi_sla,I2C_CMD_READADC) >= 0) {
                    delay_ms(5);
                    if(twi_read_small(16,twi_data,twi_sla) >= 0) {
                        int i;
                        checksum=twi_data[15];
                        twi_data[15]=0;
                        checksum_chk=0;
                        for(i=0;i<16;i++) {
                            checksum_chk+=twi_data[i];
                        }
                        if(checksum_chk==checksum) {
                            i=1;
                            tmp_adc =((unsigned short)twi_data[i++])<<8;
                            tmp_adc|=((unsigned short)twi_data[i++])&0xff;
                            printf("[%d] vpv_oc=%d\n",s,vsolar_to_volt(tmp_adc));
							
                            i=3;
                            tmp_adc =((unsigned short)twi_data[i++])<<8;
                            tmp_adc|=((unsigned short)twi_data[i++])&0xff;
                            sysval.scc_vpv[s]=tmp_adc;
        
                            tmp_adc =((unsigned short)twi_data[i++])<<8;
                            tmp_adc|=((unsigned short)twi_data[i++])&0xff;
                            sysval.scc_icharge[s]=tmp_adc;
        
                            tmp_adc =((unsigned short)twi_data[i++])<<8;
                            tmp_adc|=((unsigned short)twi_data[i++])&0xff;
                            sysval.scc_vbatt[s]=tmp_adc;
        
                            tmp_adc =((unsigned short)twi_data[i++])<<8;
                            tmp_adc|=((unsigned short)twi_data[i++])&0xff;
                            sysval.scc_iload[s]=tmp_adc;							
        
                            sysval.scc_temp[s]=twi_data[i++];							
                            tmp_adc =((unsigned short)twi_data[i++])<<8;
                            tmp_adc|=((unsigned short)twi_data[i++])&0xff;
                            printf("[%d] vpv_sp=%d\n\r",s,vsolar_to_volt(tmp_adc));
                            //printf("[%d] Vfloat=%d\n",s,vbatt_to_volt(tmp_adc));
							
                            sysval.scc_sysstate[s] |= 0x80; //communication was OK 
                        } else sysval.scc_sysstate[s] |= 0x40; //communication was ERROR
                    } else sysval.scc_sysstate[s] |= 0x20; //communication was ERROR
                } else sysval.scc_sysstate[s] |= 0x20; //communication was ERROR
            } else sysval.scc_sysstate[s] |= 0x20; //communication was ERROR
        } else sysval.scc_sysstate[s] |= 0x20; //communication was ERROR
    }
}

uint16_t scc_monitor_dump(char *output_buf, int part)
{
	uint16_t len=0;
	uint8_t twi_sla,s;
	
	if(output_buf==NULL) {
		printf_P(PSTR("\r\nSCC:    system battst solst SoC   Vpv  Icharge  Vbatt  Iload temp loadstate"));
		printf_P(PSTR("\r\n--------------+------+-----+---+------+--------+-----+------+----+---------"));
		for(s=0;s<MAX_NOF_SCC;s++) {
			twi_sla=I2C_SCC_BASE_ADDR+s;
			printf("\r\n[%2x]:",twi_sla);
			printf("%s",sysstate_status_to_str(sysval.scc_sysstate[s]));
			printf("%s",sysstate_to_str(sysval.scc_sysstate[s]));
			printf("%s",battstate_to_str(sysval.scc_battstate[s]));
			printf("%s",solarstate_to_str(sysval.scc_solarstate[s]));
			printf(" %3d ",sysval.scc_soc[s]);

			printf(" %4d ",sysval.scc_vpv[s]);
			printf(" %4d ",sysval.scc_icharge[s]);
			printf("    %4d ",sysval.scc_vbatt[s]);
			printf(" %4d ",sysval.scc_iload[s]);
			printf("    %2d ",sysval.scc_temp[s]);
			printf("%s",loadstate_to_str(sysval.scc_loadstate[s]));
		}
		printf("\r\n");	
		printf_P(PSTR("\r\nSCC:  days  ydayl   Ah   SoCC "));
		printf_P(PSTR("\r\n----------+------+------+----+"));	
		for(s=0;s<MAX_NOF_SCC;s++) {
			twi_sla=I2C_SCC_BASE_ADDR+s;
			printf("\r\n[%2x]:",twi_sla);
			printf(" %4d",sysval.scc_days[s]);
			printf("  %4d",sysval.scc_ydayl[s]);
			printf("  %5d",sysval.scc_Ah[s]);
			printf("  %3d",sysval.scc_socc[s]);		
		}
		printf("\r\n");
	} else {
		if(part==0) {
			len=sprintf(output_buf,       "\r\nSCC:    system battst solst SoC   Vpv  Icharge  Vbatt  Iload temp loadstat\r\n");
			for(s=0;s<MAX_NOF_SCC;s++) {
				twi_sla=I2C_SCC_BASE_ADDR+s;
				len+=sprintf(&output_buf[len],"[%2x]:",twi_sla);
				len+=sprintf(&output_buf[len],"%s",sysstate_status_to_str(sysval.scc_sysstate[s]));
				len+=sprintf(&output_buf[len],"%s",sysstate_to_str(sysval.scc_sysstate[s]));
				len+=sprintf(&output_buf[len],"%s",battstate_to_str(sysval.scc_battstate[s]));
				len+=sprintf(&output_buf[len],"%s",solarstate_to_str(sysval.scc_solarstate[s]));
				len+=sprintf(&output_buf[len]," %3d ",sysval.scc_soc[s]);

				len+=sprintf(&output_buf[len]," %4d ",sysval.scc_vpv[s]);
				len+=sprintf(&output_buf[len]," %4d ",sysval.scc_icharge[s]);
				len+=sprintf(&output_buf[len],"    %4d ",sysval.scc_vbatt[s]);
				len+=sprintf(&output_buf[len]," %4d ",sysval.scc_iload[s]);
				len+=sprintf(&output_buf[len],"    %2d ",sysval.scc_temp[s]);
				len+=sprintf(&output_buf[len],"%s\r\n",loadstate_to_str(sysval.scc_loadstate[s]));
			}
			len+=sprintf(&output_buf[len],"\r\n");
		} else if(part==1) {
			len=sprintf(output_buf,       "\r\nSCC:  days  ydayl   Ah   SoCC\r\n");
			for(s=0;s<MAX_NOF_SCC;s++) {
				twi_sla=I2C_SCC_BASE_ADDR+s;
				len+=sprintf(&output_buf[len],"[%2x]:",twi_sla);
				len+=sprintf(&output_buf[len]," %4d",sysval.scc_days[s]);
				len+=sprintf(&output_buf[len],"  %4d",sysval.scc_ydayl[s]);
				len+=sprintf(&output_buf[len],"  %5d",sysval.scc_Ah[s]);
				len+=sprintf(&output_buf[len],"  %3d\r\n",sysval.scc_socc[s]);				
			}
			len+=sprintf(&output_buf[len],"\r\n");
		}
	}
	return len;
}

void scc_send_cmd(uint8_t cmd, uint8_t val, uint8_t write_val)
{
    uint8_t twi_sla;
    uint8_t twi_cmd;
    uint8_t twi_data[16];
    unsigned char s,checksum,checksum_chk;

    twi_cmd=cmd;

    // send command to all SCC boards
    for(s=0;s<MAX_NOF_SCC;s++) {
        twi_sla=I2C_SCC_BASE_ADDR+s;
        delay_ms(50);
        memset(twi_data,0,16);
        sysval.scc_sysstate[s] &= 0x0f; // reset bit 7..4
        if(twi_write_one(twi_sla,twi_cmd) >= 0) {
			if(write_val) { twi_write_one(twi_sla,val); }
            delay_ms(5);
            if(twi_read_small(16,twi_data,twi_sla) >= 0) {
                int i;
                checksum=twi_data[15];
                twi_data[15]=0;
                checksum_chk=0;
                for(i=0;i<16;i++) {
                    checksum_chk+=twi_data[i];
                }
                if(checksum_chk==checksum) {
                    sysval.scc_sysstate[s] |= 0x80; //communication was OK
                } else sysval.scc_sysstate[s] |= 0x40; //communication was ERROR
            } else sysval.scc_sysstate[s] |= 0x20; //communication was ERROR
        } else sysval.scc_sysstate[s] |= 0x20; //communication was ERROR
    }
}

void scc_set_load(uint8_t onoff)
{
	printf("scc_set_load(%d)\n",onoff);
	if(onoff==0) {
		scc_send_cmd(I2C_CMD_LOAD_OFF,0,0);
	} else {
		scc_send_cmd(I2C_CMD_LOAD_ON,0,0);
	}
}


void scc_cal_temp(uint8_t temp)
{
	printf("scc_cal_temp(%d)\n",temp);
	scc_send_cmd(I2C_CMD_CALTEMP,temp,1);
}

void scc_cal_batt(int8_t offset)
{
	printf("scc_cal_batt(%d)\n",offset);
	scc_send_cmd(I2C_CMD_CALBATT,offset,1);
}

void scc_set_ah_batt(int8_t ah)
{
	printf("scc_set_ah_batt(%d)\n",ah);
	scc_send_cmd(I2C_CMD_SETAHBATT,ah,1);
}

void scc_start_equalize(void)
{
	printf("scc_start_equalize()\n");
	scc_send_cmd(I2C_CMD_EQUALIZE,0,0);
}


void scc_read_battery(void)
{
	uint8_t twi_sla;
	uint8_t twi_data[16];
	unsigned char s,checksum,checksum_chk;
	uint32_t As, As_max;
	uint16_t Ah;
	uint8_t equalizing;
		
	wdt_reset();
	
	printf("scc_read_battery:\n");
	
	for(s=0;s<MAX_NOF_SCC;s++) {
		twi_sla=I2C_SCC_BASE_ADDR+s;
		delay_ms(50);
		memset(twi_data,0,16);
		sysval.scc_sysstate[s]=0;
		if(twi_write_one(twi_sla,I2C_CMD_READBATT) >= 0) {
			delay_ms(5);
			if(twi_read_small(16,twi_data,twi_sla) >= 0) {
				int i;
				checksum=twi_data[15];
				twi_data[15]=0;
				checksum_chk=0;
				for(i=0;i<16;i++) {
					checksum_chk+=twi_data[i];
				}
				if(checksum_chk==checksum) {
					i=1;
					As =((uint32_t)twi_data[i++])<<24;
					As|=((uint32_t)twi_data[i++])<<16;
					As|=((uint32_t)twi_data[i++])<<8;
					As|=((uint32_t)twi_data[i++])&0xff;
					
					As_max =((uint32_t)twi_data[i++])<<24;
					As_max|=((uint32_t)twi_data[i++])<<16;
					As_max|=((uint32_t)twi_data[i++])<<8;
					As_max|=((uint32_t)twi_data[i++])&0xff;
					
					equalizing = twi_data[i++];
								
					printf("[%d] As=%ld As_max=%ld equalize=%d\n",s,As,As_max,equalizing);
					Ah=(uint16_t)(As/3600L);
					sysval.scc_Ah[s] = Ah;
					//printf("[%d] Ah=%d\n",s,sysval.scc_Ah[s]);
					
					if((As_max >= As) && (As > 0)) {
						As *= 100UL;
						As = As / As_max;						
						sysval.scc_socc[s] = (uint8_t)As;
						//printf("[%d] SoCC=%d\n",s,sysval.scc_socc[s]);
					}
					sysval.scc_sysstate[s] |= 0x80; //communication was OK
				} else sysval.scc_sysstate[s] |= 0x40; //communication was ERROR			
			} else sysval.scc_sysstate[s] |= 0x20; //communication was ERROR
		} else sysval.scc_sysstate[s] |= 0x20; //communication was ERROR
	}	
}
