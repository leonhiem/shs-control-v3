/*
 * DAQ 
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#if (SYSTEM_CCNEO_SCC==1)

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "samd20.h"
#include "shsctrl.h"
#include "scc.h"
#include "i2c.h"
#include "utils.h"
#include "uart.h"

extern sysVals_t sysval;
static uint8_t scc_errors=0;
static uint8_t scc_do_probe=0;
static uint8_t scc_i2c_addr[MAX_NOF_SCC];
static uint8_t scc_loadstate=0;

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

void scc_init(uint8_t onoff)
{
    scc_do_probe=onoff;
}

void scc_probe(void)
{
    int8_t i,j=0,valid_addr;
    sysval.scc_nof_scc=0;
    for (i=0;i<MAX_NOF_SCC;i++) {
        scc_i2c_addr[i]=I2C_SCC_BASE_ADDR+i;
    }    
    for (i=0;i<MAX_NOF_SCC;i++) {
        valid_addr=scc_monitor(i);
        if(valid_addr) {
            scc_i2c_addr[sysval.scc_nof_scc]=valid_addr;
            sysval.scc_nof_scc++;
        }
    }    
    for (i=sysval.scc_nof_scc;i<MAX_NOF_SCC;i++) {
        scc_i2c_addr[i]=0;
    }    
    uart_print(0,"scc: %d SCC's detected\n\r",sysval.scc_nof_scc);
    for (i=0;i<MAX_NOF_SCC;i++) {
        uart_print(0,"scc[%d]=0x%x\n\r",i,scc_i2c_addr[i]);
    }
    if(sysval.scc_nof_scc>0) scc_do_probe=0;
}

void scc_tasks(void)
{
    uint8_t tasklist;
    uint8_t subtask;
    __disable_irq();
    tasklist=sysval.scc_tasklist;
    sysval.scc_tasklist&=0x7f;    
    __enable_irq();
    subtask=sysval.scc_taskidx;

    if(tasklist & 0x80) { // task allowed to run
        if(scc_do_probe) {
            scc_probe();
            return;
        }
        scc_monitor(subtask);
        //scc_read_battery(subtask);            
        scc_update_sysval(); 
        subtask++;
                     
        if(subtask>=sysval.scc_nof_scc) sysval.scc_taskidx=0; else sysval.scc_taskidx=subtask;
    }
}

uint8_t scc_monitor(uint8_t s)
{
    uint8_t i2c_sla;
    uint8_t i2c_data[16];
    uint8_t i2c_cmd[2];
    unsigned char checksum,checksum_chk;
    int8_t step;
    unsigned short tmp_adc;    
    unsigned short day_meter,minute_meter;

    if(s>(MAX_NOF_SCC-1)) return 0;
    uart_print(0,"scc_monitor[%d]=0x%x\n\r",s,scc_i2c_addr[s]);

    if(scc_errors>5) { 
        scc_errors=0; 
        i2c_reset();
        delay_ms(20);
    }
        
    sysval.watchdog=0;
    
    i2c_sla=scc_i2c_addr[s];
    memset(i2c_data,0,16);
    sysval.scc_sysstate[s]&=0x7f;
    i2c_cmd[0]=I2C_CMD_STAT_WDT;
    i2c_cmd[1]=scc_loadstate;
    if(i2c_transact(i2c_sla,i2c_cmd,2,i2c_data,16) >= 0) {
        int i;
        checksum=i2c_data[15];
        i2c_data[15]=0;
        checksum_chk=0;
        for(i=0;i<16;i++) {
            checksum_chk+=i2c_data[i];
        }
        if(checksum_chk==checksum) {
            i=1;
            day_meter =((unsigned short)i2c_data[i++])<<8;
            day_meter|=((unsigned short)i2c_data[i++])&0xff;
            sysval.scc_days[s]       = day_meter;
                    
            minute_meter =((unsigned short)i2c_data[i++])<<8;
            minute_meter|=((unsigned short)i2c_data[i++])&0xff;
            //uart_print(0,"[%d] dayl=%d min=%d\n\r",s,day_meter,minute_meter);                    
                    
            i=5;
            sysval.scc_sysstate[s]   = i2c_data[i++];
            sysval.scc_battstate[s]  = i2c_data[i++];
            sysval.scc_solarstate[s] = i2c_data[i++];
            sysval.scc_loadstate[s]  = i2c_data[i++];
            sysval.scc_soc[s]        = i2c_data[i++];
                    
            step                     = (int8_t)i2c_data[i++]; // mppt_indicator
            uart_print(0,"MPPT=%d:",step);
            if(step<-5)     uart_print(0,"[<<<|   ]\n\r");
            else if(step<0) uart_print(0,"[  <|   ]\n\r");
            else if(step>5) uart_print(0,"[   |>>>]\n\r");
            else if(step>0) uart_print(0,"[   |>  ]\n\r");
            else            uart_print(0,"[   |   ]\n\r");
                    
            sysval.scc_door[s]       = i2c_data[i++];
            sysval.scc_daystate[s]   = i2c_data[i++];
                
            day_meter =((unsigned short)i2c_data[i++])<<8;
            day_meter|=((unsigned short)i2c_data[i++])&0xff;
            sysval.scc_ydayl[s]      = day_meter;                    
        } else uart_print(0,"1 checksum error\r\n");    
        
        delay_ms(20);
        memset(i2c_data,0,16);
        i2c_cmd[0]=I2C_CMD_READADC;
        if(i2c_transact(i2c_sla,i2c_cmd,1,i2c_data,16) >= 0) {                   
            int i;
            checksum=i2c_data[15];
            i2c_data[15]=0;
            checksum_chk=0;
            for(i=0;i<16;i++) {
                checksum_chk+=i2c_data[i];
            }
            if(checksum_chk==checksum) {
                i=1;
                tmp_adc =((unsigned short)i2c_data[i++])<<8;
                tmp_adc|=((unsigned short)i2c_data[i++])&0xff;
                uart_print(0,"[%d] vpv_oc=%d\n\r",s,vsolar_to_volt(tmp_adc));
                            
                i=3;
                tmp_adc =((unsigned short)i2c_data[i++])<<8;
                tmp_adc|=((unsigned short)i2c_data[i++])&0xff;
                sysval.scc_vpv[s]=tmp_adc;
    
                tmp_adc =((unsigned short)i2c_data[i++])<<8;
                tmp_adc|=((unsigned short)i2c_data[i++])&0xff;
                sysval.scc_icharge[s]=tmp_adc;
    
                tmp_adc =((unsigned short)i2c_data[i++])<<8;
                tmp_adc|=((unsigned short)i2c_data[i++])&0xff;
                sysval.scc_vbatt[s]=tmp_adc;
    
                tmp_adc =((unsigned short)i2c_data[i++])<<8;
                tmp_adc|=((unsigned short)i2c_data[i++])&0xff;
                sysval.scc_iload[s]=tmp_adc;                            
    
                sysval.scc_temp[s]=i2c_data[i++];                            
                            
                tmp_adc =((unsigned short)i2c_data[i++])<<8;
                tmp_adc|=((unsigned short)i2c_data[i++])&0xff;
                uart_print(0,"[%d] vpv_sp=%d\n\r",s,vsolar_to_volt(tmp_adc));
                //uart_print(0,"[%d] Vfloat=%d\n\r",s,vbatt_to_volt(tmp_adc));
                            
                uart_print(0,"scc[%d] FW=%x\n\r",s,i2c_data[i++]);                            

                sysval.scc_sysstate[s] |= 0x80; //communication was OK 
                scc_errors=0;
            } else { 
                uart_print(0,"2 checksum error\r\n"); 
                sysval.scc_sysstate[s] |= 0x40; scc_errors++; 
            }//communication was ERROR                    
        } else { 
            uart_print(0,"1 comm error\r\n"); 
            sysval.scc_sysstate[s] |= 0x20; 
            scc_errors++; 
        }//communication was ERROR
    } else { 
        uart_print(0,"2 comm error\r\n"); 
        sysval.scc_sysstate[s] |= 0x20; 
        scc_errors++;
    }//communication was ERROR

    if(scc_errors) {
        int i;
        uart_print(0,"scc i2c error:");
        for(i=0;i<16;i++) uart_print(0,"[%x]",i2c_data[i]);
        uart_print(0,"\n\r");
        return 0; 
    } else return i2c_sla;
}


void scc_send_cmd(uint8_t cmd, uint8_t val, uint8_t write_val)
{
    uint8_t i2c_sla;
    uint8_t i2c_cmd[2];
    uint8_t i2c_cmd_len;
    uint8_t i2c_data[16];
    unsigned char s,checksum,checksum_chk;

    i2c_cmd[0]=cmd;    
    if(write_val) {
        i2c_cmd[1]=val;
        i2c_cmd_len=2;
    } else {
        i2c_cmd_len=1;
    }

    // send command to all SCC boards
    for(s=0;s<sysval.scc_nof_scc;s++) {
        uint8_t retries=10;
        while(retries--) {
            if(scc_errors>5) { 
                scc_errors=0; 
                i2c_reset();
                delay_ms(20);
            }

            i2c_sla=scc_i2c_addr[s];
            memset(i2c_data,0,16);
            sysval.scc_sysstate[s] &= 0x0f; // reset bit 7..4
            if(i2c_transact(i2c_sla,i2c_cmd,i2c_cmd_len,i2c_data,16) >= 0) {
                int i;
                checksum=i2c_data[15];
                i2c_data[15]=0;
                checksum_chk=0;
                for(i=0;i<16;i++) {
                    checksum_chk+=i2c_data[i];
                }
                if(checksum_chk==checksum) {
                    sysval.scc_sysstate[s] |= 0x80; //communication was OK
                    scc_errors=0;
                } else {
                    sysval.scc_sysstate[s] |= 0x40; //communication was ERROR
                    scc_errors++;
                }
            } else {
                sysval.scc_sysstate[s] |= 0x20; //communication was ERROR
                scc_errors++;
            }
            if(scc_errors==0) break;
        }
    }
}

void scc_set_load(uint8_t onoff)
{
    if(onoff != 0) scc_loadstate=1; else scc_loadstate=0;
    uart_print(0,"scc_set_load(%d)\n\r",scc_loadstate);
}

void scc_cal_temp(uint8_t temp)
{
    uart_print(0,"scc_cal_temp(%d)\n\r",temp);
    scc_send_cmd(I2C_CMD_CALTEMP,temp,1);
}

void scc_cal_batt(int8_t offset)
{
    uart_print(0,"scc_cal_batt(%d)\n\r",offset);
    scc_send_cmd(I2C_CMD_CALBATT,offset,1);
}

void scc_set_ah_batt(int8_t ah)
{
    uart_print(0,"scc_set_ah_batt(%d)\n\r",ah);
    scc_send_cmd(I2C_CMD_SETAHBATT,ah,1);
}

void scc_start_equalize(void)
{
    uart_print(0,"scc_start_equalize()\n\r");
    scc_send_cmd(I2C_CMD_EQUALIZE,0,0);
}

uint8_t scc_read_battery(uint8_t s)
{
    uint8_t i2c_sla;
    uint8_t i2c_cmd = I2C_CMD_READBATT;
    uint8_t i2c_data[16];
    unsigned char checksum,checksum_chk;
    uint32_t As, As_max;
    uint16_t Ah;
    uint8_t equalizing;

    if(s>(MAX_NOF_SCC-1)) return 0;
    uart_print(0,"scc_read_battery[%d]\n\r",s);
            
    if(scc_errors>5) { 
        scc_errors=0; 
        i2c_reset();
        delay_ms(20);
    }

    i2c_sla=scc_i2c_addr[s];
    memset(i2c_data,0,16);
    sysval.scc_sysstate[s]&=0x7f;
    if(i2c_transact(i2c_sla,&i2c_cmd,1,i2c_data,16) >= 0) {
        int i;
        checksum=i2c_data[15];
        i2c_data[15]=0;
        checksum_chk=0;
        for(i=0;i<16;i++) {
            checksum_chk+=i2c_data[i];
        }
        if(checksum_chk==checksum) {
            i=1;
            As =((uint32_t)i2c_data[i++])<<24;
            As|=((uint32_t)i2c_data[i++])<<16;
            As|=((uint32_t)i2c_data[i++])<<8;
            As|=((uint32_t)i2c_data[i++])&0xff;
                
            As_max =((uint32_t)i2c_data[i++])<<24;
            As_max|=((uint32_t)i2c_data[i++])<<16;
            As_max|=((uint32_t)i2c_data[i++])<<8;
            As_max|=((uint32_t)i2c_data[i++])&0xff;
                
            equalizing = i2c_data[i++];
                            
            uart_print(0,"[%d] As=%ld As_max=%ld equalize=%d\n\r",s,As,As_max,equalizing);
            Ah=(uint16_t)(As/3600L);
            sysval.scc_Ah[s] = Ah;
            //uart_print(0,"[%d] Ah=%d\n\r",s,sysval.scc_Ah[s]);
                
            if((As_max >= As) && (As > 0)) {
                As *= 100UL;
                As = As / As_max;                        
                sysval.scc_socc[s] = (uint8_t)As;
                //uart_print(0,"[%d] SoCC=%d\n\r",s,sysval.scc_socc[s]);
            }
            sysval.scc_sysstate[s] |= 0x80; //communication was OK
            scc_errors=0;
        } else {
            sysval.scc_sysstate[s] |= 0x40; //communication was ERROR
            scc_errors++;
        }
    } else {
        sysval.scc_sysstate[s] |= 0x20; //communication was ERROR
        scc_errors++;
    }
    if(scc_errors) {
        int i;
        uart_print(0,"scc i2c error:");
        for(i=0;i<16;i++) uart_print(0,"[%x]",i2c_data[i]);
        uart_print(0,"\n\r");
        return 0; 
    } else return 1;
}

void scc_read_batteries(void)
{
    unsigned char s;
    for(s=0;s<sysval.scc_nof_scc;s++) {
        scc_read_battery(s);
    }
    scc_monitor_dump(NULL,1);
}

void scc_update_sysval(void)
{
    int s,nof_scc=0;
    int16_t vpv=0, icharge=0, vbatt=0, iload=0, soc=0, socc=0, temp=0;
    uint16_t ydayl=0;
    int32_t pin=0,pout=0, Ah=0;
    uint16_t dt;

    for(s=0;s<sysval.scc_nof_scc;s++) {
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
    if(nof_scc>0) {
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

        if(sysval.seconds_old==0) {
            dt=0;
        } else {
            dt=(uint16_t)(sysval.seconds - sysval.seconds_old);
        }
        sysval.seconds_old=sysval.seconds;
        sysval.Ws_in+=(uint32_t)(sysval.P_in * dt);
        sysval.Ws_out+=(uint32_t)(sysval.P_out * dt);
    }    
    scc_monitor_dump(NULL,0);
}

uint16_t scc_monitor_dump(char *output_buf, int part)
{
    uint16_t len=0;
    uint8_t i2c_sla,s;
    
    if(output_buf==NULL) {
        if(part==0) {
            uart_print(0,"\r\nSCC:    system battst solst SoC   Vpv  Icharge  Vbatt  Iload temp loadstate");
            uart_print(0,"\r\n--------------+------+-----+---+------+--------+-----+------+----+---------");
            for(s=0;s<sysval.scc_nof_scc;s++) {
                i2c_sla=scc_i2c_addr[s];
                uart_print(0,"\r\n[%2x]:",i2c_sla);
                uart_print(0,"%s",sysstate_status_to_str(sysval.scc_sysstate[s]));
                uart_print(0,"%s",sysstate_to_str(sysval.scc_sysstate[s]));
                uart_print(0,"%s",battstate_to_str(sysval.scc_battstate[s]));
                uart_print(0,"%s",solarstate_to_str(sysval.scc_solarstate[s]));
                uart_print(0," %3d ",sysval.scc_soc[s]);

                uart_print(0," %4d ",sysval.scc_vpv[s]);
                uart_print(0," %4d ",sysval.scc_icharge[s]);
                uart_print(0,"    %4d ",sysval.scc_vbatt[s]);
                uart_print(0," %4d ",sysval.scc_iload[s]);
                uart_print(0,"    %2d ",sysval.scc_temp[s]);
                uart_print(0,"%s",loadstate_to_str(sysval.scc_loadstate[s]));
            }
            uart_print(0,"\r\n");
        } else if(part==1) {    
            uart_print(0,"\r\nSCC:  days  ydayl   Ah   SoCC ");
            uart_print(0,"\r\n----------+------+------+----+");    
            for(s=0;s<sysval.scc_nof_scc;s++) {
                i2c_sla=scc_i2c_addr[s];
                uart_print(0,"\r\n[%2x]:",i2c_sla);
                uart_print(0," %4d",sysval.scc_days[s]);
                uart_print(0,"  %4d",sysval.scc_ydayl[s]);
                uart_print(0,"  %5d",sysval.scc_Ah[s]);
                uart_print(0,"  %3d",sysval.scc_socc[s]);        
            }
            uart_print(0,"\r\n");
        }        
    } else {
        if(part==0) {
            len=sprintf(output_buf,       "\r\nSCC:    system battst solst SoC   Vpv  Icharge  Vbatt  Iload temp loadstat\r\n");
            for(s=0;s<sysval.scc_nof_scc;s++) {
                i2c_sla=scc_i2c_addr[s];
                len+=sprintf(&output_buf[len],"[%2x]:",i2c_sla);
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
            for(s=0;s<sysval.scc_nof_scc;s++) {
                i2c_sla=scc_i2c_addr[s];
                len+=sprintf(&output_buf[len],"[%2x]:",i2c_sla);
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

#endif // SYSTEM_CCNEO_SCC==1
