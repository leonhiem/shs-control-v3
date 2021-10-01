/*
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#if (SYSTEM_MODULAR_DC==1 || SYSTEM_MODULAR_AC==1 || SYSTEM_MODULAR_PV==1)

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "samd20.h"
#include "shsctrl.h"
#include "mod.h"
#include "i2c.h"
#include "utils.h"
#include "uart.h"

extern sysVals_t sysval;
static uint8_t mod_errors=0;

#define MOD_FILTER_SOC_SIZE 8
uint8_t mod_filter_soc[MOD_FILTER_SOC_SIZE];
uint8_t mod_filter_soc_idx;
uint8_t mod_filter_soc_init;
void mod_filter_soc_input(uint8_t val);
uint8_t mod_filter_soc_output(void);

#if (SYSTEM_MODULAR_DC==1)
#  warning "Using System MODULAR DC"
#endif

#if (SYSTEM_MODULAR_AC==1)
#  warning "Using System MODULAR AC"
#endif

#if (SYSTEM_MODULAR_PV==1)
#  warning "Using System MODULAR PV"
#endif

#if (SYSTEM_NOF_MOD_PV > 4)
#  error "Cannot support more than 4 PV modules"
#endif

#if (SYSTEM_NOF_MOD_AC > 4)
#  error "Cannot support more than 4 AC modules"
#endif

#if (SYSTEM_NOF_MOD_DC > 4)
#  error "Cannot support more than 4 DC modules"
#endif

#if (MOD_VBATT_FULL_cV <= MOD_VBATT_EMPTY_cV)
#  error "MOD_VBATT_FULL_cV must be greater than MOD_VBATT_EMPTY_cV"
#endif

void mod_init(int8_t onoff)
{
    mod_filter_soc_idx=0;
    mod_filter_soc_init=1;
}

void mod_tasks(void)
{
    uint8_t tasklist;
    uint8_t subtask;
    uint8_t s;
    __disable_irq();
    tasklist=sysval.mod_tasklist;
    subtask=sysval.mod_taskidx;
    sysval.mod_tasklist&=0x7f;    
    __enable_irq();   
       
    if(tasklist & 0x80) { // task allowed to run        
        sysval.mod_taskidx++;        
        switch(subtask) {
            case 0:
#if (SYSTEM_MODULAR_AC==1)
              for(s=0;s<SYSTEM_NOF_MOD_AC;s++) { mod_ac_monitor(s); }
#endif
              break;              
            case 1:
#if (SYSTEM_MODULAR_DC==1)
              for(s=0;s<SYSTEM_NOF_MOD_DC;s++) { mod_dc_monitor(s); }
#endif
              break;                       
            case 2:           
#if (SYSTEM_MODULAR_PV==1)
              for(s=0;s<SYSTEM_NOF_MOD_PV;s++) { mod_pv_monitor(s); }
#endif
              break;
            default:
              sysval.mod_taskidx=0;
              break;
        }        
        mod_update_sysval();              
    }
}

void mod_ac_send_cmd(uint8_t s, uint8_t cmd)
{
    uint8_t i2c_sla;
    uint8_t i2c_data;
    uint8_t retries=10;
    i2c_sla=MOD_AC_I2C_slaveAddress+s;
    
    while(retries--) {
        if(mod_errors>5) {
            mod_errors=0;
            i2c_reset();
            delay_ms(20);
        }
        if(i2c_transact(i2c_sla,&cmd,1,&i2c_data,0)  >= 0) {
            mod_errors=0;
            break;
        } else {
            mod_errors++;
        }        
    }   
}
void mod_dc_send_cmd(uint8_t s, uint8_t cmd)
{
    uint8_t i2c_sla;
    uint8_t i2c_data;
    uint8_t retries=10;
    i2c_sla=MOD_DC_I2C_slaveAddress+s;
    while(retries--) {
        if(mod_errors>5) {
            mod_errors=0;
            i2c_reset();
            delay_ms(20);
        }
        if(i2c_transact(i2c_sla,&cmd,1,&i2c_data,0) >= 0) {
            mod_errors=0;
            break;
        } else {
            mod_errors++;
        }        
    }    
}
void mod_pv_send_cmd(uint8_t s, uint8_t cmd)
{
    uint8_t i2c_sla;
    uint8_t i2c_data;
    uint8_t retries=10;
    i2c_sla=MOD_PV_I2C_slaveAddress+s;
    while(retries--) {
        if(mod_errors>5) {
            mod_errors=0;
            i2c_reset();
            delay_ms(20);
        }
        if(i2c_transact(i2c_sla,&cmd,1,&i2c_data,0) >= 0) {
            mod_errors=0;
            break;
        } else {
            mod_errors++;
        }        
    }    
}

void mod_ac_set_relay(uint8_t s, uint8_t onoff)
{
    uart_print(0,"mod_ac_set_relay[%d](%d)\n\r",s,onoff);
    if(onoff==0) {
    	mod_ac_send_cmd(s,MOD_I2C_CMD_SET_RELAY_OFF);
    } else {
    	mod_ac_send_cmd(s,MOD_I2C_CMD_SET_RELAY_ON);
    }
    sysval.mod_load_state_ac[s]=onoff;
}
void mod_ac_set_relay_all(uint8_t onoff)
{
    uint8_t s;
    for(s=0;s<SYSTEM_NOF_MOD_AC;s++) {        
        mod_ac_set_relay(s, onoff);
    }    
}

void mod_dc_set_relay(uint8_t s, uint8_t onoff)
{
    uart_print(0,"mod_dc_set_relay[%d](%d)\n\r",s,onoff);
    if(onoff==0) {
        mod_dc_send_cmd(s,MOD_I2C_CMD_SET_RELAY_OFF);
    } else {
        mod_dc_send_cmd(s,MOD_I2C_CMD_SET_RELAY_ON);
    }
    sysval.mod_load_state_dc[s]=onoff;    
}

void mod_dc_set_relay_all(uint8_t onoff)
{
    uint8_t s;
    for(s=0;s<SYSTEM_NOF_MOD_DC;s++) {        
        mod_dc_set_relay(s, onoff);
    }    
}


void mod_ac_monitor(uint8_t s)
{
    uint8_t i2c_sla;
    uint8_t dummy, i2c_data[4];
    uint16_t adcval_curr,adcval_volt;
    uint8_t retval=0;

    if(s>3) return;
    uart_print(0,"mod_ac_monitor[%d]\n\r",s);

    if(mod_errors>5) {
        mod_errors=0;
        i2c_reset();
        delay_ms(20);
    }    
    sysval.watchdog=0;
    i2c_sla=MOD_AC_I2C_slaveAddress+s;    

    mod_ac_send_cmd(s,MOD_I2C_CMD_FREEZE_VOLT_CURR);   
    delay_ms(4);
    mod_ac_send_cmd(s,MOD_I2C_CMD_READ_ADC_CURR_LSB);    
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[0],1);    
       
    mod_ac_send_cmd(s,MOD_I2C_CMD_READ_ADC_CURR_MSB);    
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[1],1);    
    
    mod_ac_send_cmd(s,MOD_I2C_CMD_READ_ADC_VOLT_LSB);    
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[2],1);    
    
    mod_ac_send_cmd(s,MOD_I2C_CMD_READ_ADC_VOLT_MSB);    
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[3],1);

    if(retval!=4) {
        uart_print(0,"mod_ac_monitor[%d]: ERROR\n\r",s);
        mod_errors++;
        return;
    }
    mod_errors=0;
    adcval_curr=((uint16_t)i2c_data[1])<<8;
    adcval_curr|=((uint16_t)i2c_data[0]);
    adcval_volt=((uint16_t)i2c_data[3])<<8;
    adcval_volt|=((uint16_t)i2c_data[2]);
    
    sysval.mod_iload_ac[s] = adcval_curr/10; // mA to cA
    sysval.mod_vload_ac[s] = adcval_volt;
}

void mod_dc_monitor(uint8_t s)
{
    uint8_t i2c_sla;
    uint8_t dummy, i2c_data[4];
    uint16_t adcval_curr;//,adcval_volt;
    uint8_t retval=0;

    if(s>3) return;
    uart_print(0,"mod_dc_monitor[%d]\n\r",s);

    if(mod_errors>5) {
        mod_errors=0;
        i2c_reset();
        delay_ms(20);
    }    
    sysval.watchdog=0;
    i2c_sla=MOD_DC_I2C_slaveAddress+s;

    mod_dc_send_cmd(s,MOD_I2C_CMD_FREEZE_VOLT_CURR);
    delay_ms(4);
    mod_dc_send_cmd(s,MOD_I2C_CMD_READ_ADC_CURR_LSB);
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[0],1);
    
    mod_dc_send_cmd(s,MOD_I2C_CMD_READ_ADC_CURR_MSB);
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[1],1);
    
    if(retval!=2) {
        uart_print(0,"mod_dc_monitor[%d]: ERROR\n\r",s);
        mod_errors++;
        return;
    }
    mod_errors=0;
    adcval_curr=((uint16_t)i2c_data[1])<<8;
    adcval_curr|=((uint16_t)i2c_data[0]);
        
    sysval.mod_iload[s] = adcval_curr/10; // mA to cA
}

void mod_pv_monitor(uint8_t s)
{
    uint8_t i2c_sla;
    uint8_t dummy, i2c_data[6];
    uint16_t adcval_curr,adcval_volt,adcval_voltB;
    uint8_t retval=0;

    if(s>3) return;
    uart_print(0,"mod_pv_monitor[%d]\n\r",s);

    if(mod_errors>5) {
        mod_errors=0;
        i2c_reset();
        delay_ms(20);
    }
    sysval.watchdog=0;
    i2c_sla=MOD_PV_I2C_slaveAddress+s;

    mod_pv_send_cmd(s,MOD_I2C_CMD_FREEZE_VOLT_CURR);
    delay_ms(4);
    mod_pv_send_cmd(s,MOD_I2C_CMD_READ_ADC_CURR_LSB);
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[0],1);
    
    mod_pv_send_cmd(s,MOD_I2C_CMD_READ_ADC_CURR_MSB);
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[1],1);
    
    mod_pv_send_cmd(s,MOD_I2C_CMD_READ_ADC_VOLT_LSB);
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[2],1);
    
    mod_pv_send_cmd(s,MOD_I2C_CMD_READ_ADC_VOLT_MSB);
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[3],1);
    
    mod_pv_send_cmd(s,MOD_I2C_CMD_READ_ADC_VOLTB_LSB);
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[4],1);
    
    mod_pv_send_cmd(s,MOD_I2C_CMD_READ_ADC_VOLTB_MSB);
    delay_ms(1);
    retval+=i2c_transact(i2c_sla,&dummy,0,&i2c_data[5],1);
    
    if(retval!=6) {
        uart_print(0,"mod_pv_monitor[%d]: ERROR\n\r",s);
        mod_errors++;
        return;
    }
    mod_errors=0;
    adcval_curr=((uint16_t)i2c_data[1])<<8;
    adcval_curr|=((uint16_t)i2c_data[0]);
    adcval_volt=((uint16_t)i2c_data[3])<<8;
    adcval_volt|=((uint16_t)i2c_data[2]);
    adcval_voltB=((uint16_t)i2c_data[5])<<8;
    adcval_voltB|=((uint16_t)i2c_data[4]);
    
    sysval.mod_icharge[s] = adcval_curr/10; // mA to cA
    sysval.mod_vpv[s]     = adcval_volt;
    sysval.mod_vbatt[s]   = adcval_voltB;
}

void mod_update_sysval(void)
{
    int s,nof_mod_pv=0,nof_mod_ac=0,nof_mod_dc=0;
    int16_t vpv=0, icharge=0, vbatt=0, iload=0, iload_ac=0, vload_ac=0;
    int32_t pin=0,pout=0,pout_ac=0;
    uint16_t dt;
    int32_t tmp_SoC;

    sysval.sys_state=0;
    sysval.load_state=0;    
    for(s=0;s<SYSTEM_NOF_MOD_PV;s++) {
        nof_mod_pv++;
        icharge  += sysval.mod_icharge[s];
        vpv      += sysval.mod_vpv[s];
        if(sysval.mod_vbatt[s] > vbatt) {
            vbatt = sysval.mod_vbatt[s];
        }

        pin += ((int32_t)sysval.mod_vbatt[s] * (int32_t)sysval.mod_icharge[s]);
        sysval.sys_state |= ((2&0x3)<<(s*2)); // fill in bit pair. Just put '2' "charging" for now (FIXME)
    }
    if(nof_mod_pv>0) {
        sysval.Vpv       = vpv / nof_mod_pv; // average
        sysval.Vbatt     = vbatt;   // max
        sysval.I_charge  = icharge; // sum
    }
    for(s=0;s<SYSTEM_NOF_MOD_AC;s++) {
        nof_mod_ac++;
        iload_ac += sysval.mod_iload_ac[s];
        vload_ac += sysval.mod_vload_ac[s];

        pout_ac  += ((int32_t)sysval.mod_vload_ac[s] * (int32_t)sysval.mod_iload_ac[s]);
        
        sysval.sys_state  |= ((2&0x3)<<(s*2)); // fill in bit pair. Just put '2' "charging" for now (FIXME)
        sysval.load_state |= ((sysval.mod_load_state_ac[s]&0x3)<<(s*2)); 
    }
    for(s=0;s<SYSTEM_NOF_MOD_DC;s++) {
        nof_mod_dc++;
        iload    += sysval.mod_iload[s];
        pout     += ((int32_t)sysval.Vbatt * (int32_t)sysval.mod_iload[s]);
        
        sysval.sys_state  |= ((2&0x3)<<(s*2)); // fill in bit pair. Just put '2' "charging" for now (FIXME)
        sysval.load_state |= ((sysval.mod_load_state_dc[s]&0x3)<<(s*2));
    }

    sysval.SoCC      = 0; // TODO
    sysval.temp      = 0;
    sysval.ydayl     = 0;
    sysval.Ah        = 0; // TODO

    if(sysval.seconds_old==0) {
        dt=0;
    } else {
        dt=(uint16_t)(sysval.seconds - sysval.seconds_old);
    }

    if(nof_mod_ac>0) {
        sysval.I_load_ac = iload_ac;   // sum
        sysval.V_load_ac = vload_ac / nof_mod_ac; // average
        pout_ac += (int32_t)5000;     // integer roundoff +0.5
        sysval.P_out_ac = pout_ac/(int32_t)10000; // to Watt   
        sysval.Ws_out_ac+=(uint32_t)(sysval.P_out_ac * dt);
    }
    if(nof_mod_dc>0) {
        sysval.I_load    = iload;      // sum
        pout    += (int32_t)5000;     // integer roundoff +0.5
        sysval.P_out    = pout/(int32_t)10000;    // to Watt   
        sysval.Ws_out+=(uint32_t)(sysval.P_out * dt);
    }
    if(nof_mod_pv>0) {
        tmp_SoC = (int32_t)sysval.Vbatt;
        if(tmp_SoC <= MOD_VBATT_EMPTY_cV) {
            mod_filter_soc_input(0);
        } else if(tmp_SoC >= MOD_VBATT_FULL_cV) {
            mod_filter_soc_input(100);
        } else {
            tmp_SoC = tmp_SoC - MOD_VBATT_EMPTY_cV;
            tmp_SoC = tmp_SoC * 100L;
            tmp_SoC = tmp_SoC / (MOD_VBATT_FULL_cV-MOD_VBATT_EMPTY_cV);
            mod_filter_soc_input((uint8_t)tmp_SoC);
        }
        sysval.SoC = mod_filter_soc_output();

        pin          += (int32_t)5000;     // integer roundoff +0.5
        sysval.P_in   = pin/(int32_t)10000;     // to Watt
        sysval.Ws_in += (uint32_t)(sysval.P_in * dt);
    }    
    sysval.seconds_old=sysval.seconds;
    mod_monitor_dump(NULL,0);
}

uint16_t mod_monitor_dump(char *output_buf, int part)
{
    uint16_t len=0;
    uint8_t i2c_sla,s;
    
#if (SYSTEM_MODULAR_DC==1)
    for(s=0;s<SYSTEM_NOF_MOD_DC;s++) {
        i2c_sla=MOD_DC_I2C_slaveAddress+s;
        uart_print(0,"[%2x]:MOD DC curr=%d\n\r",i2c_sla,sysval.mod_iload[s]);
    }
#endif
#if (SYSTEM_MODULAR_AC==1)
    for(s=0;s<SYSTEM_NOF_MOD_AC;s++) {
        i2c_sla=MOD_AC_I2C_slaveAddress+s;    
        uart_print(0,"[%2x]:MOD AC curr=%d volt=%d\n\r",
                   i2c_sla,sysval.mod_iload_ac[s],sysval.mod_vload_ac[s]); 
    }
#endif
#if (SYSTEM_MODULAR_PV==1)
    for(s=0;s<SYSTEM_NOF_MOD_PV;s++) {
        i2c_sla=MOD_PV_I2C_slaveAddress+s;
        uart_print(0,"[%2x]:MOD PV curr=%d volt=%d vbatt=%d\n\r",
                   i2c_sla,sysval.mod_icharge[s],sysval.mod_vpv[s],sysval.mod_vbatt[s]);
    }
#endif
    return len;
}

/*
 * Filter to smoothen the SoC value:
 */
void mod_filter_soc_input(uint8_t val)
{
    if(mod_filter_soc_init) { // initialize
        memset(mod_filter_soc,val,MOD_FILTER_SOC_SIZE);
        mod_filter_soc_init=0;
    }
    // write into ringbuffer indexed by filter_soc_idx:
    if(val > 100) val=100;
    mod_filter_soc[mod_filter_soc_idx] = val;
    mod_filter_soc_idx++;
    mod_filter_soc_idx &= (MOD_FILTER_SOC_SIZE-1); // wrap around
}

uint8_t mod_filter_soc_output(void)
{
    uint16_t val=0;
    uint8_t i;
    for(i=0;i<MOD_FILTER_SOC_SIZE;i++) {
        val+=(uint16_t)mod_filter_soc[i];
    }
    val = val/MOD_FILTER_SOC_SIZE;
    return (uint8_t)val;
}

#endif // (SYSTEM_MODULAR_DC==1 || SYSTEM_MODULAR_AC==1 || SYSTEM_MODULAR_PV==1)
