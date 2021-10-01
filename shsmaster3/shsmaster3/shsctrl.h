/*
 * SHS Control
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#ifndef _SHSCTRL_H
#define _SHSCTRL_H

#define VERSION     7
#define SUBVERSION  1
#define DATAPACKET_VERSION "3.0"

// SIM900 debug messages
#define SIM_DEBUG 1


/*
 * Behaviour of green charge LED. Choose:
 * CHARGE_LED_ON_VPV: when Vpv > Vbatt
 * else
 * Icharge > 0
 */
//#define CHARGE_LED_ON_VPV 1


#define SIM_MAX_RXBUFLEN 512
#define SIM_MAX_TXBUFLEN 512
#define SERIAL_MAX_RXBUFLEN 200
#define SERIAL_MAX_TXBUFLEN 200
#define MAX_COMMAND_LEN 40
#define SIM_MAX_SMSLENGTH 160


/*
 * Bits in errorstatus
 */
#define ERROR_SMS_PARSE_ERROR  3
#define ERROR_SMS_EMPTY_OR_SPY 4
#define ERROR_SMS_ERROR        6
#define ERROR_2G_ERROR         7

/*
 * Reply destinations when receiving SMS command
 */
#define REPLY_WITH_2G   0
#define REPLY_WITH_SMS  1

/*
 * Bits in solar_state
 */
#define BATT_STATE_ALMOST_FULL     1
#define BATT_STATE_ALMOST_EMPTY    2

#define SOLAR_STATE_CHARGING       4
#define BATT_STATE_FULL            5
#define HYST_STATE_OVERDISCHARGE   7

/*
 * Pay LED modi:
 */
#define PAYLED_MODE_OFF 0
#define PAYLED_MODE_1HZ 1
#define PAYLED_MODE_2HZ 2
#define PAYLED_MODE_ON  3


// 2G lengths:
#define HOSTNAME_LEN MAX_COMMAND_LEN
#define IMEI_LEN 17
#define LOGIN_LEN 16
#define APN_LEN   LOGIN_LEN
#define USER_LEN  LOGIN_LEN
#define PWD_LEN   LOGIN_LEN

// Phone Numbers:
#define SIM_MAX_PHONE_NR_LENGTH 20
#define SIM_MIN_PHONE_NR_LENGTH 10

// Quota constants
#define QUOTA_DEFAULT (1000UL * 3600UL)
#define QUOTA_USE_RELAY 1

// Task constants:
#define TASK_SECOND   0
#define TASK_2SECOND  1
#define TASK_3SECOND  2
#define TASK_5SECOND  3
#define TASK_MINUTE   4
#define TASK_HOUR     5
#define TASK_DAY      6
#define TASK_SENDSMS  7
#define TASK_2G       8

#define MINUTES_PER_DAY 1440

// Powerfault states:
#define POWERFAULT_OK            0
#define POWERFAULT_FUSEBLOWN     1
#define POWERFAULT_OVERCURRENT   2
#define POWERFAULT_OVERDISCHARGE 3


typedef struct sysTime_s {
        uint16_t ms; /**< Current MS timer value, resets to 0 every minute. */
        uint8_t sec; /**< Current second timer value, resets to 0 every minute. */
} sysTime_t;

/*** Storage for dynamic system status values. */
typedef struct sysVals_s {
    uint32_t seconds;   // copy to eeprom
    uint32_t seconds_old;
    uint16_t minutes;
    uint16_t ydayl; // yesterday's daylength in minutes
    uint16_t tasklist;
    uint8_t scc_tasklist;
    uint8_t scc_taskidx;
    uint8_t mod_tasklist;
    uint8_t mod_taskidx;
    uint8_t sim_tasklist;     
    uint8_t cmd_tasklist;     
    uint8_t watchdog;
    uint16_t interval2G;
    char sms_phonenr[SIM_MAX_PHONE_NR_LENGTH];
    
#if (SYSTEM_CCNEO_SCC==1)
    uint8_t scc_nof_scc;
    uint8_t scc_sysstate   [MAX_NOF_SCC];
    uint8_t scc_battstate  [MAX_NOF_SCC];
    uint8_t scc_solarstate [MAX_NOF_SCC];
    uint8_t scc_loadstate  [MAX_NOF_SCC];
    uint8_t scc_daystate   [MAX_NOF_SCC];
    uint8_t scc_soc        [MAX_NOF_SCC];
    uint8_t scc_socc       [MAX_NOF_SCC];
    uint8_t scc_temp       [MAX_NOF_SCC];
    int16_t scc_vbatt      [MAX_NOF_SCC];
    int16_t scc_vpv        [MAX_NOF_SCC];
    int16_t scc_icharge    [MAX_NOF_SCC];
    int16_t scc_iload      [MAX_NOF_SCC];
    int16_t scc_door       [MAX_NOF_SCC];
    int16_t scc_Ah         [MAX_NOF_SCC];
    uint16_t scc_ydayl     [MAX_NOF_SCC]; // yesterday's daylength in minutes
    uint16_t scc_days      [MAX_NOF_SCC];
#endif
#if (SYSTEM_MODULAR_DC==1 || SYSTEM_MODULAR_AC==1 || SYSTEM_MODULAR_PV==1)
    int16_t mod_vbatt      [SYSTEM_NOF_MOD_PV];
    int16_t mod_vpv        [SYSTEM_NOF_MOD_PV];
    int16_t mod_icharge    [SYSTEM_NOF_MOD_PV];
    int16_t mod_iload      [SYSTEM_NOF_MOD_DC];
    int16_t mod_iload_ac   [SYSTEM_NOF_MOD_AC];
    int16_t mod_vload_ac   [SYSTEM_NOF_MOD_AC];
    uint8_t mod_load_state_ac[SYSTEM_NOF_MOD_AC];
    uint8_t mod_load_state_dc[SYSTEM_NOF_MOD_AC];
#endif
	
    int16_t Vpv;
    int16_t Vbatt;
    int16_t I_charge;
    int16_t I_load;
    int16_t I_load_ac;  // NEW
    int16_t V_load_ac;  // NEW
    int16_t P_in;
    int16_t P_out;
    int16_t P_out_ac;   // NEW
    uint32_t Ws_in;     // copy to eeprom
    uint32_t Ws_out;    // copy to eeprom
    uint32_t Ws_out_ac; // copy to eeprom // NEW
    uint8_t SoC;
    uint8_t SoCC;
    int32_t Ah;
    uint8_t temp;
    uint8_t solar_state;
    uint8_t load_state;
    uint8_t batt_state;
    uint8_t sys_state;
    uint8_t day_state;
    uint8_t Door_open;
    uint8_t ringing;
    uint8_t errorstatus;
    uint8_t payled_mode;
    uint8_t retry2G;
    uint8_t sim900_errors;
    int8_t  signalstrength;
	
    uint8_t has_bluetooth;
    uint8_t bluetooth_connected;
    uint8_t bluetooth_status;
} sysVals_t;

#endif
