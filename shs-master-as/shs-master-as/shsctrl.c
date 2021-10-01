/*
 * SHS Control
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 *
 * Processor: ATmega1284p or ATmega1284
 * Compiler used: Atmel Studio 7.0.1417
 * 
 * XTAL: 7.372800 MHz
 * Fuse settings: lfuse=0xEC
 *                hfuse=0xD9
 *                efuse=0xFE
 * 
 */
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
#include "quota.h"
#include "twi.h"
#include "tasks.h"
#include "scc.h"

// Set fuses: 
FUSES = 
{ 
   .high = 0xD9, 
   .low  = 0xEC, 
   .extended = 0xFE  
}; 

#define FIRMWARE_VERSION   "6.4"
#define DATAPACKET_VERSION "3.1"

#ifdef LEONS_SHS
#warning "shsctrl.c: Leon's system in Holland"
#else
#warning "shsctrl.c: SHS in Cambodia"
#endif

uint8_t  EEMEM EEPROM_ADDR_Ws_quota_enabled = 0;
uint32_t EEMEM EEPROM_ADDR_Ws_quota   = QUOTA_DEFAULT;
uint8_t  EEMEM EEPROM_ADDR_SIM900_enabled = 1;
uint8_t  EEMEM EEPROM_ADDR_LED_mode   = PAYLED_MODE_OFF;
uint8_t  EEMEM EEPROM_ADDR_onoff        = 0xa5; // default on
#ifdef LEONS_SHS
uint16_t EEMEM EEPROM_ADDR_intervalSMS  = 0;
uint16_t EEMEM EEPROM_ADDR_interval2G   = 15;
#else
uint16_t EEMEM EEPROM_ADDR_intervalSMS  = 1440;
uint16_t EEMEM EEPROM_ADDR_interval2G   = 20;
#endif
uint32_t EEMEM EEPROM_ADDR_secondsMeter = 0UL;
uint32_t EEMEM EEPROM_ADDR_Ws_in        = 0UL;
uint32_t EEMEM EEPROM_ADDR_Ws_out       = 0UL;
uint8_t  EEMEM EEPROM_ADDR_OperatorNr[MAX_NR_OPERATORS][MAX_PHONE_NR_LENGTH] = {
#ifdef LEONS_SHS
                                      "+31612622133",  // [0] Leon Holland: Hi
#endif
                                      "+85589980773",  // [0] or [1] Cambodia operator's nr
#ifndef LEONS_SHS
                                      "+14804180480",  // [1] Twilio
#endif
                                      "+85599935667",  // [2] Arjen's nr
                                      "+85511637842",  // [3] Benoit's nr
                                      "+85512426013",  // [4] Nuntium SMS gateway
                                      "+8613714323770",// [5] China operator
                                      "+85592962162"   // [6] Kamworks headphone: PROTECTED (readonly)
                                     };
#ifdef LEONS_SHS
uint8_t  EEMEM EEPROM_ADDR_hostname[HOSTNAME_LEN] = { "hanuman-stag.kamworks.com" };
#else
uint8_t  EEMEM EEPROM_ADDR_hostname[HOSTNAME_LEN] = { "hanuman.kamworks.com" };
#endif

#ifndef LEONS_SHS
/*
// default for cellcard in Cambodia
uint8_t  EEMEM EEPROM_ADDR_apn[APN_LEN]   = { "cellcard" };
uint8_t  EEMEM EEPROM_ADDR_user[USER_LEN] = { "mobitel" };
uint8_t  EEMEM EEPROM_ADDR_pwd[PWD_LEN]   = { "mobitel" };
*/

// default for China Mobile
uint8_t  EEMEM EEPROM_ADDR_apn[APN_LEN]   = { "CMNET" };
uint8_t  EEMEM EEPROM_ADDR_user[USER_LEN] = { "" };
uint8_t  EEMEM EEPROM_ADDR_pwd[PWD_LEN]   = { "" };

/*
// default for vinaphone in Vietnam 
uint8_t  EEMEM EEPROM_ADDR_apn[APN_LEN]   = { "m3-world" };
uint8_t  EEMEM EEPROM_ADDR_user[USER_LEN] = { "mms" };
uint8_t  EEMEM EEPROM_ADDR_pwd[PWD_LEN]   = { "mms" };
*/
#else
// default for KPN in Nederland 
//uint8_t  EEMEM EEPROM_ADDR_apn[APN_LEN]   = { "portalmmm.nl" };
//uint8_t  EEMEM EEPROM_ADDR_user[USER_LEN] = { "" };
//uint8_t  EEMEM EEPROM_ADDR_pwd[PWD_LEN]   = { "" };

// default for AH in Nederland 
uint8_t  EEMEM EEPROM_ADDR_apn[APN_LEN]   = { "internet" };
uint8_t  EEMEM EEPROM_ADDR_user[USER_LEN] = { "" };
uint8_t  EEMEM EEPROM_ADDR_pwd[PWD_LEN]   = { "" };
#endif

uint8_t  EEMEM EEPROM_ADDR_dummy=0xdb; // this is address 0: keep unused

volatile sysVals_t sysval;
volatile sysTime_t sysTime = {0,0}; /**< System timer (sec,ms). */
	
// This function is called upon a HARDWARE RESET:
void wdt_first(void) __attribute__((naked)) __attribute__((section(".init3")));

// Clear SREG_I on hardware reset.
void wdt_first(void)
{
	// Note that for newer devices (any AVR that has the option to also
	// generate WDT interrupts), the watchdog timer remains active even
	// after a system reset (except a power-on condition), using the fastest
	// prescaler value (approximately 15 ms). It is therefore required
	// to turn off the watchdog early during program startup.
	MCUSR = 0; // clear reset flags
	wdt_disable();
	// http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
}



/* UART buffer defines */
#define UART_SIM_RX_BUFFER_SIZE 256 /* power of 2 */
#define UART_SIM_TX_BUFFER_SIZE 512
#define UART_SIM_RX_BUFFER_MASK (UART_SIM_RX_BUFFER_SIZE - 1)

#define UART_SIM_TX_BUFFER_MASK (UART_SIM_TX_BUFFER_SIZE - 1)

#define UART_TERM_RX_BUFFER_SIZE 256 /* power of 2 */
#define UART_TERM_RX_BUFFER_MASK (UART_TERM_RX_BUFFER_SIZE - 1)

/* Static Variables */
static char UART_SIM_RxBuf[UART_SIM_RX_BUFFER_SIZE];
static volatile char UART_SIM_RxHead;
static volatile char UART_SIM_RxTail;
static char UART_SIM_TxBuf[UART_SIM_TX_BUFFER_SIZE];
static volatile char UART_SIM_TxHead;
static volatile char UART_SIM_TxTail;

static unsigned char UART_TERM_RxBuf[UART_TERM_RX_BUFFER_SIZE];
static volatile unsigned char UART_TERM_RxHead;
static volatile unsigned char UART_TERM_RxTail;

FILE term_uart_str = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

unsigned char Receive_SIM_Byte(void);
void Transmit_SIM_Byte(unsigned char data);
unsigned char Receive_TERM_Byte(void);
void Transmit_TERM_Byte(unsigned char data);

unsigned char Receive_SIM_Byte(void)
{
        unsigned char tmptail; 
        /* Wait for incoming data */
        while (UART_SIM_RxHead == UART_SIM_RxTail);
        /* Calculate buffer index */
        tmptail = (UART_SIM_RxTail + 1) & UART_SIM_RX_BUFFER_MASK;
        /* Store new index */
        UART_SIM_RxTail = tmptail; 
        /* Return data */
        return UART_SIM_RxBuf[tmptail];
}

unsigned char Receive_TERM_Byte(void)
{
        unsigned char tmptail; 
        /* Wait for incoming data */
        while (UART_TERM_RxHead == UART_TERM_RxTail);
        /* Calculate buffer index */
        tmptail = (UART_TERM_RxTail + 1) & UART_TERM_RX_BUFFER_MASK;
        /* Store new index */
        UART_TERM_RxTail = tmptail; 
        /* Return data */
        return UART_TERM_RxBuf[tmptail];
}

unsigned char Poll_TERM(void)
{
        if(UART_TERM_RxHead == UART_TERM_RxTail) return 0; else return 1;
}
unsigned char Poll_SIM(void)
{
        if(UART_SIM_RxHead == UART_SIM_RxTail) return 0; else return 1;
}


uint8_t prepare_data_message(const char *userstr, uint8_t *sms_start_offset);
int8_t send_2g_message(const char *msg, const uint8_t size);
void startup_sim900(int8_t do_delete);

volatile uint8_t twi_timeout;
#define RESTART_MICROCONTROLLER() { while(1); } // handled by watchdog

static void initIO(void) {
    PORTA = (1<<POWER_FAULT); // pull up
    DDRA  = (1<<IO_0); // output enable for external powersupply

    PORTB = (1<<IO_1) | (1<<SPI_MOSI_IO_2) | (1<<SPI_MISO) | (1<<SPI_SCK); // pull up here
    DDRB  = (1<<LED_CHARGING) | (1<<LED_100PERCENT) | (1<<LED_60PERCENT) | (1<<LED_30PERCENT);

    PORTC = (1<<I2C_SCL) | (1<<I2C_SDA); // pull up here
    DDRC  = (1<<POWERKEY) | (1<<RELAY_CLOSE) | (1<<RELAY_OPEN) | 
            (1<<LED_NOTPAYED) | (1<<LED_SHORTFAULT);

    PORTD = (1<<SERIAL_RXD) | (1<<SIM900_RXD) | (1<<SIM900_RI) | (1<<SIM900_CTS); // pull up here
    DDRD  = (1<<SERIAL_TXD) | (1<<SIM900_TXD) | (1<<SIM900_RTS);
}

/**
 * @brief Initialise hardware timers.
 */
static void initTimers(void) {
    // f_ocr0=7372800/(2*Prescaler*(1+OCR0A))  (see page 99 section 15.7.2 CTC mode)
    #define MS_TICK_OCR_VAL       99 // 7372800 Hz / ( 64 * (1+99)) = 1152 Hz
    #define MS_TICK_PS_VAL (1<<CS00)|(1<<CS01) //clock/64 prescaler
    #define MSTICKS_PER_SEC      1152UL 
    #define MSTICKS_PER_HALFSEC   576UL 
    #define MSTICKS_PER_TENTHSEC  115UL 

    #define PWM_PS_VAL (1<<CS10) //clock/1 prescaler

    //Timer 0 - ~MS Tick, ~1kHz
    TCCR0A = (1<<WGM01); //Clear on output compare match.
    OCR0A = MS_TICK_OCR_VAL;
    TIMSK0 |= (1<<OCIE0A); //Enable output compare interrupt.
    TCCR0B |= MS_TICK_PS_VAL;
}

void run_tasks(void)
{
    uint8_t sreg;
    uint16_t tasklist;
    uint8_t dosms,sms_start_offset;
    sreg=SREG; cli(); // atomic read
    tasklist=sysval.tasklist;
    sysval.tasklist=0;
    SREG=sreg; // sei()

    /* Do all pending tasks now: */

    if(tasklist & (1<<TASK_SECOND)) {
		//printf("---TASK_SECOND---\n");
        task_every_second();
		if(sysval.has_bluetooth) {
			if(sysval.bluetooth_connected) {
				char btbuf[32];
				char *btbuf_ptr=sim_bluetooth_read(btbuf, sizeof(btbuf));
				if(btbuf_ptr != NULL) {
					uint16_t bt_len=0;
					printf("Bluetooth message received:%s\n",btbuf_ptr);
					if(strncasecmp(btbuf_ptr,"SCC?",4)==0) {
						bt_len=scc_monitor_dump(UART_SIM_TxBuf,0);
					} else if(strncasecmp(btbuf_ptr,"SCCD?",5)==0) {
					    bt_len=scc_monitor_dump(UART_SIM_TxBuf,1);
					} else if(strncasecmp(btbuf_ptr,"SOC?",4)==0) {
					    bt_len=task_soc_dump(UART_SIM_TxBuf);
					} else if(strncasecmp(btbuf_ptr,"LOAD=",5)==0) { // expect: 0 or 1
					    char *ptr=&btbuf_ptr[5];
					    btbuf_ptr[6]=0;
					    task_set_load(atoi(ptr));
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"setting load\r\n");
					} else if(strncasecmp(btbuf_ptr,"TEMP=",5)==0) { // expect: number
					    char *ptr=&btbuf_ptr[5];
					    btbuf_ptr[7]=0;
					    task_cal_temp(atoi(ptr));
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"calibrate temp\r\n");
					} else if(strncasecmp(btbuf_ptr,"BATO=",5)==0) { // expect: number
					    char *ptr=&btbuf_ptr[5];
					    btbuf_ptr[7]=0;
					    task_cal_batt(atoi(ptr));
					    bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"calibrate batt Ah offset\r\n");
					} else if(strncasecmp(btbuf_ptr,"AH=",3)==0) { // expect: number
					    char *ptr=&btbuf_ptr[3];
					    btbuf_ptr[6]=0;
					    task_set_ah_batt(atoi(ptr));
					    bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"set batt Ah\r\n");
					} else if(strncasecmp(btbuf_ptr,"RUNEQ",5)==0) {
					    task_start_equalize();
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"Start equalizing\r\n");				
					} else if(strncasecmp(btbuf_ptr,"RESET",5)==0) {
					    RESTART_MICROCONTROLLER();
				    } else {
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"------ Help: -----------------\r\n");
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"SCC?   read SCCs sensors\r\n");
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"SCCD?  read SCCs other\r\n");
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"SOC?   read SoC of full system\r\n");
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"LOAD=x set load to 0 or 1\r\n");
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"TEMP=x calibrate tempsensors\r\n");
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"BATO=x set selfcurrent offset\r\n");
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"AH=x   set batteries to xAh\r\n");
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"RUNEQ  force equalization\r\n");
						bt_len+=sprintf(&UART_SIM_TxBuf[bt_len],"RESET  reset master\r\n");
					}
					sim_bluetooth_write(UART_SIM_TxBuf,bt_len);
				}
			}
		}
    }

    if(tasklist & (1<<TASK_2SECOND)) {
	}
	if(tasklist & (1<<TASK_3SECOND)) {
		//printf("---TASK_3SEC---\n");
		if(sysval.has_bluetooth) {
			if(sysval.bluetooth_connected==0) {
				sim_start_bluetooth();
				if(sysval.bluetooth_status == 25) {
					sim_bluetooth_connect();
					sysval.bluetooth_connected = 1;
					printf("connect!\n");
				}
			}
		}
	}
    if(tasklist & (1<<TASK_5SECOND)) {
		//printf("---TASK_5SEC---\n");
        task_every_5second();		
    }

    if(tasklist & (1<<TASK_MINUTE)) {
		//printf("---TASK_MIN---\n");
        task_every_minute();

        /*
         * In case of too many SIM900 errors:
         */
        // if for some reason the SIM900 module turned off then turn on again
        if(sysval.sim900_errors > 10) {
            sim_onoff_pulse(); // turn it off (next minute back on)
            sysval.sim900_errors=0;
        } else {
            // if the SIM900 module turned off before then turn on now
            if(!sim_is_on()) {
                sim_onoff_pulse();
                tasklist &= ~(1<<TASK_CREG);
            }
        }

        /*
         * periodic SMS sending:
         */
        dosms=0;
        sreg=SREG; cli(); // atomic read
        if(sysval.intervalSMS == 0) {
            sysval.intervalSMS = eeprom_read_word(&EEPROM_ADDR_intervalSMS);
            if(sysval.intervalSMS > 0) {
                dosms=1;
            }
        }
        SREG=sreg; // sei()
        if(dosms) {
			if(sysval.has_bluetooth) { sim_stop_bluetooth(); }
            // to the 1st: the operator
            uint8_t message_len = prepare_data_message("ASOC?",&sms_start_offset);
            message_len-=sms_start_offset;

            if(sim_send_sms(&UART_SIM_TxBuf[sms_start_offset],message_len,0) < 0) {
                sysval.errorstatus |= (1<<ERROR_SMS_ERROR);
            }
        }
    }


    /*
     * periodic 2G data sending:
     */
    if( ((tasklist & (1<<TASK_2G)) && sysval.interval2G != 0) ||
        ((tasklist & (1<<TASK_MINUTE)) && sysval.retry2G)) {

        uint8_t message_len;
		//printf("---TASK_2G---\n");
		if(sysval.has_bluetooth) { sim_stop_bluetooth(); }
        sysval.errorstatus &= ~(1<<ERROR_2G_ERROR);
        message_len = prepare_data_message("ASOC?",&sms_start_offset);
        if(send_2g_message(UART_SIM_TxBuf,message_len) != 0) {
            sysval.errorstatus |= (1<<ERROR_2G_ERROR);
            sysval.retry2G++;

            if(sysval.retry2G>3) {
                sysval.retry2G=0;
                sysval.sim900_errors++;
            }
        } else {
            sysval.retry2G=0;
            sysval.sim900_errors=0;
        }
    }


    /*
     * Check how good the SIM900 network connection is:
     */
    if(tasklist & (1<<TASK_CREG)) {
		//printf("---TASK_CREG---\n");
		if(sysval.has_bluetooth) { sim_stop_bluetooth(); }
        int creg = sim_read_CREG();
        if(creg!=1 && creg!=5) { // 1 or 5 is registered
                                 // Otherwise... (including in case of error or 
                                 // !sim_is_on() :
            sim_onoff_pulse();   // turn it off; next minute turn back on
        }
    }

    if(tasklist & (1<<TASK_HOUR)) {
		//printf("---TASK_HOUR---\n");
        task_hourly();
    }

    if(tasklist & (1<<TASK_DAY)) {
		//printf("---TASK_DAY---\n");
        task_daily();
		if(sysval.has_bluetooth) { sim_stop_bluetooth(); }
        startup_sim900(1); // restart the SIM900 and delete all SMSs
    }

    /*
     * Send ASOC data string to debug port:
     */
	/*
    if(tasklist & (1<<TASK_SECOND)) {
        prepare_data_message("ASOC?",&sms_start_offset);
    }
	*/
	/*
	if(tasklist & (1<<TASK_SECOND)) {
		printf("WATCHDOG=%d\n",sysval.watchdog);
	}
	*/
	
}

const char PM_URLfilepath_data[] PROGMEM="/analytics/datapoint_backend/";
const char PM_GETcmdDATA[]       PROGMEM="?data=";
const char PM_GETimei[]          PROGMEM="?imei=";
const char PM_SOCstr1[] PROGMEM=";" DATAPACKET_VERSION ";" FIRMWARE_VERSION ";%c%c%c%c;%c%c%c%c;%lu;%c%c%c%c;";
//const char PM_SOCstr1[] PROGMEM=";" DATAPACKET_VERSION ";" FIRMWARE_VERSION ";%.20s;%c%c%c%c;%lu;%c%c%c%c;";
const char PM_SOCstr2[] PROGMEM="%d;%d;%d;%d;%d;%d;%d;";

uint8_t prepare_data_message(const char *userstr, uint8_t *sms_start_offset)
{
    char sim_rxbuf[64];
    uint8_t sreg;
    uint8_t len=0;
    uint8_t tmp_ledstate;

    sreg=SREG; cli();
    eeprom_read_block(&UART_SIM_TxBuf[len],&EEPROM_ADDR_hostname,HOSTNAME_LEN);
    SREG=sreg;

    len=strlen(UART_SIM_TxBuf);

    len+=sprintf_P(&UART_SIM_TxBuf[len],PM_URLfilepath_data);
    len+=sprintf_P(&UART_SIM_TxBuf[len],PM_GETcmdDATA);

    *sms_start_offset=len;
    sprintf(&UART_SIM_TxBuf[len],"%s",sim_read_IMEI(sim_rxbuf,sizeof(sim_rxbuf)));
    len=strlen(UART_SIM_TxBuf);

    len+=sprintf_P(&UART_SIM_TxBuf[len],PM_SOCstr1,
                  daystate2str(3),daystate2str(2),daystate2str(1),daystate2str(0),
				  sysstate2str(3),sysstate2str(2),sysstate2str(1),sysstate2str(0), sysval.seconds, 
				  loadstate2str(3),loadstate2str(2),loadstate2str(1),loadstate2str(0));
    // userstr len is maximal 20 chars (truncated by sprintf_P(".... %.20s ....")

    len+=sprintf(&UART_SIM_TxBuf[len],"%d;%d;%d;%lu;%lu;%lu;%c%c%c%c;",
                  sysval.Door_open,sysval.SoC,sysval.SoCC,
                  sysval.Ah, sysval.Ws_in/3600, sysval.Ws_out/3600,
				  solarstate2str(3),solarstate2str(2),solarstate2str(1),solarstate2str(0));

    sreg=SREG; cli(); // atomic read to capture led state without race condition
    tmp_ledstate=0;
	if(bit_is_set(PORTC,LED_SHORTFAULT) == 0) tmp_ledstate|=0x08;
    if(bit_is_set(PORTB,LED_CHARGING)   == 0) tmp_ledstate|=0x10;
    if(bit_is_set(PORTB,LED_100PERCENT) == 0) tmp_ledstate|=0x20;
    if(bit_is_set(PORTB,LED_60PERCENT)  == 0) tmp_ledstate|=0x40;
    if(bit_is_set(PORTB,LED_30PERCENT)  == 0) tmp_ledstate|=0x80;	
    SREG=sreg;

    len+=sprintf(&UART_SIM_TxBuf[len],"%d;",sysval.Vbatt);
    len+=sprintf(&UART_SIM_TxBuf[len],"%d;",sysval.Vpv);
    len+=sprintf(&UART_SIM_TxBuf[len],"%d;",sysval.I_load);
    len+=sprintf(&UART_SIM_TxBuf[len],"%d;",sysval.I_charge);

    len+=sprintf(&UART_SIM_TxBuf[len],"%d;%d;%d;", sysval.P_out,sysval.P_in, sysval.temp);
    len+=sprintf_P(&UART_SIM_TxBuf[len],PM_SOCstr2,sysval.ydayl,
                  ((tmp_ledstate&0x10)==0x10), // LED_CHARGING
                  ((tmp_ledstate&0x20)==0x20), // LED_100PERCENT
                  ((tmp_ledstate&0x40)==0x40), // LED_60PERCENT
                  ((tmp_ledstate&0x80)==0x80), // LED_30PERCENT
                  ((tmp_ledstate&0x08)==0x08), // LED_SHORTFAULT (FIXME: what if it is blinking?)
                  sysval.payled_mode);
    len+=sprintf(&UART_SIM_TxBuf[len],"%d;", sim_read_signalstrength());

    // I_load channels from each SCC:
    len+=sprintf(&UART_SIM_TxBuf[len],"%d;",sysval.scc_icharge[0]);//sysval.scc_iload[0]);
    len+=sprintf(&UART_SIM_TxBuf[len],"%d;",sysval.scc_icharge[1]);//sysval.scc_iload[1]);
    len+=sprintf(&UART_SIM_TxBuf[len],"%d;",sysval.scc_icharge[2]);//sysval.scc_iload[2]);

    // Quota:
    if(sysval.Ws_quota_enabled) {
        uint32_t tmp32 = eeprom_read_dword(&EEPROM_ADDR_Ws_quota);
        len+=sprintf(&UART_SIM_TxBuf[len],"%lu;",tmp32/3600);
        len+=sprintf(&UART_SIM_TxBuf[len],"%lu;",sysval.Ws_quota/3600);
    } else {
        len+=sprintf(&UART_SIM_TxBuf[len],"0;0;");
    }
	
//#ifdef LEONS_SHS
//    printf("%s\n",&UART_SIM_TxBuf[*sms_start_offset]); // strip off: hostname    
//#else
    printf("%s\n\r",UART_SIM_TxBuf);	
//#endif 
    return len;
}

int8_t send_2g_message(const char *msg, const uint8_t size)
{
    int8_t state;
    uint8_t retries;
    uint8_t errorstatus;

    retries=0;
    state=0;
    errorstatus=0;
    while(state != 4 && retries<4) {
        state=sim_start_2g();
        if(state<0) retries++;
    }
    if(retries>3) {
        errorstatus=1;
    } else {
        // now we have a 2G connection!
        retries=0;
        while(retries<4) {
            if(sim_send_2g_msg(msg,size) != 0)
                retries++;
            else
                break;
        }
        if(retries>3) errorstatus=1;
    }
    sim_stop_2g();
    return errorstatus;
}

void write_operator_phonenr_to_eeprom(const int idx, const char *phonenr_ptr)
{
    uint8_t sreg;
    int len = strlen(phonenr_ptr);
    if(idx < MAX_NR_OPERATORS-1) { // MAX_NR_OPERATORS-1 is reserved: read only
        if(len<MAX_PHONE_NR_LENGTH) {
            sreg=SREG; cli();
            eeprom_update_block(phonenr_ptr,&EEPROM_ADDR_OperatorNr[idx],len+1);
            SREG=sreg;
        }
    }
}

void startup_sim900(int8_t do_delete)
{
    uint8_t tmp8;
    char sim_rxbuf[64];
    char *ptr=NULL;

    // turn off anyways (could be in undefined state)
    if(sim_is_on()) {
        sim_onoff_pulse();
    }

    tmp8=3;
    while(tmp8) {
        wdt_reset();
        delay_ms(500);
        wdt_reset();
        if(!sim_is_on()) {
            sim_onoff_pulse();
        }
        sim_set_textmode();
        if(do_delete) { sim_delete_all_sms(); }
        sim_set_flowcontrol();
        ptr = sim_read_IMEI(sim_rxbuf, sizeof(sim_rxbuf));
        printf("IMEI=%s\n",ptr);
        printf("Pincheck=%s\n",(sim_pincode_check() ? "OK" : "FAIL"));

        if(ptr!=NULL) break;
        tmp8--;
    }
	
	if(sim_has_bluetooth()) {
		sysval.has_bluetooth=1;	
	}
}



//////////////////////////////////// Main Loop ////////////////////////////////////
int main(void) 
{
    uint8_t sreg,x,i;

    cli();
    initIO();

    PCICR |= (1<<PCIE3); // enable pin change interrupt on PCINT31:24
    PCMSK3 |= (1<<PCINT28); // enable PCINT28 interrupt on pin change (SIM900_RI line)

    //wdt_enable(WDTO_2S); // enable watchdog	
	WDTCSR = (1<<WDCE) | (1<<WDE);   			 // Enable the WD Change Bit - configure mode
	//WDTCSR = (1<<WDIE) | (1<<WDP2) | (1<<WDP1);	 // Enable WDT Interrupt, and Set Timeout to ~1 seconds ,or use WDTO_1S
	WDTCSR = (1<<WDIE) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);	 // Enable WDT Interrupt, and Set Timeout to ~2 seconds
		


    memset((void *)&sysval,0,sizeof(sysVals_t));

    twi_init();
    twi_timeout=0;

    INIT_UARTN_8N1(1,BRR115200,U2X115200); //Configure UART SIM900
    UCSR1B |= (1<<RXCIE1); // enable interrupts


    //INIT_UARTN_8N1(0,BRR9600,U2X9600); //Configure UART Serial
    INIT_UARTN_8N1(0,BRR115200,U2X115200); //Configure UART Serial
    UCSR0B |= (1<<RXCIE0); // enable interrupts
    stdout = &term_uart_str;

    x=0;
    UART_SIM_RxTail = x;
    UART_SIM_RxHead = x;
    UART_SIM_TxTail = x;
    UART_SIM_TxHead = x;
    UART_TERM_RxTail = x;
    UART_TERM_RxHead = x;

    wdt_reset();

    // turn on all leds:
    LED_CHARGING_ON();
    LED_100PERCENT_ON();
    LED_60PERCENT_ON();
    LED_30PERCENT_ON();
    LED_NOTPAYED_ON();
    LED_SHORTFAULT_ON();
    delay_ms(500);
    // turn off all leds:
    LED_CHARGING_OFF();
    LED_100PERCENT_OFF();
    LED_60PERCENT_OFF();
    LED_30PERCENT_OFF();
    LED_NOTPAYED_OFF();
    LED_SHORTFAULT_OFF();

    initTimers();

    delay_ms(500);
    wdt_reset();


    eeprom_busy_wait();
    sysval.payled_mode = eeprom_read_byte(&EEPROM_ADDR_LED_mode);
    sysval.seconds   = eeprom_read_dword(&EEPROM_ADDR_secondsMeter);
    sysval.Ws_in     = eeprom_read_dword(&EEPROM_ADDR_Ws_in);
    sysval.Ws_out    = eeprom_read_dword(&EEPROM_ADDR_Ws_out);
    sysval.Door_open = 0;
    sysval.intervalSMS = eeprom_read_word(&EEPROM_ADDR_intervalSMS);
    sysval.interval2G = eeprom_read_word(&EEPROM_ADDR_interval2G);
    sysval.Ws_quota  = (int32_t)eeprom_read_dword(&EEPROM_ADDR_Ws_quota);
    sysval.Ws_quota_enabled = eeprom_read_byte(&EEPROM_ADDR_Ws_quota_enabled);

    delay_ms(100);
    wdt_reset();

    sei();
	
    printf_P(PSTR(">> SHS-Control " FIRMWARE_VERSION " running >>\n\r"));
#ifdef SIM_DEBUG
	printf("TESTING1\n");
#endif

    sreg=SREG; cli();
    eeprom_read_block(UART_SIM_TxBuf,&EEPROM_ADDR_hostname,HOSTNAME_LEN);
    SREG=sreg;
        printf("EEPROM_ADDR_hostname:%s\n",UART_SIM_TxBuf);
        for(i=0;i<MAX_NR_OPERATORS;i++) {
                sreg=SREG; cli();
                eeprom_read_block(UART_SIM_TxBuf,&EEPROM_ADDR_OperatorNr[i],MAX_PHONE_NR_LENGTH);         
                SREG=sreg;
                printf("EEPROM_ADDR_OperatorNr[%d]:%s\n",i,UART_SIM_TxBuf);
        }

    startup_sim900(1);

    if(bit_is_clear(PINB,SPI_MOSI_IO_2)) { // jumper: SIM900 terminal mode
        INIT_UARTN_8N1(0,BRR115200,U2X115200); //Configure UART Serial
        UCSR0B |= (1<<RXCIE0); // enable interrupts
        
        LED_100PERCENT_ON();

        while(1) {
            LED_CHARGING_ON();
            delay_ms(500);
            wdt_reset();
            LED_CHARGING_OFF();
            delay_ms(500);
        }
    } 

    //Main loop
    while(1) {
        uint8_t sms_size=0;
        uint8_t sms_start_offset=0;
        uint8_t sms_reply_to,sms_pending;
        char termbuf[512];
        char *termbuf_ptr=NULL;

        wdt_reset();		

        /*
         * Wait for SMS
         * when ringing AND RI became inactive then it is time to read sms
         */
        while(!(sysval.ringing && bit_is_set(PIND,SIM900_RI)) ) {
            /*
             * meanwhile run tasks (readout the tracer and run statistics)
             */
            wdt_reset();
            delay_ms(100);
            run_tasks();
			if(sysval.watchdog > 60) {
				printf("WATCHDOG RESET!!!\n");
				RESTART_MICROCONTROLLER();
			}
        }
		printf("message processing\n");
        wdt_reset();
        sreg=SREG; cli();
        sms_pending=sysval.ringing;
        sysval.ringing=0; // reset
        SREG=sreg; // sei()
        if(sms_pending) {
			if(sysval.has_bluetooth) { sim_stop_bluetooth(); }
            termbuf_ptr=sim_receive_sms(termbuf,sizeof(termbuf));
            if(termbuf_ptr!=NULL) {
               printf("SMS cmd:%s\n",termbuf_ptr);
            } else {
               printf("no valid SMS\n");
            }
        }

        /*
         * Read the SMS and process (if valid content)
         */
        sysval.errorstatus &= ~((1<<ERROR_SMS_PARSE_ERROR)|(1<<ERROR_SMS_OPER_ERROR)|(1<<ERROR_SMS_ERROR));
        sms_reply_to=0;

        if(termbuf_ptr!=NULL) {
            if(strncasecmp(termbuf_ptr,"SOC?",4)==0) {
                termbuf_ptr[4]=0;
                sms_size=prepare_data_message(termbuf_ptr,&sms_start_offset);
                printf("offs=%d,smssize=%d\n",sms_start_offset,sms_size);
                sms_reply_to=((1<<REPLY_WITH_SMS)|(1<<REPLY_WITH_2G));

            } else if(strncasecmp(termbuf_ptr,"LOAD=",5)==0) { // expect: 0 or 1
                char *bufptr=&termbuf_ptr[5];
                int state;
                termbuf_ptr[6]=0;
                state=atoi(bufptr);
                task_set_load(state);
                task_every_5second(); // refresh
                sms_size=prepare_data_message(termbuf_ptr,&sms_start_offset);
                sms_reply_to=((1<<REPLY_WITH_SMS)|(1<<REPLY_WITH_2G));

            } else if(strncasecmp(termbuf_ptr,"RESET",5)==0) {
                termbuf_ptr[5]=0;
                sysval.Ws_in=0;
                sysval.Ws_out=0;

                sysval.Door_open=0;
                sysval.payled_mode=PAYLED_MODE_OFF;

                sreg=SREG; cli();
                eeprom_update_dword(&EEPROM_ADDR_Ws_in,sysval.Ws_in);
                eeprom_update_dword(&EEPROM_ADDR_Ws_out,sysval.Ws_out);
                eeprom_update_byte(&EEPROM_ADDR_LED_mode,sysval.payled_mode);
                SREG=sreg;

                sms_size=prepare_data_message(termbuf_ptr,&sms_start_offset);
                sms_reply_to=((1<<REPLY_WITH_SMS)|(1<<REPLY_WITH_2G));

            } else if(strncasecmp(termbuf_ptr,"RESTART",7)==0) {
                termbuf_ptr[7]=0;
                sms_size=prepare_data_message(termbuf_ptr,&sms_start_offset); 
                send_2g_message(UART_SIM_TxBuf,sms_size);
                sms_size-=sms_start_offset;
                sim_send_sms(&UART_SIM_TxBuf[sms_start_offset],sms_size,sysval.actual_operator_idx);
                RESTART_MICROCONTROLLER();

            } else if(strncasecmp(termbuf_ptr,"OPER",4)==0) {
                char operator_phonenr[MAX_PHONE_NR_LENGTH];
                int i;
                char c;
                char *idx_ptr=termbuf_ptr+4;
                char *phonenr_ptr=termbuf_ptr+6;
                if(isdigit(idx_ptr[0])) {
                    int idx=atoi(idx_ptr);
                    while((c=*phonenr_ptr) != 0) {
                        if(!isdigit(c) && c!='+') { *phonenr_ptr=0; break; }
                        phonenr_ptr++;
                    }
                    phonenr_ptr=termbuf_ptr+6;
                    write_operator_phonenr_to_eeprom(idx,phonenr_ptr);
                }

                sms_size=0;
                for(i=0;i<MAX_NR_OPERATORS;i++) {
                    if(i>0) sms_size+=sprintf(&UART_SIM_TxBuf[sms_size],";");
                    eeprom_read_block(operator_phonenr,&EEPROM_ADDR_OperatorNr[i],
                                      MAX_PHONE_NR_LENGTH);
                    sms_size+=sprintf(&UART_SIM_TxBuf[sms_size],"%s",operator_phonenr);
                }
                sms_reply_to=(1<<REPLY_WITH_SMS);

            } else if(strncasecmp(termbuf_ptr,"host=",5)==0) { // expect: hostname
                char c;
                char *ptr=&termbuf_ptr[5];
                while((c=*ptr) != 0) {
                    if(isspace(c)) { *ptr=0; break; }
                    ptr++;
                }
                sreg=SREG; cli();
                eeprom_update_block(&termbuf_ptr[5],&EEPROM_ADDR_hostname,strlen(termbuf_ptr)-5+1); // incl \0
                SREG=sreg;
                sms_size=sprintf(UART_SIM_TxBuf,"%s",&termbuf_ptr[5]);
                sms_reply_to=(1<<REPLY_WITH_SMS);

            } else if(strncasecmp(termbuf_ptr,"apn=",4)==0) { // expect: APN
                char c;
                char *ptr=&termbuf_ptr[4];
                while((c=*ptr) != 0) {
                    if(isspace(c)) { *ptr=0; break; }
                    ptr++;
                }
                sreg=SREG; cli();
                eeprom_update_block(&termbuf_ptr[4],&EEPROM_ADDR_apn,strlen(termbuf_ptr)-4+1); // incl \0
                SREG=sreg;
                sms_size=sprintf(UART_SIM_TxBuf,"\"%s\"",&termbuf_ptr[4]);
                sms_reply_to=(1<<REPLY_WITH_SMS);

            } else if(strncasecmp(termbuf_ptr,"user=",5)==0) { // expect: user
                char c;
                char *ptr=&termbuf_ptr[5];
                while((c=*ptr) != 0) {
                    if(isspace(c)) { *ptr=0; break; }
                    ptr++;
                }
                sreg=SREG; cli();
                eeprom_update_block(&termbuf_ptr[5],&EEPROM_ADDR_user,strlen(termbuf_ptr)-5+1); // incl \0
                SREG=sreg;
                sms_size=sprintf(UART_SIM_TxBuf,"\"%s\"",&termbuf_ptr[5]);
                sms_reply_to=(1<<REPLY_WITH_SMS);

            } else if(strncasecmp(termbuf_ptr,"pwd=",4)==0) { // expect: pwd
                char c;
                char *ptr=&termbuf_ptr[4];
                while((c=*ptr) != 0) {
                    if(isspace(c)) { *ptr=0; break; }
                    ptr++;
                }
                sreg=SREG; cli();
                eeprom_update_block(&termbuf_ptr[4],&EEPROM_ADDR_pwd,strlen(termbuf_ptr)-4+1); // incl \0
                SREG=sreg;
                sms_size=sprintf(UART_SIM_TxBuf,"\"%s\"",&termbuf_ptr[4]);
                sms_reply_to=(1<<REPLY_WITH_SMS);

            } else if(strncasecmp(termbuf_ptr,"PAYLED=",7)==0) { // expect: 0,1,2 or 3
                char *bufptr=&termbuf_ptr[7];
                termbuf_ptr[8]=0;
                sysval.payled_mode=atoi(bufptr);
                sreg=SREG; cli();
                eeprom_update_byte(&EEPROM_ADDR_LED_mode,sysval.payled_mode);
                SREG=sreg;
                sms_size=prepare_data_message(termbuf_ptr,&sms_start_offset);
                sms_reply_to=((1<<REPLY_WITH_SMS)|(1<<REPLY_WITH_2G));

            } else if(strncasecmp(termbuf_ptr,"T_SMS=",6)==0) { // expect: minutes
                char *bufptr=&termbuf_ptr[6];
                int i;
                for(i=6;i<strlen(termbuf_ptr);i++) {
                    if(!isdigit(termbuf_ptr[i])) { termbuf_ptr[i]=0; break; }
                }
                sreg=SREG; cli();
                sysval.intervalSMS=(uint16_t)atol(bufptr);
                if(sysval.intervalSMS > MINUTES_PER_DAY) sysval.intervalSMS=MINUTES_PER_DAY;
                SREG=sreg;
                

                sreg=SREG; cli();
                eeprom_update_word(&EEPROM_ADDR_intervalSMS,sysval.intervalSMS);
                SREG=sreg;
                sms_size=prepare_data_message(termbuf_ptr,&sms_start_offset);
                sms_reply_to=((1<<REPLY_WITH_SMS)|(1<<REPLY_WITH_2G));

            } else if(strncasecmp(termbuf_ptr,"T_2G=",5)==0) { // expect: minutes
                char *bufptr=&termbuf_ptr[5];
                int i;
                for(i=5;i<strlen(termbuf_ptr);i++) {
                    if(!isdigit(termbuf_ptr[i])) { termbuf_ptr[i]=0; break; }
                }
                sreg=SREG; cli();
                sysval.interval2G=(uint16_t)atol(bufptr);
                if(sysval.interval2G > MINUTES_PER_DAY) sysval.interval2G=MINUTES_PER_DAY;
                SREG=sreg;

                sreg=SREG; cli();
                eeprom_update_word(&EEPROM_ADDR_interval2G,sysval.interval2G);
                SREG=sreg;
                sms_size=prepare_data_message(termbuf_ptr,&sms_start_offset);
                sms_reply_to=((1<<REPLY_WITH_SMS)|(1<<REPLY_WITH_2G));

            } else if(strncasecmp(termbuf_ptr,"QUOTA=",6)==0) { // expect: Wh or 0
                char *bufptr=&termbuf_ptr[6];
                int i;
                uint16_t tmp16;
                for(i=6;i<strlen(termbuf_ptr);i++) {
                    if(!isdigit(termbuf_ptr[i])) { termbuf_ptr[i]=0; break; }
                }
                tmp16=(uint16_t)atol(bufptr);
                if(tmp16==0) {
                    sysval.Ws_quota_enabled=0;
                    sysval.Ws_quota=QUOTA_DEFAULT;
                } else {
                    sysval.Ws_quota_enabled=1;
                    sysval.Ws_quota=(int32_t)tmp16;
                    sysval.Ws_quota*=3600;
                }
                sreg=SREG; cli();
                eeprom_update_dword(&EEPROM_ADDR_Ws_quota,sysval.Ws_quota);
                eeprom_update_byte(&EEPROM_ADDR_Ws_quota_enabled,sysval.Ws_quota_enabled);
                SREG=sreg;
                sms_size=prepare_data_message(termbuf_ptr,&sms_start_offset);
                sms_reply_to=((1<<REPLY_WITH_SMS)|(1<<REPLY_WITH_2G));

            } else if(strncasecmp(termbuf_ptr,"TIME=",5)==0) { // expect: seconds
                char *bufptr=&termbuf_ptr[5];
                int i;
                for(i=5;i<strlen(termbuf_ptr);i++) {
                    if(!isdigit(termbuf_ptr[i])) { termbuf_ptr[i]=0; break; }
                }
                sysval.seconds=(uint32_t)atol(bufptr);
                sysval.seconds_old=sysval.seconds;
                sreg=SREG; cli();
                eeprom_update_dword(&EEPROM_ADDR_secondsMeter,sysval.seconds);
                SREG=sreg;
                sms_size=prepare_data_message(termbuf_ptr,&sms_start_offset);
                sms_reply_to=((1<<REPLY_WITH_SMS)|(1<<REPLY_WITH_2G));

            } else if(strncasecmp(termbuf_ptr,"EXTPWR=",7)==0) { // expect: 0 or 1
                char *bufptr=&termbuf_ptr[7];
                int tmp8;
                termbuf_ptr[8]=0;
                tmp8=atoi(bufptr);
                if(tmp8==0) {
                    PORTA &= ~(1<<IO_0); // turn off external powersupply
                } else {
                    PORTA |= (1<<IO_0); // turn on external powersupply
                }
                sms_size=prepare_data_message(termbuf_ptr,&sms_start_offset);
                sms_reply_to=((1<<REPLY_WITH_SMS)|(1<<REPLY_WITH_2G));
            } else {
                char c;
                char *ptr=termbuf_ptr;
                sysval.errorstatus |= (1<<ERROR_SMS_PARSE_ERROR);
                while((c=*ptr) != 0) {
                    if(!isalnum(c)) *ptr=' ';
                    ptr++;
                }
                sms_size=prepare_data_message(termbuf_ptr,&sms_start_offset);
                sms_reply_to=((1<<REPLY_WITH_SMS)|(1<<REPLY_WITH_2G));
            }
        }

        sim_delete_sms();

        if(sms_size>0 && sms_pending) { // send SMS after processed valid command
            uint8_t retries=2;
            int8_t smsretval;

            if(sms_reply_to&(1<<REPLY_WITH_2G)) {
                if(send_2g_message(UART_SIM_TxBuf,sms_size) != 0) {
                    sysval.errorstatus |= (1<<ERROR_2G_ERROR);
                }
            }
            sms_size-=sms_start_offset;
            if(sms_reply_to&(1<<REPLY_WITH_SMS)) {
                while(retries > 0) {
                    smsretval=sim_send_sms(&UART_SIM_TxBuf[sms_start_offset],sms_size,sysval.actual_operator_idx);
                    if(smsretval < -1) {
                        retries--;
                    } else break;
                }
            }
        } 
        sysval.errorstatus &= ~((1<<ERROR_SMS_PARSE_ERROR)|(1<<ERROR_SMS_OPER_ERROR)|(1<<ERROR_SMS_ERROR));
    }
}

/**
 * Timer 0 Compare Match Interrupt Vector - Used for MS Tick and ADC Sampling
 */
ISR(TIMER0_COMPA_vect) {
    //Update MS Tick
    sysTime.ms++;

    if(!(sysTime.ms%MSTICKS_PER_TENTHSEC)){
        if(twi_timeout>0) twi_timeout--;
    }

    //Update Sec Tick
    if(sysTime.ms >= MSTICKS_PER_SEC) { // 1152
        sysTime.ms=0;
        sysTime.sec++;
        sysval.seconds++;
        sysval.tasklist |= (1<<TASK_SECOND);

        if((sysTime.sec % 5)==0) {
            sysval.tasklist |= (1<<TASK_5SECOND);
			sysval.watchdog++;
        }
		if((sysTime.sec % 2)==0) {
			sysval.tasklist |= (1<<TASK_2SECOND);
		}
		if((sysTime.sec % 3)==0) {
			sysval.tasklist |= (1<<TASK_3SECOND);
		}
		

        if(sysval.SIM_poweron_pulse>4) {
            PORTC |= (1<<POWERKEY);
        } else {
            PORTC &= ~(1<<POWERKEY);
        }
        if(sysval.SIM_poweron_pulse>0) {
            sysval.SIM_poweron_pulse--;
        }
    }

    // Display SoCV if Quota is disabled
    if(sysval.Ws_quota_enabled == 0) {
        if(sysval.SoC > 0 && sysval.SoC < 30) {
            LED_100PERCENT_OFF();
            LED_60PERCENT_OFF();
            LED_30PERCENT_ON();
        }
        if(sysval.SoC > 31 && sysval.SoC < 60) {
            LED_100PERCENT_OFF();
            LED_60PERCENT_ON();
            LED_30PERCENT_OFF();
        }
        if(sysval.SoC > 61) {
            LED_100PERCENT_ON();
            LED_60PERCENT_OFF();
            LED_30PERCENT_OFF();
        }
    }

    // check for any battery which could be empty:
    if((sysval.batt_state&0x03)==0x02 || (sysval.batt_state&0x0c)==0x08 || 
	   (sysval.batt_state&0x30)==0x20 || (sysval.batt_state&0xc0)==0x20 || sysval.SoC==0) {
        LED_SHORTFAULT_ON();
        LED_100PERCENT_OFF();
        LED_60PERCENT_OFF();
        LED_30PERCENT_OFF();
	// check for short circuit or fuse tripped:
    } else if( ((sysval.load_state&0x03)==0x03 || (sysval.load_state&0x0c)==0x0c ||
	            (sysval.load_state&0x30)==0x30 || (sysval.load_state&0xc0)==0xc0)	
	           && ((sysTime.ms%MSTICKS_PER_SEC)<288)) { // load fault: blink
        LED_SHORTFAULT_ON();
    } else {
        LED_SHORTFAULT_OFF();
    }

    if(sysval.payled_mode == PAYLED_MODE_ON) {
        LED_NOTPAYED_ON();
    } else if(sysval.payled_mode == PAYLED_MODE_2HZ && ((sysTime.ms%MSTICKS_PER_HALFSEC)<144)) {
        LED_NOTPAYED_ON();
    } else if(sysval.payled_mode == PAYLED_MODE_1HZ && ((sysTime.ms%MSTICKS_PER_SEC)<288)) {
        LED_NOTPAYED_ON();
    } else {
        LED_NOTPAYED_OFF();
    }

    //Update minutes
    if(sysTime.sec>=60){
        sysTime.sec=0;
        sysval.minutes++;
        sysval.tasklist |= (1<<TASK_MINUTE);

        if((sysval.minutes % 5)==0 && sysval.minutes!=0) {
            sysval.tasklist |= (1<<TASK_CREG);
        }
        if((sysval.minutes % 60)==0) {
            sysval.tasklist |= (1<<TASK_HOUR);
        }
        if(sysval.intervalSMS > 0) {
            sysval.intervalSMS--; 
        }
        if((sysval.minutes % sysval.interval2G)==0) {
            sysval.tasklist |= (1<<TASK_2G);
        }
        if(sysval.minutes >= MINUTES_PER_DAY) {
            sysval.minutes=0;
            sysval.tasklist |= (1<<TASK_DAY);
        }
    }
}


/**
 * UART Data Received Interrupt - Update receive buffer.
 * SIM900
 */
ISR(USART1_RX_vect)
{
        unsigned char data;
        unsigned char tmphead;
                
        /* Read the received data */
        data = UDR1;

        /* Calculate buffer index */
        tmphead = (UART_SIM_RxHead + 1) & UART_SIM_RX_BUFFER_MASK;
        /* Store new index */
        UART_SIM_RxHead = tmphead;
        if (tmphead == UART_SIM_RxTail) {
                /* ERROR! Receive buffer overflow */
        }
        /* Store received data in buffer */
        UART_SIM_RxBuf[tmphead] = data;
		
	    if(bit_is_clear(PINB,SPI_MOSI_IO_2)) { // jumper: SIM900 terminal mode
	        if (data == '\n') {
		        loop_until_bit_is_set(UCSR0A, UDRE0);
		        UDR0='\r';
	        }
	        loop_until_bit_is_set(UCSR0A, UDRE0);
	        UDR0=data;
	        //PORTD |= (1<<SIM900_RTS);
	        return;
	    }
}

/**
 * UART Data Received Interrupt - Update receive buffer.
 * Terminal
 */
ISR(USART0_RX_vect) {
        unsigned char data;
        unsigned char tmphead;
        
        /* Read the received data */
        data = UDR0;
		if(bit_is_clear(PINB,SPI_MOSI_IO_2)) { // jumper: SIM900 terminal mode
			// forward char to SIM900:
			//PORTD &= ~(1<<SIM900_RTS);
			loop_until_bit_is_set(UCSR1A, UDRE1);
			//loop_until_bit_is_clear(PIND, SIM900_CTS);
			UDR1=data;
			return;
		}

        /* Calculate buffer index */
        tmphead = (UART_TERM_RxHead + 1) & UART_TERM_RX_BUFFER_MASK;
        /* Store new index */
        UART_TERM_RxHead = tmphead;
        if (tmphead == UART_TERM_RxTail) {
                /* ERROR! Receive buffer overflow */
        }
        /* Store received data in buffer */
        UART_TERM_RxBuf[tmphead] = data;    

        UDR0=data;
        if(data=='\r') {
            data='\n';
            loop_until_bit_is_set(UCSR0A, UDRE0);
            UDR0=data;
        }
}


// RI line
// SIM900_RI 
ISR(PCINT3_vect) {
    if(sim_is_on() && bit_is_clear(PIND,SIM900_RI)) {
        sysval.ringing=1;
    }
}

// This is the interrupt code, called every second, unless wdt_reset() was called sooner
ISR(WDT_vect)
{
	sysval.watchdog++;
	if (sysval.watchdog < 10) { // 60 seconds limit , that is ONE MINUTE
		// start timer again (we are still in interrupt-only mode)
		wdt_reset();
	} else {
		UDR0='#';
		// go for immediate reset
		WDTCSR = (1<<WDCE) | (1<<WDE);	// Enable the WD Change Bit - configure mode
		WDTCSR = (1<<WDE) | (1<<WDP0);	// set reset flag (WDE) and 16ms (WDP0)
	}
}
