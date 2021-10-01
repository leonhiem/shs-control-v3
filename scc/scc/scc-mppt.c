/*
 * Project: Solar Charger
 * Author : LeonH <L.Hiemstra@gmail.com>
 *
 * Fuses: lfuse:0xe2
 *        hfuse:0xd5
 *
 * Config:  ATtiny861
 */

#include <avr/io.h>
#include <stdint.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include "usi_twi_slave.h"

FUSES = 
{ 
   .high = 0xD5, 
   .low  = 0xE2, 
   .extended = 0xFF  
}; 

/*Modern AVRs will restart from a watchdog reset with the timer
active with the lowest time value.  This often results in the AVR
getting stuck in a reset loop.  The following code is the work
around. */
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) \
__attribute__((naked)) \
__attribute__((section(".init3")));
void get_mcusr(void)
{
	mcusr_mirror = MCUSR;
	MCUSR = 0;
	wdt_disable();
}

#define LITHIUM_3S 1

// Default:
#ifndef LITHIUM_3S
#  define LITHIUM_3S 0 
#endif

// Default:
#ifndef LEAD_ACID_12V
#  define LEAD_ACID_12V 0
#endif



#if (LEAD_ACID_12V==1 && LITHIUM_3S==1)
#  error "choose LEAD_ACID_12V=1 or LITHIUM_3S=1, not both"
#elif (LEAD_ACID_12V==0 && LITHIUM_3S==0)
#  error "choose LEAD_ACID_12V=1 or LITHIUM_3S=1"
#endif

#if (LITHIUM_3S==1)
#  define LEAD_ACID_12V 0
#endif

#if (LEAD_ACID_12V==1)
#  define LITHIUM_3S 0
#endif




#if (LITHIUM_3S==1)
#warning "compiling for Lithium Ion 3S battery"
#else
#warning "compiling for Lead Acid battery"
#endif


#define FW_VERSION 0x28

/*
 * The ATtiny261 internal RC clocksource must be set to 8MHz.
 * - CKDIV8 is disabled (lfuse bit7=1)
 */


/*
 * These are the division factors which are filled in the 
 * timer-compare-registers.
 * -> The larger the number the lower the frequency..
 * 
 * (Remember that the PLL oscillator is initialized to 64MHz)
 * Calculate for our push-pull circuit:
 * factor = 64MHz / 2 / wishedfrequency / 2
 *                  ^-- prescaler
 */
#define FREQ_100KHZ  160
#define FREQ_70KHZ   228
#define FREQ_63KHZ   254

#define FREQ_SET     FREQ_63KHZ
#define PWM_MAX     (FREQ_SET-20)

/*
 */
#define LED_GREEN  PA3
#define LED_RED    PA2

#define LED_ON(x)     (PORTA|=(1<<(x)))
#define LED_OFF(x)    (PORTA&=~(1<<(x)))
#define LED_TOGGLE(x) (PORTA=PORTA^(1<<(x)))

#define LEDN_OFF(x)   (PORTA|=(1<<(x)))
#define LEDN_ON(x)    (PORTA&=~(1<<(x)))

//#define MOSFET_H_off_L_off() {TCCR1C &= ~((1<<COM1D0) | (1<<COM1D1)); LED_OFF(LED_RED);}
//#define MOSFET_H_on_L_off()  {TCCR1C &= ~(1<<COM1D0); TCCR1C|=(1<<COM1D1); LED_OFF(LED_RED);} 
//#define MOSFET_H_on_L_on()   {TCCR1C &= ~(1<<COM1D1); TCCR1C|=(1<<COM1D0); LED_ON(LED_RED); } 

#define MOSFET_H_off_L_off() {TCCR1C &= ~((1<<COM1D0) | (1<<COM1D1)); }
#define MOSFET_H_on_L_off()  {TCCR1C &= ~(1<<COM1D0); TCCR1C|=(1<<COM1D1); } 
#define MOSFET_H_on_L_on()   {TCCR1C &= ~(1<<COM1D1); TCCR1C|=(1<<COM1D0); } 

#define PANEL_CONNECT()    PORTB |= (1<<PB6); 
#define PANEL_DISCONNECT() PORTB &= ~(1<<PB6); 

#define LOAD_ON()     PORTA &= ~(1<<PA1); 
#define LOAD_OFF()    PORTA |= (1<<PA1); 

//#define CHARGEPUMP_ON()  {PORTA &= ~(1<<PA0); LED_ON(LED_RED); }
//#define CHARGEPUMP_OFF() {PORTA |= (1<<PA0); LED_OFF(LED_RED); }
#define CHARGEPUMP_ON()  PORTA &= ~(1<<PA0);
#define CHARGEPUMP_OFF() PORTA |= (1<<PA0);

#define HAVE_POWERFAULT()  (!(PINA & (1<<PINA2)))
#define DOORSWITCH_PRESSED() (PINA & (1<<PINA3))

#define OCR1D_STARTUP_VAL 20

#define SOLAR_OK         0
#define SOLAR_TOO_LOW    1
#define SOLAR_TOO_HIGH   2

#define TWILIGHT_TIME   30
#define PRESET_As       18000000 //0L   // guessed 50Ah battery (in As)

#define BATT_NORMAL      0
#define BATT_FULL        1
#define BATT_EMPTY       2
#define BATT_GONE        3

// sysstate:
#define STATE_INIT       0
#define STATE_WAIT       1
#define STATE_CHARGE     2
#define STATE_ERROR      3 

/* ADC defenitions */
//#define cStartAdc    ((1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0)) // fADC=8MHz/64=125kHz
#define cStartAdc    ((1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)) // fADC=8MHz/128=62.5kHz
#define cRestartAdc  ((cStartAdc)|(1<<ADSC))
#define cAdcV        0 

#define INCREASING 1
#define DECREASING 0
#define MPPT_STEPSIZE_SMALL 4 // 0.5 Volt
#define MPPT_STEPSIZE_BIG   8 // 1.0 Volt

/*
 * This variable sits in the EEPROM:
 */
#define EEPROM_ADDR_dummy          ((uint8_t *)0)  // avoid address 0
#define EEPROM_ADDR_minute_meter   ((uint16_t *)1)
#define EEPROM_ADDR_day_meter      ((uint16_t *)3)
#define EEPROM_ADDR_load_onoff     ((uint8_t *)5)
#define EEPROM_ADDR_tempoffset     ((uint8_t *)6)
#define EEPROM_ADDR_batt_As_offset ((uint8_t *)7)
#define EEPROM_ADDR_As_max         ((uint32_t *)8)
#define EEPROM_ADDR_As             ((uint32_t *)12)


/*
 * Interrupt enable/disable macros
 */
#define DISABLE_INTERRUPTS()  {sreg=SREG; cli();} 
#define ENABLE_INTERRUPTS()   {SREG=sreg;}  /* same as sei() */

/*
 * Subroutine declarations:
 */
static uint8_t timed_out(void);
static void run_second_task(void);
static void run_minute_task(void);
static uint8_t filter_soc_output(void);
static void filter_soc_input(uint8_t val);
static void Flush_TWI_TX_Buffers(void);
static void Flush_TWI_RX_Buffers(void);
static void USI_TWI_Slave_Initialise(void);
static int8_t USI_TWI_Transmit_Byte(uint8_t);
static uint8_t USI_TWI_Receive_Byte(void);
static uint8_t USI_TWI_Data_In_Receive_Buffer(void);
static void update_eeprom_loadstate(uint8_t state);


typedef struct Time_s {
    uint8_t subs;
    uint32_t uptime;
    uint32_t systimeout;
    uint8_t slowstart;
    uint16_t day_meter;
    uint16_t minute_meter;
    uint16_t daylight_meter;
    uint16_t ydaylight_meter;
    uint8_t is_daytime;
    uint8_t day_am_pm;
} Time_t;

typedef struct ADC_s {
    uint8_t adcsequence;
    uint16_t vpv_oc;
    uint16_t adc_vsolar;
    uint16_t adc_isense_in;
    uint16_t adc_isense_out;
    uint16_t adc_vbatt;
    uint16_t adc_temp;
} ADC_t;

typedef struct Task_s {
    uint8_t second1_task;
    uint8_t second2_task;
    uint8_t second10_task;
    uint8_t minute_task;
    uint8_t minute60_task;
    uint8_t do_meas_vpv_oc;
    int8_t equalizing;
} Task_t;

typedef struct State_s {
    uint8_t sysstate;
    uint8_t solarstate;
    uint8_t battstate;
    uint8_t loadstate;
    uint8_t doorswitch;
    uint8_t master_watchdog;
    uint8_t floating;
    uint8_t overload;
    uint8_t overload_protect;
    uint8_t battery_empty_protect;
    uint8_t mosfets_enable;
    uint8_t adc_pwm_update;
} State_t;

typedef struct MPPT_s {
    uint8_t ocr1d;
    uint16_t Vbatt_float;
    uint16_t Vbatt_top;
    uint16_t vbatt_setpoint;
    uint16_t setpoint_vpv;
    uint8_t  setpoint_vpv_delta;
    uint8_t  mppt_stepsize;
    int32_t V1,V2;
    int32_t I1,I2;
    int32_t P1,P2;
    int8_t mppt_indicator;
} MPPT_t;

typedef struct Engineering_s {
    uint16_t Vsolar;
    uint16_t Isense_in;
    uint16_t Isense_out;
    uint16_t Vbatt;
    int32_t As;
    int32_t As_max;
    int8_t temperature;
    int8_t temperature_offset;
    int8_t batt_As_offset;
} Engineering_t;

typedef struct SCC_s {
    Time_t t;
    ADC_t adc;
    Task_t task;
    State_t state;
    MPPT_t mppt;
    Engineering_t eng;
} SCC_t;

SCC_t scc;

#define FILTER_SOC_SIZE 8
uint8_t filter_soc[FILTER_SOC_SIZE];
uint8_t filter_soc_idx;

static uint8_t TWI_slaveAddress;
static volatile uint8_t USI_TWI_Overflow_State;

static uint8_t TWI_RxBuf[TWI_RX_BUFFER_SIZE];
static volatile uint8_t TWI_RxHead;
static volatile uint8_t TWI_RxTail;

static uint8_t TWI_TxBuf[TWI_TX_BUFFER_SIZE];
static volatile uint8_t TWI_TxHead;
static volatile uint8_t TWI_TxTail;
static volatile int8_t TWI_usi_timeout;



// for Vbatt the resistor ladder is 12k and 3k ohm
// Vcc is 3.3V
// max voltage measurable is:
// i=3.3/3000 =  0.0011000 A
// 3.3 + 12000*i =  16.5A => 1023 sample value (10bit ADC)

#define VBATT_165V      1023 
#define VBATT_153V       948
#define VBATT_152V       942
#define VBATT_151V       936
#define VBATT_150V       930
#define VBATT_149V       924
#define VBATT_148V       918
#define VBATT_147V       911
#define VBATT_146V       905
#define VBATT_145V       899
#define VBATT_144V       894
#define VBATT_143V       887  
#define VBATT_142V       880  
#define VBATT_141V       874  
#define VBATT_140V       868  // 14.0 * 1023 / 16.5
#define VBATT_139V       862
#define VBATT_138V       855
#define VBATT_137V       849
#define VBATT_136V       843
#define VBATT_135V       837
#define VBATT_134V       831
#define VBATT_133V       824
#define VBATT_128V       794
#define VBATT_126V       781
#define VBATT_125V       775
#define VBATT_121V       755
#define VBATT_120V       744
#define VBATT_115V       713
#define VBATT_112V       694
#define VBATT_111V       688
#define VBATT_110V       682
#define VBATT_93V        577
#define VBATT_80V        496


#define VSOLAR_14V       108
#define VSOLAR_145V      112
#define VSOLAR_148V      114
#define VSOLAR_15V       116
#define VSOLAR_16V       124
#define VSOLAR_50V       386
#define VSOLAR_112V      868

/*
 * http://batteryuniversity.com/learn/article/charging_at_high_and_low_temperatures
 */
#if (LITHIUM_3S==1)
/* Lithium Ion 3S:
 * No temperature compensation.
 * But no charge at low (0-2degC) temperatures
 */
const uint16_t PM_lut_Vfloat[] PROGMEM = {
// 0          1          2          3          4          5          6          7          8          9 
   VBATT_111V,VBATT_111V,VBATT_111V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,
// Columns 10 through 19:
   VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,
// Columns 20 through 29:
   VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,
// Columns 30 through 39:
   VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,
// Columns 40 through 49:
   VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,VBATT_125V,
};
const uint16_t PM_lut_Vtop[] PROGMEM = {
// 0          1          2          3          4          5          6          7          8          9 
   VBATT_112V,VBATT_112V,VBATT_112V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,
// Columns 10 through 19:
   VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,
// Columns 20 through 29:
   VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,
// Columns 30 through 39:
   VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,
// Columns 40 through 49:
   VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,VBATT_126V,
};
#else
/* lead-acid:
 * Indexed by temperature 0-49 degC:
 * (3mV*6cell)/degC
 */
const uint16_t PM_lut_Vfloat[] PROGMEM = {
// 0          1          2          3          4          5          6          7          8          9 
   VBATT_142V,VBATT_142V,VBATT_142V,VBATT_142V,VBATT_141V,VBATT_141V,VBATT_141V,VBATT_141V,VBATT_141V,VBATT_141V,
// Columns 10 through 19:
   VBATT_140V,VBATT_140V,VBATT_140V,VBATT_140V,VBATT_140V,VBATT_140V,VBATT_140V,VBATT_139V,VBATT_139V,VBATT_139V,
// Columns 20 through 29:
   VBATT_139V,VBATT_139V,VBATT_138V,VBATT_138V,VBATT_138V,VBATT_138V,VBATT_138V,VBATT_138V,VBATT_137V,VBATT_137V,
// Columns 30 through 39:
   VBATT_137V,VBATT_137V,VBATT_137V,VBATT_137V,VBATT_136V,VBATT_136V,VBATT_136V,VBATT_136V,VBATT_136V,VBATT_135V,
// Columns 40 through 49:
   VBATT_135V,VBATT_135V,VBATT_135V,VBATT_135V,VBATT_135V,VBATT_134V,VBATT_134V,VBATT_134V,VBATT_134V,VBATT_133V,
};
/*
 * Indexed by temperature 0-49 degC:
 * (5mV*6cell)/degC
 */
const uint16_t PM_lut_Vtop[] PROGMEM = {
// 0          1          2          3          4          5          6          7          8          9 
   VBATT_153V,VBATT_153V,VBATT_152V,VBATT_152V,VBATT_152V,VBATT_151V,VBATT_151V,VBATT_151V,VBATT_151V,VBATT_150V,
// Columns 10 through 19:
   VBATT_150V,VBATT_150V,VBATT_149V,VBATT_149V,VBATT_149V,VBATT_148V,VBATT_148V,VBATT_148V,VBATT_148V,VBATT_147V,
// Columns 20 through 29:
   VBATT_147V,VBATT_147V,VBATT_146V,VBATT_146V,VBATT_146V,VBATT_145V,VBATT_145V,VBATT_145V,VBATT_145V,VBATT_144V,
// Columns 30 through 39:
   VBATT_144V,VBATT_144V,VBATT_143V,VBATT_143V,VBATT_143V,VBATT_142V,VBATT_142V,VBATT_142V,VBATT_142V,VBATT_141V,
// Columns 40 through 49:
   VBATT_141V,VBATT_141V,VBATT_140V,VBATT_140V,VBATT_140V,VBATT_139V,VBATT_139V,VBATT_139V,VBATT_138V,VBATT_138V,
};
#endif

void set_battstate(void)
{
    if(scc.adc.adc_vbatt < VBATT_80V || scc.adc.adc_vbatt > VBATT_153V) {
        scc.state.battstate = BATT_GONE;
        scc.state.floating=0;
        scc.state.battery_empty_protect=1;
#if (LITHIUM_3S==1)
    } else if(scc.adc.adc_vbatt < VBATT_93V) {
#else
    // lead-acid
    } else if(scc.adc.adc_vbatt < VBATT_112V) {
#endif
        scc.state.battstate = BATT_EMPTY;
        scc.state.floating=0;
        if(scc.t.uptime>10) { scc.state.battery_empty_protect=1; }
    } else if(scc.adc.adc_vbatt > scc.mppt.Vbatt_top) {
        scc.state.battstate = BATT_FULL;
        scc.state.floating=1;
#if (LITHIUM_3S==1)
    } else if(scc.adc.adc_vbatt < VBATT_125V) {
#else
    // lead-acid
    } else if(scc.adc.adc_vbatt < VBATT_135V) {
#endif
        scc.state.battstate = BATT_NORMAL;
        scc.state.floating=0;
    }

#if (LITHIUM_3S==1)
    if(scc.state.battery_empty_protect && scc.adc.adc_vbatt > VBATT_111V) {
#else
    // lead-acid
    if(scc.state.battery_empty_protect && scc.adc.adc_vbatt > VBATT_120V) {
#endif
        scc.state.battery_empty_protect=0;
    }
}

void set_solarstate(void)
{
    if(scc.adc.adc_vsolar < VSOLAR_14V) { // <14V
        scc.state.solarstate = SOLAR_TOO_LOW;
    } else if(scc.adc.adc_vsolar > VSOLAR_50V) { // >50V
        scc.state.solarstate = SOLAR_TOO_HIGH;
    } else {
        scc.state.solarstate = SOLAR_OK;
    }
}

void measure_vpv_oc(void)
{
    scc.t.slowstart=1; // to prevent oscillations 
    scc.mppt.mppt_stepsize=MPPT_STEPSIZE_BIG;
    MOSFET_H_off_L_off();
    PANEL_DISCONNECT();
    CHARGEPUMP_ON();
    scc.mppt.ocr1d=OCR1D_STARTUP_VAL;
}

ISR(TIMER1_OVF_vect)
{
    // update new PWM synchronously with a new period
    OCR1D=scc.mppt.ocr1d;
}

/*
 * runs at 50Hz
 */
ISR(TIMER0_COMPA_vect)
{       
    uint8_t save_led_red, save_led_green;
    scc.t.subs++; 
    if(scc.t.subs > 20) scc.t.slowstart=0;

    if(scc.adc.adc_isense_out > 330) {
        scc.state.overload++; // >10.6 Ampere; counting...
    } else {
        scc.state.overload=0;
    }
    if(scc.state.overload > 12) { // 0.24 seconds in overload
        LOAD_OFF();
        scc.state.overload_protect=10; // 10 seconds OFF
    }

    if((scc.t.subs%10)==0) { // 5Hz 
        set_battstate();
        set_solarstate();

        if(scc.state.solarstate==SOLAR_OK) {
            scc.state.mosfets_enable=1;
        } else {
            scc.state.mosfets_enable=0;
        }
        if(scc.state.battstate==BATT_GONE) {
            scc.state.mosfets_enable=0;
        }


        if(TWI_usi_timeout>0) TWI_usi_timeout--;

        save_led_red = (PORTA & (1<<PA2)); // save red led state
        // check POWERFAULT
        PORTA &= ~(1<<PA2);// turn off first
        DDRA &= ~(1<<PA2); // set as input (temporary)
        PORTA |= (1<<PA2); // turn on pullup
        scc.state.loadstate = ~((PORTA & (1<<PA1))>>1); // get actual state ON/OFF
        scc.state.loadstate &= 0x1;  // reset powerfault bit

        if(HAVE_POWERFAULT() ||
          (scc.state.overload>0 || scc.state.overload_protect>0)) {
            scc.state.loadstate |= (1<<1); // set powerfault
        }
        PORTA &= ~(1<<PA2);    // turn off pullup first
        DDRA |= (1<<PA2);      // set as output again
        PORTA |= save_led_red; // restore red led state

        save_led_green = (PORTA & (1<<PA3)); // save green led state
        // check DOORSWITCH
        PORTA &= ~(1<<PA3);// turn off first
        DDRA &= ~(1<<PA3); // set as input (temporary)
        PORTA |= (1<<PA3); // turn on pullup
        if(!DOORSWITCH_PRESSED()) {
            scc.state.doorswitch=1;
        } else {
            scc.state.doorswitch=0;
        }
        PORTA &= ~(1<<PA3);      // turn off pullup first
        DDRA |= (1<<PA3);        // set as output again
        PORTA |= save_led_green; // restore green led state
    }

    if(scc.t.subs > 10) {
        if(scc.task.do_meas_vpv_oc) {
            scc.t.slowstart=1; // to prevent oscillations 
            scc.task.do_meas_vpv_oc=0;
        }
    }

    if(scc.t.subs == 25) {    // == 1st 1 second
        scc.task.second1_task=1;
    }

    if(scc.t.subs >= 50) {    // == 2nd 1 second
        scc.t.subs=0;
        scc.t.uptime+=1;
        scc.task.second2_task=1;
        if(scc.state.master_watchdog < 200) { scc.state.master_watchdog++; }
        if(scc.state.overload_protect > 0) { scc.state.overload_protect--; }

        if(scc.t.uptime%10==0) { // every 10 seconds
            scc.task.second10_task=1;
            if(scc.mppt.mppt_stepsize==MPPT_STEPSIZE_BIG) {
                scc.mppt.mppt_stepsize=MPPT_STEPSIZE_SMALL;
            } else {
                scc.mppt.mppt_stepsize=MPPT_STEPSIZE_BIG;
            }
        }


        if((scc.state.sysstate==STATE_WAIT || scc.state.sysstate==STATE_INIT) && ((scc.t.uptime%5)==0)) { 
            scc.task.do_meas_vpv_oc=1;
            measure_vpv_oc();
        }


        if(!scc.task.do_meas_vpv_oc && ((scc.t.uptime%5)==0)) {
            int32_t dPdV,dV,dI,dP;
            scc.mppt.V2 = (int32_t)scc.adc.adc_vsolar;
            scc.mppt.I2 = (int32_t)scc.adc.adc_isense_in;
            scc.mppt.P2 = scc.mppt.V2 * scc.mppt.I2;
            dV = scc.mppt.V2-scc.mppt.V1;
            dI = scc.mppt.I2-scc.mppt.I1;
            dP = scc.mppt.P2-scc.mppt.P1;
            dPdV = dP / dV;
            scc.mppt.mppt_indicator=0;

            if(scc.mppt.P2 == 0) { // lost it
                scc.mppt.setpoint_vpv = VSOLAR_15V;
                scc.task.do_meas_vpv_oc=1;
                measure_vpv_oc();
                //mppt_pwm_step=1;
            } else {
                if(dV == 0) {
                    if(dI > 0) {
                        scc.mppt.mppt_indicator=-scc.mppt.mppt_stepsize;
                        if(scc.mppt.setpoint_vpv > VSOLAR_145V) { scc.mppt.setpoint_vpv-=scc.mppt.mppt_stepsize; }
                        //mppt_pwm_step=1;
                    } else if(dI < 0) {
                        scc.mppt.mppt_indicator=scc.mppt.mppt_stepsize;
                        if(scc.mppt.setpoint_vpv < VSOLAR_50V) { scc.mppt.setpoint_vpv+=scc.mppt.mppt_stepsize; }
                        //mppt_pwm_step=-1;
                    } else {
                        //mppt_pwm_step=0;
                    }
                } else {
                    if(dPdV > 0) {
                        // go right
                        scc.mppt.mppt_indicator=scc.mppt.mppt_stepsize;
                        if(scc.mppt.setpoint_vpv < VSOLAR_50V) { scc.mppt.setpoint_vpv+=scc.mppt.mppt_stepsize; }
                        //mppt_pwm_step=-1;
                    } else if(dPdV < 0) {
                        // go left
                        scc.mppt.mppt_indicator=-scc.mppt.mppt_stepsize;
                        if(scc.mppt.setpoint_vpv > VSOLAR_145V) { scc.mppt.setpoint_vpv-=scc.mppt.mppt_stepsize; }
                        //mppt_pwm_step=1;
                    }  else {
                        //mppt_pwm_step=0;
                    }
                }
            }
            scc.mppt.V1 = scc.mppt.V2;
            scc.mppt.I1 = scc.mppt.I2;
            scc.mppt.P1 = scc.mppt.P2;
        }
    }
    if(scc.t.subs == 10) {    // == 3rd 1 second
        if((scc.t.uptime%60)==0) { // == every minute
            /*
             * Keep track of day time
             */
            if(scc.t.is_daytime == 0) {
                if(scc.t.daylight_meter > 0) { 
                    scc.t.ydaylight_meter=scc.t.daylight_meter;
                    scc.t.daylight_meter=0;
                }
            } else if(scc.t.is_daytime >= TWILIGHT_TIME) {
                scc.t.daylight_meter++;
            }
            scc.t.minute_meter++;
            scc.task.minute_task=1;
        }
    }
}

/*
 * Get ADC samples
 * regulation loops
 */
// ADC channels:
#define ADC_VINsense    6
#define ADC_VOUTsense   5
#define ADC_Icharge     4
#define ADC_Iload       3
#define ADC_TEMPsense  0xf 


//fADC=8MHz/64=125kHz
// adc conversion is 13 CK periods
// 4 channels
// 8e6/64/4/13 ~ 2403Hz
ISR(ADC_vect)
{
    uint8_t muxtmp;
    uint8_t lsb,msb;
    uint16_t sample;

    muxtmp = ADMUX;
    muxtmp &= 0xf;

    lsb = ADCL; // read the low part
    msb = ADCH; // read the high part

    sample = (uint16_t)lsb;
    sample |= ((uint16_t)msb)<<8;

    scc.state.adc_pwm_update++;

    if(muxtmp == ADC_VINsense) { // measured 1372 Hz
        /*
         * during 'do_meas_vpv_oc' toggle in between ADC_VINsense and ADC_TEMPsense
         */
        if(scc.task.do_meas_vpv_oc) {
            scc.adc.vpv_oc = sample;
            scc.adc.adcsequence=ADC_TEMPsense;
        } else {
            scc.adc.adc_vsolar = sample;
            scc.adc.adcsequence=ADC_Icharge;
        }
    } else if(muxtmp == ADC_TEMPsense) {
        scc.adc.adc_temp = sample;
        scc.adc.adcsequence=ADC_VINsense;
    } else if(muxtmp == ADC_Icharge) {
        scc.adc.adc_isense_in=sample;
        if(scc.task.do_meas_vpv_oc) { scc.adc.adcsequence=ADC_VINsense; } else { scc.adc.adcsequence=ADC_VOUTsense; }
    } else if(muxtmp == ADC_VOUTsense) {
        scc.adc.adc_vbatt = sample;
        if(scc.task.do_meas_vpv_oc) { scc.adc.adcsequence=ADC_VINsense; } else { scc.adc.adcsequence=ADC_Iload; }
    } else if(muxtmp == ADC_Iload) {
        scc.adc.adc_isense_out=sample;
        scc.adc.adcsequence=ADC_VINsense;
    }

    if(!scc.task.do_meas_vpv_oc) {
        if(scc.state.mosfets_enable) {
            if(!scc.t.slowstart) {
                if(scc.state.floating) { // floating when battery is full: keep Vbatt=~13.8V
                    if(scc.adc.adc_vbatt > (scc.mppt.Vbatt_float+5)) { // limit vbatt to ~13.8V(lead-acid:depends on temp)
                        if(scc.mppt.ocr1d > OCR1D_STARTUP_VAL) {
                            scc.mppt.ocr1d-=1;
                        }
                    } else if((scc.adc.adc_vbatt < (scc.mppt.Vbatt_float-5)) && (scc.adc.adc_vsolar >= scc.mppt.setpoint_vpv)) {
                        if((scc.mppt.ocr1d < PWM_MAX) && (scc.adc.adc_isense_in < 330)) { // charge current limit to 10A
                            scc.mppt.ocr1d+=1;
                        }
                    }

                } else { // not floating but normal charge
                    if((scc.state.adc_pwm_update%8)==0) {
                        if((scc.adc.adc_vsolar < scc.mppt.setpoint_vpv-5) /*&& mppt_pwm_step<0*/) {
                            if(scc.mppt.ocr1d > OCR1D_STARTUP_VAL) {
                                scc.mppt.ocr1d-=1;
                            }
                        } else if((scc.adc.adc_vsolar > scc.mppt.setpoint_vpv+5) /*&& mppt_pwm_step>0*/) {
                            if((scc.mppt.ocr1d < PWM_MAX) && (scc.adc.adc_isense_in < 330)) { // charge current limit to 10A
                                scc.mppt.ocr1d+=1;
                            }
                        }
                    }
                }
            }
    
            if(scc.adc.adc_isense_in > 62 && (PORTB & (1<<PB6)) ) { // > 0.2V = ~1.5A
                MOSFET_H_on_L_on();  // enable LO mosfet
            } else if(scc.adc.adc_isense_in < 50) {
                MOSFET_H_on_L_off(); // disable LO mosfet (some hysteresis based on current)
            }
        } else {
            MOSFET_H_off_L_off(); // mosfets not enabled
        }

        if(scc.adc.adc_isense_in > 10 && (scc.task.do_meas_vpv_oc==0) && (PORTB & (1<<PB6)) ) { // > ~250mA
            CHARGEPUMP_OFF();
        }
        if(scc.state.solarstate == SOLAR_OK) {
            PANEL_CONNECT();
        } else {
            PANEL_DISCONNECT();
            CHARGEPUMP_OFF();
            scc.mppt.ocr1d=OCR1D_STARTUP_VAL;
        }
    }

    /*
     * (Re)Start ADC
     */
    // Init ADC
    if(scc.adc.adcsequence==ADC_TEMPsense) { 
        ADMUX = 0x9f; // use internal 1.1 VREF
        ADCSRB |= (1<<MUX5);
        ADCSRB &= ~(1<<REFS2);
    } else {
        ADMUX = cAdcV;
        ADMUX |= scc.adc.adcsequence;
        ADCSRB &= ~(1<<MUX5);
    }
    ADCSRA = cRestartAdc; // Restart the ADC again   
}


/*
 * Main loop
 */
int main() 
{
    uint8_t j;
    cli();
    wdt_reset(); 
    wdt_disable();

    memset((void *)&scc,0,sizeof(scc));

    // init portB
    DDRB = 0xf0;
    PORTB = 0x0a; // pull-up resistors for dipswitch

    // init portA
    DDRA = 0x0f;     

    j = eeprom_read_byte(EEPROM_ADDR_load_onoff);
    if(j==1) {
        PORTA = 0x00; // chargepump ON; LOAD ON default
        scc.state.loadstate=1;
    } else {
        PORTA = 0x02; // chargepump ON; LOAD OFF default
    }

    scc.adc.adcsequence=ADC_VINsense;
    scc.adc.vpv_oc=1023;
    scc.task.do_meas_vpv_oc=1;
    scc.adc.adc_vbatt=VBATT_120V;
    scc.adc.adc_temp=300;
    scc.mppt.mppt_stepsize=MPPT_STEPSIZE_SMALL;
#if (LITHIUM_3S==1)
    scc.mppt.Vbatt_top=VBATT_126V;
    scc.mppt.Vbatt_float=VBATT_125V;
#else
    // lead-acid
    scc.mppt.Vbatt_top=VBATT_145V;
    scc.mppt.Vbatt_float=VBATT_138V;
#endif
    scc.t.slowstart=1;
    scc.t.systimeout=scc.t.uptime+6;

    scc.state.sysstate = STATE_INIT;
    scc.state.solarstate = SOLAR_TOO_LOW;
    scc.state.battstate=BATT_NORMAL;
    scc.eng.temperature=20;

    // Init USI
    memset(TWI_RxBuf,0,TWI_RX_BUFFER_SIZE);
    memset(TWI_TxBuf,0,TWI_TX_BUFFER_SIZE);
    USI_TWI_Slave_Initialise();

    memset(filter_soc,100,FILTER_SOC_SIZE);
    filter_soc_idx=0;

    // init timers
    /* PLL config: */
    /* high speed: 1MHz * 8 = 8MHz PLL */
    PLLCSR|=(1<<PLLE); /* SET PLLE bit: start PLL (high speed) (page 120)*/

    /* Wait a bit */
    for (j=0; j<100; j++) { asm volatile ("nop"); }
    while (!((PLLCSR&0x01)==0x01)) {
        /* check PLOCK-bit: wait for PLL until locked */
    }
    PLLCSR|=(1<<PCKE);/* SET PCKE-bit: PLL is clk source timer1 (page 120) */


    // H1=nOC1D
    // H2=OC1D

    /* Timer/Counter 1 Init (this channel drives the mosfets) */
    //TCCR1B=(1<<DTPS11)|(1<<DTPS10)|(1<<CS12);/* DT=(8MHz/8=1MHz) en TCNT1=(8MHz/8=1MHz) */
    TCCR1B=(1<<DTPS11)|(1<<DTPS10)|(1<<CS11);

    TCCR1A=0;
    TCCR1C=(1<<COM1D0)|(1<<PWM1D); // enable PWM on comperator OCR1D: (OC1D and nOC1D connected)
    TCCR1D=(1<<WGM10); /* phase/freq correct pwm mode */
    TCCR1E=0;
    DT1=0x33;         /* symetric dead time */

    OCR1C=FREQ_SET; // freq
    OCR1D=0; // pwm

    MOSFET_H_off_L_off();
    PANEL_DISCONNECT();
    CHARGEPUMP_ON();

    // timer0 (ADC restart + uptime)
    TCCR0A =(1<</*CTC0*/0); //  mode 0-OCR1A
    TCCR0B = (1<<CS02)|(1<<CS00); // prescaler /1024
    OCR0A = 156;  // 50Hz

    // Timer0 interrupt on compare match
    // Timer1 interrupt on overflow
    TIMSK|=(1<<OCIE0A)|(1<<TOIE1); 

    /* pin change interrupts: */
    //GIMSK |= (1<<PCIE0); // enable pin change interrupt on pins PCINT11:8
    //PCMSK1=0;

    /* Init ADC: */
    ADMUX = cAdcV;
    ADMUX |= scc.adc.adcsequence;
    ADCSRA = cRestartAdc; // Restart the ADC again  


    // digital input disable register) using ADC 3,4,5,6 (page 162)
    DIDR0=(1<<ADC6D)|(1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D);
    DIDR1=0x00;

    ACSRA |= (1<<ACD);  /* Turn off the analog comperator to safe power */

    scc.t.minute_meter=eeprom_read_word(EEPROM_ADDR_minute_meter);
    scc.t.day_meter=eeprom_read_word(EEPROM_ADDR_day_meter);
    scc.eng.temperature_offset=eeprom_read_byte(EEPROM_ADDR_tempoffset);
    scc.eng.batt_As_offset=eeprom_read_byte(EEPROM_ADDR_batt_As_offset);
    if(scc.eng.batt_As_offset < 0) { scc.eng.batt_As_offset=1; }
    scc.eng.As_max=eeprom_read_dword(EEPROM_ADDR_As_max);
    if(scc.eng.As_max < 0L) { scc.eng.As_max=PRESET_As; }
    scc.eng.As=eeprom_read_dword(EEPROM_ADDR_As);
    if(scc.eng.As < 0L) { scc.eng.As=PRESET_As; }

    wdt_enable(WDTO_2S); // enable watchdog

    /* enable interrupts: */	
    sei();		  

    LEDN_ON(LED_RED);
    LEDN_ON(LED_GREEN);

    /* Main loop: */
    while (1) {
        uint8_t sreg,tmp_solarstate,tmp_battstate;
        uint16_t tmp_voltcalc=0,tmp_voltval=0;
#define I2C_DATA_OUT_SIZE 16
        uint8_t i2c_data_out[I2C_DATA_OUT_SIZE];

        wdt_reset();

        DISABLE_INTERRUPTS();
        tmp_solarstate = scc.state.solarstate;
        tmp_battstate = scc.state.battstate;
        ENABLE_INTERRUPTS();

        if(scc.state.sysstate == STATE_INIT) {
            scc.state.master_watchdog=0;
            if(scc.task.do_meas_vpv_oc) {
                set_sleep_mode(SLEEP_MODE_IDLE);
                sleep_enable();
                sleep_mode();
                // in idle now: better noise suppression to measure temperature
                sleep_disable();

                DISABLE_INTERRUPTS();
                tmp_voltval = scc.adc.vpv_oc;
                scc.mppt.V1 = (int32_t)scc.adc.vpv_oc;
                scc.mppt.I1 = (int32_t)scc.adc.adc_isense_in;
                scc.mppt.P1 = scc.mppt.V1 * scc.mppt.I1;
                ENABLE_INTERRUPTS();

                tmp_voltcalc=tmp_voltval * 8;
                tmp_voltcalc=tmp_voltcalc / 10; // times 0.80

                scc.mppt.setpoint_vpv=tmp_voltcalc;
                scc.mppt.setpoint_vpv_delta=0;

                DISABLE_INTERRUPTS();
                tmp_voltval = scc.adc.adc_temp;
                ENABLE_INTERRUPTS();

                scc.eng.temperature = (int8_t)(tmp_voltval-273-scc.eng.temperature_offset); // convert temp to Celsius
                // look up voltage compensation for Vbatt_float and Vbatt_top:
                if(scc.eng.temperature < 0)  { scc.eng.temperature=0;  }
                if(scc.eng.temperature > 49) { scc.eng.temperature=49; }
                scc.mppt.Vbatt_float = pgm_read_word(&PM_lut_Vfloat[scc.eng.temperature]);
                scc.mppt.Vbatt_top   = pgm_read_word(&PM_lut_Vtop  [scc.eng.temperature]);
            }
            if(timed_out()) {
                scc.state.sysstate = STATE_CHARGE;
                scc.t.systimeout=scc.t.uptime+2;
                USI_TWI_Slave_Initialise();
                // Initialization done
                LEDN_OFF(LED_RED);
                LEDN_OFF(LED_GREEN);
            }
        } else {
            if(tmp_solarstate==SOLAR_TOO_LOW || tmp_battstate==BATT_GONE) {
                scc.state.sysstate = STATE_WAIT;
            }
            if(tmp_battstate == BATT_GONE) {
                scc.state.sysstate = STATE_ERROR;
            }
            if(scc.state.sysstate==STATE_ERROR && tmp_battstate!=BATT_GONE) {
                scc.state.sysstate = STATE_INIT;
                scc.t.systimeout=scc.t.uptime+1;
            }
            if(scc.state.sysstate==STATE_WAIT && tmp_solarstate==SOLAR_OK) {
                scc.state.sysstate = STATE_INIT;
                scc.t.systimeout=scc.t.uptime+6;
            }


            /*
             * Update the GREEN LED: (Solar) Charge status
             */
            if(tmp_battstate == BATT_GONE || tmp_battstate == BATT_EMPTY) {
                LEDN_OFF(LED_GREEN);
            } else if(tmp_battstate == BATT_FULL) {
                if(scc.t.subs>25) {
                    LEDN_ON(LED_GREEN);
                } else {
                    LEDN_OFF(LED_GREEN);
                }
            } else { // BATT_CHARGE
                if(scc.eng.Isense_in > 3) { // 300mA
                    LEDN_ON(LED_GREEN);
                }
                if(scc.eng.Isense_in < 1) { // 100mA
                    LEDN_OFF(LED_GREEN);
                }
            }
        }



        /*
         * Update the RED LED: (Load) Fault status
         */
        if((scc.state.loadstate&0x2) || scc.state.overload) {
            LEDN_ON(LED_RED); // have power fault
        } else {
            if(tmp_battstate == BATT_EMPTY) {
                // blink if batt empty
                if(scc.t.subs>25) {
                    LEDN_ON(LED_RED);
                } else {
                    LEDN_OFF(LED_RED);
                }
            } else {
                LEDN_OFF(LED_RED);
            }
        }

        /*
         * Check master watchdog
         */
        if(scc.state.master_watchdog > 180) { // 3 minutes
            LOAD_OFF();
        }

        /*
         * SoC calculation (once per 10 seconds):
         */
        if(tmp_battstate != BATT_GONE && scc.task.second10_task) {
            uint8_t charging;
            DISABLE_INTERRUPTS();
            scc.task.second10_task=0;
            if(scc.adc.adc_isense_in > scc.adc.adc_isense_out) {
                tmp_voltval = scc.adc.adc_isense_in - scc.adc.adc_isense_out;
                if(tmp_voltval > 160) tmp_voltval = 160; // 160 ~> 5.1A
                
                charging=(uint8_t)(tmp_voltval/2);
            } else {
                charging=0;
            }
            tmp_voltval = scc.adc.adc_vbatt;
            ENABLE_INTERRUPTS();
            if(charging > 0) {

#if (LITHIUM_3S==1)
                if(tmp_voltval < VBATT_93V) {
#else
                // lead-acid
                // Battery voltage range during charging: 11-14.5V
                if(tmp_voltval < VBATT_112V) {
#endif
                    filter_soc_input(0);
                } else if(tmp_voltval > scc.mppt.Vbatt_top) {
                    filter_soc_input(100);
                } else {
#if (LITHIUM_3S==1)
                    if(tmp_voltval >= VBATT_93V) {
                        tmp_voltval -= VBATT_93V;
#else
                    // lead-acid
                    if(tmp_voltval >= VBATT_112V) {
                        tmp_voltval -= VBATT_112V;
                        if(tmp_voltval > charging) {
                            tmp_voltval -= charging;
                        } else {
                            tmp_voltval = 0;
                        }
#endif
                    } else {
                        tmp_voltval = 0;
                    }
#if (LITHIUM_3S==1)
                    filter_soc_input((uint8_t)(tmp_voltval/2)); // VBATT_93V=>0, VBATT_125V=>100
#else
                    // lead-acid
                    //filter_soc_input((uint8_t)(tmp_voltval/2)); // VBATT_121V=>0 , VBATT_144V=>100
                    filter_soc_input((uint8_t)(tmp_voltval)); // VBATT_121V=>0 , VBATT_144V=>100
#endif
                }
            } else { // Battery voltage range during discharging: 11-12.8 V
#if (LITHIUM_3S==1)
                if(tmp_voltval < VBATT_93V) {
#else
                // lead-acid
                if(tmp_voltval < VBATT_112V) {
#endif
                    filter_soc_input(0);
#if (LITHIUM_3S==1)
                } else if(tmp_voltval > VBATT_125V) { // 775
#else
                // lead-acid
                } else if(tmp_voltval > VBATT_128V) { // 794
#endif
                    filter_soc_input(100);
                } else {
#if (LITHIUM_3S==1)
                    if(tmp_voltval >= VBATT_93V) {
                        tmp_voltval -= VBATT_93V; // 577
#else
                    // lead-acid
                    if(tmp_voltval >= VBATT_112V) {
                        tmp_voltval -= VBATT_112V; // 694
#endif
                    } else {
                        tmp_voltval = 0;
                    }
#if (LITHIUM_3S==1)
                    filter_soc_input((uint8_t)(tmp_voltval/2)); // VBATT_93V=>0, VBATT_125V=>100
#else
                    // lead-acid
                    filter_soc_input((uint8_t)tmp_voltval); // VBATT_112V=>0 , VBATT_128V=>100
#endif
                }
            }
        }

        /*
         * Tasks to do:
         * (shall take less tham 1ms, to prevent I2C under-runs during TX)
         */
        run_second_task();
        run_minute_task();

        /*
         * Send housekeeping data over I2C bus to master
         */
        if(USI_TWI_Data_In_Receive_Buffer()) {
            uint8_t i2c_cmd, i2c_val, i2c_reply=1;
            uint8_t checksum=0;
            memset(i2c_data_out,0,I2C_DATA_OUT_SIZE);
            i2c_cmd = USI_TWI_Receive_Byte();
            switch(i2c_cmd) {
                case I2C_CMD_READADC:
                  // prepare packet, collect data:
                  DISABLE_INTERRUPTS();
                  i2c_data_out[1] = (uint8_t)(scc.adc.vpv_oc>>8);
                  i2c_data_out[2] = (uint8_t)(scc.adc.vpv_oc&0xff);
                  i2c_data_out[11] = scc.eng.temperature;//ocr1d;
                  //i2c_data_out[12] = (uint8_t)(Vbatt_float>>8);
                  //i2c_data_out[13] = (uint8_t)(Vbatt_float&0xff);
                  i2c_data_out[12] = (uint8_t)(scc.mppt.setpoint_vpv>>8);
                  i2c_data_out[13] = (uint8_t)(scc.mppt.setpoint_vpv&0xff);
                  ENABLE_INTERRUPTS();

                  i2c_data_out[3] = (uint8_t)(scc.eng.Vsolar>>8);
                  i2c_data_out[4] = (uint8_t)(scc.eng.Vsolar&0xff);
                  i2c_data_out[5] = (uint8_t)(scc.eng.Isense_in>>8);
                  i2c_data_out[6] = (uint8_t)(scc.eng.Isense_in&0xff);
                  i2c_data_out[7] = (uint8_t)(scc.eng.Vbatt>>8);
                  i2c_data_out[8] = (uint8_t)(scc.eng.Vbatt&0xff);
                  i2c_data_out[9] = (uint8_t)(scc.eng.Isense_out>>8);
                  i2c_data_out[10] = (uint8_t)(scc.eng.Isense_out&0xff);
                  i2c_data_out[14] = FW_VERSION; // mcusr_mirror; // reset cause
                  break;
                case I2C_CMD_STAT_WDT:
                  i2c_val = USI_TWI_Receive_Byte(); // CMD load=0(off) or load=1(on)
                  scc.state.master_watchdog=0; // reset watchdog
                  // in case of watchdog expired: switch the load to previous state
                  if((i2c_val&1) && (scc.state.battery_empty_protect==0) && (scc.state.overload_protect==0)) {
                     LOAD_ON();
                     scc.state.loadstate|=(1<<0);
                     update_eeprom_loadstate(1);
                  } else {
                     LOAD_OFF();
                     scc.state.loadstate&=~(1<<0);
                     update_eeprom_loadstate(0);
                  }
                case I2C_CMD_READSTAT:
                  // prepare packet, collect data:
                  DISABLE_INTERRUPTS();
                  i2c_data_out[1] = (uint8_t)(scc.t.day_meter>>8);
                  i2c_data_out[2] = (uint8_t)(scc.t.day_meter&0xff);
                  //i2c_data_out[1] = (uint8_t)(scc.t.daylight_meter>>8);
                  //i2c_data_out[2] = (uint8_t)(scc.t.daylight_meter&0xff);
                  i2c_data_out[3] = (uint8_t)(scc.t.minute_meter>>8);
                  i2c_data_out[4] = (uint8_t)(scc.t.minute_meter&0xff);
                  if(scc.t.is_daytime > 0) {
                      i2c_data_out[12] = 2;
                  } else {
                      i2c_data_out[12] = 0;
                  }
                  i2c_data_out[12] |= (scc.t.day_am_pm&1);
                  ENABLE_INTERRUPTS();
                  i2c_data_out[5] = scc.state.sysstate;
                  i2c_data_out[6] = tmp_battstate;
                  i2c_data_out[7] = tmp_solarstate;
                  i2c_data_out[8] = scc.state.loadstate;
                  i2c_data_out[9] = filter_soc_output();
                  i2c_data_out[10] = scc.mppt.mppt_indicator;
                  i2c_data_out[11] = scc.state.doorswitch;

                  i2c_data_out[13] = (uint8_t)(scc.t.ydaylight_meter>>8);
                  i2c_data_out[14] = (uint8_t)(scc.t.ydaylight_meter&0xff);
                  break;
                case I2C_CMD_READBATT:
                  // prepare packet, collect data:
                  i2c_data_out[1] = (uint8_t)(scc.eng.As>>24);
                  i2c_data_out[2] = (uint8_t)(scc.eng.As>>16);
                  i2c_data_out[3] = (uint8_t)(scc.eng.As>>8);
                  i2c_data_out[4] = (uint8_t)(scc.eng.As&0xff);

                  i2c_data_out[5] = (uint8_t)(scc.eng.As_max>>24);
                  i2c_data_out[6] = (uint8_t)(scc.eng.As_max>>16);
                  i2c_data_out[7] = (uint8_t)(scc.eng.As_max>>8);
                  i2c_data_out[8] = (uint8_t)(scc.eng.As_max&0xff);
                  i2c_data_out[9] = scc.task.equalizing;
                  break;
                case I2C_CMD_CALTEMP:
                  i2c_val = USI_TWI_Receive_Byte();
                  DISABLE_INTERRUPTS();
                  scc.eng.temperature_offset = (int8_t)(scc.adc.adc_temp-273-i2c_val); 
                  eeprom_update_byte(EEPROM_ADDR_tempoffset,scc.eng.temperature_offset);
                  ENABLE_INTERRUPTS();
                  break;
                case I2C_CMD_CALBATT:
                  i2c_val = USI_TWI_Receive_Byte();
                  DISABLE_INTERRUPTS();
                  scc.eng.batt_As_offset = (int8_t)i2c_val; 
                  eeprom_update_byte(EEPROM_ADDR_batt_As_offset,scc.eng.batt_As_offset);
                  ENABLE_INTERRUPTS();
                  break;
                case I2C_CMD_SETAHBATT:
                  i2c_val = USI_TWI_Receive_Byte();
                  scc.eng.As_max=(int32_t)i2c_val;
                  scc.eng.As_max *= 3600L;
                  scc.eng.As_max *= 100L;
                  scc.eng.As=scc.eng.As_max;
                  DISABLE_INTERRUPTS();
                  eeprom_update_dword(EEPROM_ADDR_As_max,scc.eng.As_max);
                  ENABLE_INTERRUPTS();
                  break;
#if (LITHIUM_3S==0)
                // For Lead-Acid only
                case I2C_CMD_EQUALIZE:
                  scc.task.equalizing=3; // hours
                  break;
#endif
                case 0:
                  USI_TWI_Slave_Initialise();
                  i2c_reply=0;
                  break;
                default:
                  break;
            }
            if(i2c_reply) {
                i2c_data_out[0] = i2c_cmd;
                for(j=0;j<I2C_DATA_OUT_SIZE;j++) { 
                    checksum += i2c_data_out[j];
                }
                i2c_data_out[15] = checksum;
                for(j=0;j<I2C_DATA_OUT_SIZE;j++) {
                    if(USI_TWI_Transmit_Byte(i2c_data_out[j]) != 0) {
                        USI_TWI_Slave_Initialise();
                        break;
                    }
                }
            }
        }
    }
    return 0;
}

uint8_t timed_out(void)
{
    uint8_t sreg;
    DISABLE_INTERRUPTS();
    if(scc.t.uptime >= scc.t.systimeout) {
        ENABLE_INTERRUPTS();
        return 1;
    } else {
        ENABLE_INTERRUPTS();
        return 0;
    }
}

void run_second_task(void)
{
    uint8_t sreg;
    uint8_t tasksec1;
    uint8_t tasksec2;
    uint32_t v;

    DISABLE_INTERRUPTS();
    tasksec1=scc.task.second1_task;
    tasksec2=scc.task.second2_task;
    scc.task.second1_task=0;
    scc.task.second2_task=0;
    ENABLE_INTERRUPTS();

    if(tasksec1) {
        //LED_TOGGLE(LED_RED);
        DISABLE_INTERRUPTS();
        v = (uint32_t)scc.adc.adc_vbatt;
        ENABLE_INTERRUPTS();
        v *= 105600UL; // 1650*64
        v = v >> 16;   // take out byte 2 and 3
        scc.eng.Vbatt = (uint16_t)v;

        DISABLE_INTERRUPTS();
        v = (uint32_t)scc.adc.adc_vsolar;
        ENABLE_INTERRUPTS();
        v *= 848320UL; // 13255*64
        v = v >> 16;
        scc.eng.Vsolar = (uint16_t)v;
    }
    if(tasksec2) {
        //LED_TOGGLE(LED_GREEN);
        DISABLE_INTERRUPTS();
        v = (uint32_t)scc.adc.adc_isense_in;
        ENABLE_INTERRUPTS();
        v *= 211200UL; // 3300*64
        v = v >> 16;
        scc.eng.Isense_in = (uint16_t)v;

        DISABLE_INTERRUPTS();
        v = (uint32_t)scc.adc.adc_isense_out;
        ENABLE_INTERRUPTS();
        v *= 211200UL; // 3300*64
        v = v >> 16;
        scc.eng.Isense_out = (uint16_t)v;

        scc.eng.As += (int32_t)scc.eng.Isense_in;
        scc.eng.As -= (int32_t)scc.eng.Isense_out;
        scc.eng.As -= scc.eng.batt_As_offset;

        if(scc.eng.As < 0) { scc.eng.As=0L; }
        if(scc.eng.As > scc.eng.As_max) { scc.eng.As=scc.eng.As_max; }

        if(scc.state.battstate == BATT_EMPTY) {
            scc.eng.As_max -= scc.eng.As;
            if(scc.eng.As_max < 0) scc.eng.As_max=0L;
            scc.eng.As = 0L;
        }
    }
}

void run_minute_task(void)
{
    uint8_t sreg;
    uint8_t task;
    uint16_t ydaylight_meter_div2;

    DISABLE_INTERRUPTS();
    task=scc.task.minute_task;
    scc.task.minute_task=0;
    ENABLE_INTERRUPTS();

    if(task) {
        DISABLE_INTERRUPTS();
        if(scc.t.minute_meter >= 1440) { scc.t.minute_meter=0; scc.t.day_meter++; }

        if(scc.state.solarstate == SOLAR_OK) {
            if(scc.t.is_daytime < TWILIGHT_TIME) scc.t.is_daytime++;
        } else {
            if(scc.t.is_daytime > 0) scc.t.is_daytime--;
        }
        ENABLE_INTERRUPTS();

        scc.task.minute60_task++;
        if(scc.task.minute60_task>=60) {
            scc.task.minute60_task=0;

            if(scc.task.equalizing > 1) { scc.task.equalizing--; }

            DISABLE_INTERRUPTS();
            eeprom_update_word(EEPROM_ADDR_minute_meter,scc.t.minute_meter);
            eeprom_update_word(EEPROM_ADDR_day_meter,scc.t.day_meter);
            ENABLE_INTERRUPTS();

            DISABLE_INTERRUPTS();
            ydaylight_meter_div2 = scc.t.ydaylight_meter>>1;  // div2 to find 12:00 time
            if(scc.t.daylight_meter > ydaylight_meter_div2) { // 12:00 or later ?
                scc.t.day_am_pm=1; // PM 

                // start a 2 hour equalizing run weekly: 
                if(((scc.t.day_meter%7UL)==0) && (scc.task.equalizing == 0)) {
                    scc.task.equalizing=3;
                } else {
                    scc.task.equalizing=0;
                }
            } else {
                scc.t.day_am_pm=0; // AM
            }
            ENABLE_INTERRUPTS();

            DISABLE_INTERRUPTS();
            eeprom_update_dword(EEPROM_ADDR_As_max,scc.eng.As_max);
            eeprom_update_dword(EEPROM_ADDR_As,scc.eng.As);
            ENABLE_INTERRUPTS();
        }
    }
}


/*
 * Filter to smoothen the SoC value:
 */
void filter_soc_input(uint8_t val)
{
    uint8_t sreg;
    DISABLE_INTERRUPTS();
    if(scc.t.uptime < 20) { // initialize
        ENABLE_INTERRUPTS();
        memset(filter_soc,val,FILTER_SOC_SIZE);
    } else {
        ENABLE_INTERRUPTS();
    }
    // write into ringbuffer indexed by filter_soc_idx:
    if(val > 100) val=100;
    filter_soc[filter_soc_idx] = val;
    filter_soc_idx++;
    filter_soc_idx &= (FILTER_SOC_SIZE-1); // wrap around
}

uint8_t filter_soc_output(void)
{
    uint16_t val=0;
    uint8_t i;
    for(i=0;i<FILTER_SOC_SIZE;i++) {
        val+=(uint16_t)filter_soc[i];
    }
    val = val/FILTER_SOC_SIZE;
    return (uint8_t)val;
}

//********** USI_TWI I2C functions **********//
void Flush_TWI_TX_Buffers(void)
{
    TWI_TxTail = 0;
    TWI_TxHead = 0;
}
void Flush_TWI_RX_Buffers(void)
{
    TWI_RxTail = 0;
    TWI_RxHead = 0;
}

void USI_TWI_Slave_Initialise( void )
{
    // check dipswitches to set TWI_slaveAddress
    TWI_slaveAddress = 0x2c; 
    if(!(PINB & (1<<PINB1))) { TWI_slaveAddress |= 0x01; }
    if(!(PINB & (1<<PINB3))) { TWI_slaveAddress |= 0x02; }

    USI_TWI_Overflow_State=USI_SLAVE_CHECK_ADDRESS;
    Flush_TWI_TX_Buffers();
    Flush_TWI_RX_Buffers();
    TWI_usi_timeout=0;

    // --- USE THIS register to select which port USI uses on the ATtiny261: ---
    USIPP &= ~(1<<USIPOS); // use PB2:PB0 for USI on the ATtiny261 !!

    // In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will pull SCL
    // low when a start condition is detected or a counter overflow (only
    // for USIWM1, USIWM0 = 11).  This inserts a wait state.  SCL is released
    // by the ISRs (USI_START_vect and USI_OVERFLOW_vect).

    // Set SCL and SDA as output
    DDR_USI |= ( 1 << PORT_USI_SCL ) | ( 1 << PORT_USI_SDA );

    // set SCL high
    PORT_USI |= ( 1 << PORT_USI_SCL );

    // set SDA high
    PORT_USI |= ( 1 << PORT_USI_SDA );

    // Set SDA as input
    DDR_USI &= ~( 1 << PORT_USI_SDA );
    USICR =
       // enable Start Condition Interrupt
       ( 1 << USISIE ) |
       // disable Overflow Interrupt
       ( 0 << USIOIE ) |
       // set USI in Two-wire mode, no USI Counter overflow hold
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
       // Shift Register Clock Source = external, positive edge
       // 4-Bit Counter Source = external, both edges
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
       // no toggle clock-port pin
       ( 0 << USITC );

    // clear all interrupt flags and reset overflow counter
    USISR = ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC );
}

int8_t USI_TWI_Transmit_Byte( uint8_t data )
{
    uint8_t sreg;
    uint8_t tmphead,to;
    DISABLE_INTERRUPTS();
    TWI_usi_timeout=3;
    tmphead = (TWI_TxHead + 1) & TWI_TX_BUFFER_MASK;  // Calculate buffer index.
    ENABLE_INTERRUPTS();
    while (tmphead == TWI_TxTail) { // Wait for free space in buffer.
        to=TWI_usi_timeout;
        if(to==0) { 
            return -1;
        }
    }
    TWI_TxBuf[tmphead] = data;                        // Store data in buffer.
    TWI_TxHead = tmphead;                             // Store new index.
    return 0;
}

uint8_t USI_TWI_Receive_Byte( void )
{
    uint8_t sreg;
    uint8_t tmptail;
    uint8_t tmpRxTail;
    uint8_t to;
    DISABLE_INTERRUPTS();
    tmpRxTail = TWI_RxTail;
    TWI_usi_timeout=3;
    ENABLE_INTERRUPTS();
    while (TWI_RxHead == tmpRxTail) {
        to = TWI_usi_timeout;
        if(to==0) {
            return 0;
        }
    }
    tmptail = (TWI_RxTail + 1) & TWI_RX_BUFFER_MASK;  // Calculate buffer index
    TWI_RxTail = tmptail;                             // Store new index
    return TWI_RxBuf[tmptail];                        // Return data from the buffer.
}

uint8_t USI_TWI_Data_In_Receive_Buffer( void )
{
    uint8_t tmpRxTail;                 // Temporary variable to store volatile
    tmpRxTail = TWI_RxTail;
    return (TWI_RxHead != tmpRxTail);  // Return 0 (FALSE) if the receive buffer is empty.
}

/*
 * Interrupt service routines:
 */
ISR(USI_START_vect)
{
    // Set default starting conditions for new TWI package
    USI_TWI_Overflow_State = USI_SLAVE_CHECK_ADDRESS;
    DDR_USI  &= ~(1<<PORT_USI_SDA); // Set SDA as input

    // worst case timeout catched by watchdog reset
    while ( // wait for condition
        // SCL is high
        (PIN_USI & (1<<PIN_USI_SCL)) &&
        //!(PIN_USI & (1<<PIN_USI_SCL)) &&
        // and SDA is low
        !((PIN_USI & (1<<PIN_USI_SDA))) //&&
        //usi_to>0
    );// usi_to--;

    if (!(PIN_USI & (1<<PIN_USI_SDA))) {
        // a Stop Condition did not occur
        USICR =
            // keep Start Condition Interrupt enabled to detect RESTART
            ( 1 << USISIE ) |
            // enable Overflow Interrupt
            ( 1 << USIOIE ) |
            // set USI in Two-wire mode, hold SCL low on USI Counter overflow
            ( 1 << USIWM1 ) | ( 1 << USIWM0 ) |
            // Shift Register Clock Source = External, positive edge
            // 4-Bit Counter Source = external, both edges
            ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
            // no toggle clock-port pin
            ( 0 << USITC );
    } else {
        // a Stop Condition did occur
        USICR =
            // enable Start Condition Interrupt
            ( 1 << USISIE ) |
            // disable Overflow Interrupt
            ( 0 << USIOIE ) |
            // set USI in Two-wire mode, no USI Counter overflow hold
            ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
            // Shift Register Clock Source = external, positive edge
            //          // 4-Bit Counter Source = external, both edges
            ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
            // no toggle clock-port pin
            ( 0 << USITC );
    } // end if

    USISR =
        // clear interrupt flags - resetting the Start Condition Flag will release SCL
        ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) |
        ( 1 << USIPF ) |( 1 << USIDC ) |
        // set USI to sample 8 bits (count 16 external SCL pin toggles)
        ( 0x0 << USICNT0);
}

ISR(USI_OVF_vect)
{
    unsigned char tmpTxTail;
    unsigned char tmpUSIDR;

    switch (USI_TWI_Overflow_State) {
        // ---------- Address mode ----------
        // Check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK, else reset USI.
        case USI_SLAVE_CHECK_ADDRESS:
            if ((USIDR == 0) || ((USIDR>>1) == TWI_slaveAddress)) {
                if ( USIDR & 0x01 ) {
                    USI_TWI_Overflow_State = USI_SLAVE_SEND_DATA;
                } else {
                    USI_TWI_Overflow_State = USI_SLAVE_REQUEST_DATA;
                }
                SET_USI_TO_SEND_ACK();
            } else {
                SET_USI_TO_TWI_START_CONDITION_MODE();
            }
            break;

        // ----- Master write data mode ------
        // Check reply and goto USI_SLAVE_SEND_DATA if OK, else reset USI.
        case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
            if ( USIDR ) { // If NACK, the master does not want more data.
                SET_USI_TO_TWI_START_CONDITION_MODE();
                return;
            }
            // From here we just drop straight into USI_SLAVE_SEND_DATA if the master sent an ACK

        // Copy data from buffer to USIDR and set USI to shift byte. Next USI_SLAVE_REQUEST_REPLY_FROM_SEN
        case USI_SLAVE_SEND_DATA:
            // Get data from Buffer       
            tmpTxTail = TWI_TxTail;
            if (TWI_TxHead != tmpTxTail) {
                TWI_TxTail = ( TWI_TxTail + 1 ) & TWI_TX_BUFFER_MASK;
                USIDR = TWI_TxBuf[TWI_TxTail];
            } else { // If the buffer is empty then:
                SET_USI_TO_TWI_START_CONDITION_MODE();
                return;
            }
            USI_TWI_Overflow_State = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
            SET_USI_TO_SEND_DATA();
            break;

        // Set USI to sample reply from master. Next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
        case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
            USI_TWI_Overflow_State = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
            SET_USI_TO_READ_ACK();
            break;

        // ----- Master read data mode ------
        // Set USI to sample data from master. Next USI_SLAVE_GET_DATA_AND_SEND_ACK.
        case USI_SLAVE_REQUEST_DATA:
            USI_TWI_Overflow_State = USI_SLAVE_GET_DATA_AND_SEND_ACK;
            SET_USI_TO_READ_DATA();
            break;

        // Copy data from USIDR and send ACK. Next USI_SLAVE_REQUEST_DATA
        case USI_SLAVE_GET_DATA_AND_SEND_ACK:
            // Put data into Buffer
           tmpUSIDR = USIDR;
           TWI_RxHead = ( TWI_RxHead + 1 ) & TWI_RX_BUFFER_MASK;
           TWI_RxBuf[TWI_RxHead] = tmpUSIDR;

           USI_TWI_Overflow_State = USI_SLAVE_REQUEST_DATA;
           SET_USI_TO_SEND_ACK();
           break;
    }
}

void update_eeprom_loadstate(uint8_t state)
{
    uint8_t j;
    uint8_t sreg;

    DISABLE_INTERRUPTS();
    j = eeprom_read_byte(EEPROM_ADDR_load_onoff);
    ENABLE_INTERRUPTS();

    if(j != state) {
        DISABLE_INTERRUPTS();
        eeprom_update_byte(EEPROM_ADDR_load_onoff,state);
        ENABLE_INTERRUPTS();
    }
}

