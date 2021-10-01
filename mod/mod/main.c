/*
 * I2C: tested with SCL clock freq 15kHz
 */


#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#define F_OSC F_CPU


// Set fuses: 
FUSES =
{
   .high = 0xff,
   .low  = 0x7a,
};

#ifndef MOD_PV
#  define MOD_PV 0 
#endif
#ifndef MOD_DC
#  define MOD_DC 1 
#endif
#ifndef MOD_AC
#  define MOD_AC 0 
#endif

// MOD PV I2C: 0x10..0x13
// MOD DC I2C: 0x14..0x17
// MOD AC I2C: 0x18..0x1B

#if (MOD_AC==1)
#  warning "Compiling for MOD_AC"
#  define I2C_ADDR 0x18
#elif (MOD_DC==1)
#  warning "Compiling for MOD_DC"
#  define I2C_ADDR 0x14
#elif (MOD_PV==1)
#  warning "Compiling for MOD_PV"
#  define I2C_ADDR 0x10
#endif

#define I2C_slaveAddress (I2C_ADDR<<1) 



// PORTB pins:
#define ADC_I PORTB4
#if (MOD_DC==0)
#  define ADC_V PORTB3
#endif

#if (MOD_PV==1)
#  define ADC_VB PORTB2
#elif (MOD_DC==1)
#  define RELAY PORTB3
#else
#  define RELAY PORTB0
#endif


#define PORT_SDA PORTB1
#define PIN_SDA  PINB1


#if (MOD_PV==1)
#  define PORT_SCL PORTB0
#  define PIN_SCL  PINB0 
#else
#  define PORT_SCL PORTB2
#  define PIN_SCL  PINB2 
#endif


#define SDA_high() { PORTB |=  (1<<PORT_SDA); DDRB &= ~(1<<PORTB1); } // allow pullups
#define SDA_low()  { PORTB &= ~(1<<PORT_SDA); DDRB |=  (1<<PORTB1); } // drive to 0


// Small delay to create overlap time around SCL clock pulses
static inline void overlap_time(uint16_t count)
{
    asm volatile (  "cp  %A0,__zero_reg__ \n\t"  \
                 "cpc %B0,__zero_reg__ \n\t"  \
                 "breq L_Exit_%=       \n\t"  \
                 "L_LOOP_%=:           \n\t"  \
                 "sbiw %0,1            \n\t"  \
                 "brne L_LOOP_%=       \n\t"  \
                 "L_Exit_%=:           \n\t"  \
                 : "=w" (count)
                                     : "0"  (count)
               );
}


#define I2C_WAIT_FOR_START 0
#define I2C_RECEIVE_ADDR   1
#define I2C_RECEIVE_DATA   2
#define I2C_SEND_DATA      3

#define I2C_CMD_FREEZE_VOLT_CURR   0x20
#define I2C_CMD_READ_ADC_CURR_LSB  0x30
#define I2C_CMD_READ_ADC_CURR_MSB  0x31
#define I2C_CMD_READ_ADC_VOLT_LSB  0x40
#define I2C_CMD_READ_ADC_VOLT_MSB  0x41
#define I2C_CMD_READ_ADC_VOLTB_LSB 0x42
#define I2C_CMD_READ_ADC_VOLTB_MSB 0x43
#define I2C_CMD_SET_RELAY_OFF      0x50
#define I2C_CMD_SET_RELAY_ON       0x51


volatile uint8_t I2C_state = I2C_WAIT_FOR_START;
volatile uint8_t I2C_response_data = 0;

volatile uint16_t adc_volt  = 0;
volatile uint16_t adc_voltB = 0;
volatile uint16_t adc_curr  = 0;

volatile uint8_t adc_curr_lsb  = 0;
volatile uint8_t adc_curr_msb  = 0;
volatile uint8_t adc_volt_lsb  = 0;
volatile uint8_t adc_volt_msb  = 0;
volatile uint8_t adc_voltB_lsb = 0;
volatile uint8_t adc_voltB_msb = 0;



/* ADC defenitions */
#define cStartAdc    ((1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)) // fADC=1MHz/128=7.8kHz
#define cRestartAdc  ((cStartAdc)|(1<<ADSC))
#define cAdcV        (1<<REFS0) 


#define ADC_curr 2

#if (MOD_DC==0)
#  define ADC_volt 3
#endif

#if (MOD_PV==1)
#  define ADC_voltB 1
#endif

ISR(ADC_vect)
{
    uint8_t muxtmp;
    uint8_t lsb,msb;
    uint16_t sample;

    muxtmp = ADMUX;
    muxtmp &= 0x3;

    lsb = ADCL; // read the low part
    msb = ADCH; // read the high part

    sample = (uint16_t)lsb;
    sample |= ((uint16_t)msb)<<8;

#if (MOD_PV==1)
    if(muxtmp == ADC_voltB) {
        adc_voltB = sample;
        ADMUX = cAdcV | ADC_curr;
    } else
#endif
    if(muxtmp == ADC_curr) {
        adc_curr = sample;
#if (MOD_DC==1)
        ADMUX = cAdcV | ADC_curr;
#else
        ADMUX = cAdcV | ADC_volt;

    } else if(muxtmp == ADC_volt) {
        adc_volt = sample;
#endif
#if (MOD_PV==1)
        ADMUX = cAdcV | ADC_voltB;
#else
        ADMUX = cAdcV | ADC_curr;
#endif
    }
    ADCSRA = cRestartAdc; // Restart the ADC again   
}

static void adc_to_A(void)
{
    uint16_t tmp=adc_curr;
#if (MOD_AC==1)
    tmp=tmp*40; // --> mA
#elif (MOD_DC==1)
    tmp=tmp*27; // --> mA
#elif (MOD_PV==1)
    tmp=tmp*30; // --> mA
#endif
    adc_curr_lsb=(uint8_t)(tmp&0xff);
    adc_curr_msb=(uint8_t)((tmp>>8)&0xff);
}

static void adc_to_V(void)
{
    uint32_t tmp=adc_volt;
#if (MOD_AC==1)
    tmp=tmp*39; // --> cV
#elif (MOD_DC==1)
#elif (MOD_PV==1)
    tmp=tmp*6; // --> cV
#endif
    adc_volt_lsb=(uint8_t)(tmp&0xff);
    adc_volt_msb=(uint8_t)((tmp>>8)&0xff);
}

static void adc_to_V2(void)
{
    uint16_t tmp=adc_voltB;
#if (MOD_AC==1)
#elif (MOD_DC==1)
#elif (MOD_PV==1)
    tmp=(tmp)+(tmp>>1); // *1.5 // --> cV
#endif
    adc_voltB_lsb=(uint8_t)(tmp&0xff);
    adc_voltB_msb=(uint8_t)((tmp>>8)&0xff);
}

static void relay_onoff(uint8_t onoff)
{
#if (MOD_PV==0)
    if(onoff) {
        PORTB |= (1<<RELAY);
    } else {
        PORTB &= ~(1<<RELAY);
    }
#endif
}

static uint8_t process_i2c_command(uint8_t cmd)
{
  uint8_t ret=1;
  I2C_response_data = 0;

  switch(cmd) {
    /*
     * write commands
     */
    case I2C_CMD_SET_RELAY_OFF:
      relay_onoff(0);
      break;
    case I2C_CMD_SET_RELAY_ON:
      relay_onoff(1);
      break;
    case I2C_CMD_FREEZE_VOLT_CURR:
      adc_to_A();
      adc_to_V();
      adc_to_V2();
      break;
    /*
     * read commands
     */
    case I2C_CMD_READ_ADC_CURR_LSB:
      I2C_response_data = adc_curr_lsb;
      break;
    case I2C_CMD_READ_ADC_CURR_MSB:
      I2C_response_data = adc_curr_msb;
      break;
    case I2C_CMD_READ_ADC_VOLT_LSB:
      I2C_response_data = adc_volt_lsb;
      break;
    case I2C_CMD_READ_ADC_VOLT_MSB:
      I2C_response_data = adc_volt_msb;
      break;
    case I2C_CMD_READ_ADC_VOLTB_LSB:
      I2C_response_data = adc_voltB_lsb;
      break;
    case I2C_CMD_READ_ADC_VOLTB_MSB:
      I2C_response_data = adc_voltB_msb;
      break;
    default:
      ret=0; // unsupported command
      break;
  }
  return ret;
}

static void write_I2C_byte(uint8_t data)
{
  uint8_t i,d=data;
  for (i=0; i<8; i++) {
      if(d&0x80) { SDA_high(); } else { SDA_low(); } // MSB first
      loop_until_bit_is_set(PINB, PIN_SCL);
      loop_until_bit_is_clear(PINB, PIN_SCL);
      d=(d<<1);
  }
  // accept ACK or NACK
  SDA_high();
  loop_until_bit_is_set(PINB, PIN_SCL);
  loop_until_bit_is_clear(PINB, PIN_SCL);

  I2C_state = I2C_WAIT_FOR_START;
}

static uint8_t read_I2C_byte(uint8_t match_slaveaddr)
{
  uint8_t i;
  uint8_t data = 0;

  for (i=0; i<8; i++) {
      overlap_time(15);
      loop_until_bit_is_set(PINB, PIN_SCL);
      overlap_time(15);

      data = (data << 1);
      if(bit_is_set(PINB,PIN_SDA)) { data |= 1; } 

      while (bit_is_set(PINB,PIN_SCL)) {
          // If SDA changes while SCL is high, it's a
          // stop (low to high) or start (high to low) condition.
          if( ((data&1)==1) && bit_is_clear(PINB,PIN_SDA)) { 
              I2C_state = I2C_RECEIVE_ADDR;
              return 0; //I2C_START_DETECTED;
          }
          if( ((data&1)==0) && bit_is_set(PINB,PIN_SDA)) { 
              I2C_state = I2C_WAIT_FOR_START;
              return 0; //I2C_STOP_DETECTED;
          }
      }
  }
  // Send ACK -- SCL is low now
  if (match_slaveaddr) {
      if((data & 0xfe) == I2C_slaveAddress) {
          // send ACK
          SDA_low();
          loop_until_bit_is_set(PINB, PIN_SCL);
          loop_until_bit_is_clear(PINB, PIN_SCL);
          overlap_time(15);
          SDA_high();

          if (data & 1) {
              I2C_state = I2C_SEND_DATA;
          } else {
              I2C_state = I2C_RECEIVE_DATA;
          }
      } else {
          I2C_state = I2C_WAIT_FOR_START;
          return 0; //(I2C_ADDR_MISMATCH);
      }
  } else { // match command

      // accept command anyways. Best is to do process_i2c_command() first and use
      // SCL clock scretching

      // send ACK
      SDA_low();
      loop_until_bit_is_set(PINB, PIN_SCL);
      loop_until_bit_is_clear(PINB, PIN_SCL);
      overlap_time(15);
      SDA_high();

      process_i2c_command(data);
      I2C_state = I2C_WAIT_FOR_START;
  }
  return data;
}


// pin change interrupt on SDA line
ISR(PCINT0_vect)
{
  if (bit_is_clear(PINB,PIN_SDA) && bit_is_set(PINB,PIN_SCL)) {
      GIMSK &= ~(1<<PCIE); // disable pin change interrupt for SDA
      loop_until_bit_is_clear(PINB, PIN_SCL);
      I2C_state = I2C_RECEIVE_ADDR;

      while(I2C_state != I2C_WAIT_FOR_START) {
          switch(I2C_state) {
            case I2C_RECEIVE_ADDR:
              read_I2C_byte(1); // read address, 1=addr match
              break;
            case I2C_RECEIVE_DATA:
              read_I2C_byte(0);
              break;
            case I2C_SEND_DATA:
              write_I2C_byte(I2C_response_data);
              break;
            default: 
              I2C_state = I2C_WAIT_FOR_START;
              break;
          }
      }
      GIMSK |= (1<<PCIE);   // enable pin change interrupt for SDA
  } 
}

int main(void) 
{
    PORTB |= ((1<<PORT_SCL)|(1<<PORT_SDA)); // pullups on
#if (MOD_PV==0)
    DDRB  |= (1<<RELAY); // output for relay, I2C pullup
#endif

    PCMSK |= (1<<PCINT1); // enable PCINT1 interrupt on pin change (SDA line)
    GIMSK |= (1<<PCIE);   // enable pin change interrupt

/*
    adc_curr = 0;
    adc_volt = 0;
    adc_voltB = 0;
    adc_curr_lsb = 0;
    adc_curr_msb = 0;
    adc_volt_lsb = 0;
    adc_volt_msb = 0;
    adc_voltB_lsb = 0;
    adc_voltB_msb = 0;
    I2C_response_data = 0;
*/

    // Init ADC: 
#if (MOD_PV==1)
    DIDR0 |= (1<<ADC1D)|(1<<ADC2D)|(1<<ADC3D); // disable digital input buffer on ADC inputs to safe power
#elif (MOD_DC==1)
    DIDR0 |= (1<<ADC2D); // disable digital input buffer on ADC inputs to safe power
#else
    DIDR0 |= (1<<ADC2D)|(1<<ADC3D); // disable digital input buffer on ADC inputs to safe power
#endif
    ADMUX = cAdcV;
    ADMUX |= ADC_curr;
    ADCSRA = cRestartAdc; // Restart the ADC again  

    ACSR |= (1<<ACD);  // Turn off the analog comperator to safe power 

    //I2C_state = I2C_WAIT_FOR_START;

    sei(); // enable interrupts

    while(1) {
        asm volatile ("nop");
    }
}

