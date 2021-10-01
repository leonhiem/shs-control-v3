#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "adc.h"


// Taken from Arduino IDE:
// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while(ADC->STATUS.bit.SYNCBUSY == 1);
}


uint8_t analogReadExtended(uint8_t bits) {
/*
 * Allows for adc to read 8, 10, or 12 bits normally or 13-16 bits using oversampling and decimation.
 * See pages 853 & 862
 * 8,10,12 bit = 1 sample ~ 436 microseconds
 * 13 bit = 4 samples ~ 1668 microseconds
 * 14 bit = 16 samples ~ 6595 microseconds
 * 15 bit = 64 samples ~ 26308 microseconds
 * 16 bit = 256 samples ~ 105156 microseconds
 */
  switch(bits) {
    case 8:
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x0;
      ADC->AVGCTRL.bit.SAMPLENUM = 0x0;
      return 0;
    break;

    case 10:
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x0;
      ADC->AVGCTRL.bit.SAMPLENUM = 0x0;
      return 0;
    break;
    
    case 12:
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x0;
      ADC->AVGCTRL.bit.SAMPLENUM = 0x0;
      return 0;
    break;
      
    case 13:
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x1;
      ADC->AVGCTRL.bit.SAMPLENUM = 0x2;
      return 0;
    break;
    
    case 14:
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x2;
      ADC->AVGCTRL.bit.SAMPLENUM = 0x4;
      return 0;
    break;
    
    case 15:
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x1;
      ADC->AVGCTRL.bit.SAMPLENUM = 0x6;
      return 0;
    break;
    
    case 16:
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x0;
      ADC->AVGCTRL.bit.SAMPLENUM = 0x8;
      return 0;
    break;
    
    default:
      return -1;
    break;
   }
}

// same as the above function, but no error checking, no pin types are changed, and the positive and negative
// inputs are the raw values being input. The DAC is not automatically shut off either. See datasheet page
int16_t analogDifferentialRaw(uint8_t mux_pos,uint8_t mux_neg) {
  
  uint32_t value_read = 0;

  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = mux_pos; // Selection for the positive ADC input
  ADC->INPUTCTRL.bit.MUXNEG = mux_neg; // negative ADC input

  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x01; // enable adc
  ADC->CTRLB.bit.DIFFMODE = 1; // set to differential mode

  syncADC();
  ADC->SWTRIG.bit.START = 1; // start conversion

  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; // clear the data ready flag
  syncADC();

  ADC->SWTRIG.bit.START = 1; // restart conversion, as changing inputs messes up first conversion
  
  while(ADC->INTFLAG.bit.RESRDY == 0);   // Wait for conversion to complete
  value_read = ADC->RESULT.reg; // read the value

  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x00; // disable adc
  ADC->CTRLB.bit.DIFFMODE = 0; // put back into single-ended mode
  ADC->INPUTCTRL.bit.MUXNEG = ADC_PIN_GND; // set back muxneg to internal ground
  syncADC();

  return value_read;
}

// sets the gain of the ADC. See page 868. All values defined above. 
void analogGain(uint8_t gain) {
  syncADC();
  ADC->INPUTCTRL.bit.GAIN = gain;
  syncADC();
}


// set the analog reference voltage, but with all available options
// (the Arduino IDE neglects some). The Arduino IDE also changes
// the gain when analogReference() is used, but this won't. pg 861
void analogReference2(uint8_t ref) {
  syncADC();
  ADC->REFCTRL.bit.REFSEL = ref;
  syncADC();
}

// increases accuracy of gain stage by enabling the reference buffer
// offset compensation. Takes longer to start. pg 861
void analogReferenceCompensation(uint8_t val) {
  if(val>0) val = 1; 
  syncADC();
  ADC->REFCTRL.bit.REFCOMP = val;
  syncADC();
}

// sets the ADC clock relative to the peripheral clock. pg 864
void analogPrescaler(uint8_t val) {
  syncADC();
  ADC->CTRLB.bit.PRESCALER = val;
  syncADC();
}

// resets the ADC. pg 860
// note that this doesn't put back the default values set by the 
// Arduino IDE. 
void analogReset() {
  syncADC();
  ADC->CTRLA.bit.SWRST = 1; // set reset bit
  while(ADC->CTRLA.bit.SWRST==1); // wait until it's finished
  syncADC();
}

void adc_init(void)
{
    PM->APBCMASK.reg |= PM_APBCMASK_ADC;    
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(ADC_GCLK_ID) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(2);
}