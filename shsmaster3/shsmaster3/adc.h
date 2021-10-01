// Blake Felt
// ATSAMD21_ADC.h
// Adds some extra functions to the ADC, such as
// 16 bit and differential mode
//
// https://github.com/Molorius/ATSAMD21-ADC

#ifndef ATSAMD21_ADC_H
#define ATSAMD21_ADC_H

#ifndef ADC_CTRLB_RESSEL_12BIT_Val 
#define ADC_CTRLB_RESSEL_8BIT_Val   0x03
#define ADC_CTRLB_RESSEL_10BIT_Val  0x02 // default by Arduino
#define ADC_CTRLB_RESSEL_12BIT_Val  0x00
#endif
//#define ADC_CTRLB_RESSEL_16BIT_Val  0x01 // used for averaging mode output

#define ADC_PIN_TEMP                0x18 // positive mux, pg 870
#define ADC_PIN_BANDGAP             0x19
#define ADC_PIN_SCALEDCOREVCC       0x1A
#define ADC_PIN_SCALEDIOVCC         0x1B
#define ADC_PIN_DAC                 0x1C

#define ADC_PIN_GND                 0x18 // negative mux, pg 869
#define ADC_PIN_IOGND               0x19

#define ADC_GAIN_1                  0x00 // pg 868
#define ADC_GAIN_2                  0x01
#define ADC_GAIN_4                  0x02
#define ADC_GAIN_8                  0x03
#define ADC_GAIN_16                 0x04
#define ADC_GAIN1_DIV2              0x0F // default by Arduino

#define ADC_REF_INT1V               0x00 // 1.0V reference, pg 861
#define ADC_REF_INTVCC0             0x01 // 1/1.48 VDDANA
#define ADC_REF_INTVCC1             0x02 // 1/2 VDDANA (only for VDDANA > 2.0V) // default
#define ADC_REF_VREFA               0x03 // external reference
#define ADC_REF_VREFB               0x04 // external reference

#define ADC_PRESCALER_DIV4          0x00 // pg 864
#define ADC_PRESCALER_DIV8          0x01
#define ADC_PRESCALER_DIV16         0x02
#define ADC_PRESCALER_DIV32         0x03
#define ADC_PRESCALER_DIV64         0x04
#define ADC_PRESCALER_DIV128        0x05
#define ADC_PRESCALER_DIV256        0x06
#define ADC_PRESCALER_DIV512        0x07 // Arduino default


uint8_t analogReadExtended(uint8_t bits);

// same as the above function, but no error checking, no pin types are changed, and the positive and negative
// inputs are the raw values being input. The DAC is not automatically shut off either. See datasheet page
int16_t analogDifferentialRaw(uint8_t mux_pos,uint8_t mux_neg); 

// sets the gain of the ADC. See page 868. All values defined above. 
void analogGain(uint8_t gain); 

// set the analog reference voltage, but with all available options
// (the Arduino IDE neglects some). The Arduino IDE also changes
// the gain when analogReference() is used, but this won't. pg 861
void analogReference2(uint8_t ref); 

// increases accuracy of gain stage by enabling the reference buffer
// offset compensation. Takes longer to start. pg 861
void analogReferenceCompensation(uint8_t val); 

// sets the ADC clock relative to the peripheral clock. pg 864
void analogPrescaler(uint8_t val); 

// resets the ADC. pg 860
// note that this doesn't put back the default values set by the 
// Arduino IDE. 
void analogReset(); 
void adc_init(void);

#endif // ifndef ATSAMD21_ADC_H
