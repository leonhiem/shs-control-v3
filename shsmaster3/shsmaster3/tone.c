/*
 * Derived from Arduino Tone class
 * Modified from C++ to C    by LeonH
 */

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "hal_gpio.h"
#include "pitches.h"
#include "utils.h"
#include "tone.h"

//-----------------------------------------------------------------------------

/*
// notes in the melody:
#define LEN_melody_start 5
Melody melody_start[] = {
	{NOTE_A4,8}, {NOTE_B4,8}, {NOTE_C5,4}, {NOTE_D5,4}, {NOTE_E5,4}
};
*/

#define LEN_melody_success 7
Melody melody_success[] = {
  {NOTE_G5,8}, {NOTE_G5,8}, {NOTE_A5,4}, {NOTE_G5,4}, {0,4}, {NOTE_B5,4}, {NOTE_C6,4}
};

#define LEN_melody_error 2
Melody melody_error[] = {
  {NOTE_C6,8}, {NOTE_A4,2}
};

#define LEN_melody_expired 8
Melody melody_expired[] = {
	{NOTE_G5,4}, {NOTE_F5,4}, {NOTE_E5,4}, {NOTE_D5,4}, {NOTE_C5,4}, {NOTE_B4,4}, {NOTE_A4,2}, {NOTE_G4,1}
};

volatile int64_t tone_toggleCount;
volatile bool toneIsActive = false;


#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);
void resetTC2(void)
{
	// Disable TCx
	TC2->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	WAIT_TC16_REGS_SYNC(TC2);
	// Reset TCx
	TC2->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	WAIT_TC16_REGS_SYNC(TC2);
	while (TC2->COUNT16.CTRLA.bit.SWRST);
}

void noTone(void)
{
	resetTC2();
	HAL_GPIO_PIEZO_PIN_clr();
	toneIsActive = false;
}

void tone (uint32_t frequency, uint32_t duration)
{
	uint32_t prescalerConfigBits;
	uint32_t ccValue;
	uint32_t toneMaxFrequency = F_CPU / 2;
	
	if(frequency<50) return;
	
	NVIC_DisableIRQ(TC2_IRQn);
	NVIC_ClearPendingIRQ(TC2_IRQn);
	
	if (toneIsActive) noTone();

	ccValue = toneMaxFrequency / frequency - 1;
	prescalerConfigBits = TC_CTRLA_PRESCALER_DIV1;
	uint8_t i = 0;
	
	while(ccValue > 0xffff) {
		ccValue = toneMaxFrequency / frequency / (2<<i) - 1;
		i++;
		if(i == 4 || i == 6 || i == 8) //DIV32 DIV128 and DIV512 are not available
		i++;
	}	
	switch(i-1)	{
		case 0: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV2; break;		
		case 1: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV4; break;		
		case 2: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV8; break;		
		case 3: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV16; break;		
		case 5: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV64; break;		
		case 7: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV256; break;		
		case 9: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV1024; break;		
		default: break;
	}
    tone_toggleCount = (duration > 0 ? frequency * duration * 2 / 1000UL : -1);
    resetTC2();
	
	uint16_t tmpReg = 0;
	tmpReg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
	tmpReg |= TC_CTRLA_WAVEGEN_MFRQ;  // Set TONE_TC mode as match frequency
	tmpReg |= prescalerConfigBits;
	TC2->COUNT16.CTRLA.reg |= tmpReg;
	WAIT_TC16_REGS_SYNC(TC2);
	TC2->COUNT16.CC[0].reg = (uint16_t) ccValue;
	WAIT_TC16_REGS_SYNC(TC2);
	TC2->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);
    TC2->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    WAIT_TC16_REGS_SYNC(TC2);
	NVIC_EnableIRQ(TC2_IRQn);	
}

void play_nof_tones(int16_t nof_tones)
{
	for (int16_t n = 0; n < nof_tones; n++) {

		int noteDuration = 1000 / 4;
		tone(NOTE_A4, noteDuration);

		int pauseBetweenNotes = noteDuration * 1.30;
		delay_ms(pauseBetweenNotes);
		// stop the tone playing:
		noTone();
	}
}
void play_song_success(void)
{
	// iterate over the notes of the melody:
	for (int thisNote = 0; thisNote < LEN_melody_success; thisNote++) {

		// to calculate the note duration, take one second
		// divided by the note type.
		//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000 / melody_success[thisNote].duration;
		tone(melody_success[thisNote].tone, noteDuration);

		// to distinguish the notes, set a minimum time between them.
		// the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;
		delay_ms(pauseBetweenNotes);
		// stop the tone playing:
		noTone();
	}
}
void play_song_expired(void)
{
	for (int thisNote = 0; thisNote < LEN_melody_expired; thisNote++) {
		int noteDuration = 1000 / melody_expired[thisNote].duration;
		tone(melody_expired[thisNote].tone,noteDuration);

		int pauseBetweenNotes = noteDuration * 1.30;
		delay_ms(pauseBetweenNotes);
		noTone();
	}
}
void play_song_error(void)
{
	for (int thisNote = 0; thisNote < LEN_melody_error; thisNote++) {
		int noteDuration = 1000 / melody_error[thisNote].duration;
		tone(melody_error[thisNote].tone, noteDuration);

		int pauseBetweenNotes = noteDuration * 1.30;
		delay_ms(pauseBetweenNotes);
		noTone();
	}
}
/*
void play_song_start(void)
{
	for (int thisNote = 0; thisNote < LEN_melody_start; thisNote++) {
		int noteDuration = 1000 / melody_start[thisNote].duration;
		tone(melody_start[thisNote].tone, noteDuration);

		int pauseBetweenNotes = noteDuration * 1.30;
		delay_ms(pauseBetweenNotes);
		noTone();
	}
}
*/
void play_toneC(void)
{
	tone(NOTE_C6, 1000/4);
}


//void irq_handler_tc2(void){
void TC2_Handler(void){
	if (TC2->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1)) {
		if (tone_toggleCount != 0) {
			if (tone_toggleCount > 0) --tone_toggleCount;
			HAL_GPIO_PIEZO_PIN_toggle();
		} else {
			resetTC2();
			HAL_GPIO_PIEZO_PIN_clr();
			toneIsActive = false;
		}		
		TC2->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1); // clear
	}
}

void timer2_init(int8_t onoff)
{
    if(onoff == 0) {
        PM->APBCMASK.reg &= ~PM_APBCMASK_TC2;
    } else {            
	    PM->APBCMASK.reg |= PM_APBCMASK_TC2;
	    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC2_GCLK_ID) |
	    GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);
    }    
}

void tone_init(int8_t onoff)
{
    HAL_GPIO_PIEZO_PIN_out();
    HAL_GPIO_PIEZO_PIN_clr();    
    timer2_init(onoff);    
}

