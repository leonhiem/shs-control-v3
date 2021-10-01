/*
 * Kamworks Codeswitch firmware
 *
 * Author: Leon Hiemstra <l.hiemstra@gmail.com>
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "ir_receiver.h"
#include "utils.h"
#include "hal_gpio.h"


volatile irparams_t irparams;
unsigned long myticks;

uint8_t irrecv_get_irparams_rcvstate(void)
{
    return irparams.rcvstate;
}

void irrecv_reset() {
    irparams.rcvstate = STATE_IDLE;
    irparams.rawlen = 0;    
}

// NECs have a repeat only 4 items long
long decodeNEC(decode_results *results) {
	long data = 0;
	int offset = 1; // Skip first space
	// Initial mark
	if (!MATCH_MARK(results->rawbuf[offset], NEC_HDR_MARK)) {
        //uart_print(0,"bad hdr mark\r\n");
		return IRERR;
	}
	offset++;
	// Check for repeat
	if (irparams.rawlen == 4 &&
	MATCH_SPACE(results->rawbuf[offset], NEC_RPT_SPACE) &&
	MATCH_MARK(results->rawbuf[offset+1], NEC_BIT_MARK)) {
		results->bits = 0;
		results->value = REPEAT;
		return DECODED;
	}
	if (irparams.rawlen < 2 * NEC_BITS + 4) {
        //uart_print(0,"bad nec\r\n");
		return IRERR;
	}
	// Initial space
	if (!MATCH_SPACE(results->rawbuf[offset], NEC_HDR_SPACE)) {
        //uart_print(0,"bad hdr space\r\n");
		return IRERR;
	}
	offset++;
	for (int i = 0; i < NEC_BITS; i++) {
		if (!MATCH_MARK(results->rawbuf[offset], NEC_BIT_MARK)) {
            //uart_print(0,"IRerr1=%d offs=%d\r\n",i,offset);
			return IRERR;
		}
		offset++;
		if (MATCH_SPACE(results->rawbuf[offset], NEC_ONE_SPACE)) {
			data = (data << 1) | 1;
		} else if (MATCH_SPACE(results->rawbuf[offset], NEC_ZERO_SPACE)) {
			data <<= 1;
		} else {
            //uart_print(0,"IRerr2=%d offs=%d\r\n",i,offset);
			return IRERR;
		}
		offset++;
	}
	// Success
	results->bits = NEC_BITS;
	results->value = data;
	return DECODED;
}

// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results
int irrecv_decode(decode_results *results) 
{
    long ret;
    results->rawbuf = irparams.rawbuf;
    results->rawlen = irparams.rawlen;
    if(irparams.rcvstate != STATE_STOP) return IRERR;
    ret=decodeNEC(results);
    irrecv_reset();
    if(ret) return DECODED;
    else    return IRERR;
}

uint8_t irrecv_convert(uint32_t value)
{
	char ch,valid=0;
	
	switch(value) {
		case 0x10ed00ff: // STAR Times remote controller
		case 0xFFA25D:   // Arduino RC 1 (with bubbles under. newer?)
		case 0xfd00ff:   // Arduino RC 2
		ch='1'; valid=1; break;
		
		case 0x10ed807f:
		case 0xFF629D:
		case 0xfd807f: 
		ch='2'; valid=1; break;
		
		case 0x10ed40bf:
		case 0xFFE21D:
		case 0xfd40bf: 
		ch='3'; valid=1; break;
		
		case 0x10ed20df:
		case 0xFF22DD:
		case 0xfd20df: 
		ch='4'; valid=1; break;
		
		case 0x10eda05f:
		case 0xFF02FD:
		case 0xfda05f: 
		ch='5'; valid=1; break;
		
		case 0x10ed609f:
		case 0xFFC23D:
		case 0xfd609f: 
		ch='6'; valid=1; break;
		
		case 0x10ed10ef:
		case 0xFFE01F:
		case 0xfd10ef: 
		ch='7'; valid=1; break;
		
		case 0x10ed906f:
		case 0xFFA857:
		case 0xfd906f: 
		ch='8'; valid=1; break;
		
		case 0x10ed50af:
		case 0xFF906F:
		case 0xfd50af: 
		ch='9'; valid=1; break;
		
		case 0x10ed08f7:
		case 0xFF6897:
		case 0xfd30cf: 
		ch='*'; valid=1; break;
		
		case 0x10edb04f:
		case 0xFF9867:
		case 0xfdb04f: 
		ch='0'; valid=1; break;

		case 0x10ed48b7:
		case 0xFFB04F:
		case 0xfd708f: 
		ch='#'; valid=1; break;		
		
		case 0xFF18E7:
		case 0xfd8877: 
		ch='^'; break;		
		
		case 0xFF10EF:
		case 0xfd28d7: 
		ch='<'; break;		
		
		case 0xFF38C7:
		case 0xfda857: 
		ch='!'; break;		
		
		case 0xFF5AA5:
		case 0xfd6897: 
		ch='>'; break;		
		
		case 0xFF4AB5:
		case 0xfd9867: 
		ch='v'; break;
		
		case 0xFFFFFFFF: 
		ch='r'; break;
		
		default: ch='?';

/*
		//case 0xFF6897: // from internet example
		case 0x8F742bd:  // Leon's fancy remote controller
		case 0xFFA25D:   // Arduino RC 1 (with bubbles under. newer?)
		case 0xfd00ff:   // Arduino RC 2
		ch='1'; valid=1; break;
		
		//case 0xFF9867:
		case 0x8F7C23D:
		case 0xFF629D:
		case 0xfd807f: 
		ch='2'; valid=1; break;
		
		//case 0xFFB04F:
		case 0x8F7E817:
		case 0xFFE21D:
		case 0xfd40bf: 
		ch='3'; valid=1; break;
		
		//case 0xFF30CF:
		case 0x8F702FD:
		case 0xFF22DD:
		case 0xfd20df: 
		ch='4'; valid=1; break;
		
		//case 0xFF18E7:
		case 0x8F7827D:
		case 0xFF02FD:
		case 0xfda05f: 
		ch='5'; valid=1; break;
		
		//case 0xFF7A85:
		case 0x8F76897:
		case 0xFFC23D:
		case 0xfd609f: 
		ch='6'; valid=1; break;
		
		//case 0xFF10EF:
		case 0x8F740BF:
		case 0xFFE01F:
		case 0xfd10ef: 
		ch='7'; valid=1; break;
		
		//case 0xFF38C7:
		case 0x8F7C03F:
		case 0xFFA857:
		case 0xfd906f: 
		ch='8'; valid=1; break;
		
		//case 0xFF5AA5:
		case 0x8F7A857:
		case 0xFF906F:
		case 0xfd50af: 
		ch='9'; valid=1; break;
		
		//case 0xFF42BD:
		case 0x8F700FF:
		case 0xFF6897:
		case 0xfd30cf: 
		ch='*'; valid=1; break;
		
		//case 0xFF4AB5:
		case 0x8F7807F:
		case 0xFF9867:
		case 0xfdb04f: 
		ch='0'; valid=1; break;
		
		//case 0xFF52AD:
		case 0x8F728D7:
		case 0xFFB04F:
		case 0xfd708f: 
		ch='#'; valid=1; break;
		
		//case 0xFF629D:
		case 0x8F7906F:
		case 0xFF18E7:
		case 0xfd8877: 
		ch='^'; break;
		
		//case 0xFF22DD:
		case 0x8F710EF:
		case 0xFF10EF:
		case 0xfd28d7: 
		ch='<'; break;
		
		//case 0xFF02FD:
		case 0x8F7708F:
		case 0xFF38C7:
		case 0xfda857: 
		ch='!'; break;
		
		//case 0xFFC23D:
		case 0x8F7D02F:
		case 0xFF5AA5:
		case 0xfd6897: 
		ch='>'; break;
		
		//case 0xFFA857:
		case 0x8F750AF:
		case 0xFF4AB5:
		case 0xfd9867: 
		ch='v'; break;
		
		case 0xFFFFFFFF: 
		ch='r'; break;
		
		default: ch='?';
*/
	}
	if(valid==1) {
		return (0x80 | ch); // use bit 7 to identify valid
	} else {
		return (uint8_t)ch;
	}
}

void TC1_Handler(void)
{
    if (TC1->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1)) {
        // begin my code
        uint8_t irdata = HAL_GPIO_RC_PIN_read();        
        
#if (MASTER_PCB_v32==0)
        //HAL_GPIO_LED_RED_write(irdata);
        HAL_GPIO_LED_RED_clr();
#endif
    
        myticks++;
        if (irparams.rawlen >= RAWBUF) {
            // Buffer overflow
            irparams.rcvstate = STATE_STOP;
        }
        switch(irparams.rcvstate) {
          case STATE_IDLE: // In the middle of a gap
            if (irdata == MARK) {                
                // gap just ended, record duration and start recording transmission
                irparams.rawlen = 0;
                irparams.rawbuf[irparams.rawlen++] = myticks;
                myticks = 0;
                irparams.rcvstate = STATE_MARK;                
            }
            break;
          case STATE_MARK: // timing MARK
            if (irdata == SPACE) {   // MARK ended, record time
                irparams.rawbuf[irparams.rawlen++] = myticks;
                myticks = 0;
                irparams.rcvstate = STATE_SPACE;
            }
            break;
          case STATE_SPACE: // timing SPACE
            if (irdata == MARK) { // SPACE just ended, record it
                irparams.rawbuf[irparams.rawlen++] = myticks;
                myticks = 0;
                irparams.rcvstate = STATE_MARK;
            } else { // SPACE
                if (myticks > GAP_TICKS) {
                    // big SPACE, indicates gap between codes
                    // Mark current code as ready for processing
                    // Switch to STOP
                    // Don't reset timer; keep counting space width
                    irparams.rcvstate = STATE_STOP;                                        
                }
            }
            break;
          case STATE_STOP: // waiting, measuring gap
            TC1->COUNT16.CTRLA.reg &= ~(TC_CTRLA_ENABLE);
            NVIC_DisableIRQ(TC1_IRQn);
            
            if (irdata == MARK) { // reset gap timer                  
                myticks = 0;                
            }             
            break;
        }
        TC1->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
    }
}
//-----------------------------------------------------------------------------
static void timer1_init(int8_t onoff)
{
    if(onoff==0) {
        PM->APBCMASK.reg &= ~PM_APBCMASK_TC1;
    } else {    
        PM->APBCMASK.reg |= PM_APBCMASK_TC1;
        GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC1_GCLK_ID) |
                            GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);
  
        TC1->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
                                 TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_PRESCSYNC_RESYNC;
  
        TC1->COUNT16.COUNT.reg = 0;

        TC1->COUNT16.CC[0].reg = 48; // 20kHz
        TC1->COUNT16.COUNT.reg = 0;

        TC1->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);

        //TC1->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
        //NVIC_EnableIRQ(TC1_IRQn);
    }    
}

void irrecv_init(int8_t onoff)
{    
    HAL_GPIO_RC_PIN_in();
    HAL_GPIO_RC_PIN_pullup();
    timer1_init(onoff);
    irrecv_reset();
}
