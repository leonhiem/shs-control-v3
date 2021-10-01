/*
 * Kamworks Codeswitch firmware
 *
 * Author: Leon Hiemstra <l.hiemstra@gmail.com>
 */

#ifndef _IR_RECEIVER_H
#define _IR_RECEIVER_H

// begin IR receiver:
#define RAWBUF 70 // Length of raw duration buffer (65 is too small)
// information for the interrupt handler
typedef struct {
	uint8_t rcvstate;            // state machine
	unsigned int rawbuf[RAWBUF]; // raw data
	uint8_t rawlen;              // counter of entries in rawbuf
} irparams_t;

#define USECPERTICK 50  // microseconds per clock interrupt tick
// receiver states
#define STATE_IDLE     2
#define STATE_MARK     3
#define STATE_SPACE    4
#define STATE_STOP     5
// IR detector output is active low
#define MARK  0
#define SPACE 1
#define _GAP 80000 // 100000 // Minimum gap between transmissions  us
#define GAP_TICKS (_GAP/USECPERTICK)
// Decoded value for NEC when a repeat code is received
#define REPEAT 0xffffffff
#define IRERR 0
#define DECODED 1
#define TOLERANCE 25  // percent tolerance in measurements
#define LTOL (1.0 - TOLERANCE/100.)
#define UTOL (1.0 + TOLERANCE/100.)
#define TICKS_LOW(us) (int) (((us)*LTOL/USECPERTICK))
#define TICKS_HIGH(us) (int) (((us)*UTOL/USECPERTICK + 1))
#define MATCH(measured_ticks, desired_us) ((measured_ticks) >= TICKS_LOW(desired_us) && (measured_ticks) <= TICKS_HIGH(desired_us))
#define MATCH_MARK(measured_ticks, desired_us) MATCH(measured_ticks, (desired_us) + MARK_EXCESS)
#define MATCH_SPACE(measured_ticks, desired_us) MATCH((measured_ticks), (desired_us) - MARK_EXCESS)
#define NEC 1
#define NEC_BITS 32
#define NEC_HDR_MARK	9000
#define NEC_HDR_SPACE	4500
#define NEC_BIT_MARK	560
#define NEC_ONE_SPACE	1500 //1600
#define NEC_ZERO_SPACE	600  //560  //not 500!
#define NEC_RPT_SPACE	2250
// Marks tend to be 100us too long, and spaces 100us too short
// when received due to sensor lag.
#define MARK_EXCESS 100
// Results returned from the decoder
typedef struct  {
	unsigned long value; // Decoded value
	int bits; // Number of bits in decoded value
	volatile unsigned int *rawbuf; // Raw intervals in .5 us ticks
	int rawlen; // Number of records in rawbuf.
} decode_results;

void irrecv_reset(void);
void irrecv_reset_cpy(void);
long decodeNEC(decode_results *results);
int irrecv_decode(decode_results *results);
uint8_t irrecv_get_irparams_rcvstate(void);
uint8_t irrecv_convert(uint32_t val);
void irrecv_init(int8_t onoff);

#endif // _IR_RECEIVER_H
