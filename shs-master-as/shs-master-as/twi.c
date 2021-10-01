#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <util/twi.h>		/* Note [1] */


/*
 * Maximal number of iterations to wait for a device to respond for a
 * selection.  Should be large enough to allow for a pending write to
 * complete, but low enough to properly abort an infinite loop in case
 * a slave is broken or not present at all.  With 100 kHz TWI clock,
 * transfering the start condition and SLA+R/W packet takes about 10
 * µs.  The longest write period is supposed to not exceed ~ 10 ms.
 * Thus, normal operation should not require more than 100 iterations
 * to get the device to respond to a selection.
 */
#define MAX_ITER	10
#define PAGE_SIZE 8


/*
 * Saved TWI status register, for error messages only.  We need to
 * save it in a variable, since the datasheet only guarantees the TWSR
 * register to have valid contents while the TWINT bit in TWCR is set.
 */
uint8_t twst;
extern volatile uint8_t twi_timeout;

#define TWI_TIMEOUT 8 // 8/10 second

void twi_init(void)
{
    /* Initialize TWI */
    //TWBR = 0xff; 
    //TWBR = 0x80; 
    //TWBR = 0x40; 
    TWBR = 0x20; 

    //TWSR = 0x3; // (measured) when TWBR=0xff SCL=220 Hz
    TWSR = 0x2;  // (measured) when TWBR=0xff SCL=910Hz,when TWBR=0x80 SCL=1818Hz,when TWBR=0x40 SCL=3636Hz
                 //            when TWBR=0x20 SCL=7143Hz
}


int twi_write_one(uint8_t slave_addr, uint8_t data)
{
    uint8_t n = 0;
    int rv = 0;

restart:
    if (n++ >= MAX_ITER) goto error;//return -1;

begin:
    //printf("enter\n\r");

    /* Note [15] */
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
    twi_timeout=TWI_TIMEOUT;
    while ((TWCR & _BV(TWINT)) == 0) { 
       // printf("twi_timeout=%d\n\r",twi_timeout);
        if(twi_timeout==0) { goto error; } 
    } /* wait for transmission */
    //printf("1\n\r");
    switch ((twst = TW_STATUS)) {
       case TW_REP_START:		/* OK, but should not happen */
       case TW_START:
         break;

       case TW_MT_ARB_LOST:
         goto begin;

       default:
         return -1;		/* error: not in start condition */
				/* NB: do /not/ send stop condition */
    }

    /* send SLA+W */
    TWDR = (slave_addr<<1);
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
    twi_timeout=TWI_TIMEOUT;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    //printf("2\n\r");
    switch ((twst = TW_STATUS)) {
       case TW_MT_SLA_ACK:
         //printf("TW_MT_SLA_ACK\n\r");
         break;

       case TW_MT_SLA_NACK:	/* nack during select: device busy writing */
         //printf("TW_MT_SLA_NACK\n\r");
         goto restart;

       case TW_MT_ARB_LOST:	/* re-arbitrate */
         //printf("TW_MT_ARB_LOST\n\r");
         goto begin;

       default:
         goto error;		/* must send stop condition */
    }

    

    TWDR = data; 
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transm */
    twi_timeout=TWI_TIMEOUT;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    //printf("3\n\r");
    switch ((twst = TW_STATUS)) {
      case TW_MT_DATA_ACK:
        break;

      case TW_MT_DATA_NACK:
        goto quit;

      case TW_MT_ARB_LOST:
        goto begin;

      default:
        goto error;		/* must send stop condition */
    }
    
quit:
    //printf("stop\n\r");
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */

    return rv;

error:
    //printf("error\n\r");
    rv = -1;
    goto quit;
}



int twi_read_small( int len, uint8_t *buf, uint8_t slave_addr)
{
    uint8_t twcr;
    int rv = 0;


begin:
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
    twi_timeout=TWI_TIMEOUT;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
       case TW_REP_START:          /* OK, but should not happen */
       case TW_START:
         break;

       case TW_MT_ARB_LOST:        /* Note [9] */
         goto begin;

       default:
         return -1;                /* error: not in start condition */
                                   /* NB: do /not/ send stop condition */
    }

    /* send SLA+R */
    TWDR = (slave_addr<<1) | 1;
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
    twi_timeout=TWI_TIMEOUT;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
      case TW_MR_SLA_ACK:
        break;

      case TW_MR_SLA_NACK:
        goto quit;

      case TW_MR_ARB_LOST:
        goto begin;

      default:
        goto error;
    }


    for (twcr = _BV(TWINT) | _BV(TWEN) | _BV(TWEA) /* Note [13] */;
         len > 0;
         len--)
    {
        if (len == 1) twcr = _BV(TWINT) | _BV(TWEN); /* send NAK this time */

        TWCR = twcr;		/* clear int to start transmission */
        twi_timeout=TWI_TIMEOUT;
        while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
        switch ((twst = TW_STATUS)) {
  	  case TW_MR_DATA_NACK:
	    len = 0;		/* force end of loop */
	    /* FALLTHROUGH */
	  case TW_MR_DATA_ACK:
	    *buf++ = TWDR;
	    rv++;
	    break;

	  default:
	    goto error;
	}
    }
quit:

    /* Note [14] */
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */

    return rv;

error:
    rv = -1;
    goto quit;
}

