

#ifndef _TYPES_H
#define _TYPES_H

#include <stdint.h>

typedef uint8_t bool_t;
#define TRUE 1
#define FALSE (!TRUE)

#ifndef NULL
#define NULL ((void *)0)
#endif

typedef struct sysTime_s {
	uint16_t ms; /**< Current MS timer value, resets to 0 every minute. */
	uint8_t sec; /**< Current second timer value, resets to 0 every minute. */
} sysTime_t;

#endif