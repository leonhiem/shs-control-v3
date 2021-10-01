/*
 * Kamworks Codeswitch firmware
 *
 * Author: Leon Hiemstra <l.hiemstra@gmail.com>
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

void delay_us(unsigned long us)
{
	unsigned long cnt;
	for(cnt=0;cnt<(us);cnt++) asm volatile ("nop"); // roughly microseconds
}

void delay_1us(void)
{
	asm volatile ("nop"); // roughly microsecond
}

void delay_ms(unsigned long ms)
{
#define DELAY_1ms (850)  // 1/F_CPU * 850   (F_CPU=8e6) (measured with oscilloscope)
	unsigned long cnt;
	for(cnt=0;cnt<(DELAY_1ms*ms);cnt++) asm volatile ("nop");
}

char is_digit(char c)
{
	if(c&0xc0) return 0;
	if(c<0x30) return 0;
	if(c>0x39) return 0;
	return 1;
}

const uint8_t seqlut[] = {
	35, 86, 69, 93, 99, 63, 74, 15, 84, 88,
	82, 26, 50, 43, 58, 55, 97, 71, 52, 96,
	13, 67, 56, 20, 27, 46,  9, 19, 73, 38,
	1,  2, 77, 64,  5, 57,  3, 25, 21, 31,
	61, 16,  8, 75, 24, 92, 78, 30, 33,  6,
	10, 44, 72, 83, 53, 95, 49, 11, 18, 42,
	40, 28, 76, 17,  0, 14, 12, 80, 47, 87,
	68, 98, 32, 51, 39, 81, 62, 34, 45, 54,
	85, 36,  4, 37, 41, 66, 91, 70, 89, 60,
	79, 94, 29, 90, 22,  7, 65, 48, 59, 23
};

const uint8_t digitlut[] = {1, 7, 2, 6, 4, 0, 9, 8, 5, 3};

void decrypt_digits(char *code_str)
{
	int i,c;
	for(i=0;i<14;i++) {
		for(c=0;c<10;c++) {
			if(code_str[i] == (digitlut[c]+'0')) break;
		}
		code_str[i]=c+'0';
	}
}
int decode_rcode(char *code_str, uint8_t *seq, uint32_t *id, uint16_t *days)
{
	uint8_t seqc,seqcode;
	int i;
	char str[16];
	char str1[16];
	char str2[16];

	decrypt_digits(code_str);

	seqcode = strtoul(&code_str[12],NULL,10);
	if(seqcode < 0 || seqcode > 99) {
		return -1; // bad sequence number
	}
	strcpy(str,code_str);
	str[12]='\0';
	// rol <<
	for(i=0;i<seqcode;i++) {
		strncpy(str1,str,1); // take most left digit
		str1[1]='\0';
		strncpy(str2,&str[1],11); // take other right digits
		str2[11]='\0';

		strcpy(str,str2);
		strcat(str,str1);
	}
	
	for(seqc=0;seqc<100;seqc++) {
		if(seqcode==seqlut[seqc]) break;
	}
	*seq=seqc;

	strcpy(str1,str);
	strcpy(str2,str);
	// 123456789012
	// 0        9
	str1[9]='\0';
	str2[8]=' ';

	*id = strtoul(str1,NULL,10);
	*days = strtoul(&str2[9],NULL,10);
	return 1;
}

