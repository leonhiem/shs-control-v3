/*
 * SIM Switch
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "shsctrl.h"
#include "uart.h"
#include "delay.h"
#include "io.h"


extern sysVals_t sysval;
extern uint8_t EEMEM EEPROM_ADDR_OperatorNr[][MAX_PHONE_NR_LENGTH];
extern uint8_t EEMEM EEPROM_ADDR_apn[APN_LEN];
extern uint8_t EEMEM EEPROM_ADDR_user[USER_LEN];
extern uint8_t EEMEM EEPROM_ADDR_pwd[PWD_LEN];



#ifdef LEONS_SHS
#warning "sim900.c: Leon's system in Holland"
#else
#warning "sim900.c: SHS in Cambodia"
#endif


uint8_t sim_is_on(void)
{
    return (bit_is_set(PINC,SIM900_STATUS));
}

char sim_rxflush(void)
{
	char ch=0;
	int timeout_centisec=5;
#ifdef SIM_DEBUG
//	printf("\nrxflush:");
#endif
	while(timeout_centisec>0) {
		wdt_reset();
		delay_ms(10);
		if(Poll_SIM()) {
			ch=Receive_SIM_Byte();
#ifdef SIM_DEBUG
//			printf("[%x]",ch);
#endif
		} else {
			delay_ms(100);
			timeout_centisec--;
		}
	}
#ifdef SIM_DEBUG
//	printf("\n");
#endif
	return ch;
}

/*
 * returns:
 *  0 if len=0 or 'expect' not found
 * >0 if len>0
 */
uint16_t sim_read(char *sim_bufptr, const char *expect, int timeout_centisec, uint16_t maxlen)
{
    uint16_t len=0;
	int timeout=timeout_centisec;
	char ch=0, found=0;

    wdt_reset();
#ifdef SIM_DEBUG
//    printf("\nsim_read:");
#endif
    while(timeout>0) {
		if(len>=(maxlen-1)) break;	 
		wdt_reset();
		delay_ms(1);
		if(Poll_SIM()) {
			ch = Receive_SIM_Byte();			
#ifdef SIM_DEBUG
//			if(ch==0) printf("\\0");
//			else if(ch=='\n') printf("\\n");
//			else if(ch=='\r') printf("\\r");
//			else printf("%c",ch);
#endif
			sim_bufptr[len]=ch;
			len++;
			sim_bufptr[len]=0;
			if(strstr(sim_bufptr,expect)!=NULL) { found=1; break; }
		} else {
			delay_ms(100);
			timeout--;
			if(timeout <= 0) {
			    break;
			}
		}
    }			 
    if(timeout <= 0) {
       printf("timeout\n");
       len=0;
    }
    sim_bufptr[len]=0;
    len=strlen(sim_bufptr);

    /*
     * Response looks like:
     * echo\r\nresponse\r\n\r\nOK\r\n
     *
     * echo=echo of given command
     * response=response of that command
     * OK=status, but can also be something else.
     */
#ifdef SIM_DEBUG
//    printf("\nlen=%d\n",len);	
#endif
    if(found) {  
        return len;
	} else {
		return 0;
	}
}

/*
 * returns:
 *  NULL if none in 'expect' is found
 *  *expect string if that one is found
 */
char * sim_read_multi(char *sim_bufptr, const char *expect[], const int nof_expect, int timeout_centisec, uint16_t maxlen)
{
    uint16_t len=0;
	int i,timeout=timeout_centisec;
	char ch=0, found=0;
	char *found_ptr=NULL;

    wdt_reset();
#ifdef SIM_DEBUG
//    printf("\nsim_read_multi:");
#endif
    while(timeout>0) {
		if(len>=(maxlen-1)) break;	 
		wdt_reset();
		delay_ms(1);
		if(Poll_SIM()) {
			ch = Receive_SIM_Byte();			
#ifdef SIM_DEBUG
//			if(ch==0) printf("\\0");
//			else if(ch=='\n') printf("\\n");
//			else if(ch=='\r') printf("\\r");
//			else printf("%c",ch);
#endif
			sim_bufptr[len]=ch;
			len++;
			sim_bufptr[len]=0;
			for(i=0;i<nof_expect;i++) {
			    if((found_ptr=strstr(sim_bufptr,expect[i]))!=NULL) { found=1; break; }
			}
			if(found) break;
		} else {
			delay_ms(100);
			timeout--;
			if(timeout <= 0) {
			    break;
			}
		}
    }			 
    if(timeout <= 0) {
       printf("timeout\n");
       found_ptr=NULL;
    }
	return found_ptr;
}


void sim_tx_char(char c)
{
    uint16_t timeout;
    
    timeout=2000;
    do { 
        delay_us(10);
        timeout--;
        if(timeout==0) break;
    } while (bit_is_clear(UCSR1A, UDRE1));


    loop_until_bit_is_clear(PIND, SIM900_CTS);
    timeout=2000;
    do { 
        delay_us(10);
        timeout--;
        if(timeout==0) break;
    } while (bit_is_set(PIND, SIM900_CTS));

    UDR1=c;
#ifdef SIM_DEBUG
//    if(c==0) printf("\\0");
//	else if(c=='\n') printf("\\n");
//	else if(c=='\r') printf("\\r");
//	else printf("%c",c);
#endif
}

void sim_tx(const char *buf, int len)
{
    char c;
    int i;
#ifdef SIM_DEBUG
//	printf("\nsim_write:");
#endif
    for(i=0;i<len;i++) {
        c=buf[i];
        sim_tx_char(c);
    }
}

uint8_t sim_onoff_pulse(void)
{
    uint8_t waiting=1;
    uint8_t ison;
    uint8_t sreg;
	
#ifdef SIM_DEBUG
	printf("\nsim_onoff_pulse: ");
#endif
    sreg=SREG;
    cli();
    sysval.SIM_poweron_pulse=6;
    SREG=sreg; // sei()

    while(waiting) {
        wdt_reset();
        sreg=SREG;
        cli();
        if(sysval.SIM_poweron_pulse==0) waiting=0;
        SREG=sreg; // sei()
        delay_ms(1);
    }
    ison=sim_is_on();
    if(ison) {
        sim_tx("AT\r",3); // help sim with autobauding
        wdt_reset();
        delay_ms(500);
        sim_tx("AT\r",3); // help sim with autobauding
    }
    sim_rxflush();
#ifdef SIM_DEBUG
    printf(" sim_onoff_pulse done\n");
#endif
    return ison;
}

int phonenr_ok(const char *nr)
{
    int i;
    if(strlen(nr) < MIN_PHONE_NR_LENGTH) return 0;
    if(strlen(nr) > MAX_PHONE_NR_LENGTH) return 0;
    if(nr[0] != '+') return 0;
    for(i=1;i<strlen(nr);i++) {
        if(!isdigit(nr[i])) return 0;
    }
    return 1;
}

int8_t sim_send_sms(const char *sms_msg, uint8_t size, uint8_t to_phone_idx)
{
    uint8_t sreg;
	char sim_rxbuf[64];
    char operator_phonenr[MAX_PHONE_NR_LENGTH];
    char buf[MAX_PHONE_NR_LENGTH];
    uint8_t len;
    int8_t ret=0;

    printf("sendsms:%s\n",sms_msg);
    sreg=SREG;
    cli();
    eeprom_read_block(operator_phonenr,
                      &EEPROM_ADDR_OperatorNr[to_phone_idx],
                      MAX_PHONE_NR_LENGTH);
    SREG=sreg;
    if(!phonenr_ok(operator_phonenr)) return -1;

    sim_tx("AT+CMGS=\"",9);
    len=sprintf(buf,"%s\"\r",operator_phonenr);
    sim_tx(buf,len);
    
    if(sim_read(sim_rxbuf,">",5,sizeof(sim_rxbuf))==0) {
        printf("abort\n");
        len=sprintf(buf,"%c",0x1b); // abort by send ESC
        sim_tx(buf,len);
        sim_rxflush();
        return -3;
    }

    /*
     * Now write SMS:
     */
    sim_tx(sms_msg,size); // send prepared sms message
    len=sprintf(buf,"%c",0x1a); // end SMS
    sim_tx(buf,len);

    sim_rxflush();
#ifdef SIM_DEBUG
    printf("result\n");
#endif
    ret=sim_read(sim_rxbuf,"+CMGS",60,sizeof(sim_rxbuf));
#ifdef SIM_DEBUG
    printf("send ret=%d\n",ret);
#endif
    sim_rxflush();
    return ret;
}

char * sim_receive_sms(char *sim_rxbuf, uint16_t maxlen)
{
    uint8_t sreg;
    char operator_phonenr[MAX_PHONE_NR_LENGTH];
    char *phonenr_ptr=NULL;
	char *sim_bufptr=NULL;
    char *ptr;
    char c;
    uint8_t match;
    int i;

    sysval.actual_operator_idx=0;

    sim_tx("AT+CMGL\r",8);
	delay_ms(500);
	wdt_reset();
	
    if(sim_read(sim_rxbuf,"REC UNREAD",30,maxlen) == 0) return NULL;
	
	if(sim_read(sim_rxbuf,"\r\nOK\r\n",7,maxlen) == 0) return NULL;	
    	
    // message response comes in like:
    // first pass:  AT+CMGL\r\r\n+CMGL: 1,"REC UNREAD
	// second pass: ","+31612622133","Leon","13/05/10,13:16:52+08"\r\nSOC?\r\n\r\nOK\r\n
    sim_bufptr=sim_rxbuf;
    match=0;
    while((c=*sim_bufptr) != 0) {
        if(c=='\"') match++;
        sim_bufptr++;
        if(match==2) {
			match++;
			printf("phonenr: ");
            phonenr_ptr=sim_bufptr;
        }
        if(match==4) {
            *(sim_bufptr-1)=0; // eos for phone number
            sim_bufptr++;
			printf("%s\n",phonenr_ptr);
            break;
        }
    }
    sim_bufptr=strstr(sim_bufptr,"\r\n");
    if(sim_bufptr==NULL) { printf("nothing\n"); return NULL; }

    sim_bufptr+=2; // point to message

    // remove "\r\n\r\nOK\r\n" after message:
    ptr=sim_bufptr;
    while((c=*ptr) != 0) {
        if(c=='\r') {
            *ptr=0;
            break;
        }
        ptr++;
    }
    printf("sms:%s\n",sim_bufptr);

    // verify if sender is in OPER list:
    for(i=0;i<MAX_NR_OPERATORS;i++) {
        sreg=SREG;
        cli();
        eeprom_read_block(operator_phonenr,&EEPROM_ADDR_OperatorNr[i],
                          MAX_PHONE_NR_LENGTH);
        SREG=sreg;
        if(strcmp(phonenr_ptr,operator_phonenr) == 0) {
            sysval.actual_operator_idx=i;
			printf("phone number OK\n");
            break;
        }
    }
	return sim_bufptr;
}

uint8_t sim_delete_sms(void)
{
	char sim_rxbuf[32];
	uint16_t len;
    sim_tx("AT+CMGD=1\r",10);
	sim_read(sim_rxbuf,"AT+CMGD=1\r",7,sizeof(sim_rxbuf)); // read echo
	len=sim_read(sim_rxbuf,"OK\r\n",7,sizeof(sim_rxbuf));	
	return (uint8_t)len;
}

uint8_t sim_delete_all_sms(void)
{
	char sim_rxbuf[32];
	uint16_t len;
	char *cmd="AT+CMGDA=\"DEL ALL\"\r";
    sim_tx(cmd,strlen(cmd));
	sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
	len=sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf));	
	return (uint8_t)len;
}

char * sim_read_IMEI(char *sim_rxbuf, uint16_t maxlen)
{
	char *ptr_imei,*ptr,c;
    uint8_t retry=5;
    char *cmd="AT+GSN\r";
	
    while(retry) {
        sim_tx(cmd,strlen(cmd));
		sim_read(sim_rxbuf,cmd,5,maxlen); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",5,maxlen)==0) {
            retry--;
            delay_ms(100);
			printf("bad imei");
            sim_rxflush();
            continue;
        }
		// skip heading \r\n
		ptr_imei=sim_rxbuf;
		while((c=*ptr_imei) != 0) {
			if(c=='\r' || c=='\n') {
				ptr_imei++;
				continue;
			} else break;
		}
		// remove "\r\n\r\nOK\r\n" after message:
		ptr=ptr_imei;
		while((c=*ptr) != 0) {
			if(c=='\r') {
				*ptr=0;
				break;
			}
			ptr++;
		}
		
        if(strlen(ptr_imei)>10) break;
        else {
            retry--;
            delay_ms(100);
            sim_rxflush();
            continue;
        }
    }
    if(retry==0) return "NO-IMEI";
    else return ptr_imei;
}

int sim_read_signalstrength(void)
{
	char sim_rxbuf[32];
    char *ptr;
    int found,signal,ber;
	char *cmd="AT+CSQ\r";

    sim_tx(cmd,strlen(cmd));
	sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
    if(sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf))==0)
        return -3;

    // response looks like: AT+CSQ\r\r\n+CSQ: 28,0
    ptr=sim_rxbuf;
    found=sscanf(ptr,"\r\n+CSQ: %d,%d",&signal,&ber);
    if(found==2) {
#ifdef SIM_DEBUG
        printf("CSQ:%d\n",signal);
#endif
        return signal;
    } else return -4;
}

char * sim_2g_state(char *sim_rxbuf, uint16_t maxlen)
{
	char *cmd="AT+CIPSTATUS\r";
    sim_tx(cmd,strlen(cmd));
	sim_read(sim_rxbuf,cmd,20,maxlen); // read echo
	// response looks like: AT+CIPSTATUS\r\r\nOK\r\n\r\nSTATE: IP INITIAL\r\n
    if(sim_read(sim_rxbuf,"STATE: ",20,maxlen)==0)
        return NULL;
	if(sim_read(sim_rxbuf,"\r\n",20,maxlen)==0)
	    return NULL;
	else 
	    return sim_rxbuf;
}

int8_t sim_start_2g(void)
{
	char sim_rxbuf[256];
    uint8_t retries;
    int8_t ret_state=0;
    uint8_t sreg;
    char login_data[LOGIN_LEN];
    char buf[LOGIN_LEN+4];
    uint8_t len;
    char *state;
    char *cmd1="AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r";
	char *cmd2="AT+SAPBR=3,1,\"APN\",\"";
	char *cmd3="AT+SAPBR=3,1,\"USER\",\"";
	char *cmd4="AT+SAPBR=3,1,\"PWD\",\"";
	char *cmd5="AT+CSTT=\"";
	char *cmd6="AT+CIICR\r";
	char *cmd7="AT+CIFSR\r";
	char *cmd8="AT+SAPBR=1,1\r";
	char *cmd9_shut="AT+CIPSHUT\r";
	
    sim_tx(cmd1,strlen(cmd1));
	sim_read(sim_rxbuf,"AT+SAPBR=3,1,",2,sizeof(sim_rxbuf)); // read echo
    if(sim_read(sim_rxbuf,"OK\r\n",2,sizeof(sim_rxbuf))==0) {
        return -3;
    }

    // find out current state:
    state=sim_2g_state(sim_rxbuf,sizeof(sim_rxbuf));
	
//#ifdef SIM_DEBUG
    printf("2g:state=%s\n",state);
//#endif

    if(strncmp(state,"IP INITIAL",10)==0) {

        sreg=SREG; cli();
        eeprom_read_block(login_data,&EEPROM_ADDR_apn,LOGIN_LEN);
        SREG=sreg;
        sim_tx(cmd2,strlen(cmd2));
        len=sprintf(buf,"%s\"\r",login_data);
        sim_tx(buf,len);
		sim_read(sim_rxbuf,"AT+SAPBR=3,1,",2,sizeof(sim_rxbuf)); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",2,sizeof(sim_rxbuf))==0) {
            return -4;
        }

        sreg=SREG; cli();
        eeprom_read_block(login_data,&EEPROM_ADDR_user,LOGIN_LEN);
        SREG=sreg;
        sim_tx(cmd3,strlen(cmd3));
        len=sprintf(buf,"%s\"\r",login_data);
        sim_tx(buf,len);
		sim_read(sim_rxbuf,"AT+SAPBR=3,1,",2,sizeof(sim_rxbuf)); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",2,sizeof(sim_rxbuf))==0) {
            return -5;
        }

        sreg=SREG; cli();
        eeprom_read_block(login_data,&EEPROM_ADDR_pwd,LOGIN_LEN);
        SREG=sreg;
        sim_tx(cmd4,strlen(cmd4));
        len=sprintf(buf,"%s\"\r",login_data);
        sim_tx(buf,len);
		sim_read(sim_rxbuf,"AT+SAPBR=3,1,",2,sizeof(sim_rxbuf)); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",2,sizeof(sim_rxbuf))==0) {
            return -6;
        }

        sim_tx(cmd5,strlen(cmd5));
        sreg=SREG; cli();
        eeprom_read_block(login_data,&EEPROM_ADDR_apn,LOGIN_LEN);
        SREG=sreg;
        len=sprintf(buf,"%s\",\"",login_data);
        sim_tx(buf,len);

        sreg=SREG; cli();
        eeprom_read_block(login_data,&EEPROM_ADDR_user,LOGIN_LEN);
        SREG=sreg;
        len=sprintf(buf,"%s\",\"",login_data);
        sim_tx(buf,len);

        sreg=SREG; cli();
        eeprom_read_block(login_data,&EEPROM_ADDR_pwd,LOGIN_LEN);
        SREG=sreg;
        len=sprintf(buf,"%s\"\r",login_data);
        sim_tx(buf,len);
        sim_read(sim_rxbuf,"AT+CSTT=",15,sizeof(sim_rxbuf)); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",15,sizeof(sim_rxbuf))==0) {
            return -7;
        }

        ret_state=1;
    } else if(strncmp(state,"IP START",8)==0) {
        // bring up wireless connection with GPRS
        retries=0;
        while(retries<4) {
            sim_tx(cmd6,strlen(cmd6));
            delay_ms(1000);
            wdt_reset();
            delay_ms(500);
			sim_read(sim_rxbuf,cmd6,50,sizeof(sim_rxbuf)); // read echo
            if(sim_read(sim_rxbuf,"OK\r\n",50,sizeof(sim_rxbuf))==0) {
                retries++;
            } else break;
        }
        if(retries>3) return -8;

        ret_state=2;
    } else if(strncmp(state,"IP GPRSACT",10)==0) {
        // get an IP address
        sim_tx(cmd7,strlen(cmd7));
		sim_read(sim_rxbuf,cmd7,15,sizeof(sim_rxbuf)); // read echo
        if(sim_read(sim_rxbuf,".",15,sizeof(sim_rxbuf))==0) { // response is the ipadress
            return -9;
        }
		sim_rxflush();
        ret_state=3;
    } else if(strncmp(state,"IP STATUS",9)==0) {
        retries=0;
        while(retries<4) {
            // open bearer
            sim_tx(cmd8,strlen(cmd8));
			sim_read(sim_rxbuf,cmd8,70,sizeof(sim_rxbuf)); // read echo
            if(sim_read(sim_rxbuf,"OK\r\n",70,sizeof(sim_rxbuf))==0) {
                retries++;
            } else break;
        }
        if(retries>3) return -10;

        ret_state=4;
    } else if(strncmp(state,"PDP DEACT",9)==0) {
		sim_tx(cmd9_shut,strlen(cmd9_shut));
		sim_read(sim_rxbuf,cmd9_shut,40,sizeof(sim_rxbuf)); // read echo
		wdt_reset();
		delay_ms(500);
		sim_read(sim_rxbuf,"SHUT OK\r\n",40,sizeof(sim_rxbuf));
		return -11;
	} else return -12;

    return ret_state;
}

int8_t sim_send_2g_msg(const char *msg, uint16_t size)
{
	char sim_rxbuf[384];
    uint8_t retries;
    int8_t ret=0;
    uint8_t waiting;
    char *cmd1_init     ="AT+HTTPINIT\r";
	char *cmd2_term     ="AT+HTTPTERM\r";
	char *cmd3_para_cid ="AT+HTTPPARA=\"CID\",1\r";
	char *cmd4_para_url ="AT+HTTPPARA=\"URL\",\"";
	char *cmd5_action   ="AT+HTTPACTION=0\r";
	char *cmd6_status   ="AT+HTTPSTATUS?\r";
	
    delay_ms(500);
    wdt_reset();
    sim_tx(cmd1_init,strlen(cmd1_init));
	sim_read(sim_rxbuf,cmd1_init,15,sizeof(sim_rxbuf)); // read echo
    if(sim_read(sim_rxbuf,"OK\r\n",15,sizeof(sim_rxbuf))==0) {
		// reason of failure could be unterminated HTTP. Terminate it:
		sim_rxflush();
		sim_tx(cmd2_term,strlen(cmd2_term));
		sim_read(sim_rxbuf,"OK\r\n",40,sizeof(sim_rxbuf));
        ret= -2;
        return ret;
    }

    retries=0;
    while(retries<4) {
        sim_tx(cmd3_para_cid,strlen(cmd3_para_cid));
		sim_read(sim_rxbuf,"AT+HTTPPARA=",20,sizeof(sim_rxbuf)); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf))==0) {
            retries++;
        } else break;
    }
    if(retries>3) {
        ret = -3;
        goto sim_send_2g_msg_exit;
    }

    retries=0;
    while(retries<4) {
        sim_tx(cmd4_para_url,strlen(cmd4_para_url));
        sim_tx(msg,size); // send prepared command+msg
        sim_tx("\"\r",2); // end of command
		sim_read(sim_rxbuf,"AT+HTTPPARA=",20,sizeof(sim_rxbuf)); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf))==0) {
            retries++;
        } else break;
    }
    if(retries>3) {
        ret = -3;
        goto sim_send_2g_msg_exit;
    }

    retries=0;
    while(retries<4) {
        sim_tx(cmd5_action,strlen(cmd5_action));
		sim_read(sim_rxbuf,cmd5_action,20,sizeof(sim_rxbuf)); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf))==0) {
            retries++;
        } else break;
    }
    if(retries>3) {
        ret = -4;
        goto sim_send_2g_msg_exit;
    }

    delay_ms(500);
    wdt_reset();

    // wait until URL is sent
    waiting=15;
    while(waiting) {
        delay_ms(500);
        sim_tx(cmd6_status,strlen(cmd6_status));
		// response is: AT+HTTPSTATUS?\r\r\n+HTTPSTATUS: GET,0,0,0\r\n\r\nOK\r\n
		sim_read(sim_rxbuf,cmd6_status,40,sizeof(sim_rxbuf)); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",40,sizeof(sim_rxbuf))==0) {
            waiting--;
            continue;
        }
		if(strstr(sim_rxbuf,"GET,0,0,0")==NULL) { // response should become 0,0,0		
            waiting--;
            continue;
        } else break;
    }
    if(waiting==0) ret=-5;
sim_send_2g_msg_exit:
    waiting=15;
	delay_ms(500);
	wdt_reset();
	sim_rxflush();
    while(waiting) {
        sim_tx(cmd2_term,strlen(cmd2_term));
		sim_read(sim_rxbuf,cmd2_term,40,sizeof(sim_rxbuf)); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",40,sizeof(sim_rxbuf)) > 0) {
            break;
        }
        waiting--;
        delay_ms(100);
    }
    return ret;
}

void sim_stop_2g(void)
{
	char sim_rxbuf[64];
    uint8_t waiting=10;
	char *cmd1_sapbr="AT+SAPBR=0,1\r";
	char *cmd2_shut="AT+CIPSHUT\r";

    while(waiting) {
		wdt_reset();
        sim_tx(cmd1_sapbr,strlen(cmd1_sapbr));
		sim_read(sim_rxbuf,cmd1_sapbr,40,sizeof(sim_rxbuf)); // read echo
        if(sim_read(sim_rxbuf,"OK\r\n",40,sizeof(sim_rxbuf)) > 0) {
            break;
        }
        waiting--;
        delay_ms(100);
    }
    sim_tx(cmd2_shut,strlen(cmd2_shut));
	sim_read(sim_rxbuf,cmd2_shut,40,sizeof(sim_rxbuf)); // read echo
	wdt_reset();
	delay_ms(500);
	sim_read(sim_rxbuf,"SHUT OK\r\n",40,sizeof(sim_rxbuf));
}

int sim_read_CREG(void)
{
	char sim_rxbuf[64];
    char *ptr;
    int found,stat;
	char *cmd="AT+CREG?\r";

    sim_tx(cmd,strlen(cmd));
	sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
    if(sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf))==0)
        return -1;

    // response looks like: +CREG: 0,1\r\nOK
    ptr=sim_rxbuf;
    found=sscanf(ptr,"\r\n+CREG: 0,%d",&stat);
    if(found==1) {
        printf("CREG:%d\n",stat);
        return stat;
    } else return -1;
}

uint8_t sim_set_flowcontrol(void)
{
	char sim_rxbuf[32];
	char *cmd1="AT+IFC=2,2\r";
	char *cmd2="AT+CSCLK=0\r";
	char *cmd3="AT&D0\r";
	
    sim_tx(cmd1,strlen(cmd1)); // set hardware flow control
    sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf));

    sim_tx(cmd2,strlen(cmd2)); // do not go to sleep
    sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf));

    sim_tx(cmd3,strlen(cmd3)); // ignore DTR status
    return (uint8_t)sim_read(sim_rxbuf,"OK\r\n",10,sizeof(sim_rxbuf));
}

/*
 * Returns:
 *  0 if error or not found "OK"
 * >0 if found "OK": means success
 */
uint8_t sim_set_textmode(void)
{
	char sim_rxbuf[32];
	char *cmd="AT+CMGF=1\r"; 
    sim_tx(cmd,strlen(cmd)); // select text mode
    return (uint8_t)sim_read(sim_rxbuf,"OK\r\n",5,sizeof(sim_rxbuf));
}

/*
 * Returns:
 *  0 if error or not found "READY"
 * >0 if found "READY": means pincode check OK; no pending pincode
 */
uint8_t sim_pincode_check(void)
{
	char sim_rxbuf[32];
	uint16_t len;
	char *cmd="AT+CPIN?\r";
	
    sim_tx(cmd,strlen(cmd));
	delay_ms(500);
	wdt_reset();	
    len=sim_read(sim_rxbuf,"READY",30,sizeof(sim_rxbuf));
	sim_rxflush();
	return (uint8_t)len;
}

// --------------------
// Bluetooth functions:
// --------------------
uint8_t sim_has_bluetooth(void)
{
	char sim_rxbuf[60];
	char *ptr;
	char *cmd="AT+CGMR\r"; // read firmware version. ends with "_BT" if has bluetooth
	
	sim_tx(cmd,strlen(cmd));
	delay_ms(500);
	wdt_reset();
	if(sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf))==0)
	    return 0;
	
	ptr=sim_rxbuf;
	if(strstr(ptr,"BT")!=NULL) {
		return 1;
	} else {
		return 0;
	}
}


int sim_read_bluetooth_status(void)
{
	char sim_rxbuf[60];
	char *ptr;
	int found,stat;
	char *cmd="AT+BTSTATUS?\r";

	sim_tx(cmd,strlen(cmd));
	sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
	sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf));

	// response looks like: +BTSTATUS: 5\r\nOK
	ptr=sim_rxbuf;
	found=sscanf(ptr,"\r\n+BTSTATUS: %d",&stat);
	sim_rxflush();
	if(found==1) {
		printf("BTSTATUS:%d\n",stat);
		return stat;
	} else return -1;
}

uint8_t sim_bluetooth_power_on(void)
{
	char sim_rxbuf[32];
	char *cmd="AT+BTPOWER=1\r";
	printf("sim_bluetooth_power_on\n");	
	sim_tx(cmd,strlen(cmd)); // power on BT
	sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
	wdt_reset();
	delay_ms(500);
	wdt_reset();
	delay_ms(500);
	wdt_reset();
	delay_ms(500);
	wdt_reset();
	return (uint8_t)sim_read(sim_rxbuf,"OK\r\n",40,sizeof(sim_rxbuf));
}

uint8_t sim_bluetooth_power_off(void)
{
	char sim_rxbuf[32];
	char *cmd="AT+BTPOWER=0\r"; // power off BT
	printf("sim_stop_bluetooth\n");
	sim_tx(cmd,strlen(cmd));
	sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
	wdt_reset();
	delay_ms(500);
	return (uint8_t)sim_read(sim_rxbuf,"OK\r\n",40,sizeof(sim_rxbuf));
}

uint8_t sim_set_bluetooth(void)
{
	char sim_rxbuf[50];
	char *cmd_gethost="AT+BTHOST?\r";
	char *cmd_sethost="AT+BTHOST=Kamworks_SHS\r";
	char *bthostname =          "Kamworks_SHS";
	char *cmd_btpaircfg = "AT+BTPAIRCFG=1,0000\r"; // sets the pairing mode to 1 with pincode="0000"
	char *cmd_btsppcfg = "AT+BTSPPCFG=MC,0\r"; // do not support multiconnections
	char *found_ptr=NULL;
	
	sim_tx(cmd_gethost,strlen(cmd_gethost)); // get BT hostname
	sim_read(sim_rxbuf,cmd_gethost,10,sizeof(sim_rxbuf)); // read echo
	sim_read(sim_rxbuf,"OK\r\n",20,sizeof(sim_rxbuf));
	// response looks like: +BTHOST: Kamworks_SHS,38:1c:4a:98:b4:24\r\n\r\nOK
	found_ptr = strstr(sim_rxbuf,bthostname);
	sim_rxflush();
	if(found_ptr != NULL) {
		printf("sim_set_bluetooth: bthost already set\n");
		} else {
		printf("sim_set_bluetooth: setting up\n");
		sim_tx(cmd_sethost,strlen(cmd_sethost)); // set BT hostname
		sim_read(sim_rxbuf,"OK\r\n",5,sizeof(sim_rxbuf));
		
		sim_tx(cmd_btsppcfg,strlen(cmd_btsppcfg)); // disable multiconnections
		sim_read(sim_rxbuf,"OK\r\n",5,sizeof(sim_rxbuf));
		
		sim_tx(cmd_btpaircfg,strlen(cmd_btpaircfg)); // set BT pairing mode and pincode (needs bt reboot)
		sim_read(sim_rxbuf,"OK\r\n",5,sizeof(sim_rxbuf));
		sim_rxflush();
	}
	return 1;
}


void sim_start_bluetooth(void)
{
	printf("sim_start_bluetooth\n");
	sysval.bluetooth_status=sim_read_bluetooth_status();
	if(sysval.bluetooth_status==0) {
		sim_set_bluetooth();
		sim_bluetooth_power_on();		
	    delay_ms(500);
	    wdt_reset();	
	    sysval.bluetooth_status=sim_read_bluetooth_status();
	}
	printf("Bluetooth status=%d\n",sysval.bluetooth_status);
}

int sim_bluetooth_connect(void)
{
	char sim_rxbuf[32];
	char *cmd="AT+BTACPT=1\r"; // accept connection
	printf("sim_bluetooth_connect\n");
	sim_tx(cmd,strlen(cmd));
	sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
	return (uint8_t)sim_read(sim_rxbuf,"OK\r\n",40,sizeof(sim_rxbuf));
}

char * sim_bluetooth_read(char *sim_rxbuf, uint16_t maxlen)
{
	char *sim_bufptr=NULL;
	char *ptr;
	char c;
	uint8_t match;
	int found,bt_ch,bt_len;
	const char *expect[] = { "+BTSPPDATA:", 
		                     "+BTDISCONN:",
							 "+BTCONNECT" };
    wdt_reset();
    sim_bufptr=sim_read_multi(sim_rxbuf, expect, 3, 10,maxlen);
	if(sim_bufptr==NULL) return NULL;
	
	printf("sim_bluetooth_read:%s\n",sim_bufptr);
	
	if(strncasecmp(expect[2],sim_rxbuf,strlen(expect[2])) == 0) {
		sim_rxflush();
		if(sysval.bluetooth_connected==0) {
		    sim_bluetooth_connect();
		    sysval.bluetooth_connected = 1;
		    printf("sim_bluetooth_read: connect!\n");
		}
		return NULL;
	} else if(strncasecmp(expect[1],sim_bufptr,strlen(expect[1])) == 0) {
		sim_rxflush();
		sysval.bluetooth_connected = 0;
		printf("sim_bluetooth_read: disconnect!\n");
		return NULL;
	}	
	if(sim_read(sim_rxbuf,"\r\n",7,maxlen) == 0) return NULL;
	
	// message comes in like: +BTSPPDATA: 1,7,message\r\n	
	
	sim_bufptr=sim_rxbuf;
	
	// response looks like: +CREG: 0,1\r\nOK
	ptr=sim_rxbuf;
	found=sscanf(ptr,"%d,%d",&bt_ch,&bt_len);
	if(found!=2) {
		return NULL;
	}
	
	match=0;
	while((c=*sim_bufptr) != '\r') {
		if(c==',') match++;
		sim_bufptr++;
		if(match==2) break;
	}
	
	// remove "\r\n" after message:
	ptr=sim_bufptr;
	while((c=*ptr) != '\r') {
		if(c=='\r') {
			*ptr=0;
			break;
		}
		ptr++;
	}
	return sim_bufptr;
}

uint8_t sim_bluetooth_write(const char *msg, uint16_t size)
{
	char c;
	char sim_rxbuf[128], txbuf[32];
	uint8_t txlen;
	uint16_t oldidx=0,idx=0;
	
	// send line by line:
	while(idx < size) {
		c=msg[idx++];
		if(c=='\n') {
			txlen=sprintf(txbuf,"AT+BTSPPSEND=%d\r",idx-oldidx);
			printf("sim_bluetooth_write\n");
			sim_tx(txbuf,txlen);
			sim_read(sim_rxbuf,"> ",10,sizeof(sim_rxbuf)); // read prompt
			sim_tx(&msg[oldidx],idx-oldidx);
			delay_ms(500);
			wdt_reset();
			sim_read(sim_rxbuf,"SEND OK\r\n",50,sizeof(sim_rxbuf));
			oldidx=idx;
		}
	}
	return 1;
}

void sim_stop_bluetooth(void)
{
	printf("sim_stop_bluetooth\n");
	if(sysval.bluetooth_connected) {
		sim_bluetooth_write("bye, (busy 15sec)\r\n",19);
	}
	sysval.bluetooth_connected=0;
	sysval.bluetooth_status=sim_read_bluetooth_status();
	if(sysval.bluetooth_status==0) {
		printf("Bluetooth already off\n");
		return;
	}
	sim_bluetooth_power_off();
	sysval.bluetooth_status=sim_read_bluetooth_status();
	printf("Bluetooth status=%d\n",sysval.bluetooth_status);
}
