/*
 * SIM Switch
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd20.h"
#include "sim.h"
#include "uart.h"
#include "utils.h"
#include "hal_gpio.h"
#include "shsctrl.h"
#include "flash.h"
#include "bt.h"

extern sysVals_t sysval;
extern sim_task_queue_t sim_task_queue;

int bt_read_bluetooth_status(void)
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
		uart_print(0,"BTSTATUS:%d\n\r",stat);
		return stat;
	} else return -1;
}

uint8_t bt_bluetooth_power_on(void)
{
	char sim_rxbuf[32];
	char *cmd="AT+BTPOWER=1\r";
	uart_print(0,"sim_bluetooth_power_on\n\r");	
	sim_tx(cmd,strlen(cmd)); // power on BT
	sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
	//wdt_reset();
	delay_ms(500);
	//wdt_reset();
	delay_ms(500);
	//wdt_reset();
	delay_ms(500);
	//wdt_reset();
	return (uint8_t)sim_read(sim_rxbuf,"OK\r\n",40,sizeof(sim_rxbuf));
}

uint8_t bt_bluetooth_power_off(void)
{
	char sim_rxbuf[32];
	char *cmd="AT+BTPOWER=0\r"; // power off BT
	uart_print(0,"sim_stop_bluetooth\n\r");
	sim_tx(cmd,strlen(cmd));
	sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
	//wdt_reset();
	delay_ms(500);
	return (uint8_t)sim_read(sim_rxbuf,"OK\r\n",40,sizeof(sim_rxbuf));
}

uint8_t bt_set_bluetooth(void)
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
		uart_print(0,"sim_set_bluetooth: bthost already set\n\r");
	} else {
		uart_print(0,"sim_set_bluetooth: setting up\n\r");
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


void bt_start_bluetooth(void)
{
	uart_print(0,"sim_start_bluetooth\n\r");
	sysval.bluetooth_status=bt_read_bluetooth_status();
	if(sysval.bluetooth_status==0) {
		bt_set_bluetooth();
		bt_bluetooth_power_on();		
	    delay_ms(500);
	    //wdt_reset();	
	    sysval.bluetooth_status=bt_read_bluetooth_status();
	}
	uart_print(0,"Bluetooth status=%d\n\r",sysval.bluetooth_status);
}

int bt_bluetooth_connect(void)
{
	char sim_rxbuf[32];
	char *cmd="AT+BTACPT=1\r"; // accept connection
	uart_print(0,"sim_bluetooth_connect\n\r");
	sim_tx(cmd,strlen(cmd));
	sim_read(sim_rxbuf,cmd,10,sizeof(sim_rxbuf)); // read echo
	return (uint8_t)sim_read(sim_rxbuf,"OK\r\n",40,sizeof(sim_rxbuf));
}

char * bt_bluetooth_read(char *sim_rxbuf, uint16_t maxlen)
{
	char *sim_bufptr=NULL;
	char *ptr;
	char c;
	uint8_t match;
	int found,bt_ch,bt_len;
	const char *expect[] = { "+BTSPPDATA:", 
                             "+BTDISCONN:",
                             "+BTCONNECT" };
    //wdt_reset();
    sim_bufptr=sim_read_multi(sim_rxbuf, expect, 3, 10,maxlen);
	if(sim_bufptr==NULL) return NULL;
	
	uart_print(0,"sim_bluetooth_read:%s\n\r",sim_bufptr);
	
	if(strncasecmp(expect[2],sim_rxbuf,strlen(expect[2])) == 0) {
		sim_rxflush();
		if(sysval.bluetooth_connected==0) {
		    bt_bluetooth_connect();
		    sysval.bluetooth_connected = 1;
		    uart_print(0,"sim_bluetooth_read: connect!\n\r");
		}
		return NULL;
	} else if(strncasecmp(expect[1],sim_bufptr,strlen(expect[1])) == 0) {
		sim_rxflush();
		sysval.bluetooth_connected = 0;
		uart_print(0,"sim_bluetooth_read: disconnect!\n\r");
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

uint8_t bt_bluetooth_write(const char *msg, uint16_t size)
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
			uart_print(0,"sim_bluetooth_write\n\r");
			sim_tx(txbuf,txlen);
			sim_read(sim_rxbuf,"> ",10,sizeof(sim_rxbuf)); // read prompt
			sim_tx(&msg[oldidx],idx-oldidx);
			delay_ms(500);
			//wdt_reset();
			sim_read(sim_rxbuf,"SEND OK\r\n",50,sizeof(sim_rxbuf));
			oldidx=idx;
		}
	}
	return 1;
}

void bt_stop_bluetooth(void)
{
	uart_print(0,"sim_stop_bluetooth\n\r");
	if(sysval.bluetooth_connected) {
		bt_bluetooth_write("bye, (busy 15sec)\r\n",19);
	}
	sysval.bluetooth_connected=0;
	sysval.bluetooth_status=bt_read_bluetooth_status();
	if(sysval.bluetooth_status==0) {
		uart_print(0,"Bluetooth already off\n\r");
		return;
	}
	bt_bluetooth_power_off();
	sysval.bluetooth_status=bt_read_bluetooth_status();
	uart_print(0,"Bluetooth status=%d\n\r",sysval.bluetooth_status);
}
