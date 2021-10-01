/*
 * SIM Switch
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#ifndef _SIM900_H
#define _SIM900_H
uint8_t sim_read(char *sim_bufptr, const char *expect, int timeout_centisec, uint16_t maxlen);
uint8_t sim_is_on(void);
uint8_t sim_onoff_pulse(void);
int8_t sim_send_sms(const char *sms_msg, uint8_t size, uint8_t to_phone_idx);
char * sim_receive_sms(char *buf, uint16_t maxlen);
uint8_t sim_delete_sms(void);
uint8_t sim_delete_all_sms(void);
char * sim_read_IMEI(char *sim_rxbuf, uint16_t maxlen);
int sim_read_signalstrength(void);
int8_t sim_start_2g(void);
int8_t sim_send_2g_msg(const char *msg, uint16_t size);
void sim_stop_2g(void);
int sim_read_CREG(void);
int sim_set_flowcontrol(void);
uint8_t sim_pincode_check(void);
uint8_t sim_set_textmode(void);

// Bluetooth functions:
uint8_t sim_has_bluetooth(void);
uint8_t sim_set_bluetooth(void);
void sim_start_bluetooth(void);
void sim_stop_bluetooth(void);
int sim_read_bluetooth_status(void);
int sim_bluetooth_connect(void);
char * sim_bluetooth_read(char *sim_rxbuf, uint16_t maxlen);
uint8_t sim_bluetooth_write(const char *msg, uint16_t size);

#endif // _SIM900_H
