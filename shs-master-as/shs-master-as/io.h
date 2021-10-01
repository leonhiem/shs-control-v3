/*
 * SIM Switch
 *
 * Author: Leon Hiemstra <leon@hiemstra-electronics.com>
 * Date:   
 */

#ifndef _IO_H
#define _IO_H


/**
 * @file io.h
 * IO mapping
 *
 * Changelog:
 *
 */
 
 
 /** @section Port A */
 
 /*-----+------------------*
  | PA0 | Isense_load0     |
  | PA1 | Isense_load1     |
  | PA2 | Isense_load2     |
  | PA3 | Isense_charge    |
  | PA4 | Vsense_Vpv       |
  | PA5 | Vsense_Vbatt     |
  | PA6 | POWER_FAULT      |
  | PA7 | IO_0             |
  *-----+------------------*/
  
#define Isense_load0     0
#define Isense_load1     1
#define Isense_load2     2
#define Isense_charge    3
#define Vsense_Vpv       4
#define Vsense_Vbatt     5
#define POWER_FAULT      6
#define IO_0             7

 /** @section Port B */
 
 /*-----+------------------*
  | PB0 | LED_100PERCENT   |
  | PB1 | LED_CHARGING     |
  | PB2 | LED_60PERCENT    |
  | PB3 | LED_30PERCENT    |
  | PB4 | IO_1             |
  | PB5 | SPI_MOSI_IO_2    |
  | PB6 | SPI_MISO         |
  | PB7 | SPI_SCK          |
  *-----+------------------*/
  
#define LED_100PERCENT 0
#define LED_CHARGING   1
#define LED_60PERCENT  2
#define LED_30PERCENT  3
#define IO_1           4
#define SPI_MOSI_IO_2  5
#define SPI_MISO       6
#define SPI_SCK        7

#define LED_CHARGING_ON()    (PORTB &= ~(1<<LED_CHARGING))
#define LED_CHARGING_OFF()   (PORTB |= (1<<LED_CHARGING))

#define LED_100PERCENT_ON()  (PORTB &= ~(1<<LED_100PERCENT))
#define LED_100PERCENT_OFF() (PORTB |= (1<<LED_100PERCENT))

#define LED_60PERCENT_ON()   (PORTB &= ~(1<<LED_60PERCENT))
#define LED_60PERCENT_OFF()  (PORTB |= (1<<LED_60PERCENT))

#define LED_30PERCENT_ON()   (PORTB &= ~(1<<LED_30PERCENT))
#define LED_30PERCENT_OFF()  (PORTB |= (1<<LED_30PERCENT))


 /** @section Port C */
 
 /*-----+------------------*
  | PC0 | I2C_SCL          |
  | PC1 | I2C_SDA          |
  | PC2 | SIM900_STATUS    |
  | PC3 | POWERKEY         |
  | PC4 | RELAY_CLOSE      |
  | PC5 | RELAY_OPEN       |
  | PC6 | LED_NOTPAYED     |
  | PC7 | LED_SHORTFAULT   |
  *-----+------------------*/
  
#define I2C_SCL        0
#define I2C_SDA        1
#define SIM900_STATUS  2
#define POWERKEY       3
#define RELAY_CLOSE    4
#define RELAY_OPEN     5
#define LED_NOTPAYED   6
#define LED_SHORTFAULT 7

#define LED_NOTPAYED_ON()    (PORTC &= ~(1<<LED_NOTPAYED))
#define LED_NOTPAYED_OFF()   (PORTC |= (1<<LED_NOTPAYED))

#define LED_SHORTFAULT_ON()  (PORTC &= ~(1<<LED_SHORTFAULT))
#define LED_SHORTFAULT_OFF() (PORTC |= (1<<LED_SHORTFAULT))

#define LOAD_ON() {PORTC|=(1<<RELAY_CLOSE); delay_ms(200); PORTC&=~(1<<RELAY_CLOSE); sysval.loadON |=0x80; }
#define LOAD_OFF() {PORTC|=(1<<RELAY_OPEN); delay_ms(200); PORTC&=~(1<<RELAY_OPEN); sysval.loadON &=0x7f; }



 /** @section Port D */
 
 /*-----+------------------*
  | PD0 | SERIAL_RXD       |
  | PD1 | SERIAL_TXD       |
  | PD2 | SIM900_RXD       |
  | PD3 | SIM900_TXD       |
  | PD4 | SIM900_RI        |
  | PD5 | SIM900_DCD       |
  | PD6 | SIM900_CTS       |
  | PD7 | SIM900_RTS       |
  *-----+------------------*/
  
#define SERIAL_RXD  0
#define SERIAL_TXD  1
#define SIM900_RXD  2
#define SIM900_TXD  3
#define SIM900_RI   4
#define SIM900_DCD  5
#define SIM900_CTS  6
#define SIM900_RTS  7

 
#endif // _IO_H
