
/*
 * USI TWI definitions:
 */
// I2C (TWI) commands
#define I2C_CMD_READADC      0xa5
#define I2C_CMD_READSTAT     0xa6
#define I2C_CMD_STAT_WDT     0xa7
#define I2C_CMD_RESET        0xaa
#define I2C_CMD_CALTEMP      0xab
#define I2C_CMD_READBATT     0xac
#define I2C_CMD_EQUALIZE     0xad
#define I2C_CMD_CALBATT      0xae
#define I2C_CMD_SETAHBATT    0xaf

////////////////////////////////////////////////////////////////
/////////////// Driver Buffer Definitions //////////////////////
////////////////////////////////////////////////////////////////
// 1,2,4,8,16,32,64,128 or 256 bytes are allowed buffer sizes

#define TWI_RX_BUFFER_SIZE  (8)
#define TWI_RX_BUFFER_MASK ( TWI_RX_BUFFER_SIZE - 1 )
#define TWI_TX_BUFFER_SIZE  (16) 
#define TWI_TX_BUFFER_MASK ( TWI_TX_BUFFER_SIZE - 1 )

#if ( TWI_RX_BUFFER_SIZE & TWI_RX_BUFFER_MASK )
    #error TWI RX buffer size is not a power of 2
#endif
#if ( TWI_TX_BUFFER_SIZE & TWI_TX_BUFFER_MASK )
    #error TWI TX buffer size is not a power of 2
#endif

#define USI_SLAVE_CHECK_ADDRESS                (0x00)
#define USI_SLAVE_SEND_DATA                    (0x01)
#define USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA (0x02)
#define USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA   (0x03)
#define USI_SLAVE_REQUEST_DATA                 (0x04)
#define USI_SLAVE_GET_DATA_AND_SEND_ACK        (0x05)

#define DDR_USI             DDRB
#define PORT_USI            PORTB
#define PIN_USI             PINB
#define PORT_USI_SDA        PB0
#define PORT_USI_SCL        PB2
#define PIN_USI_SDA         PINB0
#define PIN_USI_SCL         PINB2
#define USI_START_COND_INT  USISIF

/*
 * These are the macros for USI-TWI:
 */
#define SET_USI_TO_SEND_ACK( ) \
{ \
   /* prepare ACK */ \
   USIDR = 0; \
   /* set SDA as output */ \
   DDR_USI |= ( 1 << PORT_USI_SDA ); \
   /* clear all interrupt flags, except Start Cond */ \
   USISR = \
        ( 0 << USI_START_COND_INT ) | \
        ( 1 << USIOIF ) | ( 1 << USIPF ) | \
        ( 1 << USIDC )| \
        /* set USI counter to shift 1 bit */ \
        ( 0x0E << USICNT0 ); \
}

#define SET_USI_TO_READ_ACK( ) \
{ \
   /* set SDA as input */ \
   DDR_USI &= ~( 1 << PORT_USI_SDA ); \
   /* prepare ACK */ \
   USIDR = 0; \
   /* clear all interrupt flags, except Start Cond */ \
   USISR = \
        ( 0 << USI_START_COND_INT ) | \
        ( 1 << USIOIF ) | \
        ( 1 << USIPF ) | \
        ( 1 << USIDC ) | \
        /* set USI counter to shift 1 bit */ \
        ( 0x0E << USICNT0 ); \
} 

#define SET_USI_TO_TWI_START_CONDITION_MODE( ) \
{ \
   USICR = \
        /* enable Start Condition Interrupt, disable Overflow Interrupt */ \
        ( 1 << USISIE ) | ( 0 << USIOIE ) | \
        /* set USI in Two-wire mode, no USI Counter overflow hold */ \
        ( 1 << USIWM1 ) | ( 0 << USIWM0 ) | \
        /* Shift Register Clock Source = External, positive edge */ \
        /* 4-Bit Counter Source = external, both edges */ \
        ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) | \
        /* no toggle clock-port pin */ \
        ( 0 << USITC ); \
   USISR = \
         /* clear all interrupt flags, except Start Cond */ \
         ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
         ( 1 << USIDC ) | ( 0x0 << USICNT0 ); \
}
#define SET_USI_TO_SEND_DATA( ) \
{ \
   /* set SDA as output */ \
   DDR_USI |=  ( 1 << PORT_USI_SDA ); \
   /* clear all interrupt flags, except Start Cond */ \
   USISR    =  \
        ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
        ( 1 << USIDC) | \
        /* set USI to shift out 8 bits */ \
        ( 0x0 << USICNT0 ); \
} 

#define SET_USI_TO_READ_DATA( ) \
{ \
   /* set SDA as input */ \
   DDR_USI &= ~( 1 << PORT_USI_SDA ); \
   /* clear all interrupt flags, except Start Cond */ \
   USISR    = \
        ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | \
        ( 1 << USIPF ) | ( 1 << USIDC ) | \
        /* set USI to shift out 8 bits */ \
        ( 0x0 << USICNT0 ); \
} 

