#ifndef _I2C_H_
#define _I2C_H_

void i2c_init(int8_t onoff);
void i2c_reset(void);
int i2c_transact(const uint8_t i2c_slave, const uint8_t *cmd, const uint8_t cmd_len, uint8_t *data, const uint8_t data_len);

#endif //_I2C_H_


