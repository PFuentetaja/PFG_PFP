#ifndef I2C__H
#define I2C__H

#define MAX_SIZE_BUFFER 32

#define BMI160_I2C_ADDRESS  0x69
#define BME280_I2C_ADDR   	0x77
#define OPT3001_I2C_ADDR  	0x47
#define BMM150_I2C_ADDR    	0x13
#define TS_I2C_ADDR         0x38

extern int fd_bmi160;
extern int fd_bme280;
extern int fd_opt3001;
extern int fd_ts;
extern int fd_i2c;

int open_i2c_bus(uint8_t dev_addr);
int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len); // len en bytes
int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);


#endif
