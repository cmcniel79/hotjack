/* Header file for AMG8833 sensor functions */

#ifndef AMG8833_H_
#define AMG8833_H_

#define AMG8833_ADDRESS 0x69
#define THERMISTOR_TTHL 0x0E // array of register address for the lower thermistor register TTHL
#define THERMISTOR_TTHH 0x0F // array of register address for the higher thermistor register TTHH
#define THERMISTOR_MASK 0x7FF // mask used to remove sign (12th bit) from 12-bit temperature reading
#define FRAME_RATE  0x02
#define PIXEL_MASK  0x7FF
#define PIXEL_START 0x80

void handle_error(void);
int amg8833_i2c_frame_rate_set(uint8_t rate);
float amg8833_i2c_thermistor_read(void);
void amg8833_i2c_config_registers(void);
void amg8833_i2c_8x8_read(float *pixels);

uint8_t therm_buffer[22];

// extern void I2CReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *value);
// extern void I2CReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *value);
// extern void I2CReadByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t *value);
// extern void I2CReadBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *value);
// extern void I2CWriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t value);
// extern void I2CWriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t value);
// extern void I2CWriteByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t value);
// extern void I2CWriteBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *value);
// extern void I2CWriteWord(uint8_t slaveAddr, uint8_t regAddr, uint16_t value);
// extern void I2CWriteWords(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint16_t *value);


#endif /* AMG8833_H_ */
