#include <stdint.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "resource_map.h"

/***************************************
 *            Constants
 ****************************************/
#define PACKET_SOP_POS (0UL)

/* I2C bus frequency */
#define I2C_FREQ (400000UL)

/***************************************
 *          Global Variables
 ****************************************/
cyhal_i2c_t mI2C;

/*******************************************************************************
 * Function Name: handle_error
 ********************************************************************************
 * Summary:
 * User defined error handling function
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void handle_error(void)
{
	/* Disable all interrupts. */
	__disable_irq();
	CY_ASSERT(0);
	printf("An Error Occurred With an I2C Function\r\n\n");
}

void I2C_init()
{
	cy_rslt_t result;

	/* PSOC Will be I2C Master */
	cyhal_i2c_cfg_t mI2C_cfg;

	printf(">> Configuring I2C..... ");
	mI2C_cfg.is_slave = false;
	mI2C_cfg.address = 0;
	mI2C_cfg.frequencyhal_hz = I2C_FREQ;

	/* Initialize */
	if (cyhal_i2c_init(&mI2C, mI2C_SDA, mI2C_SCL, NULL) != CY_RSLT_SUCCESS)
	{
		handle_error();
	}

	/* Configure */
	if (cyhal_i2c_configure(&mI2C, &mI2C_cfg) != CY_RSLT_SUCCESS)
	{
		handle_error();
	}
	printf("Done with I2C Init\r\n");
}

/***************************************
 *          I2C Read Functions
 ****************************************/

void I2CReadBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *value)
{	
	/* Need to write a register to MPU6050 first 
		so that it knows what info to send back */
	uint8_t write_buffer_length = 1;
	uint8_t i2c_write_buffer[write_buffer_length];
	i2c_write_buffer[PACKET_SOP_POS] =  regAddr;
	
	/* Setup response packet settings */
	uint8_t read_buffer_length = length;
	uint8_t i2c_read_buffer[read_buffer_length];

	/* Write the our specified register to MPU6050. 
	Need to make sure send_stop flag is set to 
	false since we're reading right after */
	if (CY_RSLT_SUCCESS == cyhal_i2c_master_write(&mI2C, slaveAddr, i2c_write_buffer, write_buffer_length, 0, false))
	{
		/* Read response packet from the slave. */
		if (CY_RSLT_SUCCESS == cyhal_i2c_master_read(&mI2C, slaveAddr, i2c_read_buffer, read_buffer_length, 0, true))
		{
				/* Send data back to calling function */
				uint8_t i = 0;
				while (i < length)
				{
					*value++ = i2c_read_buffer[i];
					i++;
				}
		}
		else
		{
			printf("Failed at I2CWriteBytes()\r\n\n");
		}
	}
}

void I2CReadByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t *value)
{
	I2CReadBytes(slaveAddr, regAddr, 1, value);
}

void I2CReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *value)
{
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	I2CReadByte(slaveAddr, regAddr, value);
	*value &= mask;
	*value >>= (bitStart - length + 1);
}

void I2CReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *value)
{
	I2CReadByte(slaveAddr, regAddr, value);
	*value = *value & (1 << bitNum);
}

/***************************************
 *          I2C Write Functions
 ****************************************/

void I2CWriteBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *value)
{
	uint8_t write_buffer_length = length + 1;
	uint8_t i2c_write_buffer[write_buffer_length];
	i2c_write_buffer[PACKET_SOP_POS] = regAddr;

	uint8_t i = 0;
	while (i < length)
	{
		i2c_write_buffer[1 + i] = *value++;
		i++;
	}
	if (cyhal_i2c_master_write(&mI2C, slaveAddr, i2c_write_buffer, write_buffer_length, 0, true) != CY_RSLT_SUCCESS)
	{
		printf("Failed at I2CWriteBytes()\r\n\n");
	}
}

void I2CWriteByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t value)
{
	I2CWriteBytes(slaveAddr, regAddr, 1, &value);
}

void I2CWriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t value)
{
	uint8_t b;
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	I2CReadByte(slaveAddr, regAddr, &b);
	value <<= (bitStart - length + 1);
	value &= mask;
	b &= ~(mask);
	b |= value;
	I2CWriteByte(slaveAddr, regAddr, b);
}

void I2CWriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t value)
{
	uint8_t b;
	I2CReadByte(slaveAddr, regAddr, &b);
	b = (value != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	I2CWriteByte(slaveAddr, regAddr, b);
}
