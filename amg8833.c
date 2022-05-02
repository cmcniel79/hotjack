/******************************************************************************
* File Name:   amg8833.c
*******************************************************************************
* Description: contains functions for performing the following tasks for the AMG8833 thermal sensor breakout board from Adafruit:
*			   - reading the thermistor data from the on-board Grid-EYE sensor.
*			   - reading the 8x8 temperature grid data from the Grid-Eye sensor.
*
* Related Document: See README.md */

#include <stdint.h>
#include "amg8833.h"

// Global Variables
uint8_t pixel_regs[127]; // declares the array of 128 register address for all of the lower and upper registers of each pixel

/*******************************************************************************
* Function Name: amg833_i2c_thermistor_read
********************************************************************************
* Summary:
*  Reads the thermistor data from the Grid-EYE sensor
*
* Parameters:
*  void
*
* Return:
*  float value of the thermistor reading in degrees C
*
*******************************************************************************/
float amg8833_i2c_thermistor_read(void) {

	// Reading data from the lower TTHL thermistor register in the Grid-EYE
    I2CReadByte(AMG8833_ADDRESS, THERMISTOR_TTHL, therm_buffer);
	uint8_t thermistor_tthl = therm_buffer[0];

	// Reading data from the higher TTHH thermistor register in the Grid-EYE
    I2CReadByte(AMG8833_ADDRESS, THERMISTOR_TTHH, therm_buffer);
	uint8_t thermistor_tthh = therm_buffer[0];

	// Formatting the separate register readings into one thermistor reading
	uint16_t thermistor_temp = thermistor_tthh << 8 | thermistor_tthl; // combining the lower and higher thermistor readings (minus the sign bit)
	int sign = thermistor_temp >> 11; // pick up the sign value of thermistor reading (12th bit)
		// Determining the sign value of the temperature reading
		if (sign == 1) {
			sign = -1;
		}
		else {
			sign = 1;
		}

	// Performing the final conversion and formatting into a degree C temperature value
	thermistor_temp = THERMISTOR_MASK & thermistor_temp; // removing the sign bit (12th bit)
	float thermistor_temp_final = sign * thermistor_temp * 0.0625; // converting the binary / decimal thermistor reading to a temperature value in degrees C

	return thermistor_temp_final;
}


/*******************************************************************************
* Function Name: amg8833_i2c_frame_rate_set
********************************************************************************
* Summary:
*  Function to set the frame rate and the read back the successfully set frame rate
*
* Parameters:
*  integer value of the frame rate
*
* Return:
*  integer value of the frame rate that was successfully set
*
*******************************************************************************/
int amg8833_i2c_frame_rate_set(uint8_t rate) {

	// Set the frame rate
    I2CWriteByte(AMG8833_ADDRESS, FRAME_RATE, rate);

	// Read data from the frame rate register
    I2CReadByte(AMG8833_ADDRESS, FRAME_RATE, therm_buffer);

	// Return the frame rate value
	return therm_buffer[0];
}

/*******************************************************************************
* Function Name: amg8833_i2c_config_registers
********************************************************************************
* Summary:
* Get the array ready for the pixel registers
*
* Parameters:
*  void
*
* Return:
*  float
*
*******************************************************************************/
void amg8833_i2c_config_registers(void) {
	// Generates the complete register addresses for all 64 pixels
	pixel_regs[0] = PIXEL_START; // sets the start register address for the pixel register array "pixel_regs"
	for(uint8_t i = 1; i < 128; i++) {
		pixel_regs[i] = pixel_regs[i-1] + 1;
	}
}

/*******************************************************************************
* Function Name: amg8833_i2c_8x8_read
********************************************************************************
* Summary:
*  An array of 64 temperature readings representing the 64x pixels on the 8x8
*  grid
*
* Parameters:
*  void
*
* Return:
*  float
*
*******************************************************************************/
void amg8833_i2c_8x8_read(float *value) {

	// Performing the combination of lower and upper registers for all 64 pixels and storing them in an array
	for(uint8_t n = 0; n < 63; n++) {
		uint8_t pixel_lower_reg = pixel_regs[2 * n]; // updates the address variable for the lower register
		uint8_t pixel_upper_reg = pixel_regs[2 * n + 1]; // updates the address variable for the upper register

		uint8_t pixel_lower;
		uint8_t pixel_upper;

		// Read data from the lower register of a given pixel
    	I2CReadByte(AMG8833_ADDRESS, pixel_lower_reg, therm_buffer);
		pixel_lower = therm_buffer[0];

		// Read data from the upper register of a given pixel
		I2CReadByte(AMG8833_ADDRESS, pixel_upper_reg, therm_buffer);
		pixel_upper = therm_buffer[0];

		// Format the separate register readings into one pixel temperature reading reading
		uint16_t pixel_temp_combined = pixel_upper << 8 | pixel_lower; // combining the lower and upper pixel temperature readings (minus the sign bit)

		// Performing the final conversion and formatting into a degree C temperature value and assigning the sign value
		// Determining the sign value of the temperature reading
		uint16_t sign = pixel_temp_combined >> 11; // pick up the sign value of pixel temperature reading (12th bit)
		if (sign == 1) {
			sign = -1;
		} else {
			sign = 1;
		}

		float pixel_temp_grid = PIXEL_MASK & pixel_temp_combined; // removing the sign bit (12th bit)

		// IMPORTANT!! Took out the .25 factor to speed up sensor measurements
		// The .25 factor can be multiplied back on the labview side
		pixel_temp_grid = sign * pixel_temp_grid; // converting the binary / decimal pixel temperature reading to a temperature value in degrees C
		
		/* Send the float value back to calling function */
		*value++ = pixel_temp_grid;
		// Below is for debug
		// printf("Pixel %d Temperature Reading (deg C): %.2f\r\n", n+1, pixel_temp_grid); // prints out the individual pixel temperature readings
	}
}
