/**
 * @file lm75.c
 * @brief Functions to interact with the LM75 temperature sensor via I2C.
 *
 * This file contains functions to initialize the I2C communication for the LM75 sensor,
 * read temperature data from it, and write configurations to it.
 *
 * @author Naafiul Hossain
 * @date 2025-04-02
 */

#include "lm75.h"
#include <avr/io.h>
/**
 * @brief Initializes the I2C interface for communication with the LM75 sensor.
 *
 * Sets the master baud rate and control register to prepare the I2C interface
 * for communication. It also forces the I2C bus state to idle to ensure a clean start.
 *
 * @code
 * TWI0_LM75_init(); // Initialize the I2C bus for LM75 communication
 * @endcode
 */

void TWI0_LM75_init(void) {
	TWI0_MBAUD = 0x01;
	TWI0_MCTRLA = 0x01;
	TWI0.MSTATUS = 0x01;  // Force bus state to idle
}
/**
 * @brief Reads the temperature data from the LM75 sensor.
 *
 * Sends the read command to the LM75 sensor and reads back two bytes of temperature data.
 * It manages the I2C bus state throughout the operation to ensure proper reception of data.
 *
 * @param saddr The slave address of the LM75 sensor.
 * @return The 16-bit raw temperature data read from the sensor.
 *
 * @code
 * uint16_t temperature = TWI0_LM75_read(LM75_ADDR);
 * @endcode
 */
uint16_t TWI0_LM75_read(uint8_t saddr) {
	uint8_t temp_reg_high, temp_reg_low;
	while((TWI0.MSTATUS & 0x03) != 0x01); // Wait until idle

	TWI0.MADDR = ((saddr << 1) | 0x01); // Send slave address and read command
	while((TWI0.MSTATUS & 0x80) == 0); // Wait for RIF flag, byte received
	temp_reg_high = TWI0.MDATA; // Clear the RIF flag

	TWI0.MCTRLB = 0x02; // Issue ACK followed by a byte read operation
	while((TWI0.MSTATUS & 0x80) == 0); // Wait for next byte
	temp_reg_low = TWI0.MDATA; // Clear the RIF flag

	TWI0.MCTRLB = 0x07; // Issue NACK followed by a stop
	return (uint16_t)((temp_reg_high << 8) | temp_reg_low);
}
/**
 * @brief Writes a data byte to a specific register of the LM75 sensor.
 *
 * This function is used to configure the LM75 sensor by writing to its registers.
 * It handles the entire write operation including sending the slave address, register address,
 * and the data byte, followed by issuing a stop condition.
 *
 * @param saddr The slave address of the LM75 sensor.
 * @param raddr The register address to write to.
 * @param data The data byte to write to the register.
 * @return Always returns 0 indicating success.
 *
 * @code
 * TWI0_LM75_write(LM75_ADDR, LM75_CONFIG_REGISTER, new_config_value);
 * @endcode
 */

int TWI0_LM75_write(uint8_t saddr, uint8_t raddr, uint8_t data) {
	while((TWI0.MSTATUS & 0x03) != 0x01); // Wait until idle

	TWI0.MADDR = saddr << 1; // Send address for write
	while((TWI0.MSTATUS & 0x40) == 0); // Wait until address sent

	TWI0.MDATA = raddr; // Send memory address
	while((TWI0.MSTATUS & 0x40) == 0); // Wait until memory address sent

	TWI0.MDATA = data; // Send data
	while((TWI0.MSTATUS & 0x40) == 0); // Wait until data sent

	TWI0.MCTRLB |= 0x03; // Issue a stop
	return 0;
}
