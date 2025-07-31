/**
 * @file main.c
 * @author Naafiul Hossain
 * @date 2025-03-30
 * @brief Main file for the temperature measurement system.
 *
 * @mainpage Description
 * This project involves a temperature measurement system using the LM75 temperature sensor.
 * The system reads temperature data and displays it on an LCD. It involves managing SPI and I2C
 * communication protocols.
 *
 * @section library_sec Libraries Used in this Project
 * @subsection library1 LM75 Library
 * This library handles interaction with the LM75 temperature sensor.
 *
 * @subsection library2 LCD Library
 * This library manages the LCD display to show temperature data.
 */


#include "lm75.h"
#include "lcd.h"
#include <util/delay.h>
#include <stdio.h>

#define F_CPU 4000000UL  // Clock frequency for delay functions
#define LM75_ADDR 0x48     // LM75 I2C address


int main(void) {
	init_spi0_SerLCD();
	TWI0_LM75_init();
	clear_display_buffs();

	while (1) {
		uint16_t LM75_temp_reg = TWI0_LM75_read(LM75_ADDR);
		int16_t LM75_temp_celsius = LM75_temp_reg >> 7;
		float temperature_celsius = LM75_temp_celsius / 2.0f;
		float temperature_fahrenheit = temperature_celsius * 9.0f / 5.0f + 32.0f;

		// Suppose you want to use temperature_fahrenheit, here's how you might log it:
		
		sprintf(dsp_buff1, "Temp = %.1f C", temperature_celsius); // Display temperature in Celsius
		sprintf(dsp_buff2, "Temp = %.1f F", temperature_fahrenheit); // Display temperature in Fahrenheit

		// Update LCD display with temperature data
		update_SerLCD();

		_delay_ms(1000); // Delay for readability
	}
	return 0;
}

