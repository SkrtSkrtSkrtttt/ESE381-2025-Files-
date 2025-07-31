/**
 * @file lcd.c
 * @brief LCD display management for the temperature measurement system.
 *
 * This file contains all the functions necessary for initializing and managing
 * the LCD display via SPI communication, including sending data to the display,
 * clearing display buffers, and updating the display with new information.
 *
 * @author Naafiul Hossain
 * @date 2025-04-02
 */


#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <string.h>
//#define F_CPU 4000000UL  // Clock frequency for delay functions
#include <util/delay.h>

#include "lcd.h"


char dsp_buff1[21]; // Buffer for LCD display line 1 - Global variable, static storage, no linkage
char dsp_buff2[21]; // Buffer for LCD display line 2 - Global variable, static storage, no linkage
char dsp_buff3[21]; // Buffer for LCD display line 3 - Global variable, static storage, no linkage
char dsp_buff4[21]; // Buffer for LCD display line 4 - Global variable, static storage, no linkage

/**
 * @brief Initializes the SPI interface for LCD communication.
 *
 * Sets up the SPI0 hardware module for communication with the LCD using master mode.
 * Configures the direction of SPI pins and initializes SPI control registers.
 *
 * @code
 * init_spi0_SerLCD();
 * @endcode
 */

void init_spi0_SerLCD(void) {
	PORTA.DIRSET = PIN7_bm | PIN6_bm | PIN4_bm; // Set SPI pins as output
	PORTA.DIRCLR = PIN5_bm; // Set MISO pin as input
	SPI0.CTRLA = SPI_MASTER_bm | SPI_PRESC_DIV16_gc | SPI_ENABLE_bm; // Enable SPI, master mode
	SPI0.CTRLB = SPI_SSD_bm | SPI_MODE_0_gc; // Set SPI mode 0
}
/**
 * @brief Sends a byte of data to the LCD via SPI.
 *
 * This function transmits a single byte to the LCD using SPI communication,
 * ensuring the slave select line is appropriately managed before and after the transmission.
 *
 * @param data The data byte to be sent to the LCD.
 *
 * @code
 * write_spi0_SerLCD('H'); // Send character 'H' to the LCD
 * @endcode
 */
void write_spi0_SerLCD(unsigned char data) {
	select_SS();
	SPI0.DATA = data;
	while (!(SPI0.INTFLAGS & SPI_IF_bm));
	deselect_SS();
}

/**
 * @brief Selects the LCD as the SPI slave device.
 *
 * Activates the slave select (SS) line specific to the LCD to initiate SPI communication.
 *
 * @code
 * select_SS(); // Activate the LCD SS line before sending data
 * @endcode
 */
void select_SS(void) {
	PORTA.OUT &= ~PIN7_bm; // Select slave (active low)
}
/**
 * @brief Deselects the LCD as the SPI slave device.
 *
 * Deactivates the slave select (SS) line specific to the LCD to end SPI communication.
 *
 * @code
 * deselect_SS(); // Deactivate the LCD SS line after sending data
 * @endcode
 */
void deselect_SS(void) {
	PORTA.OUT |= PIN7_bm; // Deselect slave
}
/**
 * @brief Updates the content displayed on the LCD.
 *
 * Sends the contents of display buffers to the LCD via SPI, updating each line of the display.
 * This function should be called whenever the display data needs to be refreshed.
 *
 * @code
 * clear_display_buffs();   // Clear the display buffers
 * sprintf(dsp_buff1, "Temperature: %dC", temp); // Prepare line 1
 * sprintf(dsp_buff2, "Status: %s", status);     // Prepare line 2
 * update_SerLCD();         // Send the updated buffer to the LCD
 * @endcode
 */

void update_SerLCD(void) {
	write_spi0_SerLCD(0xFE);
	write_spi0_SerLCD(0x80); // First line
	for (uint8_t i = 0; i < 20; i++) {
		write_spi0_SerLCD(dsp_buff1[i]);
	}
_delay_ms(20);
	write_spi0_SerLCD(0xFE);
	write_spi0_SerLCD(0xC0); // Second line
	for (uint8_t i = 0; i < 20; i++) {
		write_spi0_SerLCD(dsp_buff2[i]);
	}
_delay_ms(20);
	write_spi0_SerLCD(0xFE);
	write_spi0_SerLCD(0x94); // Third line
	for (uint8_t i = 0; i < 20; i++) {
		write_spi0_SerLCD(dsp_buff3[i]);
	}
_delay_ms(20);
	write_spi0_SerLCD(0xFE);
	write_spi0_SerLCD(0xD4); // Fourth line
	for (uint8_t i = 0; i < 20; i++) {
		write_spi0_SerLCD(dsp_buff4[i]);
	}
}
/**
 * @brief Clears the display buffers.
 *
 * Fills the display buffer arrays with spaces to clear previous content,
 * and sets the end character to null to properly terminate the string for display.
 *
 * @code
 * clear_display_buffs(); // Clear the display buffers before writing new content
 * @endcode
 */

void clear_display_buffs(void) {
	memset(dsp_buff1, ' ', 20);
	dsp_buff1[20] = '\0';
	memset(dsp_buff2, ' ', 20);
	dsp_buff2[20] = '\0';
	memset(dsp_buff3, ' ', 20);
	dsp_buff3[20] = '\0';
	memset(dsp_buff4, ' ', 20);
	dsp_buff4[20] = '\0';
}