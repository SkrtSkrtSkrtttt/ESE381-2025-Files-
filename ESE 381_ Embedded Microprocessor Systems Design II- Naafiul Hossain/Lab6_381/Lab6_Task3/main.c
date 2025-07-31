/*
 * display_LM75_temp.c
 *
 * Created: 3/26/2025 12:41:18 AM
 * Author : Naafiul Hossain
 */ 

#include <avr/io.h>
#define F_CPU 4000000UL  // Clock frequency for delay functions
#include <util/delay.h>
#include <stdio.h>

#define LM75_ADDR 0x48     // LM75 I2C address
#define TEMPERATURE_ADDR 0x00  // Address of the temperature register in LM75

uint8_t temp_reg_high;	//high byte of LM75 Temperature register
uint8_t temp_reg_low;	//low byte of LM75 Temperature register
uint16_t LM75_temp_reg = 0;	//LM75 Temperature register contents
int16_t LM75_temp = 0;	//right adjusted 2's complement, resolution 0.1C
//char dsp_buff1[21];		//buffer for line 1 of LCD image

// SPI and LCD specific definitions
void init_spi0_SerLCD(void);
void write_spi0_SerLCD(unsigned char data);
void select_SS(void);
void deselect_SS(void);
void update_SerLCD(void);
void clear_display_buffs(void);

// I2C and LM75 specific definitions
void TWI0_LM75_init(void);
uint16_t TWI0_LM75_read(uint8_t saddr);
int TWI0_LM75_write(uint8_t saddr, uint8_t raddr, uint8_t data);



// Global display buffers
char dsp_buff1[21];
char dsp_buff2[21];
char dsp_buff3[21];
char dsp_buff4[21];

// Initialize SPI for SerLCD
void init_spi0_SerLCD(void) {
	PORTA.DIRSET = PIN7_bm | PIN6_bm | PIN4_bm;  // PA7: SS, PA6: SCK, PA4: MOSI
	PORTA.DIRCLR = PIN5_bm;                     // PA5: MISO
	SPI0.CTRLA = SPI_MASTER_bm | SPI_PRESC_DIV16_gc | SPI_ENABLE_bm; // SPI mode settings
	SPI0.CTRLB = SPI_SSD_bm | SPI_MODE_0_gc;    // More SPI settings
	SPI0.CTRLB &= ~SPI_BUFEN_bm;                // Normal unbuffered mode
}

// Initialize I2C for LM75
void TWI0_LM75_init(void) {
	TWI0_MBAUD = 0x01;
	TWI0_MCTRLA = 0x01;
	TWI0.MSTATUS = 0x01;  // Force bus state to idle
}

uint16_t TWI0_LM75_read(uint8_t saddr)
{
	while((TWI0.MSTATUS & 0x03) != 0x01) ;	// wait until idle
	
	TWI0.MADDR = ((saddr << 1) | 0x01);		// send slave address and read command
	
	while((TWI0.MSTATUS & 0x80) == 0);		// RIF flag, wait until byte is received
	// WIF flag does not work here
	temp_reg_high = TWI0.MDATA;				//clears the RIF flag
	
	TWI0.MCTRLB = 0x02;					//MCMD - issue ack followed by a byte read operation
	
	while((TWI0.MSTATUS & 0x80) == 0);		// RIF flag, wait until data received
	
	temp_reg_low = TWI0.MDATA;				//clears the RIF flag

	TWI0.MCTRLB = 0x07;					//MCMD issue nack followed by a stop
	return (uint16_t)((temp_reg_high << 8) | (temp_reg_low & 0x80));	//read data from received data buffer
}


int TWI0_LM75_write(uint8_t saddr, uint8_t raddr, uint8_t data) {
	while((TWI0.MSTATUS & 0x03) != 0x01) ; /* wait until idle */
	
	TWI0.MADDR = saddr << 1;         /* send address for write */
	while((TWI0.MSTATUS & 0x40) == 0); /* WIF flag, wait until saddr sent */
	
	//The next write clears the WIF flag
	TWI0.MDATA = raddr;              /* send memory address */
	while((TWI0.MSTATUS & 0x40) == 0); /* WIF flag, wait until raddr sent */
	
	//The next write clears the WIF flag
	TWI0.MDATA = data;               /* send data */
	while((TWI0.MSTATUS & 0x40) == 0); /* WIF flag, wait until data sent */
	
	//The next write clears the WIF flag
	TWI0.MCTRLB |= 0x03;          /* issue a stop */

	return 0;
}

// Main function where both devices are used
int main(void) {
	init_spi0_SerLCD();  // Initialize SPI for SerLCD
	TWI0_LM75_init();    // Initialize I2C for LM75

	clear_display_buffs();  // Clear display buffers for LCD

	while (1) {
		// Read temperature from LM75 using I2C
		uint16_t LM75_temp_reg = TWI0_LM75_read(LM75_ADDR);
		int16_t LM75_temp = ((int16_t)LM75_temp_reg) >> 7;
		sprintf(dsp_buff1, "Temp = %.1f C", ((float)(LM75_temp) / 2.0)); // Display temperature

		// Update SerLCD display with SPI
		sprintf(dsp_buff2, "More data 1");
		sprintf(dsp_buff3, "More data 2");
		sprintf(dsp_buff4, "More data 3");
		update_SerLCD();  // Send updated buffers to SerLCD via SPI

		_delay_ms(1000);  // Delay to slow down updates (optional)
	}
}

// Functions for SPI communication
void write_spi0_SerLCD(unsigned char data) {
	select_SS();
	SPI0.DATA = data;
	while (!(SPI0.INTFLAGS & SPI_IF_bm));
	deselect_SS();
}

void select_SS(void) { PORTA.OUT &= ~PIN7_bm; }
void deselect_SS(void) { PORTA.OUT |= PIN7_bm; }

void update_SerLCD(void) {
	write_spi0_SerLCD(0xFE);
	write_spi0_SerLCD(0x80); // First line
	for (uint8_t i = 0; i < 20; i++) {
		write_spi0_SerLCD(dsp_buff1[i]);
	}

	write_spi0_SerLCD(0xFE);
	write_spi0_SerLCD(0xC0); // Second line
	for (uint8_t i = 0; i < 20; i++) {
		write_spi0_SerLCD(dsp_buff2[i]);
	}

	write_spi0_SerLCD(0xFE);
	write_spi0_SerLCD(0x94); // Third line
	for (uint8_t i = 0; i < 20; i++) {
		write_spi0_SerLCD(dsp_buff3[i]);
	}

	write_spi0_SerLCD(0xFE);
	write_spi0_SerLCD(0xD4); // Fourth line
	for (uint8_t i = 0; i < 20; i++) {
		write_spi0_SerLCD(dsp_buff4[i]);
	}
}

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
