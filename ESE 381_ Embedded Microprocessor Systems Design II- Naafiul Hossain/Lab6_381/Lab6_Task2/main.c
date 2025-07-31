/*
 * read_LM75_temp_test.c
 *
 * Created: 3/23/2025 5:32:27 PM
 * Author : Naafiul Hossain
 */ 

#include <avr/io.h>
#define F_CPU 4000000	//Frequency used by delay macros.
#include <util/delay.h>	//Header for delay macros and functions.
#include <string.h>
#include <stdio.h>

#define LM75_ADDR 0x48     // LM75 address
#define TEMPERATURE_ADDR 0x00	//slave address of temperature register


uint8_t temp_reg_high;	//high byte of LM75 Temperature register
uint8_t temp_reg_low;	//low byte of LM75 Temperature register
uint16_t LM75_temp_reg = 0;	//LM75 Temperature register contents
int16_t LM75_temp = 0;	//right adjusted 2's complement, resolution 0.1C
char dsp_buff1[17];		//buffer for line 1 of LCD image
//function protypoes 

void TWI0_LM75_init(void);	//Initialize TWI0 module to talk to LM75
uint16_t TWIO_LM75_read(uint8_t saddr);
int TWI0_LM75_write(unsigned char saddr, unsigned char raddr, unsigned char data);

void TWI0_LM75_init(){
	TWI0_MBAUD=0x01;
	TWI0_MCTRLA=0X01;
	TWI0.DBGCTRL = 0x01;
	TWI0.MSTATUS = 0x01; //Force bus state to idle
	
	
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


int main(void)
{
   TWI0_LM75_init();
    while (1) 
    {
		while((TWI0.MSTATUS & 0x03) != 0x01) ; /* wait until I2C bus idle */
		LM75_temp_reg = TWI0_LM75_read(LM75_ADDR);
		LM75_temp = ((int16_t)LM75_temp_reg) >> 7;
		//		sprintf(dsp_buff1, "Temp = %4d", (LM75_temp >> 1)); //integer result
		//		sprintf(dsp_buff1, "Temp = %4d.%d", (LM75_temp >> 1),((LM75_temp%2) ? 5 : 0) ); //only for pos temps
		sprintf(dsp_buff1, "Temp = %.1f", ((float)(LM75_temp)/2.0)); //requires vprintf library
		//		_delay_ms(1000);
    }
}

