//Naafiul Hossain
//Lab6Task1 SerLCDWRITE

#define F_CPU 4000000
#include <util/delay.h>
#include <avr/io.h>
#define SLAVE_ADDR 0x72 // the default address of the SerLCD
#include <stdio.h>
#include <string.h>


// Setting up display buffers for the SerLCD
char dsp_buff1[21];
char dsp_buff2[21];
char dsp_buff3[21];
char dsp_buff4[21];

char *dsp_buffs[4] = {dsp_buff1, dsp_buff2, dsp_buff3, dsp_buff4};

volatile uint8_t efficiency_mode = 0;  // global flag for efficiency

void init_twi0_SerLCD(){
	TWI0.MBAUD = 0x01; // this is the calculated baud value of 15
	TWI0.MCTRLA |= TWI_ENABLE_bm; // enable the TWI module
	TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc; // set the bus state to 'idle'
}

// function to send a character to the SerLCD
int write_twi0_SerLCD(uint8_t saddr, uint8_t data){
	// check if we need to send slave address again
	if(!efficiency_mode){
		// Send one address byte then the data byte
		TWI0.MADDR = (SLAVE_ADDR << 1); // sends the 7 bit slave address and the write command
		while ((TWI0.MSTATUS & TWI_WIF_bm) == 0); // wait for write to complete successfully
		if (TWI0.MSTATUS & TWI_RXACK_bm){ // checks if slave acknowledged the address
			TWI0.MCTRLB = TWI_MCMD_STOP_gc; // sends stop signal if no acknowledge
			return -1;
		}
		TWI0.MDATA = data; // writes the data to be send to LCD
		while ((TWI0.MSTATUS & TWI_WIF_bm) == 0); // wait for write to complete successfully
		if (TWI0.MSTATUS & TWI_RXACK_bm){ // checks if slave acknowledged the data
			TWI0.MCTRLB = TWI_MCMD_STOP_gc; // sends stop signal if no acknowledge
			return -1;
		}
		TWI0.MCTRLB = TWI_MCMD_STOP_gc; // sends stop signal
		_delay_us(500);
		return 0;
	}
	else {
		// efficiency mode, do not send slave address
		TWI0.MDATA = data;  // writes the data to be send to LCD
		while ((TWI0.MSTATUS & TWI_WIF_bm) == 0); // wait for write to complete successfully
		if (TWI0.MSTATUS & TWI_RXACK_bm){ // checks if slave acknowledged the data
			TWI0.MCTRLB = TWI_MCMD_STOP_gc; // sends stop signal
			return -1;
		}
		return 0;
	}
}

void update_SerLCD(void){
	efficiency_mode = 0;
	/*TWI0.MADDR = ((SLAVE_ADDR << 1) & 0xFE); // sends the 7 bit slave address and the write command
	//while (!(TWI0.MSTATUS & TWI_WIF_bm)); // wait for write to complete successfully
	if (TWI0.MSTATUS & TWI_RXACK_bm) // checks if slave acknowledged the data
	{
	TWI0.MCTRLB = TWI_MCMD_STOP_gc;
	efficiency_mode = 0;
	return;
	}*/
	// Send control bytes
	write_twi0_SerLCD(SLAVE_ADDR,'|');  // The saddr parameter is ignored in efficient mode.
	write_twi0_SerLCD(SLAVE_ADDR, '-');
	// loop through the buffers and send the characters
	for (uint8_t buffer = 0; buffer < 4; buffer++) {
		volatile uint8_t *ptr = (volatile uint8_t *)dsp_buffs[buffer];
		for (uint8_t i = 0; i < 20; i++) {
			//_delay_ms(10);
			write_twi0_SerLCD(0, *ptr++);
		}
	}
	TWI0.MCTRLB = TWI_MCMD_STOP_gc; // sends stop signal, ending transmission
}

int main(void)
{
	init_twi0_SerLCD(); // initialize the twi0 module for
	// fill the buffers
	sprintf(dsp_buff1, "Line 1: Hello, World");
	sprintf(dsp_buff2, "Line 2: AVR128DB48  ");
	sprintf(dsp_buff3, "Line 3: SerLCD Demo ");
	sprintf(dsp_buff4, "Line 4: TWI Success ");
	update_SerLCD();
	while (1)
	{
		//update_SerLCD();
		//_delay_ms(1000); // wait 1 second
	}
	return 0;
}

