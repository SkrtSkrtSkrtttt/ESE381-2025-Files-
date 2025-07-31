/*
 * NHLab10Task3.c
 *
 * Created: 4/25/2025 4:37:12 PM
 * Author : userESD
 */ 


#define F_CPU 4000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "scd41.h"
#include "lcd.h"

int main(void) {
	uint16_t co2_ppm, temp_raw, hum_raw;

	init_twi0();
	init_spi0_SerLCD();
	clear_display_buffs();
	update_SerLCD();  // Clear screen on boot

	scd41_start_periodic_measurement();

	while (1) {
		//clear_display_buffs();

		if (scd41_is_data_ready()) {
			scd41_read_raw_measurements(&co2_ppm, &temp_raw, &hum_raw);

			// Convert here
			volatile float temp_conv;
			temp_conv = -45.0 + 175.0 * ((float)temp_raw / 65535.0);

			volatile float hum_conv;
			hum_conv = 100.0 * ((float)hum_raw / 65535.0);

			volatile uint16_t co2_ppm_conv;
			co2_ppm_conv = co2_ppm;  // optional if needed as float
			
			
			// Format values
			sprintf(dsp_buff1, "CO2: %4u ppm", co2_ppm_conv);
			sprintf(dsp_buff2, "Temp: %.1f C", temp_conv);
			sprintf(dsp_buff3, "Humidity: %.1f%%", hum_conv);
			} 

		update_SerLCD();
		_delay_ms(20);
	}
}