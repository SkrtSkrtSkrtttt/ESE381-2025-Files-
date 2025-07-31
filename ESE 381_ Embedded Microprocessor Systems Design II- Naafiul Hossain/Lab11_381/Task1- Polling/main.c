//Naafiul Hossain
//MAIN


#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "scd4x_i2c.h"
#include "sensirion_i2c_hal.h"
#include "lcd.h"

/*
// Global flag for conditional STOP after write
extern volatile bool send_stop_write;

int main(void) {
	// Init I2C for SCD41 sensor
	sensirion_i2c_hal_init();
	scd4x_init(0);  // 0 = default address or pressure, depending on your implementation

	// Init SPI for LCD 
	init_spi0_SerLCD();
	clear_display_buffs();

	// Start CO2 sensor measurements 
	scd4x_start_periodic_measurement();
	_delay_ms(5000);  // start-up time (idk why we chose to keep this long but the datasheet recommends this)

	// === Measurement variables ===
	bool data_ready = false;
	uint16_t co2 = 0;
	int32_t temperature = 0;
	int32_t humidity = 0;

	while (1) {
		// Set flag: no STOP between write-read sequence
		send_stop_write = false;
		scd4x_get_data_ready_status(&data_ready);

		if (data_ready) {
			// Read CO2, temperature, and humidity
			send_stop_write = true;  // Ensure STOP after read
			scd4x_read_measurement(&co2, &temperature, &humidity);

			// === Format values into LCD buffers ===
			snprintf(dsp_buff1, 21, "CO2: %u ppm", co2);
			snprintf(dsp_buff2, 21, "Temp: %ld.%02ld C", temperature / 1000, (temperature % 1000) / 10);
			snprintf(dsp_buff3, 21, "RH: %ld.%02ld %%", humidity / 1000, (humidity % 1000) / 10);
			snprintf(dsp_buff4, 21, "Air Monitor Ready");

			// === Send formatted data to LCD ===
			update_SerLCD();
		}

		// Small delay before next poll
		sensirion_i2c_hal_sleep_usec(5000); // 5ms
	}
}

*/

#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include <inttypes.h>  // PRIx64
#include <stdio.h>     // printf

#define sensirion_hal_sleep_us sensirion_i2c_hal_sleep_usec

void convert_and_print_serial(uint16_t* serial_raw) {
	uint64_t serial_as_int = 0;
	sensirion_common_to_integer((uint8_t*)serial_raw, (uint8_t*)&serial_as_int,
	LONG_INTEGER, 6);
	//printf("0x%" PRIx64, serial_as_int);
}

int main(void) {
	init_spi0_SerLCD();
	clear_display_buffs();
	
	int16_t error = NO_ERROR;
	sensirion_i2c_hal_init();
	scd4x_init(SCD41_I2C_ADDR_62);

	uint16_t serial_number[3] = {0};
	//_delay_ms(3);
	// Ensure sensor is in clean state
	error = scd4x_wake_up();
	if (error != NO_ERROR) {
		// printf("error executing wake_up(): %i\n", error);
	}

	error = scd4x_stop_periodic_measurement();
	if (error != NO_ERROR) {
		// printf("error executing stop_periodic_measurement(): %i\n", error);
	}

	error = scd4x_reinit();
	if (error != NO_ERROR) {
	//	printf("error executing reinit(): %i\n", error);
	}

	// Read out information about the sensor
	error = scd4x_get_serial_number(serial_number, 3);
	if (error != NO_ERROR) {
		// printf("error executing get_serial_number(): %i\n", error);
		return error;
	}
	printf("serial number: ");
	convert_and_print_serial(serial_number);
	printf("\n");
	//
	// If temperature offset and/or sensor altitude compensation
	// is required, you should call the respective functions here.
	// Check out the header file for the function definitions.
	// Start periodic measurements (5sec interval)
	error = scd4x_start_periodic_measurement();
	if (error != NO_ERROR) {
		// printf("error executing start_periodic_measurement(): %i\n", error);
		return error;
	}
	//
	// If low-power mode is required, switch to the low power
	// measurement function instead of the standard measurement
	// function above. Check out the header file for the definition.
	//
	bool data_ready = false;
	uint16_t co2_concentration = 0;
	int32_t temperature = 0;
	int32_t relative_humidity = 0;
	uint16_t repetition = 0;
	for (repetition = 0; repetition < 50; repetition++) {
		//
		// Slow down the sampling to 0.2Hz.
		//
	//	sensirion_hal_sleep_us(5000000);
		//
		// If ambient pressure compensation during measurement
		// is required, you should call the respective functions here.
		// Check out the header file for the function definition.
		error = scd4x_get_data_ready_status(&data_ready);
		if (error != NO_ERROR) {
			//printf("error executing get_data_ready_status(): %i\n", error);
			continue;
		}
		while (!data_ready) {
			sensirion_hal_sleep_us(100000);
			error = scd4x_get_data_ready_status(&data_ready);
			if (error != NO_ERROR) {
			//	printf("error executing get_data_ready_status(): %i\n", error);
				continue;
			}
		}
		error = scd4x_read_measurement(&co2_concentration, &temperature,
		&relative_humidity);
		if (error != NO_ERROR) {
			//printf("error executing read_measurement(): %i\n", error);
			continue;
		}
		//
		
		
		// Convert here
			volatile float temp_conv;
			temp_conv = -45.0 + 175.0 * ((float)temperature / 65535.0);

			volatile float hum_conv;
			hum_conv = 100.0 * ((float)relative_humidity / 65535.0);

			volatile uint16_t co2_ppm_conv;
			co2_ppm_conv = co2_concentration;  // optional if needed as float
		// === Send formatted data to LCD ===
		
		// Print results in physical units.
		sprintf(dsp_buff1,"CO2: %4u ppm", co2_ppm_conv);
		sprintf(dsp_buff2,"Temp: %.1f C", temp_conv);
		sprintf(dsp_buff3,"Humidity: %.1f%", hum_conv);
		
		
		update_SerLCD();
		
	}

	return NO_ERROR;
}