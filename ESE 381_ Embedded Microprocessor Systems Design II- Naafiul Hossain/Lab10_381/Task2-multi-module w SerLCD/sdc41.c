/*
 * scd41.c
 *
 * Implements SCD41 communication over TWI0 (I²C) for Lab 10 Task 3.
 * Handles setup, data polling, and measurement retrieval.
 *
 * Author: Naafiul Hossain
 */

#include "scd41.h"
#include <util/delay.h>

// I²C address and command constants
#define SCD41_I2C_ADDR      0x62
#define CMD_START_MEAS      0x21B1
#define CMD_DATA_READY      0xE4B8
#define CMD_READ_MEAS       0xEC05

/**
 * @brief Initializes TWI0 on alternate pins PA2 (SDA) and PA3 (SCL) for I²C communication.
 *        Sets baud rate to 400kHz assuming F_CPU is 4 MHz.
 */
void init_twi0(void) {
	PORTMUX.TWIROUTEA = PORTMUX_TWI0_ALT1_gc;     // Route TWI0 to PA2/PA3
	TWI0.MBAUD = 0x01;                             // 400kHz I²C baud for F_CPU = 4MHz
	TWI0.MCTRLA = TWI_ENABLE_bm;                   // Enable TWI master
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;           // Set bus state to idle
	PORTA.PIN2CTRL |=PORT_PULLUPEN_bm;
	PORTA.PIN3CTRL |=PORT_PULLUPEN_bm;
}


/**
 * @brief Sends a 16-bit command to the SCD41 sensor over I²C.
 * 
 * @param cmd The 16-bit command to be sent.
 */
void scd41_send_command(uint16_t cmd) {
	uint8_t cmd_high = (cmd >> 8);
	uint8_t cmd_low = (cmd & 0xFF);

	TWI0.MADDR = (SCD41_I2C_ADDR << 1);  // I²C write address
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MDATA = cmd_high;
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MDATA = cmd_low;
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MCTRLB = TWI_MCMD_STOP_gc; //stop
}

/**
 * @brief Starts periodic CO?, temperature, and humidity measurements.
 *        Includes a 5-second delay for sensor warm-up.
 */
void scd41_start_periodic_measurement(void) {
    scd41_send_command(CMD_START_MEAS);
    _delay_ms(5000);  // Warm-up time
}

/**
 * @brief Checks if new measurement data is ready from the SCD41 sensor.
 * 
 * @return uint8_t Returns non-zero if data is ready, 0 otherwise.
 */
/**
 * @brief Polls the sensor to check if new data is ready.
 * @return 1 if data is ready, 0 otherwise.
 */
uint8_t scd41_is_data_ready(void) {
    // === Write: CMD_DATA_READY (0xE4B8), no STOP ===
    uint8_t cmd_high = (CMD_DATA_READY >> 8);
    uint8_t cmd_low  = (CMD_DATA_READY & 0xFF);
uint8_t crc_byte;

volatile uint8_t status_high;
    TWI0.MADDR = (SCD41_I2C_ADDR << 1);  // Write mode
    while (!(TWI0.MSTATUS & TWI_WIF_bm));

TWI0.MDATA = cmd_high;
    while (!(TWI0.MSTATUS & TWI_WIF_bm));

TWI0.MDATA = cmd_low;
    while (!(TWI0.MSTATUS & TWI_WIF_bm));  // No STOP — keep bus active

    _delay_ms(2);  // Let sensor process command

    // === Read: 3 bytes (status high, status low, CRC) ===
    TWI0.MADDR = (SCD41_I2C_ADDR << 1) | 0x01;  // Read mode
    while (!(TWI0.MSTATUS & TWI_RIF_bm));

status_high = TWI0.MDATA;
TWI0.MCTRLB=0x02; //ACK
    while (!(TWI0.MSTATUS & TWI_RIF_bm));

uint8_t status_low  = TWI0.MDATA;
TWI0.MCTRLB=0x02; //ACK
    while (!(TWI0.MSTATUS & TWI_RIF_bm));

crc_byte = TWI0.MDATA;
//TWI0.MCTRLB=0X02; //ACK
// TWI0.MDATA;  // CRC (not used

    TWI0.MCTRLB = 0x07;  // Stop after complete read

    uint16_t status = ((uint16_t)status_high << 8) | status_low;
    return (status & 0x07FF);  // Return 1 if any of lower 11 bits are set
}

/**
 * @brief Reads raw CO?, temperature, and humidity values from the SCD41 sensor.
 *        Each measurement is 16-bit, received in MSB+LSB format.
 * 
 * @param co2 Pointer to variable to store CO? ppm.
 * @param temp Pointer to variable to store temperature raw value.
 * @param hum Pointer to variable to store humidity raw value.
 */
void scd41_read_raw_measurements(uint16_t *co2, uint16_t *temp, uint16_t *hum) {
	scd41_send_command(CMD_READ_MEAS);
	_delay_ms(1);  // Allow sensor to prepare data

	uint8_t crc_byte;

	TWI0.MADDR = (SCD41_I2C_ADDR << 1) | 0x01;
	// Read CO2
	while (!(TWI0.MSTATUS & TWI_RIF_bm));
	uint8_t co2_msb = TWI0.MDATA;
	TWI0.MCTRLB=0X02; //ACK

	while (!(TWI0.MSTATUS & TWI_RIF_bm));
	uint8_t co2_lsb = TWI0.MDATA;
	TWI0.MDATA;  // CRC (not used)
	TWI0.MCTRLB=0X02; //ACK

	while (!(TWI0.MSTATUS & TWI_RIF_bm));
	crc_byte = TWI0.MDATA;
	TWI0.MCTRLB=0X02; //ACK
	// TWI0.MDATA;  // CRC (not used)


	// Read temperature
	while (!(TWI0.MSTATUS & TWI_RIF_bm));
	uint8_t temp_msb = TWI0.MDATA;
	TWI0.MCTRLB=0X02; //ACK
	while (!(TWI0.MSTATUS & TWI_RIF_bm));
	uint8_t temp_lsb = TWI0.MDATA;
	TWI0.MDATA;  // CRC (not used)
	TWI0.MCTRLB=0X02; //ACK

	while (!(TWI0.MSTATUS & TWI_RIF_bm));
	crc_byte = TWI0.MDATA;
	TWI0.MCTRLB=0X02; //ACK
	// TWI0.MDATA;  // CRC (not used)


	// Read humidity
	while (!(TWI0.MSTATUS & TWI_RIF_bm));
	uint8_t hum_msb = TWI0.MDATA;
	TWI0.MCTRLB=0X02; //ACK
	while (!(TWI0.MSTATUS & TWI_RIF_bm));
	uint8_t hum_lsb = TWI0.MDATA;
	
	TWI0.MCTRLB=0X02; //ACK
	while (!(TWI0.MSTATUS & TWI_RIF_bm));
	crc_byte = TWI0.MDATA;
	//TWI0.MCTRLB=0X02; //ACK
	// TWI0.MDATA;  // CRC (not used)


	TWI0.MCTRLB = 0x07;  // Stop after complete read


	*co2  = ((uint16_t)co2_msb << 8) | co2_lsb;
	*temp = ((uint16_t)temp_msb << 8) | temp_lsb;
	*hum  = ((uint16_t)hum_msb << 8) | hum_lsb;


	/*temp = -45.0f + 175.0f * ((float)(*temp) / 65535.0f);
	*hum = 100.0f * ((float)(*hum) / 65535.0f) ;*/
	;

}
