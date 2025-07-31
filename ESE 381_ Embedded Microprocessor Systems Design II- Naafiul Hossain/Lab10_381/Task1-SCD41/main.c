/*
 * lab10_task2_scd41_read.c
 *
 * Task 2 – Read raw CO?, temperature, and humidity values from the SCD41 sensor
 * using TWI0 (I²C) on the AVR128DB48 and display them in Studio 7 watch window.
 *
 * Author: Naafiul Hossain
 * Date: 2025-04-21
 */

#include <avr/io.h>
#define F_CPU 4000000UL
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

// I²C address and command constants from the SCD41 datasheet
#define SCD41_I2C_ADDR      0x62  //7-bit I²C address of the sensor
#define CMD_START_MEAS      0x21B1 //Start periodic measurement
#define CMD_STOP            0x3f86
#define CMD_DATA_READY      0xE4B8 //Check if measurement data is ready
#define CMD_READ_MEAS       0xEC05 //Read the latest measured values

volatile uint16_t co2_ppm = 0;
volatile    uint16_t temp_raw = 0;
volatile    uint16_t hum_raw = 0;
/**
 * @brief Initializes TWI0 (I²C) on PA2 (SDA) and PA3 (SCL).
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
 * @brief Sends a 16-bit command to the SCD41 sensor via TWI0.
 * @param cmd 16-bit command to send.
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
 * @brief Polls the sensor to check if new data is ready.
 * @return 1 if data is ready, 0 otherwise.
 
 uint8_t scd41_is_data_ready(void) {
scd41_send_command(CMD_DATA_READY);
_delay_ms(2);  // Wait for sensor to process

TWI0.MADDR = (SCD41_I2C_ADDR << 1) | 0x01;  // I²C read address
while (!(TWI0.MSTATUS & TWI_RIF_bm));
uint8_t status_high = TWI0.MDATA;
while (!(TWI0.MSTATUS & TWI_RIF_bm));
uint8_t status_low = TWI0.MDATA;
while (!(TWI0.MSTATUS & TWI_RIF_bm));
uint8_t crc = TWI0.MDATA;  // Not used in this task

TWI0.MCTRLB = TWI_MCMD_STOP_gc;

uint16_t status = ((uint16_t)status_high << 8) | status_low;
return (status & 0x07FF);  // Check readiness flag in lower 11 bits
 }
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
 * @brief Reads CO2, temperature, and humidity values from the SCD41.
 * @param[out] co2  CO? value in ppm
 * @param[out] temp Raw temperature value (convert later)
 * @param[out] hum  Raw humidity value (convert later)
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

/**
 * @brief Main loop – initializes TWI and SCD41, then polls and reads data.
 */
int main(void) {
   

    init_twi0();
    _delay_ms(100);  // Let everything stabilize


//scd41_send_command(CMD_STOP);
    scd41_send_command(CMD_START_MEAS);
    //_delay_ms(5000);  // Allow sensor to warm up

    while (1) {
        if (scd41_is_data_ready()) {


            scd41_read_raw_measurements(&co2_ppm, &temp_raw, &hum_raw);
// Convert here
volatile float temp_conv;
temp_conv = -45.0 + 175.0 * ((float)temp_raw / 65535.0);

volatile float hum_conv;
hum_conv = 100.0 * ((float)hum_raw / 65535.0);

volatile float co2_ppm_conv;
co2_ppm_conv = (float)co2_ppm;  // optional if needed as float


            // You can watch co2_ppm, temp_raw, and hum_raw in Studio 7's watch window
        }

        _delay_ms(500);
    }
}
