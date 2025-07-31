/*
 * scd41.h
 *
 * Created: 4/25/2025 4:39:12 PM
 *  Author: userESD
 */ 
/*
 * scd41.h
 *
 * Interface for communicating with the SCD41 CO?, temperature, and humidity sensor via TWI0.
 * Used in Lab 10 Task 3 for a modular multifile implementation.
 *
 * Author: Naafiul Hossain
 */

#ifndef SCD41_H
#define SCD41_H

#include <avr/io.h>
#include <stdint.h>

// === Public API ===
void init_twi0(void);
void scd41_start_periodic_measurement(void);
uint8_t scd41_is_data_ready(void);
void scd41_read_raw_measurements(uint16_t *co2, uint16_t *temp, uint16_t *hum);

#endif // SCD41_H