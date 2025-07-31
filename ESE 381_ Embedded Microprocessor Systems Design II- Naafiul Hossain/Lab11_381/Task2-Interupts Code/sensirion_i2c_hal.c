#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "sensirion_config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define I2C_BUFFER_SIZE 32

// =============================
// Global array-based buffers & control flags
// =============================
volatile uint8_t i2c_tx_buffer[I2C_BUFFER_SIZE];
volatile uint8_t i2c_rx_buffer[I2C_BUFFER_SIZE];
volatile uint8_t i2c_tx_len = 0;
volatile uint8_t i2c_rx_len = 0;
volatile uint8_t i2c_tx_index = 0;
volatile uint8_t i2c_rx_index = 0;
volatile uint8_t i2c_done_flag = 0;
volatile uint8_t i2c_error = 0;
volatile bool i2c_send_stop = true;  // Conditional STOP control

// =============================
// I2C Bus Selection (optional)
// =============================
int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx) {
	return 0; // Only one bus used
}

// =============================
// Initialize I2C (TWI0)
// =============================
void sensirion_i2c_hal_init(void) {
	PORTMUX.TWIROUTEA = PORTMUX_TWI0_ALT1_gc;
	TWI0.MBAUD = 0x01; // 400kHz for F_CPU = 4MHz
	TWI0.MCTRLA = TWI_ENABLE_bm | TWI_WIEN_bm | TWI_RIEN_bm; // Enable TWI with interrupts
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;

	PORTA.PIN2CTRL |= PORT_PULLUPEN_bm; // SDA pull-up
	PORTA.PIN3CTRL |= PORT_PULLUPEN_bm; // SCL pull-up

	sei(); // Enable global interrupts
}

// =============================
// Release I2C
// =============================
void sensirion_i2c_hal_free(void) {
	TWI0.MCTRLA &= ~TWI_ENABLE_bm;
}

// =============================
// Write Transaction with Optional STOP
// =============================
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint8_t count) {
	if (count > I2C_BUFFER_SIZE) return -1;

	for (uint8_t i = 0; i < count; i++) {
		i2c_tx_buffer[i] = data[i];
	}

	i2c_tx_len = count;
	i2c_tx_index = 0;
	i2c_done_flag = 0;
	i2c_error = 0;

	TWI0.MADDR = (address << 1); // Start write

	while (!i2c_done_flag);

	if (i2c_send_stop) {
		TWI0.MCTRLB = TWI_MCMD_STOP_gc;
	}

	return i2c_error ? -1 : 0;
}

// =============================
// Read Transaction with STOP
// =============================
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
	if (count > I2C_BUFFER_SIZE) return -1;

	i2c_rx_len = count;
	i2c_rx_index = 0;
	i2c_done_flag = 0;
	i2c_error = 0;

	TWI0.MADDR = (address << 1) | 0x01; // Start read

	while (!i2c_done_flag);

	for (uint8_t i = 0; i < count; i++) {
		data[i] = i2c_rx_buffer[i];
	}

	return i2c_error ? -1 : 0;
}

// =============================
// Delay Function
// =============================
void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
	while (useconds--) _delay_us(1);
}

// =============================
// ISR for TWI0
// =============================
ISR(TWI0_TWIM_vect) {
	// Handle errors
	if (TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm)) {
		i2c_error = 1;
		i2c_done_flag = 1;
		TWI0.MCTRLB = TWI_MCMD_STOP_gc;
		return;
	}

	// Write operation
	if (TWI0.MSTATUS & TWI_WIF_bm) {
		if (i2c_tx_index < i2c_tx_len) {
			TWI0.MDATA = i2c_tx_buffer[i2c_tx_index++];
			} else {
			i2c_done_flag = 1; // No STOP yet, main code decides
		}
	}

	// Read operation
	if (TWI0.MSTATUS & TWI_RIF_bm) {
		if (i2c_rx_index < i2c_rx_len) {
			i2c_rx_buffer[i2c_rx_index++] = TWI0.MDATA;
			if (i2c_rx_index == i2c_rx_len) {
				TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc; // NACK + STOP
				i2c_done_flag = 1;
				} else {
				TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc; // ACK + continue
			}
		}
	}
}
