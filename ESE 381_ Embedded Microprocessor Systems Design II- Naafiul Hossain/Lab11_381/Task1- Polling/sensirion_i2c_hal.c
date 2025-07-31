/*
 * sensirion_i2c.c
 *
 * Created: 4/27/2025 2:16:57 AM
 *  Author: Naafiul Hossain
 */ 
#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "sensirion_config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>  // for the STOP flag

// === Optional Global STOP control flag...idea is that it knows when to stop write 
volatile bool send_stop_write = true;

/**
 * Select I2C bus (not needed for single-bus setup accoridng to prof short )
 */
int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx) {
    return NOT_IMPLEMENTED_ERROR;
}

/**
 * Initialize TWI0 (I2C) for 400kHz on alternate PA2/PA3 pins
 */
void sensirion_i2c_hal_init(void) {
    PORTMUX.TWIROUTEA = PORTMUX_TWI0_ALT1_gc; // Use alternate SDA/SCL
    TWI0.MBAUD = 0x01;                        // 400kHz for 4MHz clock
    TWI0.MCTRLA = TWI_ENABLE_bm;              // Enable TWI
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;      // Mark bus as idle
    PORTA_PIN2CTRL |= PORT_PULLUPEN_bm;       // Enable pull-ups on SDA
    PORTA_PIN3CTRL |= PORT_PULLUPEN_bm;       // Enable pull-ups on SCL
}

/**
 * i think short said  (not required here)
 */
void sensirion_i2c_hal_free(void) {
    TWI0.MCTRLA &= ~TWI_ENABLE_bm;
}

/**
 * Read bytes over I2C (polling)
 */
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
    TWI0.MADDR = (address << 1) | 0x01; // Send read address

    for (uint8_t i = 0; i < count; i++) {
        while (!(TWI0.MSTATUS & TWI_RIF_bm)); // Wait for read flag

        data[i] = TWI0.MDATA; // Read incoming byte

        if (i < count - 1) {
            TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc; // ACK + continue
        } else {
            TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc; // NACK + STOP on last byte
			//TWIO.MCTRLB=0x02;
        }
    }

    return 0;
}

/**
 * Write bytes over I2C (polling)
 */
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint8_t count) {
    TWI0.MADDR = (address << 1); // Send write address
    while (!(TWI0.MSTATUS & TWI_WIF_bm)); // Wait for first write ready

    for (uint8_t i = 0; i < count; i++) {
        TWI0.MDATA = data[i]; // Send next byte
        while (!(TWI0.MSTATUS & TWI_WIF_bm)); // Wait for ACK/NACK

        // Check for arbitration loss or bus error
        if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm)) {
            TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            return -1;
        }

        // Check for NACK from slave
        if (TWI0.MSTATUS & TWI_RXACK_bm) {
            TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            return -1;
        }
    }

    // Only send STOP if allowed (for combined write-read sequences)
    if (send_stop_write) {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
    }

    return 0;
}

/**
 * Microsecond delay loop (per short instructions for lab precision)
 */
void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    while (useconds--) {
        _delay_us(1);
    }
}
