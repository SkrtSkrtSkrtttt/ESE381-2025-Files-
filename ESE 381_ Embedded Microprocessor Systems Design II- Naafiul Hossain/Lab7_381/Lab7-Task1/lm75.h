/*
 * lm75.h
 *
 * Created: 4/2/2025 12:47:55 AM
 *  Author: Naafiul Hossain
 */ 


#ifndef LM75_H
#define LM75_H

#include <stdint.h>
#include <stdio.h>

void TWI0_LM75_init(void);
uint16_t TWI0_LM75_read(uint8_t saddr);
int TWI0_LM75_write(uint8_t saddr, uint8_t raddr, uint8_t data);

#endif