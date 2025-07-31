/*
 * usart3_init_test.c
 *
 * Created: 4/9/2025 12:47:07 AM
 * Author : Naafiul Hossain
 */ 

#define F_CPU 4000000 // Frequency of the CPU in Hz
#define USART3_BAUD_RATE(BAUD_RATE) ((float)((F_CPU * 64 )/ (16 * (float)BAUD_RATE)) + .5) // Macro to calculate baud rate for USART

#include <avr/io.h> // Includes the definitions of the register names
#include <avr/interrupt.h> // Includes AVR interrupt definitions
#include <util/delay.h> // Includes functions for delays
#include <string.h> // Includes functions for string manipulation

volatile uint8_t cntrlcBM; // Variable to hold the USART control configuration mask

// Function to wait until the USART data register is empty (ready to transmit)
void waitTxReady(void)
{
	// Loop until the data register empty flag is set
	while (!(USART3.STATUS & USART_DREIF_bm))
	{
		asm volatile ("nop"); // Do nothing operation
	}
}

// Function to send a character over USART
void USART3_Send(char c)
{
	waitTxReady(); // Wait for the transmit buffer to be ready
	USART3.TXDATAL = c; // Put the character into the USART data register
}

// Function to initialize USART3
void USART3_Init(uint16_t baud, uint8_t data_bits, unsigned char parity )
{
	PORTB.DIR = 0x01; // Set the first pin of PORTB as output, others as input

	USART3.BAUD = (uint16_t)USART3_BAUD_RATE(baud); // Set the baud rate using the macro

	USART3.CTRLB = 0b11000000; // Enable receiver and transmitter

	cntrlcBM = 0x00; // Start with a default control mask of 0

	// Set the number of data bits
	switch(data_bits) {
		case 5:
		cntrlcBM |= 0x00; // lsb data bits
		break;
		case 6:
		cntrlcBM |= 0x01; // 6 data bits
		break;
		case 7:
		cntrlcBM |= 0x02; // 7 data bits
		break;
		case 8:
		cntrlcBM |= 0x03; // msb data bits
		break;
		default:
		cntrlcBM |= 0x00; // Default to 5 data bits if invalid selection
		break;
	}

	// Set parity mode
	switch(parity) {
		case 'D':
		cntrlcBM |= 0x00; // Disabled parity
		break;
		case 'E':
		cntrlcBM |= 0x20; // Even parity
		break;
		case 'O':
		cntrlcBM |= 0x30; // Odd parity
		break;
		default:
		cntrlcBM |= 0x00; // Default to no parity if invalid selection
		break;
	}
	
	USART3.CTRLC = cntrlcBM; // Set the control mask to the USART control register
}

int main(void)
{
	volatile char data = 'S'; // Data to send
	USART3_Init(9600, 8, 'E'); // Initialize USART with 9600 baud, 8 data bits, even parity
	
	while (1)
	{
		USART3_Send(data); // Send the data
		_delay_ms(50); // Wait for 50 milliseconds
	}
}
