/*
 * bidirec_com.c
 *
 * Created: 4/18/2025 4:03:03 PM
 * Author : userESD
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#define F_CPU 4000000UL
#define BAUD_RATE 115200
#define USART1_BAUD ((uint16_t)((float)(F_CPU * 64) / (16.0f * BAUD_RATE) + 0.5f))

#define BENCH_NUM 7
#define PC_ADDRESS (BENCH_NUM + 30)  // Destination LW2 address (PC side)

#define RX_BUFFER_SIZE 80
#define TX_BUFFER_SIZE 64

// === RX and TX Buffers ===
char rxU1_buff[RX_BUFFER_SIZE] = {0};  // Receive from LW1
char *rxU1_ptr = rxU1_buff;
uint8_t rxU1_index = 0;
uint8_t rxU1_buff_dav = 0;             // Data Available Flag

char txU1_buff[TX_BUFFER_SIZE] = {0};  // Transmit to LW1
char *txU1_ptr = txU1_buff;
uint8_t txU1_index = 0;
uint8_t txU1_buff_dav = 0;             // Transmission ready flag

// === Parsed Message Fields ===
char RCV_preamble[10];
volatile uint16_t txmtr_address;
volatile uint16_t rcv_data_len;
char payload[40];                      // Payload from message
volatile int16_t RSSI;
volatile int16_t SNR;
uint8_t payload_index = 0;

void USART1_init(void);
void parse_rxU1_buff(void);

int main(void) {
USART1_init();
sei();  // Enable global interrupts

while (1) {
if (rxU1_buff_dav) {
parse_rxU1_buff();  // Parse and prepare AT+SEND reply
}
}
}

// === USART1 Initialization ===
void USART1_init(void) {
 PORTC.DIR |= PIN0_bm;  // PC0 = TX1
 PORTC.DIR &= ~ PIN1_bm;  // PC1 = RX1
 PORTC.OUTSET = PIN0_bm;  // PB0 = TX1
USART1.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
USART1.CTRLC = USART_CHSIZE_8BIT_gc;
USART1.BAUD = USART1_BAUD;
USART1.CTRLA = USART_RXCIE_bm;  // Enable RX interrupt
}

// === USART1 RX ISR: Receives characters into rxU1_buff ===
ISR(USART1_RXC_vect) {
char c = USART1.RXDATAL;

if (rxU1_index < RX_BUFFER_SIZE - 1) {
rxU1_buff[rxU1_index++] = c;

if (c == '\n') {
rxU1_buff[rxU1_index] = '\0';  // Null terminate
rxU1_buff_dav = 1;             // Message complete
rxU1_index = 0;
USART1.CTRLA &= ~USART_RXCIE_bm;  // Disable RX interrupt until processed
}
} else {
rxU1_index = 0;  // Prevent overflow
}
}

// === USART1 TX ISR: Transmit txU1_buff one byte at a time ===
ISR(USART1_DRE_vect) {
	if (txU1_buff_dav && txU1_buff[txU1_index] != '\0') {
		USART1.TXDATAL = txU1_buff[txU1_index++];
		} else {
		USART1.CTRLA &= ~USART_DREIE_bm;  // Disable TX interrupt when done
		txU1_buff_dav = 0;
	}
}



// === Parse incoming +RCV=... and construct AT+SEND=... ===
// === Parse received +RCV=... message and prepare AT+SEND reply ===
void parse_rxU1_buff(void) {
	// Parse LoRa message: +RCV=<addr>,<len>,<payload>,<RSSI>,<SNR>
	int parsed = sscanf(rxU1_buff, "%[^=]=%u,%u,%[^,],%d,%d",
	RCV_preamble, &txmtr_address, &rcv_data_len,
	payload, &RSSI, &SNR);

	// Clear RX flag and buffer
	rxU1_buff_dav = 0;
	memset(rxU1_buff, 0, sizeof(rxU1_buff));

	// Validate message and fields
	if (parsed == 6 &&
	strncmp(RCV_preamble, "+RCV", 4) == 0 &&
	txmtr_address != 0 &&
	rcv_data_len != 0 &&
	payload[0] != '\0') {

		// Format response: AT+SEND=<dest>,<length>,<payload>\r\n
		snprintf(txU1_buff, sizeof(txU1_buff),
		"AT+SEND=%d,%d,%s\r\n", BENCH_NUM, rcv_data_len, payload);

		// Set flags and reset TX index
		txU1_buff_dav = 1;
		txU1_index = 0;

		// Manually send first byte to kickstart TX
		while (!(USART1.STATUS & USART_DREIF_bm));
		USART1.TXDATAL = txU1_buff[txU1_index++];

		// Now enable interrupt for remaining characters
		USART1.CTRLA |= USART_DREIE_bm;
	}

	// Re-enable RX so new messages can be received
	USART1.CTRLA |= USART_RXCIE_bm;
}