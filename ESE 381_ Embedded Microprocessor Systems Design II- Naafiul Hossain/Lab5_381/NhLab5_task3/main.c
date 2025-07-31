/*
 * read_LM74_SPI.c
 *
 * Created: 3/14/2025 4:40:59 PM
 * Author : userESD
 */

#include <avr/io.h>
#include <stdio.h>
#define F_CPU 4000000UL
#include <util/delay.h>

void init_spi0_LM74(void);
uint8_t read_spi0_LM74(void);
uint16_t read_word_LM74(void);
void write_spi0_SerLCD(unsigned char data);
void clear_display_buffs(void);
void update_SerLCD(void);
void select_SS_SerLCD(void);
void deselect_SS_SerLCD(void);
void select_SS_LM74(void);
void deselect_SS_LM74(void);

char dsp_buff1[21];
char dsp_buff2[21];
char dsp_buff3[21];
char dsp_buff4[21];

int main(void) {
init_spi0_LM74();
clear_display_buffs();
sprintf (dsp_buff1, "LM74 Binary Output. ");
update_SerLCD();
while(1) {
uint16_t temp_binary = read_word_LM74();
for (uint16_t i=0; i<16; i++) { // extract msb and store in buffer at a time
if (temp_binary & (1 << (15-i))) {
dsp_buff2[i] = '1';
}
else {
dsp_buff2[i] = '0';
}
}
dsp_buff2[16] = '\0';
update_SerLCD();
_delay_ms(100);
}
}

void init_spi0_LM74(void) {
PORTA.DIRSET = PIN7_bm | PIN6_bm | PIN4_bm; // PA7 for lcd ss, PA6 for SCK, PA4 for MOSI
PORTD.DIRSET = PIN6_bm; // PD6 for tempsensor ss
PORTA.DIRCLR = PIN5_bm; // PA5 for SPI0 MISO
SPI0.CTRLA = SPI_MASTER_bm | SPI_PRESC_DIV128_gc | SPI_ENABLE_bm; // enable SPI with master mode
SPI0.CTRLB = SPI_MODE_0_gc; // set up SPI mode 0
SPI0.CTRLB &= ~SPI_BUFEN_bm; // normal unbuffered mode
deselect_SS_LM74();
deselect_SS_SerLCD();
}

uint8_t read_spi0_LM74(void) {
SPI0_DATA = 0XFF;
while (!(SPI0.INTFLAGS & SPI_IF_bm)) {}
return SPI0_DATA;
}

uint16_t read_word_LM74(void) {
uint16_t temp_reading = 0x0000;
select_SS_LM74();
temp_reading = read_spi0_LM74() << 8;
temp_reading |= read_spi0_LM74();
deselect_SS_LM74();
return temp_reading;
}

void write_spi0_SerLCD(unsigned char data) {
select_SS_SerLCD();
SPI0.DATA = data; // writes data to SPI0_DATA register and clears IF after
while (!(SPI0.INTFLAGS & SPI_IF_bm)) {} // wait for transfer to complete
deselect_SS_SerLCD();
}

void clear_display_buffs(void) { // fills buffer with space characters
for (uint8_t i=0; i<20; i++) {
dsp_buff1[i] = ' ';
dsp_buff2[i] = ' ';
dsp_buff3[i] = ' ';
dsp_buff4[i] = ' ';
}

dsp_buff1[20] = '\0';
dsp_buff2[20] = '\0';
dsp_buff3[20] = '\0';
dsp_buff4[20] = '\0';
}

void update_SerLCD(void) {
write_spi0_SerLCD(0XFE); // lcd spi transmit command
write_spi0_SerLCD(0X80); // first line
for (uint8_t i=0; i<20; i++) {
write_spi0_SerLCD(dsp_buff1[i]);
}
//write_spi0_SerLCD(0XFE); // lcd spi transmit command
//write_spi0_SerLCD(0XC0); // second line
for (uint8_t i=0; i<20; i++) {
write_spi0_SerLCD(dsp_buff2[i]);
}
//write_spi0_SerLCD(0XFE); // lcd spi transmit command
//write_spi0_SerLCD(0X94); // third line
for (uint8_t i=0; i<20; i++) {
write_spi0_SerLCD(dsp_buff3[i]);
}
//write_spi0_SerLCD(0XFE); // lcd spi transmit command
//write_spi0_SerLCD(0XD4); // fourth line
for (uint8_t i=0; i<20; i++) {
write_spi0_SerLCD(dsp_buff4[i]);
}
}

void select_SS_SerLCD(void) {PORTA.OUT &= ~PIN7_bm;}
void deselect_SS_SerLCD(void) {PORTA.OUT |= PIN7_bm;}
void select_SS_LM74(void) {PORTD.OUT &= ~PIN6_bm;}

void deselect_SS_LM74(void) {PORTD.OUT |= PIN6_bm;}