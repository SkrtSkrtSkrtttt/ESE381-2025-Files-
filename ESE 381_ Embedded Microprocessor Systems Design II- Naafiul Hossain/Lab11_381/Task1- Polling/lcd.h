/*
 * lcd.h
 *
 * Created: 4/2/2025 1:20:53 AM
 *  Author: Naafiul Hossain
 */ 
#ifndef LCD_H
#define LCD_H

 extern char dsp_buff1[21]; // Buffer for LCD display line 1 - Global variable, static storage, no linkage
 extern char dsp_buff2[21]; // Buffer for LCD display line 2 - Global variable, static storage, no linkage
 extern char dsp_buff3[21]; // Buffer for LCD display line 3 - Global variable, static storage, no linkage
 extern char dsp_buff4[21]; // Buffer for LCD display line 4 - Global variable, static storage, no linkage


void init_spi0_SerLCD(void);
void write_spi0_SerLCD(unsigned char data);
void update_SerLCD(void);
void clear_display_buffs(void);
void select_SS(void);  // Add this line
void deselect_SS(void);  // Add this line

#endif // LCD_H