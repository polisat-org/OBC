#include "lcd.h"

void i2c_write_byte(uint8_t addr, uint8_t data) {
	HAL_I2C_Master_Transmit(&hi2c1, addr << 1, &data, 1, HAL_MAX_DELAY);
}

void lcd_send_nibble(uint8_t nibble, uint8_t rs) {
	uint8_t data = nibble << 4;
	if (rs) data |= LCD_RS;
	data |= LCD_BACKLIGHT;

	i2c_write_byte(LCD_ADDR, data | LCD_E);
	HAL_Delay(1);
	i2c_write_byte(LCD_ADDR, data);
	HAL_Delay(1);
}

void lcd_send_byte(uint8_t byte, uint8_t rs) {
	lcd_send_nibble(byte >> 4, rs);
	HAL_Delay(1);
	lcd_send_nibble(byte & 0x0F, rs);
	HAL_Delay(1);
}

void lcd_init(void) {
	HAL_Delay(50);

	i2c_write_byte(LCD_ADDR, LCD_NOBACKLIGHT);
	HAL_Delay(1000);

	lcd_send_nibble(0x03, 0);
	HAL_Delay(5);

	lcd_send_nibble(0x03, 0);
	HAL_Delay(5);

	lcd_send_nibble(0x03, 0);
	HAL_Delay(5);

	lcd_send_nibble(0x02, 0);
	HAL_Delay(5);

	uint8_t displayFunc = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;

	lcd_send_byte(LCD_FUNCTIONSET | displayFunc, 0);
	lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF, 0);
	lcd_send_byte(LCD_CLEARDISPLAY, 0);
	HAL_Delay(2);
	lcd_send_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT, 0);
	lcd_send_byte(LCD_RETURNHOME, 0);
	HAL_Delay(2);
}

void lcd_print(char* s) {
	while (*s) lcd_send_byte(*s++, 1);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
	uint8_t addr;
	if (row == 0) addr = col;
	else addr = 0x40 + col;

	lcd_send_byte(0x80 | addr, 0);
}

void lcd_clear(void) {
	lcd_send_byte(LCD_CLEARDISPLAY, 0);
	HAL_Delay(2);
}
