/**
 * @file        lcd_md21605.c
 * @brief       MIDAS with ST7032i controller
 *
 * @details
 *
 * @author      Renaud Gaglione
 * @date        13 décembre 2024
 * @version     1.0.0
 *
 * @copyright   WTF
 *
 * @note
 * @warning     Check i2c speed and pullups.
 */
#include <lcd_st7032i.h>

I2C_HandleTypeDef _I2cHandler ;


void lcd_init(I2C_HandleTypeDef i2cHandler)
{
	_I2cHandler = i2cHandler ;
	// Init sequence from manual
	lcd_instruction(FUNC_SET|DL|N) ;
	WAIT(1) ;
	lcd_instruction(FUNC_SET|DL|N|IS) ;
	WAIT(1) ;
	lcd_instruction(SET_FREQ|0b100) ;	// 4 is ~183Hz@3V
	WAIT(1) ;
	lcd_set_contrast(0xC) ;
	WAIT(1) ;
	lcd_instruction(POW_ICON_CONTRAST|ION|BON|0b10) ; // contrast MSB do not change
	WAIT(1) ;
	lcd_instruction(FOLLOWER_CTRL|FON|0b011) ;
	WAIT(200) ;
	//lcd_instruction(DISPLAY_ON|D|C|B) ;
	lcd_instruction(DISPLAY_ON|D) ;
	WAIT(1) ;
	lcd_clear() ;
	lcd_instruction(ENTRY_MODE|I_D) ;
	WAIT(1) ;
}

void lcd_set_contrast(uint8_t data)
{	// LSByte sent first by HAL i2c
	lcd_instruction(CONTRAST|data) ;
}
void lcd_clear(void)
{
	lcd_instruction(CLEAR) ;
	WAIT(2) ;
}

void lcd_set_display(bool on_off, bool cursor_on_off, bool cursor_blink_on_off)
{	// based on lcd_instruction(DISPLAY_ON|D) ;
	lcd_instruction(DISPLAY_ON| on_off << D_BIT | cursor_on_off << C_BIT | cursor_blink_on_off << B_BIT) ;
}


void send_lcd_command(uint16_t data)
{
	HAL_I2C_Master_Transmit(&_I2cHandler, MD21605_I2C_ADDRESS, (uint8_t *)&data, sizeof(data), 1) ; //timeout in ms
}

void lcd_instruction(uint8_t inst)
{	// LSByte sent first by HAL i2c
	send_lcd_command(inst<<8|INSTRUCTION) ;
}
void lcd_data(uint8_t data)
{	// LSByte sent first by HAL i2c
	send_lcd_command(data << 8 | DATA_TO_RAM) ;
}


void lcd_reset(void)
{}

void lcd_put_cursor(uint8_t line, uint8_t col)	// BACKLIGHT 1
{	/*
	pos	  1  2  3  4  5 ... 39 40
	l1@	 00 01 02 03 04 ... 26 27 (hex)
	l2@  40 41 42 43 44 ... 66 67
	 */
	uint8_t data = line * 0x40 + col ;
	lcd_instruction(SET_DRAM_ADDR | data) ;
}


void lcd_write(char *str)
{	uint8_t i=0 ;
	while(str[i]!='\0')
	{
		lcd_data(str[i]) ; // not optimized : i2c stop after each char
		WAIT(1) ;
		i++;
	}
}












