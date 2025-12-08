/**
 * @file        lcd_st7032i.h
 * @brief       ST7032i controller
 *
 * @details
 *
 * @author      Renaud Gaglione
 * @date        20241213
 * @version     1.0.0
 *
 * @copyright
 * 
 * @note        No CGRAM or ICON implemented yet
 * @warning     Check i2c speed and pullups.
 */
#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"
#include <stdbool.h>

//Only one 7-bit slave addresses (0111110) is reserved for the ST7032. The R/W is assigned to 0 for Write only.
#define MD21605_I2C_ADDRESS (0x3E<<1)
#define WAIT(time_ms) HAL_Delay(time_ms)
////////////////////////////////////////////////////////////////////////
// CONFIGURE screen bits
#define FUNC_SET	0x30	// function set bit
// DL : Interface data length control bit : 1:8b 0:4b (8b in ST7032i)
#define DL (1<<DL_BIT)
// N : Display line number control bit 1:2L 0:1L
#define N (1<<N_BIT)
// DH : Double height font type control bit 0:normal
#define DH (1<<DH_BIT)
// IS : normal/extension instruction select 1:extended 0:normal
#define IS (1<<IS_BIT)
// Bit position
#define DL_BIT 4 // Data format
#define N_BIT 3  // number of lines
#define DH_BIT 2 // character font
                 // bit 1 recerved
#define IS_BIT 0 // instruction set
////////////////////////////////////////////////////////////////////////
#define SET_FREQ	0x10
// BS : Bias 1:1/4 0:1/5
#define BS (1<<BS_BIT)
#define BS_BIT 3
////////////////////////////////////////////////////////////////////////
#define CONTRAST	0x70
////////////////////////////////////////////////////////////////////////
#define POW_ICON_CONTRAST	0x50
#define ION (1<<ION_BIT)	// ICON ON/OFF
#define BON (1<<BON_BIT)	// BOOSTER ON/OFF
#define ION_BIT	3
#define BON_BIT 2
////////////////////////////////////////////////////////////////////////
#define FOLLOWER_CTRL 0x60
#define FON (1<<FON_BIT)	// Internal follower ON
#define FON_BIT 3
////////////////////////////////////////////////////////////////////////
#define DISPLAY_ON 0x08
// D: display 1:on 0:off
#define D (1<<D_BIT)
// C: cursor 1:on 0:off
#define C (1<<C_BIT)
// B: cursor blink 1:on 0:off
#define B (1<<B_BIT)
// Bit position
#define D_BIT 2 // display on
#define C_BIT 1 // cursor on
#define B_BIT 0 // cursor blink on
////////////////////////////////////////////////////////////////////////
#define CLEAR	0x01
////////////////////////////////////////////////////////////////////////
#define HOME	0x02
////////////////////////////////////////////////////////////////////////
#define ENTRY_MODE	0x04
#define I_D (1<<I_D_BIT)	// 1:increment(right) 0:decrement(left)
#define S	(1<<S_BIT)	// 1: shift entire display 0:none
#define I_D_BIT 1
#define S_BIT	0
////////////////////////////////////////////////////////////////////////
#define SET_DRAM_ADDR	0x80
////////////////////////////////////////////////////////////////////////
// Command instruction in I2C
#define INSTRUCTION 0x00
#define DATA_TO_RAM 0x40
////////////////////////////////////////////////////////////////////////

//--------------- USER LCD FUNCTIONS ---------------
void lcd_init(I2C_HandleTypeDef i2cHandler) ;
void lcd_put_cursor(uint8_t line, uint8_t col);
void lcd_write(char *str);

void lcd_set_display(bool on_off, bool cursor_on_off, bool cursor_blink_on_off) ;

void lcd_set_contrast(uint8_t data) ;
void lcd_clear(void) ;
void lcd_reset(void) ;

//--------------- USER LCD FUNCTIONS ---------------

void send_lcd_command(uint16_t data) ;
void lcd_instruction(uint8_t inst) ;
void lcd_data(uint8_t data) ;


#endif /* INC_LCD_H_ */
