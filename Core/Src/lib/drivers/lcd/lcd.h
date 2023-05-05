/*
 * lcd.h
 *
 *  Created on: 4 трав. 2023 р.
 *      Author: yevhen.surkov
 */

#ifndef SRC_LIB_DRIVERS_LCD_LCD_H_
#define SRC_LIB_DRIVERS_LCD_LCD_H_

#include "stdint.h"
#include "../I2C/I2C.h"

void lcd_write_nibble(i2c_t* objPL_this, uint8_t nibble, uint8_t rs);

void lcd_send_cmd(i2c_t* objPL_this, uint8_t cmd);

void lcd_send_data(i2c_t* objPL_this, uint8_t data);

void lcd_init(i2c_t* objPL_this);

void lcd_write_string(i2c_t* objPL_this, char *str);

void lcd_set_cursor(i2c_t* objPL_this, uint8_t row, uint8_t column);

void lcd_clear(i2c_t* objPL_this);

void lcd_backlight(uint8_t state);

void lcd_display(i2c_t* objPL_this);

void lcd_handler(i2c_t* objP_this);


#endif /* SRC_LIB_DRIVERS_LCD_LCD_H_ */
