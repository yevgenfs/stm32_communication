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

void lcd_init(void);

void lcd_write_string(char *str);

void lcd_set_cursor(uint8_t row, uint8_t column);

void lcd_clear(void);

void lcd_backlight(uint8_t state);

void lcd_handler(i2c_t* objP_this);


#endif /* SRC_LIB_DRIVERS_LCD_LCD_H_ */
