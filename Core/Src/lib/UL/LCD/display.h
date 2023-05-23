/*
 * lcd.h
 *
 *  Created on: May 6, 2023
 *      Author: yevhen.surkov
 */

#ifndef SRC_LIB_UL_LCD_DISPLAY_H_
#define SRC_LIB_UL_LCD_DISPLAY_H_

#include "../../drivers/I2C/I2C.h"

void display_init(void);

void display_write(void);

void display_run(i2c_t* objP_this);

#endif /* SRC_LIB_UL_LCD_DISPLAY_H_ */
