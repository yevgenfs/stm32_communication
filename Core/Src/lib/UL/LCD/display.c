/*
 * lcd.c
 *
 *  Created on: May 6, 2023
 *      Author: yevhen.surkov
 */

#include "display.h"
#include "../../drivers/lcd/lcd.h"
#include "stdint.h"

void display_init(void)
{
  lcd_init();
}

void display_run(i2c_t* objP_this)
{
  lcd_handler(objP_this);
}

void display_write(void)
{
  char *text  = "EmbeddedTher3";
  char *text2 = "EmbeddedTher2";
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_write_string(text);
  lcd_set_cursor(1, 0);
  lcd_write_string(text2);
}
