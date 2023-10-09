/*
 * lcd.c
 *
 *  Created on: May 6, 2023
 *      Author: yevhen.surkov
 */

#include "display.h"
#include "../../drivers/lcd/lcd.h"
#include "stdint.h"
#include "../HTU21D/HTU21D.h"
#include <stdio.h>
#include <stdlib.h>

void display_init(QueueHandle_t* objPL_display_buff)
{
  if (objPL_display_buff == NULL)
  {
    return;
  }

  lcd_init(objPL_display_buff);
}

void display_run(i2c_t* objP_this)
{
  lcd_handler(objP_this);
}

void display_write_test(void)
{
  char *text  = "EmbeddedTher4";
  char *text2 = "EmbeddedTher2";
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_write_string(text);
  lcd_set_cursor(1, 0);
  lcd_write_string(text2);
}

void display_write(void)
{
  htu21d_t* opjPL_sensor_value = HTU21D_get_sensor_data();
  char row1[16] = {0};  // to store strings..
  char row2[16] = {0};  // to store strings..

  /* Writing text */
  snprintf(row1, 16, "temp = %d.%d", opjPL_sensor_value->s16_temperature / 10,
	    abs(opjPL_sensor_value->s16_temperature) % 10);

  /* Writing text */
  snprintf(row2, 16, "hum  = %d.%d ", opjPL_sensor_value->s16_humidity / 10,
	    abs(opjPL_sensor_value->s16_humidity) % 10);

  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_write_string(row1);
  lcd_set_cursor(1, 0);
  lcd_write_string(row2);
}
