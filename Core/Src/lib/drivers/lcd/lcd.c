/*
 * lcd.c
 *
 *  Created on: 4 трав. 2023 р.
 *      Author: yevhen.surkov
 */

#include "lcd.h"

#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit

#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD

// Define global variable for backlight state
static uint8_t backlight_state = 1;

void lcd_write_nibble(i2c_t* objP_this, uint8_t nibble, uint8_t rs)
{
  uint8_t data = nibble << D4_BIT;
  data |= rs << RS_BIT;
  data |= backlight_state << BL_BIT; // Include backlight state in data
  data |= 1 << EN_BIT;
  i2c_master_send_blocking(objP_this, I2C_ADDR, &data, 1);
  HAL_Delay(1);
  data &= ~(1 << EN_BIT);
  i2c_master_send_blocking(objP_this, I2C_ADDR, &data, 1);
}

void lcd_send_cmd(i2c_t* objP_this, uint8_t cmd)
{
  uint8_t upper_nibble = cmd >> 4;
  uint8_t lower_nibble = cmd & 0x0F;
  lcd_write_nibble(objP_this, upper_nibble, 0);
  lcd_write_nibble(objP_this,lower_nibble, 0);
  if (cmd == 0x01 || cmd == 0x02)
  {
    HAL_Delay(2);
  }
}

void lcd_send_data(i2c_t* objP_this, uint8_t data)
{
  uint8_t upper_nibble = data >> 4;
  uint8_t lower_nibble = data & 0x0F;
  lcd_write_nibble(objP_this, upper_nibble, 1);
  lcd_write_nibble(objP_this, lower_nibble, 1);
}

void lcd_init(i2c_t* objP_this)
{
//  HAL_Delay(50);
  lcd_write_nibble(objP_this, 0x03, 0);
  HAL_Delay(5);
  lcd_write_nibble(objP_this, 0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(objP_this, 0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(objP_this, 0x02, 0);
  lcd_send_cmd(objP_this, 0x28);
  lcd_send_cmd(objP_this, 0x0C);
  lcd_send_cmd(objP_this, 0x06);
  lcd_send_cmd(objP_this, 0x01);
  HAL_Delay(2);
}

void lcd_write_string(i2c_t* objP_this, char *str)
{
  while (*str)
  {
    lcd_send_data(objP_this, *str++);
  }
}

void lcd_set_cursor(i2c_t* objP_this, uint8_t row, uint8_t column)
{
  uint8_t address;
  switch (row)
  {
    case 0:
      address = 0x00;
      break;
    case 1:
      address = 0x40;
      break;
    default:
      address = 0x00;
  }

  address += column;
  lcd_send_cmd(objP_this, 0x80 | address);
}

void lcd_clear(i2c_t* objP_this)
{
  lcd_send_cmd(objP_this, 0x01);
  HAL_Delay(2);
}

void lcd_backlight(uint8_t state)
{
  if (state)
  {
    backlight_state = 1;
  }
  else
  {
    backlight_state = 0;
  }
}
