/*
 * lcd.c
 *
 *  Created on: 4 трав. 2023 р.
 *      Author: yevhen.surkov
 */

#include "lcd.h"
#include "../../utils/ring_buffer/ring_buffer.h"
#include "stdint.h"

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
static ring_buffer_t ring_buffer;

static void lcd_write_nibble(uint8_t nibble, uint8_t rs);
static void lcd_send_data(uint8_t data);
static void lcd_send_cmd(uint8_t cmd);

void lcd_init(void)
{
  create_ring_buffer(&ring_buffer, 500);
  HAL_Delay(1);
  lcd_write_nibble(0x03, 0);
  lcd_write_nibble(0x03, 0);
  lcd_write_nibble(0x03, 0);
  lcd_write_nibble(0x02, 0);
  lcd_send_cmd(0x28);
  lcd_send_cmd(0x0C);
  lcd_send_cmd(0x06);
  lcd_send_cmd(0x01);
}

static void lcd_write_nibble(uint8_t nibble, uint8_t rs)
{
  uint8_t data = nibble << D4_BIT;
  data |= rs << RS_BIT;
  data |= backlight_state << BL_BIT; // Include backlight state in data
  data |= 1 << EN_BIT;
  en_ring_buffer(&ring_buffer, &data);
  data &= ~(1 << EN_BIT);
  en_ring_buffer(&ring_buffer, &data);
}

void lcd_handler(i2c_t* objP_this)
{
  uint8_t u8L_data;
  if(de_ring_buffer(&ring_buffer, &u8L_data) == e_ring_buffer_err_ok && i2c_get_state(objP_this) == eI2C_err_ok)
  {
    i2c_master_send(objP_this, I2C_ADDR, &u8L_data, 1);
    HAL_Delay(1);
  }
}

void lcd_display(void)
{
  char *text  = "EmbeddedTher1";
  char *text2 = "EmbeddedTher2";
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_write_string(text);
  lcd_set_cursor(1, 0);
  lcd_write_string(text2);
}

static void lcd_send_cmd(uint8_t cmd)
{
  uint8_t upper_nibble = cmd >> 4;
  uint8_t lower_nibble = cmd & 0x0F;
  lcd_write_nibble(upper_nibble, 0);
  lcd_write_nibble(lower_nibble, 0);
  if (cmd == 0x01 || cmd == 0x02)
  {
    HAL_Delay(2);
  }
}

static void lcd_send_data(uint8_t data)
{
  uint8_t upper_nibble = data >> 4;
  uint8_t lower_nibble = data & 0x0F;
  lcd_write_nibble(upper_nibble, 1);
  lcd_write_nibble(lower_nibble, 1);
}

void lcd_write_string(char *str)
{
  while (*str)
  {
    lcd_send_data(*str++);
  }
}

void lcd_set_cursor(uint8_t row, uint8_t column)
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
  lcd_send_cmd(0x80 | address);
}

void lcd_clear(void)
{
  lcd_send_cmd(0x01);
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
