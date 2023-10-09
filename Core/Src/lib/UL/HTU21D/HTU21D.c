/*
 * SHT1.c
 *
 */

#include "HTU21D.h"
#include "stdint.h"
#include "../LCD/display.h"

#define HTU21D_Adress (0x40)

static int16_t get_temperature(i2c_t* objPL_this);
static int16_t get_humidity(i2c_t* objPL_this);

static htu21d_t objS_htu21d = { .s16_humidity = 0, .s16_temperature = 0 };

const uint8_t u8C_temp_Cmd = 0xE3;
const uint8_t u8C_humi_Cmd = 0xE5;

void HTU21D_handler(i2c_t* objPL_this)
{
  objS_htu21d.s16_temperature = get_temperature(objPL_this);
  vTaskDelay(100);
  objS_htu21d.s16_humidity = get_humidity(objPL_this);
  vTaskDelay(100);

  display_write();
}

htu21d_t* HTU21D_get_sensor_data(void)
{
  return &objS_htu21d;
}

static int16_t get_temperature(i2c_t* objPL_this)
{
  int16_t s16L_temperature = 0;
  uint16_t u16L_ADC_Raw    = 0;
  static uint8_t u8SL_RX_data[2] = { 0 };
  i2c_mem_read(objPL_this, HTU21D_Adress, u8C_temp_Cmd, I2C_MEMADD_SIZE_8BIT, u8SL_RX_data, 2);
  u16L_ADC_Raw = ((uint16_t)(u8SL_RX_data[0] << 8) | (u8SL_RX_data[1]));
  s16L_temperature = (int16_t)(((u16L_ADC_Raw * 175.72 / 65536.00) - 46.85) * 10);
  return s16L_temperature;
}

static int16_t get_humidity(i2c_t* objPL_this)
{
  int16_t s16L_humidity = 0;
  uint16_t u16L_ADC_Raw = 0;
  static uint8_t u8SL_RX_data[2] = { 0 };
  i2c_mem_read(objPL_this, HTU21D_Adress, u8C_humi_Cmd, I2C_MEMADD_SIZE_8BIT, u8SL_RX_data, 2);
  u16L_ADC_Raw = ((uint16_t)(u8SL_RX_data[0] << 8) | (u8SL_RX_data[1]));
  s16L_humidity = (int16_t)(((u16L_ADC_Raw * 125.0 / 65536.0) - 6.0) * 10);
  return s16L_humidity;
}
