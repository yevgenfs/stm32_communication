/*
 * I2C.c
 */

#include "I2C.h"

static uint16_t dev_address_shift(uint16_t u16L_dev_address);

i2c_err_t i2c_init(i2c_t* objP_this)
{
  if (objP_this != NULL && objP_this->i2c_handler != NULL)
  {
    objP_this->i2c_handler->Instance = I2C1;
    objP_this->i2c_handler->Init.Timing = 0x00808CD2;
    objP_this->i2c_handler->Init.OwnAddress1 = 0;
    objP_this->i2c_handler->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    objP_this->i2c_handler->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    objP_this->i2c_handler->Init.OwnAddress2 = 0;
    objP_this->i2c_handler->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    objP_this->i2c_handler->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    objP_this->i2c_handler->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(objP_this->i2c_handler) != HAL_OK)
    {
      Error_Handler();
    }

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(objP_this->i2c_handler, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      Error_Handler();
    }

    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(objP_this->i2c_handler, 0) != HAL_OK)
    {
      Error_Handler();
    }

    return eI2C_err_ok;
  }

  return eI2C_err_init_fail;
}

i2c_err_t i2c_mem_read(i2c_t* objP_this, uint16_t u16L_dev_address, uint16_t u16L_mem_address,
                       uint16_t u16L_mem_add_size, uint8_t* u8PL_data, uint16_t u16L_size)
{
  if (objP_this != NULL && objP_this->i2c_handler != NULL && u8PL_data != NULL)
  {
    HAL_I2C_Mem_Read_IT(objP_this->i2c_handler, dev_address_shift(u16L_dev_address), u16L_mem_address, u16L_mem_add_size, u8PL_data, u16L_size);
    return eI2C_err_ok;
  }

  return eI2C_err_send_fail;
}

i2c_err_t i2c_master_send_blocking(i2c_t* objP_this, uint16_t u16L_dev_address, uint8_t* u8PL_data, uint16_t u16L_size)
{
  if (objP_this != NULL && objP_this->i2c_handler != NULL && u8PL_data != NULL)
  {
    HAL_I2C_Master_Transmit(objP_this->i2c_handler, dev_address_shift(u16L_dev_address), u8PL_data, u16L_size, 1);
//    HAL_I2C_Master_Transmit_IT(objP_this->i2c_handler, dev_address_shift(u16L_dev_address), u8PL_data, u16L_size);
    return eI2C_err_ok;
  }

  return eI2C_err_send_fail;
}

static uint16_t dev_address_shift(uint16_t u16L_dev_address)
{
  return (u16L_dev_address << 1);
}
