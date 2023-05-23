/*
 * I2C.h
 */

#ifndef SRC_LIB_DRIVERS_I2C_I2C_H_
#define SRC_LIB_DRIVERS_I2C_I2C_H_

#include "stm32f7xx_hal.h"
#include "main.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief I2C_err_t enum which express I2C erorrs
typedef enum I2cErr
{
  eI2C_err_ok,
  eI2C_err_init_fail,
  eI2C_err_deinit_fail,
  eI2C_err_send_fail,
  eI2C_err_receive_fail,
  eI2C_err_busy,
  eI2C_err_timeout,
}i2c_err_t;

/// @brief obj_uart_t struct which express instance of uart
typedef struct
{
  I2C_HandleTypeDef* i2c_handler;
  I2C_TypeDef*       i2c_type;
} i2c_t;

/**
 @brief function which init i2c

 @param[in] objP_this instance of i2c which should  init

 @return return type of error or ok if work correctly
*/
i2c_err_t i2c_init(i2c_t* objPL_this);

i2c_err_t i2c_mem_read(i2c_t* objPL_this, uint16_t u16L_dev_address, uint16_t u16L_mem_address,
                       uint16_t u16L_mem_add_size, uint8_t* u8PL_data, uint16_t u16L_size);

i2c_err_t i2c_master_send(i2c_t* objPL_this, uint16_t u16L_dev_address, uint8_t* u8PL_data, uint16_t u16L_size);

i2c_err_t i2c_get_state(i2c_t* objPL_this);

#ifdef __cplusplus
}
#endif

#endif /* SRC_LIB_DRIVERS_I2C_I2C_H_ */
