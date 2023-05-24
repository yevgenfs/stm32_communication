/*
 * SPI.h
 *
 *  Created on: 23 трав. 2023 р.
 *      Author: yevhen.surkov
 */

#ifndef SRC_LIB_DRIVERS_SPI_SPI_H_
#define SRC_LIB_DRIVERS_SPI_SPI_H_

#include "stdint.h"
#include "stm32f7xx_hal.h"

#define SPI_TIMEOUT 100

/// @brief I2C_err_t enum which express I2C erorrs
typedef enum SPIErr
{
  eSPI_err_ok,
  eSPI_err_init_fail,
  eSPI_err_deinit_fail,
  eSPI_err_send_fail,
  eSPI_err_receive_fail,
  eSPI_err_busy,
  eSPI_err_timeout,
}spi_err_t;

/// @brief obj_uart_t struct which express instance of uart
typedef struct
{
  SPI_HandleTypeDef* obj_spi_handler;
  SPI_TypeDef*       obj_spi_type;
} spi_t;

spi_err_t SPI_init(spi_t* objPL_this);

/* slave select */
void SPI_select(GPIO_TypeDef * objPL_port, uint16_t u16L_pin);

/* slave deselect */
void SPI_deselect(GPIO_TypeDef * objPL_port, uint16_t u16L_pin);

/* SPI transmit a byte */
void SPI_tx_byte(spi_t* objPL_this, uint8_t u8L_data);

/* SPI transmit buffer */
void SPI_tx_buffer(spi_t* objPL_this, uint8_t* u8PL_buffer, uint16_t u16L_len);

/* SPI receive a byte */
uint8_t SPI_rx_byte(spi_t* objPL_this);

/* SPI receive a byte via pointer */
void SPI_rx_byte_ptr(spi_t* objPL_this, uint8_t* u8PL_buff);

#endif /* SRC_LIB_DRIVERS_SPI_SPI_H_ */
