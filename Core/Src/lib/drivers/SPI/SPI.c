/*
 * SPI.c
 *
 *  Created on: 23 трав. 2023 р.
 *      Author: yevhen.surkov
 */

#include "SPI.h"
#include "stm32f7xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef 	hspi1;
#define HSPI_SDCARD		&hspi1

spi_err_t SPI_init(spi_t* objPL_this)
{
  if (objPL_this != NULL && objPL_this->spi_handler != NULL)
  {
    objPL_this->spi_handler->Instance = SPI1;
    objPL_this->spi_handler->Init.Mode = SPI_MODE_MASTER;
    objPL_this->spi_handler->Init.Direction = SPI_DIRECTION_2LINES;
    objPL_this->spi_handler->Init.DataSize = SPI_DATASIZE_8BIT;
    objPL_this->spi_handler->Init.CLKPolarity = SPI_POLARITY_LOW;
    objPL_this->spi_handler->Init.CLKPhase = SPI_PHASE_1EDGE;
    objPL_this->spi_handler->Init.NSS = SPI_NSS_SOFT;
    objPL_this->spi_handler->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    objPL_this->spi_handler->Init.FirstBit = SPI_FIRSTBIT_MSB;
    objPL_this->spi_handler->Init.TIMode = SPI_TIMODE_DISABLE;
    objPL_this->spi_handler->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    objPL_this->spi_handler->Init.CRCPolynomial = 7;
    objPL_this->spi_handler->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    objPL_this->spi_handler->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

    if (HAL_SPI_Init (&hspi1) != HAL_OK)
    {
      Error_Handler();
    }

    return eSPI_err_ok;
  }

  return eSPI_err_init_fail;
}

/* slave select */
void SPI_select(GPIO_TypeDef* objPL_port, uint16_t u16L_pin)
{
  HAL_GPIO_WritePin (objPL_port, u16L_pin, GPIO_PIN_RESET);
  HAL_Delay (1);
}

/* slave deselect */
void SPI_deselect(GPIO_TypeDef* objPL_port, uint16_t u16L_pin)
{
  HAL_GPIO_WritePin (objPL_port, u16L_pin, GPIO_PIN_SET);
  HAL_Delay (1);
}

/* SPI transmit a byte */
void SPI_tx_byte(spi_t* objPL_this, uint8_t data)
{
  while (!__HAL_SPI_GET_FLAG(objPL_this->spi_handler, SPI_FLAG_TXE));
  HAL_SPI_Transmit (objPL_this->spi_handler, &data, 1, SPI_TIMEOUT);
}

/* SPI transmit buffer */
void SPI_tx_buffer(spi_t* objPL_this, uint8_t *buffer, uint16_t len)
{
  while (!__HAL_SPI_GET_FLAG(objPL_this->spi_handler, SPI_FLAG_TXE));
  HAL_SPI_Transmit (objPL_this->spi_handler, buffer, len, SPI_TIMEOUT);
}

/* SPI receive a byte */
uint8_t SPI_rx_byte(spi_t* objPL_this)
{
  uint8_t dummy, data;
  dummy = 0xFF;

  while (!__HAL_SPI_GET_FLAG(objPL_this->spi_handler, SPI_FLAG_TXE));
  HAL_SPI_TransmitReceive (objPL_this->spi_handler, &dummy, &data, 1, SPI_TIMEOUT);

  return data;
}

/* SPI receive a byte via pointer */
void SPI_rx_byte_ptr(spi_t* objPL_this, uint8_t *buff)
{
  *buff = SPI_rx_byte (objPL_this);
}