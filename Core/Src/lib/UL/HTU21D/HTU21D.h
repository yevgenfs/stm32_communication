/*
 * SHT1.h
 *
 */

#ifndef SRC_LIB_UL_HTU21D_H_
#define SRC_LIB_UL_HTU21D_H_

#include "main.h"
#include "../../drivers/I2C/I2C.h"

/// @brief htu21d_t struct which express instance of htu21d_t
typedef struct
{
  int16_t s16_temperature;
  int16_t s16_humidity;
} htu21d_t;

void HTU21D_handler(i2c_t* objPL_this);
htu21d_t* HTU21D_get_sensor_data(void);

#endif /* SRC_LIB_UL_HTU21D_H_ */
