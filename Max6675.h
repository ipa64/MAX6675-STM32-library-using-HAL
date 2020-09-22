/*
 * Max6675.h
 *
 *  Created on: Sep 21, 2020
 *      Author: iPa64
 */

#ifndef INC_MAX6675_H_
#define INC_MAX6675_H_

#include "stm32f1xx_hal.h"
#include "stdbool.h"


#define SPI_TIMEOUT 1000

/**
 * @brief  Thermocouple structure
 */
typedef struct {
	SPI_HandleTypeDef *hspi;	/* ^SPI */
	GPIO_TypeDef* Thx_CS_Port;  /*!< GPIO for CS */
	uint16_t Thx_CS_Pin;   		/*!< Pin for CS */
	uint16_t Thx_rawdata; 		/*!< Last raw data */
	float	Thx_celcius;		/*!< Last celsius value */
	bool 	connected;			/* FALSE if open, TRUE if connected */
} ThermoCouple;

void ReadThermoCouple(ThermoCouple *Th);

#endif /* INC_MAX6675_H_ */
