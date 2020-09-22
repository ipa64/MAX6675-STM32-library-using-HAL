/*
 * Max6675.c
 *
 *  Created on: Sep 21, 2020
 *      Author: iPa64
 */

#define TRUE  1
#define FALSE 0
#define bool BYTE

#include "Max6675.h"



/* SPI Receive one Byte */
static uint8_t SPI_RxByte(SPI_HandleTypeDef *hspi)
{
  uint8_t dummy, data;
  dummy = 0xFF;
  data = 0;

  while ((HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY));
  HAL_SPI_TransmitReceive(hspi, &dummy, &data, 1, SPI_TIMEOUT);

  return data;
}

void ReadThermoCouple(ThermoCouple *Th)
{
	unsigned short data;


	HAL_GPIO_WritePin(Th->Thx_CS_Port, Th->Thx_CS_Pin, GPIO_PIN_RESET);	//Chip Select level low

	data = SPI_RxByte(Th->hspi);
	data <<= 8;
	data |= SPI_RxByte(Th->hspi);

	HAL_GPIO_WritePin(Th->Thx_CS_Port, Th->Thx_CS_Pin, GPIO_PIN_SET);	//Chip select level high

	Th->Thx_rawdata = data;

	if (data & 4) Th->connected = FALSE;
	else Th->connected = TRUE;

	data  >>= 3;
	Th->Thx_celcius = data * 0.25;

}




