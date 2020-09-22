
/**
  ******************************************************************************
  * Main.c MAX6675 library example
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *  Created on: Sep 21, 2020
  *      Author: iPa64
  ******************************************************************************
  */

/*----------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#include "main.h"

/*----------------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "Max6675.h"


/*----------------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
ThermoCouple sensor_th1;


/*----------------------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/*----------------------------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void InitThermocouple1(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/*----------------------------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/



/************************************************************************************************/
/************************************************************************************************/
/* MAIN              																			*/
/************************************************************************************************/
/************************************************************************************************/
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /*-------------------------------------------------------------------------*/
  /* MCU Configuration-------------------------------------------------------*/
  /*-------------------------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /*-------------------------------------------------------------------------*/
  /* Configure the system clock */
  /*-------------------------------------------------------------------------*/
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /*-------------------------------------------------------------------------*/
  /* Initialize all configured peripherals */
  /*-------------------------------------------------------------------------*/
  MX_GPIO_Init();
  MX_SPI1_Init();

  InitThermocouple1();

  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  // INFINITE LOOP         																	             //
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  float Th1_temp =0;
  int intpart, fracpart;
  char txt1[11];

  while (1)
  {
 	  ReadThermoCouple(&sensor_th1);
 	  if (sensor_th1.connected){
 		  Th1_temp = sensor_th1.Thx_celcius;
 		  intpart = (int)Th1_temp;
 		  fracpart = (int)((Th1_temp - intpart) * 100);
 		  snprintf(txt1, 11, "%3d.%02d", intpart, fracpart);
 		  // PRINT TO OLED Display
 		  // SSD1306_GotoXY (1, 1);
 		  // SSD1306_Puts (txt1, &Font_16x26, 1);
 		  // SSD1306_UpdateScreen(); // update screen*/
 	  }
 	  // else {
 	  // 	 SSD1306_GotoXY (1, 1);
 	  // 	 SSD1306_Puts ("disconnected", &Font_7x10, 1);
 	  // 	 SSD1306_UpdateScreen(); // update screen
 	  // }
	  HAL_Delay(300);
  }
}





/*#################################################################################################*/
/*#################################################################################################*/
/*## Functions                                                                                   ##*/
/*#################################################################################################*/
/*#################################################################################################*/

/*******************************************
  * @init structure thermocouple 1
  * @retval None
******************************************/
void InitThermocouple1(void)
{
	  sensor_th1.Thx_CS_Pin = Th1_CS_Pin;			// CS Pin
	  sensor_th1.Thx_CS_Port = Th1_CS_GPIO_Port;	// CS GPIO PORT
	  sensor_th1.hspi = &hspi1;						// SPI1
}




/*******************************************
  * @brief System Clock Configuration
  * @retval None
******************************************/
/*void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

    Wait till HSI is ready
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

    Wait till System clock is ready
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

    Update the time base
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}*/

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/*******************************************
  * @brief SPI Initialization Function
  * @param None
  * @retval None
  ******************************************/
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}



/*******************************************
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
 ******************************************/
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Th1_CS_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = Th1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */




/*******************************************
  * @brief  This function is executed in case of error occurrence.
  * @retval None
******************************************/
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
