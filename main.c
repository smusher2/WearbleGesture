/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_lpm.h"
#include "stm32_seq.h"
#include "dbg_trace.h"
#include "hw_conf.h"
#include "otp.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;

}ImuDataTypeDef;

typedef struct
{
	uint16_t spi_cs_pin;
	GPIO_TypeDef *spi_cs_port;
	SPI_HandleTypeDef *bmi323_spi;
	ImuDataTypeDef imu_data;

}bmi323TypeDef;

typedef enum
{
	BMI323_OK = 0,
	BMI323_SPI_ERROR,
	BMI323_SPI_CHIP_ERROR,
	BMI323_CONFIG_FILE_ERROR,
}bmi323_StatusTypeDef;

bmi323_StatusTypeDef bmi323_init(bmi323TypeDef* sensor);
bmi323_StatusTypeDef bmi323_read_data(bmi323TypeDef* sensor);
void bmi323_write_spi(bmi323TypeDef* sensor, uint8_t reg, uint8_t* data, uint16_t len);

SPI_HandleTypeDef hspi2;
IPCC_HandleTypeDef hipcc;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
bmi323_StatusTypeDef imu_status;
bmi323_StatusTypeDef feature_status;
bmi323_StatusTypeDef imu_chip_status;

bmi323TypeDef imu_sensor = {
		.bmi323_spi = &hspi2,
		.spi_cs_pin = SPI2_CS_Pin,
		.spi_cs_port = SPI2_CS_GPIO_Port
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_IPCC_Init(void);
static void MX_RF_Init(void);
/* USER CODE BEGIN PFP */
void PeriphClock_Config(void);
/* USER CODE END PFP */

int main(void)
{

  HAL_Init();
  MX_APPE_Config();
  SystemClock_Config();
  PeriphCommonClock_Config();
  MX_IPCC_Init();
  PeriphClock_Config();
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_RF_Init();
  MX_APPE_Init();
  HAL_Delay(1000);
  imu_status = bmi323_init(&imu_sensor);
  HAL_Delay(1000);

  while(1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();
    char msg1[200];

	 	  snprintf(msg1, sizeof(msg1), "hello world\r\n");

	 	    // 3. Transmit over UART (blocking mode)
	 	  HAL_UART_Transmit(&huart1, (uint8_t*)msg1, strlen(msg1), HAL_MAX_DELAY);

	   	  HAL_Delay(1000);    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }


}


static void MX_RF_Init(void)
{


}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

static void MX_RTC_Init(void)
{


 //chunk
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  HAL_PWR_EnableBkUpAccess();

  // Option A: Use LSI (no crystal needed)

  RCC_OscInitTypeDef osc = {0};
  RCC_PeriphCLKInitTypeDef pclk = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_LSI2;
  osc.LSIState = RCC_LSI_ON;
  osc.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&osc);

  pclk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  pclk.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&pclk);


  __HAL_RCC_RTC_ENABLE();
  //chunk

  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
        Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* Disable RTC registers write protection */

  LL_RTC_DisableWriteProtection(RTC);
  
  LL_RTC_WAKEUP_SetClock(RTC, CFG_RTC_WUCKSEL_DIVIDER);
  
  /* Enable RTC registers write protection */
  LL_RTC_EnableWriteProtection(RTC);
  /* USER CODE END RTC_Init 2 */

}


void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLED_PIN_Pin|GLED_PIN_Pin|RLED_PIN_Pin, GPIO_PIN_SET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_CS_GPIO_Port, GPIO_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HAPTIC_PIN_GPIO_Port, HAPTIC_PIN_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);


  /*Configure GPIO pins : BLED_PIN_Pin GLED_PIN_Pin RLED_PIN_Pin */
  GPIO_InitStruct.Pin = BLED_PIN_Pin|GLED_PIN_Pin|RLED_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HAPTIC_PIN_Pin */
  GPIO_InitStruct.Pin = HAPTIC_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HAPTIC_PIN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  //init SCK pin
  GPIO_InitStruct.Pin = SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;

  HAL_GPIO_Init(SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_Pin*/
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 EXTI Init */
  GPIO_InitStruct.Pin = TAP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TAP_INT_GPIO_Port, &GPIO_InitStruct);


}

static void MX_SPI2_Init(void)
{

  //__HAL_RCC_SPI2_CLK_ENABLE();   // <— add this line

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}
/* USER CODE BEGIN 4 */
void PeriphClock_Config(void)
{
  #if (CFG_USB_INTERFACE_ENABLE != 0)
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
	RCC_CRSInitTypeDef RCC_CRSInitStruct = { 0 };

  LL_HSEM_1StepLock( HSEM, 5 );

  LL_RCC_HSI48_Enable();

	while(!LL_RCC_HSI48_IsReady());

	/* Select HSI48 as USB clock source */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	/*Configure the clock recovery system (CRS)**********************************/

	/* Enable CRS Clock */
	__HAL_RCC_CRS_CLK_ENABLE();

	/* Default Synchro Signal division factor (not divided) */
	RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;

	/* Set the SYNCSRC[1:0] bits according to CRS_Source value */
	RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;

	/* HSI48 is synchronized with USB SOF at 1KHz rate */
	RCC_CRSInitStruct.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT;
	RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;

	RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;

	/* Set the TRIM[5:0] to the default value*/
	RCC_CRSInitStruct.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;

	/* Start automatic synchronization */
	HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
#endif

	return;
}

/* USER CODE END 4 */


void bmi323_activate_spi(bmi323TypeDef* sensor)
{
	HAL_GPIO_WritePin(sensor->spi_cs_port, sensor->spi_cs_pin, GPIO_PIN_RESET);
}

void bmi323_deactivate_spi(bmi323TypeDef* sensor)
{
	HAL_GPIO_WritePin(sensor->spi_cs_port, sensor->spi_cs_pin, GPIO_PIN_SET);
}





void bmi323_write_spi(bmi323TypeDef* sensor, uint8_t reg, uint8_t* data, uint16_t len)
{
    uint8_t buf[1 + len];
    buf[0] = reg & 0x7F;  // write = MSB = 0
    memcpy(&buf[1], data, len);

    bmi323_activate_spi(sensor);
    HAL_SPI_Transmit(sensor->bmi323_spi, buf, sizeof(buf), HAL_MAX_DELAY);
    bmi323_deactivate_spi(sensor);
}

static void bmi323_receive_spi(bmi323TypeDef* sensor, uint8_t offset_reg, uint8_t* data, uint16_t len)
{
	uint8_t data_temp[20], data_temp_byte;
	bmi323_activate_spi(sensor); //10
	data_temp[0] = offset_reg | 0x80;
	HAL_SPI_Transmit(sensor->bmi323_spi, data_temp, 1, 1000);
	HAL_SPI_Receive(sensor->bmi323_spi, &data_temp_byte, 1, 1000);
	HAL_SPI_Receive(sensor->bmi323_spi, data, len, 1000);
	bmi323_deactivate_spi(sensor);

}




// Assumes: FEATURE_DATA_ADDR=0x41, FEATURE_DATA_TX=0x42, FEATURE_DATA_STATUS=0x43


bmi323_StatusTypeDef bmi323_init(bmi323TypeDef* sensor)
{
	extern UART_HandleTypeDef huart1;  // ✅ use the real one from usart.c

	uint8_t temp_data[2] = {0};
	char msg[32];  // buffer for UART printing

	bmi323_activate_spi(sensor);
	HAL_Delay(10);
	bmi323_deactivate_spi(sensor);
	HAL_Delay(10);

	bmi323_receive_spi(sensor, REG_CHIP_ID_BMI323, temp_data, 1); //dummy read for SPI configuration

	bmi323_receive_spi(sensor, REG_CHIP_ID_BMI323, temp_data, 1); //actual read

	snprintf(msg, sizeof(msg), "Chip ID read: 0x%02X\r\n", temp_data[0]);
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	if ((temp_data[0] != 0x43) && (temp_data[0]!= 0xA6) && (temp_data[0] != 0x41))
	{

	    return BMI323_SPI_CHIP_ERROR;
	}


	//READ POWER STATUS
	bmi323_receive_spi(sensor, REG_ERR, temp_data, 1);
	snprintf(msg, sizeof(msg), "POWER STATUS read: 0x%02X\r\n", temp_data[0]);
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	if (temp_data[0] != 0x00) {  // if any error bits are set
	    return BMI323_SPI_ERROR;
	}


	//READ STATUS FLAG
	bmi323_receive_spi(sensor, REG_SENSOR_STATUS, temp_data, 1);
	snprintf(msg, sizeof(msg), "SENSOR STATUS read: 0x%02X\r\n", temp_data[0]);
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//	if ((temp_data[0]) != 0b1) {  // ready bit
//	    return BMI323_SPI_ERROR;
//	}


	return BMI323_OK;
}





/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
