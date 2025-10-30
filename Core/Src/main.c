/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_ble.h"

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

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

void Haptic_Buzz_Check2STOP(uint32_t* haptic_deadline, uint8_t* haptic_active);
void Haptic_Buzz_Start(uint32_t* haptic_deadline, uint8_t* haptic_active);
void OnRLED(void);
void OffRLED(void);
void OnGLED(void);
void OffGLED(void);
void OnBLED(void);
void OffBLED(void);
bmi323_StatusTypeDef bmi323_init(bmi323TypeDef* sensor);
bmi323_StatusTypeDef bmi323_read_data(bmi323TypeDef* sensor);
bmi323_StatusTypeDef BMI323_Feature_Engine_Enable(bmi323TypeDef* sensor);
void bmi323_write_spi(bmi323TypeDef* sensor, uint8_t reg, uint8_t* data, uint16_t len);
void bmi323_enable_tap(bmi323TypeDef* sensor);
void BMI323_HandleTapEvent(bmi323TypeDef* sensor, uint32_t* haptic_deadline, uint8_t* haptic_active);


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
IPCC_HandleTypeDef hipcc;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
bmi323_StatusTypeDef imu_status;
bmi323_StatusTypeDef feature_status;
bmi323_StatusTypeDef imu_chip_status;
bmi323TypeDef imu_sensor = {
		.bmi323_spi = &hspi2,
		.spi_cs_pin = SPI2_CS_Pin,
		.spi_cs_port = SPI2_CS_GPIO_Port
};
int16_t ax, ay, az, gx, gy, gz;
float ax_m, ay_m, az_m, gx_m, gy_m, gz_m;
volatile uint8_t g_tap_irq = 0;
uint32_t haptic_deadline;
uint8_t haptic_active = 0;
static const float ACC_LSB_PER_G  = 4096.0f;      // ±8 g
static const float GYR_LSB_PER_DPS = 16.384f;     // ±2000 dps
static const float G_TO_MPS2  =    9.80665f;

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

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */
  PeriphClock_Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();
  HAL_Delay(1000);
  imu_status = bmi323_init(&imu_sensor);
  feature_status = BMI323_Feature_Engine_Enable(&imu_sensor);          // write IO2=0x012C, IO_STATUS=0x0001,

    uint8_t gyr[2];
    gyr[0] = (0<<7) | (0x4<<4) | 0xA;   // 0x28: BW=0, RANGE=±2000, ODR=100 Hz
    gyr[1] = 0x40;                       // mode = 0b100 (enabled)
    bmi323_write_spi(&imu_sensor, REG_GYR_CONF_BMI323, gyr, 2);
    HAL_Delay(10);

    uint8_t acc[2];
    acc[0] = (0<<7) | (0x2<<4) | 0xA;  // BW=0, ±8g, ODR=200 Hz
    acc[1] = 0x40;                      // normal/enable
    bmi323_write_spi(&imu_sensor, REG_ACC_CONF_BMI323, acc, 2);
    HAL_Delay(10);

    bmi323_enable_tap(&imu_sensor);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();

  	  if (g_tap_irq)
  	  {
  		  g_tap_irq = 0;
  		  BMI323_HandleTapEvent(&imu_sensor, &haptic_deadline, &haptic_active);   // read FEATURE_EVENT_EXT and act
  	  }
      Haptic_Buzz_Check2STOP(&haptic_deadline, &haptic_active);


 	  bmi323_read_data(&imu_sensor);
   	  ax = imu_sensor.imu_data.ax;
   	  ay = imu_sensor.imu_data.ay;
   	  az = imu_sensor.imu_data.az;
   	  gx = imu_sensor.imu_data.gx;
   	  gy = imu_sensor.imu_data.gy;
   	  gz = imu_sensor.imu_data.gz;

   	  ax_m = ax * G_TO_MPS2 / ACC_LSB_PER_G;
   	  ay_m = ay * G_TO_MPS2 / ACC_LSB_PER_G;
   	  az_m = az * G_TO_MPS2 / ACC_LSB_PER_G;
   	  gx_m = gx / GYR_LSB_PER_DPS;
   	  gy_m = gy / GYR_LSB_PER_DPS;
   	  gz_m = gz / GYR_LSB_PER_DPS;



      char msg1[200];

 	  snprintf(msg1, sizeof(msg1), "AX: %.3f, AY: %.3f, AZ: %.3f, GX: %.3f, GY: %.3f, GZ: %.3f \r\n", ax_m, ay_m, az_m, gx_m, gy_m, gz_m);

 	    // 3. Transmit over UART (blocking mode)
 	  HAL_UART_Transmit(&huart1, (uint8_t*)msg1, strlen(msg1), HAL_MAX_DELAY);



   	  HAL_Delay(10);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

#if (CFG_USE_SMPS != 0)
  /**
   *  Configure and enable SMPS
   *
   *  The SMPS configuration is not yet supported by CubeMx
   *  when SMPS output voltage is set to 1.4V, the RF output power is limited to 3.7dBm
   *  the SMPS output voltage shall be increased for higher RF output power
   */
  LL_PWR_SMPS_SetStartupCurrent(LL_PWR_SMPS_STARTUP_CURRENT_80MA);
  LL_PWR_SMPS_SetOutputVoltageLevel(LL_PWR_SMPS_OUTPUT_VOLTAGE_1V40);
  LL_PWR_SMPS_Enable();
#endif

  /* USER CODE END Smps */
}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  HAL_PWR_EnableBkUpAccess();
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
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
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
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

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case GPIO_PIN_12:
            /* SW button 1 */
            APP_BLE_Key_Button1_Action();
            break;

        case GPIO_PIN_13:
            /* SW button 2 */
            APP_BLE_Key_Button2_Action();
            break;

        case TAP_INT_Pin:
            /* BMI323 tap interrupt */
            // Read BMI323 INT status + FEATURE_EVENT_EXT here
            // Decide single vs double tap, then clear as needed
            g_tap_irq = 1;
            break;

        default:
            break;
    }
}

static void MX_SPI2_Init(void)
{

  __HAL_RCC_SPI2_CLK_ENABLE();   // <— add this line

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

	/**
   * This prevents the CPU2 to disable the HSI48 oscillator when
   * it does not use anymore the RNG IP
   */
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




void bmi323_enable_tap(bmi323TypeDef* sensor)
{

	//extern UART_HandleTypeDef huart1;  // ✅ use the real one from usart.c
	//char msg[32];  // buffer for UART printing

	uint8_t temp_data[2] = {0};
	bmi323_receive_spi(sensor, FEATURE_CTRL_REG, temp_data, 2);
	temp_data[0] |= 0x01;
	bmi323_write_spi(sensor, FEATURE_CTRL_REG, temp_data, 2);

	uint8_t tap_en[2] = {0};
	bmi323_receive_spi(sensor, FEATURE_IO0_REG, tap_en, 2);
	tap_en[1] |= (1u<<4) | (1u<<5);
	bmi323_write_spi(sensor, FEATURE_IO0_REG, tap_en, 2);

	uint8_t map_int[2] = {0};
	bmi323_receive_spi(sensor, INT_MAP2, map_int, 2);
    map_int[0] = (uint8_t)((map_int[0] & ~0x03u) | 0x01u);
	bmi323_write_spi(sensor, INT_MAP2, map_int, 2);


	uint8_t io[2] = {0};
	bmi323_receive_spi(sensor, IO_INT_CTRL, io, 2);
	io[0] |=  (1u<<2);   // int1_output_en = 1  👈 required
	io[0] &= ~(1u<<1);   // int1_od = 0 (push-pull)
	io[0] |=  (1u<<0);   // int1_lvl = 1 (active high)
	bmi323_write_spi(sensor, IO_INT_CTRL, io, 2);

    /*
	//configure modes in extended reg field
	uint8_t tap1_addr = TAP_1;
	uint8_t tap2_addr = TAP_2;


	uint8_t mode1[2] = {0};
	uint8_t mode2[2] = {0};
	feature_read_word(sensor, tap1_addr, mode1);
	mode1[0] = (uint8_t)((mode1[0] & ~(0x3u << 6)) | (0x2u << 6));
	feature_write_word(sensor, tap1_addr, mode1);

	feature_read_word(sensor, tap2_addr, mode2);
	mode2[0] = (uint8_t)(mode2[0] | (0xFFu));
	mode2[1] = (uint8_t)(mode2[1] | (0x3u));
	feature_write_word(sensor, tap2_addr, mode2);
	HAL_Delay(20);
	feature_read_word(sensor, tap2_addr, mode2);


	snprintf(msg, sizeof(msg), "reg read: 0x%02X 0x%02X \r\n", mode2[1], mode2[0]);
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    */
	uint8_t dummy[2];
	bmi323_receive_spi(sensor, FEATURE_IO_STATUS_REG, dummy, 2); // read to clear
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

	//soft reset the BMI
	/*
	uint8_t reset_cmd[2] = {0xAF, 0xDE};  // 0xDEAF little endian
	bmi323_write_spi(sensor, REG_CMD_BMI323, reset_cmd, 2);
	HAL_Delay(20);
	*/

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



	//Normal Power Mode Operation: write data bits and read
	//temp_data[0] = 0x27;
	//temp_data[1] = 0x40;
	//bmi323_write_spi(sensor, REG_ACC_CONF_BMI323, temp_data, 2);
	//HAL_Delay(10);

	//temp_data[0] = 0x4B;
	//temp_data[1] = 0x40;
	//bmi323_write_spi(sensor, REG_GYR_CONF_BMI323, temp_data, 2);
	//HAL_Delay(10);


	return BMI323_OK;
}

bmi323_StatusTypeDef bmi323_read_data(bmi323TypeDef* sensor)
{
	extern UART_HandleTypeDef huart1;  // ✅ use the real one from usart.c


	uint8_t temp_data[12];
	bmi323_receive_spi(sensor, REG_ACC_DATA_X, temp_data, 12);
	sensor->imu_data.ax = (temp_data[1] << 8) | temp_data[0];
	sensor->imu_data.ay = (temp_data[3] << 8) | temp_data[2];
	sensor->imu_data.az = (temp_data[5] << 8) | temp_data[4];
	sensor->imu_data.gx = (temp_data[7] << 8) | temp_data[6];
	sensor->imu_data.gy = (temp_data[9] << 8) | temp_data[8];
	sensor->imu_data.gz = (temp_data[11] << 8) | temp_data[10];



	return BMI323_OK;


}

void BMI323_HandleTapEvent(bmi323TypeDef* sensor, uint32_t* haptic_deadline, uint8_t* haptic_active)
{

	extern UART_HandleTypeDef huart1;  // ✅ use the real one from usart.c

	char msg[32];  // buffer for UART printing

    uint8_t tap_evt[2];
    bmi323_receive_spi(sensor, FEATURE_EVENT_EXT, tap_evt, 2); // [LSB, MSB]
    uint8_t tap_type = tap_evt[0];
    if (tap_type & (1u<<4)) {
        // double tap
    	snprintf(msg, sizeof(msg), "Double Tap\r\n");
    	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    } else if (tap_type & (1u<<3)) {
        // single tap
    	snprintf(msg, sizeof(msg), "Single Tap\r\n");
    	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    // (reading FEATURE_EVENT_EXT clears the event indicators)
    Haptic_Buzz_Start(haptic_deadline, haptic_active);

}

// Call this once, before bmi323_enable_tap() and before enabling acc/gyro.
bmi323_StatusTypeDef BMI323_Feature_Engine_Enable(bmi323TypeDef* sensor)
{
    // 1) FEATURE_IO2 = 0x012C  (LSB first)
    uint8_t io2[2] = { 0x2C, 0x01 };
    bmi323_write_spi(sensor, FEATURE_IO2_REG, io2, 2);

    // 2) FEATURE_IO_STATUS = 0x0001 (LSB first)
    uint8_t iostat[2] = { 0x01, 0x00 };
    bmi323_write_spi(sensor, FEATURE_IO_STATUS_REG, iostat, 2);

    // 3) FEATURE_CTRL.engine_en = 1
    uint8_t fctrl[2] = {0};
    bmi323_receive_spi(sensor, FEATURE_CTRL_REG, fctrl, 2);
    fctrl[0] |= 0x01;                 // bit0 = engine_en
    bmi323_write_spi(sensor, FEATURE_CTRL_REG, fctrl, 2);

    // 4) Poll FEATURE_IO1.error_status for 0b001 (datasheet’s “engine ready”)
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 50) {     // ~50 ms timeout
        uint8_t io1[2] = {0};
        bmi323_receive_spi(sensor, FEATURE_IO1_REG, io1, 2);
        if ((io1[0] & 0x0F) == 0x01) {      // error_status==0b001
            return BMI323_OK;
        }
    }
    return BMI323_CONFIG_FILE_ERROR;
}

void Haptic_Buzz_Start(uint32_t* haptic_deadline, uint8_t* haptic_active)
{
    // If already buzzing, choose behavior: extend or ignore.
    if (*haptic_active) {
        // ignore
        return;
    }

    HAL_GPIO_WritePin(HAPTIC_PIN_GPIO_Port, HAPTIC_PIN_Pin, GPIO_PIN_SET); // ON
    *haptic_deadline = HAL_GetTick() + 100;
    *haptic_active = 1;
}

void Haptic_Buzz_Check2STOP(uint32_t* haptic_deadline, uint8_t* haptic_active)
{
    // signed subtraction handles HAL_GetTick wrap-around safely
    if (*haptic_active && (int32_t)(HAL_GetTick() - *haptic_deadline) >= 0) {
        HAL_GPIO_WritePin(HAPTIC_PIN_GPIO_Port, HAPTIC_PIN_Pin, GPIO_PIN_RESET); // OFF
        *haptic_active = 0;
    }
}

void OnRLED(void)
{
    HAL_GPIO_WritePin(RLED_PIN_GPIO_Port, RLED_PIN_Pin, GPIO_PIN_RESET); // OFF
}
void OffRLED(void)
{
    HAL_GPIO_WritePin(RLED_PIN_GPIO_Port, RLED_PIN_Pin, GPIO_PIN_SET); // OFF
}


void OnBLED(void)
{
    HAL_GPIO_WritePin(BLED_PIN_GPIO_Port, BLED_PIN_Pin, GPIO_PIN_RESET); // OFF
}
void OffBLED(void)
{
    HAL_GPIO_WritePin(BLED_PIN_GPIO_Port, BLED_PIN_Pin, GPIO_PIN_SET); // OFF
}


void OnGLED(void)
{
    HAL_GPIO_WritePin(GLED_PIN_GPIO_Port, GLED_PIN_Pin, GPIO_PIN_RESET); // OFF
}
void OffGLED(void)
{
    HAL_GPIO_WritePin(GLED_PIN_GPIO_Port, GLED_PIN_Pin, GPIO_PIN_SET); // OFF
}

/* USER CODE END 4 */

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
