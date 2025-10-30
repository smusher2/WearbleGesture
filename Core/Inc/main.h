/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef  USE_STM32WBXX_NUCLEO
#include "stm32wbxx_nucleo.h"
#endif
#ifdef  USE_X_NUCLEO_EPD
#include "x_nucleo_epd.h"
#endif
#ifdef USE_STM32WB5M_DK
#include "stm32wb5mm_dk.h"
#endif


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define QSPI_BK_IO0_Pin GPIO_PIN_9
#define QSPI_BK_IO0_GPIO_Port GPIOB
#define LPUART1_RX_MCU_Pin GPIO_PIN_0
#define LPUART1_RX_MCU_GPIO_Port GPIOC
#define VCP_RX_Pin GPIO_PIN_7
#define VCP_RX_GPIO_Port GPIOB
#define LPUART1_TX_MCU_Pin GPIO_PIN_5
#define LPUART1_TX_MCU_GPIO_Port GPIOB
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define User_B1_Pin GPIO_PIN_12
#define User_B1_GPIO_Port GPIOC
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define USB_P_Pin GPIO_PIN_12
#define USB_P_GPIO_Port GPIOA
#define USB_N_Pin GPIO_PIN_11
#define USB_N_GPIO_Port GPIOA
#define I2C3_SCL_Pin GPIO_PIN_13
#define I2C3_SCL_GPIO_Port GPIOB
#define TSC_G4_IO1_Pin GPIO_PIN_6
#define TSC_G4_IO1_GPIO_Port GPIOC
#define VCP_TX_Pin GPIO_PIN_6
#define VCP_TX_GPIO_Port GPIOB
#define User_B2_Pin GPIO_PIN_13
#define User_B2_GPIO_Port GPIOC
#define I2C3_SDA_Pin GPIO_PIN_11
#define I2C3_SDA_GPIO_Port GPIOB
#define SAI1_CK2_Pin GPIO_PIN_8
#define SAI1_CK2_GPIO_Port GPIOA
#define SAI1_D2_Pin GPIO_PIN_9
#define SAI1_D2_GPIO_Port GPIOA
#define QSPI_BK_SCK_Pin GPIO_PIN_3
#define QSPI_BK_SCK_GPIO_Port GPIOA
#define CS_DISP_Pin GPIO_PIN_0
#define CS_DISP_GPIO_Port GPIOH
#define GPIO_SELECT2_Pin GPIO_PIN_1
#define GPIO_SELECT2_GPIO_Port GPIOH
#define DRDY_Pin GPIO_PIN_1
#define DRDY_GPIO_Port GPIOE
#define INT1_Pin GPIO_PIN_2
#define INT1_GPIO_Port GPIOD
#define D_C_DISP_Pin GPIO_PIN_9
#define D_C_DISP_GPIO_Port GPIOC
#define QSPI_BK_NCS_Pin GPIO_PIN_3
#define QSPI_BK_NCS_GPIO_Port GPIOD
#define TSC_G4_IO2_Pin GPIO_PIN_7
#define TSC_G4_IO2_GPIO_Port GPIOC
#define INT2_Pin GPIO_PIN_9
#define INT2_GPIO_Port GPIOD
#define TSC_G6_IO1_Pin GPIO_PIN_10
#define TSC_G6_IO1_GPIO_Port GPIOD
#define GPIO_SELECT1_Pin GPIO_PIN_2
#define GPIO_SELECT1_GPIO_Port GPIOE
#define QSPI_BK_IO1_Pin GPIO_PIN_5
#define QSPI_BK_IO1_GPIO_Port GPIOD
#define QSPI_BK_IO2_Pin GPIO_PIN_6
#define QSPI_BK_IO2_GPIO_Port GPIOD
#define TSC_G6_IO2_Pin GPIO_PIN_11
#define TSC_G6_IO2_GPIO_Port GPIOD
#define RST_DISP_Pin GPIO_PIN_8
#define RST_DISP_GPIO_Port GPIOC

#define RLED_PIN_Pin GPIO_PIN_13
#define RLED_PIN_GPIO_Port GPIOB
#define GLED_PIN_Pin GPIO_PIN_4
#define GLED_PIN_GPIO_Port GPIOB
#define BLED_PIN_Pin GPIO_PIN_8
#define BLED_PIN_GPIO_Port GPIOB
#define Transmitting_LED_Pin GPIO_PIN_13
#define Transmitting_LED_GPIO_Port GPIOB
#define Power_LED_Pin GPIO_PIN_4
#define Power_LED_GPIO_Port GPIOB
#define Connected_LED_Pin GPIO_PIN_8
#define Connected_LED_GPIO_Port GPIOB
#define HAPTIC_PIN_Pin GPIO_PIN_10
#define HAPTIC_PIN_GPIO_Port GPIOA
#define TAP_INT_Pin GPIO_PIN_8
#define TAP_INT_GPIO_Port GPIOD
#define GPIO_CS_Pin GPIO_PIN_0
#define GPIO_CS_GPIO_Port GPIOD
#define SPI2_CS_Pin GPIO_PIN_0
#define SPI2_CS_GPIO_Port GPIOD
#define SPI2_SCK_Pin GPIO_PIN_1
#define SPI2_SCK_GPIO_Port GPIOD

#define REG_CHIP_ID_BMI323 0x00
#define REG_ERR 0x01
#define REG_SENSOR_STATUS 0x02
#define REG_ACC_DATA_X 0x03
#define REG_ACC_DATA_Y 0x04
#define REG_ACC_DATA_Z 0x05
#define REG_GYR_DATA_X 0x06
#define REG_GYR_DATA_Y 0x07
#define REG_GYR_DATA_Z 0x08
#define REG_TEMP_DATA 0x09
#define REG_CMD_BMI323     0x7E
#define REG_GYR_CONF_BMI323 0x21
#define REG_ACC_CONF_BMI323 0x20
#define FEATURE_IO0_REG 0x10
#define FEATURE_IO1_REG 0x11
#define FEATURE_IO2_REG 0x12
#define FEATURE_IO_STATUS_REG 0x14
#define FEATURE_CTRL_REG 0x40
#define INT_MAP2 0x3B
#define IO_INT_CTRL 0x38
#define FEATURE_EVENT_EXT  0x47
#define TAP_1 0x1E
#define TAP_2 0x1F
#define FEATURE_DATA_ADDR  0x41
#define FEATURE_DATA_TX  0x42
#define FEATURE_DATA_STATUS 0x43






#define CHIP_ID_BMI323   0x43
#define SOFT_RESET_CMD_BMI323 0xDEAF

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
