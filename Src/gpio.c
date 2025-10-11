/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : gpio.c
  * @brief          : gpio program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void Haptic_Buzz_Start(uint32_t* haptic_deadline, uint8_t* haptic_active)
{
    // If already buzzing, choose behavior: extend or ignore.
    if (*haptic_active) {
        // ignore
        return;
    }

    HAL_GPIO_WritePin(HAPTIC_PIN_GPIO_Port, HAPTIC_PIN_Pin, GPIO_PIN_SET); // ON
    *haptic_deadline = HAL_GetTick() + 500;
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

