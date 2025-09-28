/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "bmi_323.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
static void bmi323_activate_spi(bmi323TypeDef* sensor)
{
	HAL_GPIO_WritePin(sensor->spi_cs_port, sensor->spi_cs_pin, GPIO_PIN_RESET);
}

static void bmi323_deactivate_spi(bmi323TypeDef* sensor)
{
	HAL_GPIO_WritePin(sensor->spi_cs_port, sensor->spi_cs_pin, GPIO_PIN_SET);
}

static void bmi323_write_spi(bmi323TypeDef* sensor, uint8_t offset_reg, uint8_t* data, uint16_t data_size)
{
	uint8_t data_temp[20];
	bmi323_activate_spi(sensor);
	data_temp[0] = offset_reg;
	HAL_SPI_Transmit(sensor->bmi323_spi, data_temp, 1, 1000);
	HAL_SPI_Transmit(sensor->bmi323_spi, data, data_size, 1000);
	bmi323_deactivate_spi(sensor);
}

static void bmi323_receive_spi(bmi323TypeDef* sensor, uint8_t offset_reg, uint8_t* data, uint16_t data_size)
{
	uint8_t data_temp[20], data_temp_byte;
	bmi323_activate_spi(sensor);
	data_temp[0] = offset_reg | 0x80;
	HAL_SPI_Transmit(sensor->bmi323_spi, data_temp, 1, 1000);
	HAL_SPI_Receive(sensor->bmi323_spi, &data_temp_byte, 1, 1000);
	HAL_SPI_Receive(sensor->bmi323_spi, data, data_size, 1000);
	bmi323_deactivate_spi(sensor);
}

bmi323_StatusTypeDef bmi323_init(bmi323TypeDef* sensor)
{
	uint8_t temp_data[3] = {0};

	bmi323_receive_spi(sensor, REG_CHIP_ID_BMI323, temp_data, 1);
	if(temp_data[0] != CHIP_ID_BMI323)
	{
		return BMI323_SPI_ERROR;
	}

	//soft reset the BMI
	/*
	temp_data[0] = SOFT_RESET_CMD_BMI323;
	bmi323_write_spi(sensor, REG_CMD_BMI323, temp_data, 1);
	HAL_delay(10);

	bmi323_activate_spi(sensor);
	HAL_delay(10);
	bmi323_deactivate_spi(sensor);
	HAL_delay(10);
	*/

	memset(temp_data, 0, sizeof(temp_data));
	bmi323_receive_spi(sensor, REG_PWR_ERR_REG, temp_data, 1);
	if(temp_data[0] != 0b0)
	{
		return BMI323_SPI_ERROR;
	}

	memset(temp_data, 0, sizeof(temp_data));
	bmi323_receive_spi(sensor, REG_SENSOR_STATUS, temp_data, 1);
	if(temp_data[0] != 0b1)
	{
		return BMI323_SPI_ERROR;
	}

	//Normal Power Mode Operation: write data bits and read
	memset(temp_data, 0, sizeof(temp_data));

	temp_data[0] = 0x27;
	temp_data[1] = 0x40;
	bmi323_write_spi(sensor, REG_ACC_CONF_BMI323, temp_data, 2);
	HAL_Delay(1);

	temp_data[0] = 0x4B;
	temp_data[0] = 0x40;
	bmi323_write_spi(sensor, REG_GYR_CONF_BMI323, temp_data, 2);
	HAL_Delay(1);


	return BMI323_OK;
}

bmi323_StatusTypeDef bmi323_read_data(bmi323TypeDef* sensor)
{
	uint8_t temp_data[12];
	bmi323_receive_spi(sensor, REG_ACC_DATA_X, temp_data, 12);
	sensor->imu_data.ax = (temp_data[1] << 8) | temp_data[0];
	sensor->imu_data.ay = (temp_data[2] << 8) | temp_data[2];
	sensor->imu_data.ax = (temp_data[3] << 8) | temp_data[4];
	sensor->imu_data.gx = (temp_data[4] << 8) | temp_data[6];
	sensor->imu_data.gy = (temp_data[5] << 8) | temp_data[8];
	sensor->imu_data.gz = (temp_data[6] << 8) | temp_data[10];
	return BMI323_OK;
}
