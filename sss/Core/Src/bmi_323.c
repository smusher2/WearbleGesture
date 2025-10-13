/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bmi323.c
  * @brief          : bmi323 program body
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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void bmi323_activate_spi(bmi323TypeDef* sensor)
{
	HAL_GPIO_WritePin(sensor->spi_cs_port, sensor->spi_cs_pin, GPIO_PIN_RESET);
}

void bmi323_deactivate_spi(bmi323TypeDef* sensor)
{
	HAL_GPIO_WritePin(sensor->spi_cs_port, sensor->spi_cs_pin, GPIO_PIN_SET);
}

/*
 static void bmi323_write_spi(bmi323TypeDef* sensor, uint8_t offset_reg, uint8_t* data, uint16_t data_size)
{
	uint8_t data_temp[20];
	bmi323_activate_spi(sensor);
	data_temp[0] = offset_reg;
	HAL_SPI_Transmit(sensor->bmi323_spi, data_temp, 1, 1000);
	HAL_SPI_Transmit(sensor->bmi323_spi, data, data_size, 1000);
	bmi323_deactivate_spi(sensor);
}
 */


/*
 *static void bmi323_receive_spi(bmi323TypeDef* sensor, uint8_t offset_reg, uint8_t* data, uint16_t data_size)
{
	uint8_t data_temp[20], data_temp_byte;
	data_temp[0] = offset_reg | 0x80;
	bmi323_activate_spi(sensor);
	HAL_SPI_Transmit(sensor->bmi323_spi, data_temp, 1, 1000);
	//HAL_SPI_Receive(sensor->bmi323_spi, &data_temp_byte, 1, 1000);
	//HAL_SPI_Receive(sensor->bmi323_spi, data, data_size, 1000);
	uint8_t dummy_tx = 0xFF;
	HAL_SPI_TransmitReceive(sensor->bmi323_spi, &dummy_tx, &data_temp_byte, 1, 1000);
	HAL_SPI_TransmitReceive(sensor->bmi323_spi, &dummy_tx, data, data_size, 1000);
	bmi323_deactivate_spi(sensor);
}
 */

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
	temp_data[0] = 0x27;
	temp_data[1] = 0x40;
	bmi323_write_spi(sensor, REG_ACC_CONF_BMI323, temp_data, 2);
	HAL_Delay(10);

	temp_data[0] = 0x4B;
	temp_data[1] = 0x40;
	bmi323_write_spi(sensor, REG_GYR_CONF_BMI323, temp_data, 2);
	HAL_Delay(10);


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

/*
 *
 * 	char msg[200];
	snprintf(msg, sizeof(msg),
	         "DATA: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
	         temp_data[0], temp_data[1], temp_data[2], temp_data[3],
	         temp_data[4], temp_data[5], temp_data[6], temp_data[7],
	         temp_data[8], temp_data[9], temp_data[10], temp_data[11]);
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
 */


	return BMI323_OK;


}

