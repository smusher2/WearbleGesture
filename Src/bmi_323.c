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

static void feature_wait_ready(bmi323TypeDef* sensor, uint32_t to_ms)
{
    uint32_t t0 = HAL_GetTick();
    for (;;) {
        uint8_t st = 0;
        bmi323_receive_spi(sensor, FEATURE_DATA_STATUS, &st, 1);
        if ((st & 0x02)) return;      // BUSY=0 → ready
        if ((HAL_GetTick() - t0) >= to_ms) return;
    }
}

static void feature_write_word(bmi323TypeDef* sensor, uint8_t feat_addr, uint8_t* val)
{
    uint8_t addr = feat_addr;
    bmi323_write_spi(sensor, FEATURE_DATA_ADDR, &addr, 1);
    feature_wait_ready(sensor, 10);

    bmi323_write_spi(sensor, FEATURE_DATA_TX, val, 2);
    feature_wait_ready(sensor, 10);

    // Optional: poll FEATURE_DATA_STATUS for not-busy here
}

static void feature_read_word(bmi323TypeDef* sensor, uint8_t feat_addr, uint8_t* val)
{
    uint8_t addr = feat_addr;
    bmi323_write_spi(sensor, FEATURE_DATA_ADDR, &addr, 1);
    feature_wait_ready(sensor, 5);

    bmi323_receive_spi(sensor, FEATURE_DATA_TX, val, 2);
    feature_wait_ready(sensor, 5);

}


void bmi323_enable_tap(bmi323TypeDef* sensor)
{

	extern UART_HandleTypeDef huart1;  // ✅ use the real one from usart.c
	char msg[32];  // buffer for UART printing

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



