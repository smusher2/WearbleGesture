
//BMI_323 H File
//written by Yifan Wang
//9/27/2025


#ifndef INC_BMI_323_H_
#define INC_BMI_323_H_

#include "main.h"
#include "gpio.h"
#include "stm32wbxx_hal.h"



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







#define CHIP_ID_BMI323   0x43
#define SOFT_RESET_CMD_BMI323 0xDEAF

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
bmi323_StatusTypeDef BMI323_Feature_Engine_Enable(bmi323TypeDef* sensor);
void bmi323_write_spi(bmi323TypeDef* sensor, uint8_t reg, uint8_t* data, uint16_t len);
void bmi323_enable_tap(bmi323TypeDef* sensor);
void BMI323_HandleTapEvent(bmi323TypeDef* sensor, uint32_t* haptic_deadline, uint8_t* haptic_active);





#endif /* INC_BMI_323_H_ */
