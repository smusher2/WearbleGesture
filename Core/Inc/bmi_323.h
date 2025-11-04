
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
#define TAP_1                   0x1E
#define TAP_2                   0x1F
#define TAP_3                   0x20

/* =========================== TUNING KNOBS ============================== */
/* --- Tap enable switches (FEATURE_IO0) --- */
#define TAP_ENABLE_SINGLE             1   /* 1=enable single tap */
#define TAP_ENABLE_DOUBLE             1   /* 1=enable double tap */
#define TAP_ENABLE_TRIPLE             0   /* 1=enable triple tap (if used) */

/* --- TAP_1 fields --- */
#define TAP1_MODE                     2   /* 0=basic, 1=balanced, 2=robust */
#define TAP1_MAX_PEAKS_FOR_TAP        3   /* 0..7 (datasheet default 6). 2–3 resists shake */

/* --- TAP_2 fields --- */
#define TAP2_THRESH_SCALE_X100        120 /* scale current threshold by % (200=2x, 300=3x) */
#define TAP2_MAX_GESTURE_DUR_MS       350 /* double-tap window in ms (40 ms/LSB; clamp 0..2520) */


#define TAP3_MAX_DUR_BETWEEN_PEAKS    2   /* default 4 → try 2 (narrow single tap impulse) */
#define TAP3_TAP_SHOCK_SETTLING_DUR   9   /* default 6 → 8–10 (ignore post-shock ringing)  */
#define TAP3_MIN_QUIET_BETWEEN_TAPS   12  /* default 8 → 10–12 (require quiet)            */
#define TAP3_QUIET_TIME_AFTER_GESTURE 10  /* default 6 → 8–10 (cooldown)                   */

/* --- Optional: make single/double stricter at runtime without changing T2 ---
 * If arm pumps still false-trigger, try increasing scale to 300 and/or
 * reducing TAP2_MAX_GESTURE_DUR_MS to ~180.
 */

/* --- Gyro gate (alternative to accel span) --- */
#define GYRO_GATE_ENABLE              0
#define GYRO_GATE_THRESH_SUM_DPS      250  /* ignore taps if |gx|+|gy|+|gz| > this */

/* USER CODE BEGIN Private defines */
#define FEATURE_DATA_ADDR       0x41
#define FEATURE_DATA_TX         0x42
#define FEATURE_DATA_STATUS     0x43
#define FE_ST_TX_READY          0x02u  /* bit1 == data_tx_ready */

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
const char* BMI323_HandleTapEvent(bmi323TypeDef* sensor, uint32_t* haptic_deadline, uint8_t* haptic_active);


#endif /* INC_BMI_323_H_ */
