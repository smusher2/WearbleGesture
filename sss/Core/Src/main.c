/* ======================= Merged + Lean main.c ============================
   - Prints ONLY the detected gesture (conf) when it changes
   - Tap IRQ path now RETURNS a label; main prints it
   ======================================================================= */

#include "main.h"
#include "bmi_323.h"
#include "gesture.h"
#include "gpio.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ==== Cube peripherals ==== */
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;

/* ==== IMU scaling (match your BMI323 configs) ==== */
/* If BMI323 gyro is ±2000 dps -> 16.384 LSB/dps (your first file) */
/* If you change ranges, update these two constants. */
#define GYR_LSB_PER_DPS   16.384f   /* ±2000 dps  */
#define ACC_LSB_PER_G     4096.0f   /* ±8 g       */

/* Convert BMI323 raw to units expected by gesture.c
   - gesture.c expects accel ~1000 = 1g (legacy int)
   - gyro as integer dps */
static inline int16_t acc_raw_to_legacy(int16_t raw) {
    float v = (raw * 1000.0f) / ACC_LSB_PER_G;     /* 1000 ≈ 1 g */
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)(v + (v >= 0 ? 0.5f : -0.5f));
}
static inline int16_t gyr_raw_to_dps_i16(int16_t raw) {
    float dps = raw / GYR_LSB_PER_DPS;
    if (dps > 32767.0f) dps = 32767.0f;
    if (dps < -32768.0f) dps = -32768.0f;
    return (int16_t)(dps + (dps >= 0 ? 0.5f : -0.5f));
}

/* ==== BMI323 instance ==== */
bmi323TypeDef imu_sensor = {
    .bmi323_spi = &hspi2,
    .spi_cs_pin = SPI2_CS_Pin,
    .spi_cs_port = SPI2_CS_GPIO_Port
};

/* ==== Tap IRQ + optional haptics ==== */
volatile uint8_t g_tap_irq = 0;
uint32_t haptic_deadline = 0;
uint8_t  haptic_active   = 0;

/* ==== Prototypes (Cube) ==== */
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

//PRINTING HELPER
static void pretty_label(const char *in, char *out, size_t outsz) {
    if (!in || !out || outsz == 0) return;
    size_t j = 0;
    int new_word = 1;
    for (size_t i = 0; in[i] && j + 1 < outsz; ++i) {
        char c = in[i];
        if (c == '_' || c == '-' || c == ' ') {          // word separator
            if (j && out[j-1] != ' ') out[j++] = ' ';
            new_word = 1;
            continue;
        }
        if (new_word) {
            if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
            new_word = 0;
        } else {
            if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
        }
        out[j++] = c;
    }
    out[j] = '\0';
}


/* ==== UART helper ==== */
static void uart_print(const char *s) {
    HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

/* ==== Configure BMI323 like your first file (ODR/range/mode) ==== */
static void bmi323_config_default(void)
{
    (void)bmi323_init(&imu_sensor);
    (void)BMI323_Feature_Engine_Enable(&imu_sensor);  /* feature engine on */

    /* Gyro: BW=0, RANGE=±2000 dps, ODR≈100 Hz, mode=enable */
    uint8_t gyr[2];
    gyr[0] = (0<<7) | (0x4<<4) | 0xA;   /* field mapping per your driver */
    gyr[1] = 0x40;                      /* normal/enable */
    bmi323_write_spi(&imu_sensor, REG_GYR_CONF_BMI323, gyr, 2);
    HAL_Delay(10);

    /* Accel: BW=0, RANGE=±8 g, ODR≈200 Hz, mode=enable */
    uint8_t acc[2];
    acc[0] = (0<<7) | (0x2<<4) | 0xA;
    acc[1] = 0x40;                      /* normal/enable */
    bmi323_write_spi(&imu_sensor, REG_ACC_CONF_BMI323, acc, 2);
    HAL_Delay(10);

    /* Tap detection engine */
    bmi323_enable_tap(&imu_sensor);
}

/* ==== Read & convert one BMI323 sample for gesture engine ==== */
static void read_one_sample_for_gesture(int16_t *ax, int16_t *ay, int16_t *az,
                                        int16_t *gx, int16_t *gy, int16_t *gz)
{
    bmi323_read_data(&imu_sensor);

    /* Raw from driver */
    int16_t axr = imu_sensor.imu_data.ax;
    int16_t ayr = imu_sensor.imu_data.ay;
    int16_t azr = imu_sensor.imu_data.az;
    int16_t gxr = imu_sensor.imu_data.gx;
    int16_t gyr = imu_sensor.imu_data.gy;
    int16_t gzr = imu_sensor.imu_data.gz;

    /* Convert to gesture.c expectations */
    *ax = acc_raw_to_legacy(axr);  /* ~1000 = 1 g */
    *ay = acc_raw_to_legacy(ayr);
    *az = acc_raw_to_legacy(azr);
    *gx = gyr_raw_to_dps_i16(gxr); /* integer dps   */
    *gy = gyr_raw_to_dps_i16(gyr);
    *gz = gyr_raw_to_dps_i16(gzr);
}

/* ============================ Application ============================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    PeriphCommonClock_Config();

    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_USART1_UART_Init();

    HAL_Delay(1000);           /* give peripherals a moment */
    bmi323_config_default();

    uart_print("{ready}\r\n");

    /* Only print when gesture label changes or confidence band changes */
    char last_label[32] = "";
    int  last_conf = -1;
    const int MIN_CONF = 50;       /* suppress low-confidence noise */
    const uint32_t STEP_MS = 10;   /* pace ~100 Hz to match gyro ODR */

    while (1)
    {
        /* ---- TAP EVENTS: handled here (main prints) ---- */
        if (g_tap_irq) {
            g_tap_irq = 0;
            const char* tap_label = BMI323_HandleTapEvent(&imu_sensor, &haptic_deadline, &haptic_active);
            if (tap_label) {
                char tline[48];
                int tn = snprintf(tline, sizeof(tline), "%s\r\n", tap_label);
                HAL_UART_Transmit(&huart1, (uint8_t*)tline, (uint16_t)tn, HAL_MAX_DELAY);
            }
        }

        /* Optional: keep haptic buzz timing updated even without new taps */
        Haptic_Buzz_Check2STOP(&haptic_deadline, &haptic_active);

        /* ---- CONTINUOUS GESTURE CLASSIFIER ---- */
        int16_t ax, ay, az, gx, gy, gz;
        read_one_sample_for_gesture(&ax, &ay, &az, &gx, &gy, &gz);

        gesture_event_t evt = gesture_process(ax, ay, az, gx, gy, gz);
        /* evt.gesture: C-string label, evt.confidence: int (0..100) */

        int should_print = 0;

        /* 1) New label vs last label */
        if (strcmp(evt.gesture, last_label) != 0) {
            if (evt.confidence >= MIN_CONF) should_print = 1;
            else {
                /* If classifier cleared to "none"/"idle", still show it once */
                if (last_label[0] != '\0' && evt.gesture[0] == '\0') should_print = 1;
            }
        }
        /* 2) Same label but confidence changed band (every 10 points) */
        else if (evt.confidence >= MIN_CONF && evt.confidence != last_conf) {
            int prev_band = (last_conf < 0) ? -1 : (last_conf / 10);
            int curr_band = evt.confidence / 10;
            if (curr_band != prev_band) should_print = 1;
        }

        if (should_print) {
            char pretty[64];
            const char *raw = (evt.gesture[0] ? evt.gesture : "none");
            pretty_label(raw, pretty, sizeof(pretty));
            int n = snprintf(pretty, sizeof(pretty), "%s\r\n", pretty);  // reuse buffer for transmit
            HAL_UART_Transmit(&huart1, (uint8_t*)pretty, (uint16_t)n, HAL_MAX_DELAY);

            /* remember */
            strncpy(last_label, evt.gesture, sizeof(last_label)-1);
            last_label[sizeof(last_label)-1] = '\0';
            last_conf = evt.confidence;
        }

        HAL_Delay(STEP_MS);
    }
}

/* ===================== CubeMX init code (unchanged) ===================== */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}

void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }
}

static void MX_SPI2_Init(void)
{
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
  if (HAL_SPI_Init(&hspi2) != HAL_OK) { Error_Handler(); }
}

static void MX_USART1_UART_Init(void)
{
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
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
  HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8);
  HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8);
  HAL_UARTEx_DisableFifoMode(&huart1);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* LED pins */
  HAL_GPIO_WritePin(GPIOB, BLED_PIN_Pin|GLED_PIN_Pin|RLED_PIN_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = BLED_PIN_Pin|GLED_PIN_Pin|RLED_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Generic CS & Haptic pins */
  HAL_GPIO_WritePin(GPIO_CS_GPIO_Port, GPIO_CS_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_CS_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(HAPTIC_PIN_GPIO_Port, HAPTIC_PIN_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = HAPTIC_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HAPTIC_PIN_GPIO_Port, &GPIO_InitStruct);

  /* SPI2 CS default high */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /* SPI2 SCK / MISO / MOSI */
  GPIO_InitStruct.Pin = SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;     /* PB14=MISO, PB15=MOSI */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* TAP INT pin as rising-edge interrupt */
  GPIO_InitStruct.Pin  = TAP_INT_Pin;                /* e.g., PD8 */
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TAP_INT_GPIO_Port, &GPIO_InitStruct);

  /* NVIC for EXTI lines 9..5 (covers PD8) */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* ---- EXTI callback: flag tap IRQ, handled in main loop ---- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == TAP_INT_Pin) {
        g_tap_irq = 1;
    }
}

/* Standard error handler */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
