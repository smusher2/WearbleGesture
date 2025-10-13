/* Combined main.c: BMI323 real streaming + paused SIM CLI + gesture detection */

#include "main.h"
#include "bmi_323.h"
#include "gesture.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ==== Peripherals from Cube ==== */
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;

/* ==== Scaling (adjust to your BMI323 gyro full-scale!) ==== */
/* If BMI323 gyro is ±1000 dps -> 32.768 LSB/dps
   If BMI323 gyro is ±500  dps -> 65.536 LSB/dps  */
#define GYR_LSB_PER_DPS   16.384f   /* <-- set to 65.536f if FS=±500 dps */
#define ACC_LSB_PER_G     4096.0f   /* accelerometer LSB/g if using ±8 g config; change if different */

/* For accel gate in gesture.c: it expects ~1000 = 1g.
   So we convert raw LSB -> legacy by: (raw / ACC_LSB_PER_G) * 1000. */
static inline int16_t acc_raw_to_legacy(int16_t raw) {
    float v = (raw * 1000.0f) / ACC_LSB_PER_G;
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

static char uart_buf[192];

/* ==== Prototypes (Cube + local) ==== */
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

/* ==== Simple help text ==== */
static void print_help(void)
{
    const char* msg =
        "\r\n==== IMU CONTROL (Paused by default) ====\r\n"
        "r: start REAL streaming (BMI323 @~100Hz)\r\n"
        "1: single REAL sample (step)\r\n"
        "p: pause\r\n"
        "t: intent_twist   (SIM detect)\r\n"
        "s: shake          (SIM reject)\r\n"
        "a: arm_swing      (SIM reject)\r\n"
        "b: bump           (SIM reject)\r\n"
        "o: other_axis     (SIM reject)\r\n"
        "q: quiet segment  (SIM)\r\n"
        "i: single quiet sample (SIM step)\r\n"
        "h: help\r\n"
        "CSV: ts_ms,source,gy_dps,th_hi,gesture,confidence\r\n\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, (uint16_t)strlen(msg), HAL_MAX_DELAY);
}

/* ==== Run source state ==== */
typedef enum { SRC_PAUSED=0, SRC_REAL, SRC_SIM } run_src_t;
static run_src_t run_src = SRC_PAUSED;

/* Timestamps:
   - For REAL, we use real time (HAL_GetTick) relative to start.
   - For SIM, we advance a synthetic time only when a sim sample is emitted. */
static uint32_t t0_real_ms = 0;
static uint32_t sim_ts_ms = 0;
#define STEP_MS 10u  /* ~100 Hz pacing for both real and sim */

/* ==== BMI323 helpers ==== */
static void bmi323_config_default(void)
{
    /* Basic init */
    (void)bmi323_init(&imu_sensor);

    /* Example: configure gyro ODR/range/mode.
       NOTE: Adjust for your exact driver/register map.
       The following matches your earlier code (ODR=100 Hz, RANGE=±500, MODE=normal enable).
       If you truly want ±1000 dps, change RANGE field accordingly and update GYR_LSB_PER_DPS. */
    uint8_t gyr_cfg[2];
    /* GYR_CONF: [7]=0, [6:4]=range, [3:0]=ODR/bw (example values) */
    gyr_cfg[0] = (0<<7) | (0x4<<4) | 0x8;  // RANGE=0x3 (±500 dps on your map), ODR ~100 Hz
    gyr_cfg[1] = 0x40;                      // mode = 0b100 (enabled)
    bmi323_write_spi(&imu_sensor, REG_GYR_CONF_BMI323, gyr_cfg, 2);
}

/* Read one sample from BMI323 and convert to gesture units */
static void read_real_one_sample(int16_t *ax_out, int16_t *ay_out, int16_t *az_out,
                                 int16_t *gx_out, int16_t *gy_out, int16_t *gz_out)
{
    bmi323_read_data(&imu_sensor);

    /* Raw integer from driver */
    int16_t axr = imu_sensor.imu_data.ax;
    int16_t ayr = imu_sensor.imu_data.ay;
    int16_t azr = imu_sensor.imu_data.az;
    int16_t gxr = imu_sensor.imu_data.gx;
    int16_t gyr = imu_sensor.imu_data.gy;
    int16_t gzr = imu_sensor.imu_data.gz;

    /* Convert to units expected by gesture.c */
    *ax_out = acc_raw_to_legacy(axr);  /* ~1000 = 1 g */
    *ay_out = acc_raw_to_legacy(ayr);
    *az_out = acc_raw_to_legacy(azr);
    *gx_out = gyr_raw_to_dps_i16(gxr); /* dps integer */
    *gy_out = gyr_raw_to_dps_i16(gyr);
    *gz_out = gyr_raw_to_dps_i16(gzr);
}

/* ==== Application entry ==== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    PeriphCommonClock_Config();

    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_USART1_UART_Init();

    bmi323_config_default();

    /* CLI + CSV */
    print_help();
    int n = snprintf(uart_buf, sizeof(uart_buf),
                     "ts_ms,source,gy_dps,th_hi,gesture,confidence\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, (uint16_t)n, HAL_MAX_DELAY);

    /* Start paused */
    run_src = SRC_PAUSED;
    sim_set_case(SC_NONE);
    const char* paused = "{status:paused}\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)paused, (uint16_t)strlen(paused), HAL_MAX_DELAY);

    /* Real-time base for REAL mode */
    t0_real_ms = HAL_GetTick();
    sim_ts_ms = 0;

    while (1) {
        /* ----- 1) Poll UART (non-blocking) for a single-key command ----- */
        uint8_t rx;
        if (HAL_UART_Receive(&huart1, &rx, 1, 0) == HAL_OK) {
            switch (rx) {
            case 'h': case 'H':
                print_help();
                break;
            case 'p': case 'P':
                run_src = SRC_PAUSED;
                sim_set_case(SC_NONE);
                HAL_UART_Transmit(&huart1, (uint8_t*)paused, (uint16_t)strlen(paused), HAL_MAX_DELAY);
                break;

            /* ---- REAL controls ---- */
            case 'r': case 'R':
                run_src = SRC_REAL;
                t0_real_ms = HAL_GetTick(); /* restart relative clock */
                HAL_UART_Transmit(&huart1, (uint8_t*)"{status:running_real}\r\n", 22, HAL_MAX_DELAY);
                break;

            case '1': { /* single REAL sample step (while staying paused) */
                int16_t ax, ay, az, gx, gy, gz;
                read_real_one_sample(&ax, &ay, &az, &gx, &gy, &gz);

                gesture_event_t evt = gesture_process(ax, ay, az, gx, gy, gz);

                uint32_t ts = HAL_GetTick() - t0_real_ms;
                float gy_f = (float)evt.gy;               // ensure floating-point precision
                int th_i = (int)(gesture_dbg_th_hi() + 0.5f);

                n = snprintf(uart_buf, sizeof(uart_buf),
                             "%lu,real,%.3f,%d,%s,%d\r\n",  // <-- use %.3f to show decimals
                             (unsigned long)ts, gy_f, th_i, evt.gesture, evt.confidence);

                HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, (uint16_t)n, HAL_MAX_DELAY);
                /* remain paused */
                break;
            }


            /* ---- SIM controls ---- */
            case 't': case 's': case 'a': case 'b': case 'o': case 'q':
                run_src = SRC_SIM;
                sim_set_case_by_char((char)rx);
                HAL_UART_Transmit(&huart1, (uint8_t*)"{status:running_sim}\r\n", 22, HAL_MAX_DELAY);
                break;

            case 'i': case 'I': { /* single SIM quiet sample */
                sim_set_case(SC_QUIET); /* will produce a single quiet sample */
                int16_t ax, ay, az, gx, gy, gz;
                imu_generate_sim(&ax, &ay, &az, &gx, &gy, &gz);
                gesture_event_t evt = gesture_process(ax, ay, az, gx, gy, gz);

                sim_ts_ms += STEP_MS;  /* synthetic time advances only on emission */
                int th_i = (int)(gesture_dbg_th_hi() + 0.5f);
                const char* scen = imu_sim_label(); /* will read "paused" after consumption, fine */

                n = snprintf(uart_buf, sizeof(uart_buf),
                             "%u,%s,%d,%d,%s,%d\r\n",
                             (unsigned)sim_ts_ms, scen, (int)evt.gy, th_i, evt.gesture, evt.confidence);
                HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, (uint16_t)n, HAL_MAX_DELAY);

                sim_set_case(SC_NONE); /* stay paused */
                break; }

            default:
                /* ignore unknown keys */
                break;
            }
        }

        /* ----- 2) Active sources ----- */
        if (run_src == SRC_REAL) {
            int16_t ax, ay, az, gx, gy, gz;
            read_real_one_sample(&ax, &ay, &az, &gx, &gy, &gz);

            gesture_event_t evt = gesture_process(ax, ay, az, gx, gy, gz);
            uint32_t ts = HAL_GetTick() - t0_real_ms;
            int th_i = (int)(gesture_dbg_th_hi() + 0.5f);

            int n2 = snprintf(uart_buf, sizeof(uart_buf),
                              "%u,real,%d,%d,%s,%d\r\n",
                              (unsigned)ts, (int)evt.gy, th_i, evt.gesture, evt.confidence);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, (uint16_t)n2, HAL_MAX_DELAY);

            HAL_Delay(STEP_MS); /* pace near ODR */

        } else if (run_src == SRC_SIM && sim_is_active()) {
            int16_t ax, ay, az, gx, gy, gz;
            imu_generate_sim(&ax, &ay, &az, &gx, &gy, &gz);

            gesture_event_t evt = gesture_process(ax, ay, az, gx, gy, gz);
            const char* scen = imu_sim_label();
            int th_i = (int)(gesture_dbg_th_hi() + 0.5f);

            sim_ts_ms += STEP_MS; /* synthetic time */
            int n2 = snprintf(uart_buf, sizeof(uart_buf),
                              "%u,%s,%d,%d,%s,%d\r\n",
                              (unsigned)sim_ts_ms, scen, (int)evt.gy, th_i,
                              evt.gesture, evt.confidence);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, (uint16_t)n2, HAL_MAX_DELAY);

            HAL_Delay(STEP_MS);

            if (!sim_is_active()) {
                run_src = SRC_PAUSED;
                HAL_UART_Transmit(&huart1, (uint8_t*)paused, (uint16_t)strlen(paused), HAL_MAX_DELAY);
            }
        } else {
            HAL_Delay(1); /* paused: ultra-light idle */
        }
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

  HAL_GPIO_WritePin(GPIOB, BLED_PIN_Pin|GLED_PIN_Pin|RLED_PIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO_CS_GPIO_Port, GPIO_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HAPTIC_PIN_GPIO_Port, HAPTIC_PIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BLED_PIN_Pin|GLED_PIN_Pin|RLED_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = HAPTIC_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HAPTIC_PIN_GPIO_Port, &GPIO_InitStruct);

  /* SPI2 pins */
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

  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
