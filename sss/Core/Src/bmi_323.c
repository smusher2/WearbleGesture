/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bmi_323.c
  * @brief          : BMI323 driver (SPI + feature engine + tap)
  ******************************************************************************
  * @attention
  * Copyright (c) 2025.
  * This software is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "bmi_323.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ====================== Feature engine status bits ====================== */
#define FEATURE_DATA_ADDR     0x41
#define FEATURE_DATA_TX       0x42
#define FEATURE_DATA_STATUS   0x43
/* datasheet: bit1 == data_tx_ready (1 = ready) */
#define FE_ST_TX_READY        0x02u

/* ========================= Local prototypes ============================ */
static void     bmi323_activate_spi(bmi323TypeDef* s);
static void     bmi323_deactivate_spi(bmi323TypeDef* s);
static void     bmi323_receive_spi(bmi323TypeDef* s, uint8_t reg, uint8_t* data, uint16_t len);

static void     fe_wait_ready(bmi323TypeDef* s, uint32_t to_ms);
static void     fe_set_addr(bmi323TypeDef* s, uint16_t addr11);
static void     feature_write_word(bmi323TypeDef* s, uint16_t addr, const uint8_t *val);
static void     feature_read_word (bmi323TypeDef* s, uint16_t addr, uint8_t *out);

static void     debug_feature_engine_status(bmi323TypeDef* s);

/* ========================= SPI helpers ================================= */
static void bmi323_activate_spi(bmi323TypeDef* s)
{
    HAL_GPIO_WritePin(s->spi_cs_port, s->spi_cs_pin, GPIO_PIN_RESET);
}

static void bmi323_deactivate_spi(bmi323TypeDef* s)
{
    HAL_GPIO_WritePin(s->spi_cs_port, s->spi_cs_pin, GPIO_PIN_SET);
}

/* One-address read with dummy byte (CS held low throughout) */
static void bmi323_receive_spi(bmi323TypeDef* s, uint8_t reg, uint8_t* data, uint16_t len)
{
    if (!data || !len) return;

    uint8_t addr = (uint8_t)(reg | 0x80u);
    uint8_t dummy;

    bmi323_activate_spi(s);
    HAL_SPI_Transmit(s->bmi323_spi, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive (s->bmi323_spi, &dummy, 1, HAL_MAX_DELAY); /* throwaway */
    HAL_SPI_Receive (s->bmi323_spi, data, len, HAL_MAX_DELAY);
    bmi323_deactivate_spi(s);
}

void bmi323_write_spi(bmi323TypeDef* s, uint8_t reg, uint8_t* data, uint16_t len)
{
    uint8_t hdr = (uint8_t)(reg & 0x7Fu);
    bmi323_activate_spi(s);
    HAL_SPI_Transmit(s->bmi323_spi, &hdr, 1, HAL_MAX_DELAY);
    if (len && data) HAL_SPI_Transmit(s->bmi323_spi, data, len, HAL_MAX_DELAY);
    bmi323_deactivate_spi(s);
}

/* ======================= Feature engine (mailbox) ====================== */
/* Poll readiness BEFORE a transfer starts (safe anytime). */
static void fe_wait_ready(bmi323TypeDef* s, uint32_t to_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < to_ms) {
        uint8_t st = 0;
        bmi323_receive_spi(s, FEATURE_DATA_STATUS, &st, 1);
        if (st & FE_ST_TX_READY) return;
    }
}

/* 0x41 is 16-bit; only lower 11 bits are address. Write BOTH bytes, LSB first. */
static void fe_set_addr(bmi323TypeDef* s, uint16_t addr11)
{
    uint8_t a[2] = { (uint8_t)(addr11 & 0xFFu), (uint8_t)((addr11 >> 8) & 0x07u) };
    bmi323_write_spi(s, FEATURE_DATA_ADDR, a, 2);
}

/* DO NOT touch any other register between ADDR and TX for the same transfer */
static void feature_write_word(bmi323TypeDef* s, uint16_t addr, const uint8_t *val)
{
    if (!val) return;
    fe_wait_ready(s, 10);                       /* safe to poll before */
    fe_set_addr(s, addr);                       /* 1) set which word */
    bmi323_write_spi(s, FEATURE_DATA_TX, (uint8_t*)val, 2); /* 2) write LSB,MSB */
    /* transfer complete */
}

static void feature_read_word(bmi323TypeDef* s, uint16_t addr, uint8_t *out)
{
    if (!out) return;
    fe_wait_ready(s, 10);                       /* safe to poll before */
    fe_set_addr(s, addr);                       /* 1) set which word */
    bmi323_receive_spi(s, FEATURE_DATA_TX, out, 2); /* 2) read LSB,MSB */
    /* transfer complete */
}

/* ========================= Debug helper ================================ */
static void debug_feature_engine_status(bmi323TypeDef* s)
{
    extern UART_HandleTypeDef huart1;
    char msg[96];
    uint8_t io1[2] = {0}, fctrl[2] = {0}, st = 0;

    bmi323_receive_spi(s, FEATURE_IO1_REG, io1, 2);
    bmi323_receive_spi(s, FEATURE_CTRL_REG, fctrl, 2);
    bmi323_receive_spi(s, FEATURE_DATA_STATUS, &st, 1);

    snprintf(msg, sizeof(msg),
             "FE dbg: IO1[0]=0x%02X IO1[1]=0x%02X CTRL[0]=0x%02X STATUS=0x%02X\r\n",
             io1[0], io1[1], fctrl[0], st);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/* ========================= Public API ================================= */

bmi323_StatusTypeDef bmi323_init(bmi323TypeDef* s)
{
    extern UART_HandleTypeDef huart1;
    uint8_t v = 0;
    char msg[64];

    /* SPI mode “latch” wiggle */
    bmi323_activate_spi(s); HAL_Delay(2);
    bmi323_deactivate_spi(s); HAL_Delay(2);

    /* Chip ID */
    bmi323_receive_spi(s, REG_CHIP_ID_BMI323, &v, 1);  /* dummy */
    bmi323_receive_spi(s, REG_CHIP_ID_BMI323, &v, 1);  /* real  */
    snprintf(msg, sizeof(msg), "BMI323 Chip ID: 0x%02X\r\n", v);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    if ((v != CHIP_ID_BMI323) && (v != 0xA6) && (v != 0x41)) return BMI323_SPI_CHIP_ERROR;

    /* ERR */
    bmi323_receive_spi(s, REG_ERR, &v, 1);
    snprintf(msg, sizeof(msg), "BMI323 ERR: 0x%02X\r\n", v);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    if (v != 0x00) return BMI323_SPI_ERROR;

    /* SENSOR_STATUS */
    bmi323_receive_spi(s, REG_SENSOR_STATUS, &v, 1);
    snprintf(msg, sizeof(msg), "BMI323 SENSOR_STATUS: 0x%02X\r\n", v);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    return BMI323_OK;
}

bmi323_StatusTypeDef bmi323_read_data(bmi323TypeDef* s)
{
    uint8_t b[12];
    bmi323_receive_spi(s, REG_ACC_DATA_X, b, sizeof b);

    s->imu_data.ax = (int16_t)((b[1] << 8) | b[0]);
    s->imu_data.ay = (int16_t)((b[3] << 8) | b[2]);
    s->imu_data.az = (int16_t)((b[5] << 8) | b[4]);
    s->imu_data.gx = (int16_t)((b[7] << 8) | b[6]);
    s->imu_data.gy = (int16_t)((b[9] << 8) | b[8]);
    s->imu_data.gz = (int16_t)((b[11] << 8) | b[10]);
    return BMI323_OK;
}

/* Call this once before programming feature words (TAP_x, etc.) */
bmi323_StatusTypeDef BMI323_Feature_Engine_Enable(bmi323TypeDef* s)
{
    /* 1) FEATURE_IO2 = 0x012C (LSB first) */
    uint8_t io2[2] = { 0x2C, 0x01 };
    bmi323_write_spi(s, FEATURE_IO2_REG, io2, 2);

    /* 2) FEATURE_IO_STATUS = 0x0001 (LSB first) */
    uint8_t iostat[2] = { 0x01, 0x00 };
    bmi323_write_spi(s, FEATURE_IO_STATUS_REG, iostat, 2);

    /* 3) FEATURE_CTRL.engine_en = 1 */
    uint8_t fctrl[2] = {0};
    bmi323_receive_spi(s, FEATURE_CTRL_REG, fctrl, 2);
    fctrl[0] |= 0x01u;
    bmi323_write_spi(s, FEATURE_CTRL_REG, fctrl, 2);

    /* 4) Wait for engine “ready” on IO1[3:0] == 0b0001 */
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 50) {
        uint8_t io1[2] = {0};
        bmi323_receive_spi(s, FEATURE_IO1_REG, io1, 2);
        if ((io1[0] & 0x0Fu) == 0x01u) {
            debug_feature_engine_status(s);
            return BMI323_OK;
        }
    }
    debug_feature_engine_status(s);
    return BMI323_CONFIG_FILE_ERROR;
}


void bmi323_reduce_shake_false_taps(bmi323TypeDef* s)
{
    extern UART_HandleTypeDef huart1;
    char msg[128];

    /* 1) Force Robust mode: TAP_1.mode = 0b10 (bits [7:6]) */
    uint8_t t1[2] = {0};
    feature_read_word(s, TAP_1, t1);
    t1[0] = (uint8_t)((t1[0] & ~(0x3u << 6)) | (0x2u << 6));
    feature_write_word(s, TAP_1, t1);

    /* 2) Disable single-tap, keep double-tap enabled (FEATURE_IO0 MSB bits 4/5) */
    uint8_t io0[2] = {0};
    bmi323_receive_spi(s, FEATURE_IO0_REG, io0, 2);
    io0[1] &= ~(1u << 4);  /* single tap OFF */
    io0[1] |=  (1u << 5);  /* double tap ON  */
    bmi323_write_spi(s, FEATURE_IO0_REG, io0, 2);

    /* 3) Raise TAP_2 peak threshold (bits [9:0]) by +50% to ignore low-g shakes */
    uint8_t t2b[2] = {0};
    feature_read_word(s, TAP_2, t2b);
    uint16_t t2 = (uint16_t)((t2b[1] << 8) | t2b[0]);
    uint16_t th = (uint16_t)(t2 & 0x03FFu);                /* current threshold */
    uint16_t new_th = (uint16_t)(th + th/2);               /* +50% */
    if (new_th > 0x03FFu) new_th = 0x03FFu;                /* clamp to 10 bits */
    t2 = (uint16_t)((t2 & ~0x03FFu) | new_th);
    t2b[0] = (uint8_t)(t2 & 0xFF);
    t2b[1] = (uint8_t)(t2 >> 8);
    feature_write_word(s, TAP_2, t2b);

    /* 4) Read-back & print summary (also shows current double-tap window) */
    feature_read_word(s, TAP_2, t2b);
    t2 = (uint16_t)((t2b[1] << 8) | t2b[0]);
    th = (uint16_t)(t2 & 0x03FFu);
    uint8_t dur_code = (uint8_t)((t2 >> 10) & 0x3F);       /* 40 ms / LSB */
    uint16_t dur_ms = (uint16_t)(dur_code * 40);

    snprintf(msg, sizeof(msg),
             "Anti-shake: singleTap=OFF, doubleTap=ON, TAP2.thr=%u (0x%03X), window~%u ms\r\n",
             th, th, dur_ms);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// Sets double-tap window (EXT.TAP_2.max_gesture_dur bits [15:10]) in ms.
// Quantized at 40 ms/LSB; the value is floored (e.g., 350 → 320 ms).
void bmi323_set_double_tap_window_ms(bmi323TypeDef* s, uint16_t window_ms)
{
    // 1) Read TAP_2 (LSB first)
    uint8_t t2[2] = {0};
    feature_read_word(s, TAP_2, t2);

    // 2) Update duration field (bits [15:10]) and keep threshold bits [9:0]
    uint16_t t2u = (uint16_t)((t2[1] << 8) | t2[0]);
    uint8_t  dur = (uint8_t)(window_ms / 40u);   // 40 ms per LSB
    if (dur > 63u) dur = 63u;                    // 6-bit field
    t2u = (uint16_t)((t2u & 0x03FFu) | ((uint16_t)dur << 10));

    // 3) Write back (LSB first)
    t2[0] = (uint8_t)(t2u & 0xFFu);
    t2[1] = (uint8_t)(t2u >> 8);
    feature_write_word(s, TAP_2, t2);
}


void bmi323_enable_tap(bmi323TypeDef* s)
{
    extern UART_HandleTypeDef huart1;
    char buf[128];

    /* 1) Keep feature engine enabled */
    {
        uint8_t fctrl[2] = {0};
        bmi323_receive_spi(s, FEATURE_CTRL_REG, fctrl, 2);
        fctrl[0] |= 0x01u;
        bmi323_write_spi(s, FEATURE_CTRL_REG, fctrl, 2);
    }

    /* 2) Enable TAP events in FEATURE_IO0 (we’ll later disable single-tap in anti-shake) */
    {
        uint8_t io0[2] = {0};
        bmi323_receive_spi(s, FEATURE_IO0_REG, io0, 2);
        io0[1] |= (1u<<4) | (1u<<5);                  // single + double tap enable for now
        bmi323_write_spi(s, FEATURE_IO0_REG, io0, 2);
    }

    /* 3) Map to INT1 via INT_MAP2, push-pull, active-high */
    {
        uint8_t map2[2] = {0};
        bmi323_receive_spi(s, INT_MAP2, map2, 2);
        map2[0] = (uint8_t)((map2[0] & ~0x03u) | 0x01u);
        bmi323_write_spi(s, INT_MAP2, map2, 2);

        uint8_t io[2] = {0};
        bmi323_receive_spi(s, IO_INT_CTRL, io, 2);
        io[0] |=  (1u<<2);          // int1_output_en
        io[0] &= ~(1u<<1);          // push-pull
        io[0] |=  (1u<<0);          // active high
        bmi323_write_spi(s, IO_INT_CTRL, io, 2);
    }

    /* 4) (Optional) print defaults once */
    {
        uint8_t t1[2]={0}, t2[2]={0}, t3[2]={0};
        feature_read_word(s, TAP_1, t1);
        feature_read_word(s, TAP_2, t2);
        feature_read_word(s, 0x20, t3);
        snprintf(buf, sizeof(buf),
                 "TAP defaults LSB: T1=%02X %02X  T2=%02X %02X  T3=%02X %02X\r\n",
                 t1[0], t1[1], t2[0], t2[1], t3[0], t3[1]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    }

    /* 5) Do tap tuning INSIDE the driver (no need to touch main.c) */
    // after mapping INTs and (optionally) setting your double-tap window:
    bmi323_set_double_tap_window_ms(s, 350);   // optional, you already have this
    bmi323_reduce_shake_keep_single(s);        // <-- use this one (keeps single ON)
          // robust mode, singleTap OFF, raise threshold

    // After your existing tap tuning in bmi323_enable_tap():
    {
        uint8_t t2b[2]; feature_read_word(s, TAP_2, t2b);
        uint16_t t2 = (uint16_t)((t2b[1] << 8) | t2b[0]);
        uint16_t thr = (uint16_t)(t2 & 0x00FFu);       // lower 10 bits = peak threshold
        uint16_t new_thr = thr * 1u;                   // 2× (try 3× if still too sensitive)
        if (new_thr > 0x03FFu) new_thr = 0x03FFu;
        t2 = (uint16_t)((t2 & ~0x03FFu) | new_thr);
        t2b[0] = (uint8_t)(t2 & 0xFF);
        t2b[1] = (uint8_t)(t2 >> 8);
        feature_write_word(s, TAP_2, t2b);
    }

    /* 6) Read-back & print final settings */
    {
        uint8_t t1[2]={0}, t2[2]={0};
        feature_read_word(s, TAP_1, t1);
        feature_read_word(s, TAP_2, t2);
        uint16_t t2u = (uint16_t)((t2[1] << 8) | t2[0]);
        uint8_t  dur_code = (uint8_t)((t2u >> 10) & 0x3F);
        uint16_t dur_ms   = (uint16_t)(dur_code * 40);
        snprintf(buf, sizeof(buf),
                 "TAP1=%02X%02X TAP2=%02X%02X | window~%u ms\r\n",
                 t1[1], t1[0], t2[1], t2[0], dur_ms);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    }

    /* 7) Clear any latched feature status (optional) */
    {
        uint8_t dummy[2];
        bmi323_receive_spi(s, FEATURE_IO_STATUS_REG, dummy, 2);
    }
}

// Return 1 if device is clearly "shaking" right now, else 0.
// Samples N times over ~24 ms and looks at |ax|+|ay|+|az| span.
// Assumes ±8 g (≈4096 LSB/g). Adjust SPAN_THRESH if you change range.
static int bmi323_is_shaking(bmi323TypeDef* s)
{
    const int N = 12;                 // ~24 ms @ 2 ms/sample
    int32_t min_sum = INT32_MAX;
    int32_t max_sum = INT32_MIN;

    for (int i = 0; i < N; ++i) {
        uint8_t b[6];
        bmi323_receive_spi(s, REG_ACC_DATA_X, b, 6);
        int16_t ax = (int16_t)((b[1] << 8) | b[0]);
        int16_t ay = (int16_t)((b[3] << 8) | b[2]);
        int16_t az = (int16_t)((b[5] << 8) | b[4]);

        // L1 norm (cheap): |ax|+|ay|+|az|
        int32_t sum = (ax < 0 ? -ax : ax) + (ay < 0 ? -ay : ay) + (az < 0 ? -az : az);
        if (sum < min_sum) min_sum = sum;
        if (sum > max_sum) max_sum = sum;

        HAL_Delay(2);
    }

    int32_t span = max_sum - min_sum;
    // Heuristic: if span > ~0.8 g total across axes, call it a shake.
    // With ±8g → 4096 LSB/g → 0.8 g ≈ 3277 LSB.
    const int32_t SPAN_THRESH = 3200;
    return (span > SPAN_THRESH) ? 1 : 0;
}


void BMI323_HandleTapEvent(bmi323TypeDef* sensor, uint32_t* haptic_deadline, uint8_t* haptic_active)
{
    extern UART_HandleTypeDef huart1;

    // Quick motion gate: if it's a shake burst, ignore the tap event.
    if (bmi323_is_shaking(sensor)) {
        const char* m = "Tap ignored (shake detected)\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)m, strlen(m), HAL_MAX_DELAY);

        // Still read/clear the latched event
        uint8_t dump[2];
        bmi323_receive_spi(sensor, FEATURE_EVENT_EXT, dump, 2);
        return;
    }

    // Normal event handling
    uint8_t tap_evt[2] = {0};
    bmi323_receive_spi(sensor, FEATURE_EVENT_EXT, tap_evt, 2); // [LSB, MSB]
    if (tap_evt[0] & (1u<<4)) {
        const char* s = "Double Tap\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
        // TODO: trigger your haptic here if desired
    } else if (tap_evt[0] & (1u<<3)) {
        const char* s = "Single Tap\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
    }
}

// Harden tap detection while KEEPING single-tap enabled.
// - Forces Robust mode (TAP_1.mode = 0b10).
// - Ensures single & double tap are both enabled in FEATURE_IO0.
// - Raises the TAP_2 peak threshold (bits [9:0]) by +50% (tweakable).
void bmi323_reduce_shake_keep_single(bmi323TypeDef* s)
{
    extern UART_HandleTypeDef huart1;
    char msg[128];

    /* 1) Robust mode */
    uint8_t t1[2] = {0};
    feature_read_word(s, TAP_1, t1);
    t1[0] = (uint8_t)((t1[0] & ~(0x3u << 6)) | (0x2u << 6));  // mode=Robust
    feature_write_word(s, TAP_1, t1);

    /* 2) Make sure SINGLE and DOUBLE are both enabled */
    uint8_t io0[2] = {0};
    bmi323_receive_spi(s, FEATURE_IO0_REG, io0, 2);
    io0[1] |= (1u << 4);  // single tap ON
    io0[1] |= (1u << 5);  // double tap ON
    bmi323_write_spi(s, FEATURE_IO0_REG, io0, 2);

    /* 3) Raise TAP_2 peak threshold (lower 10 bits) to reduce shake hits */
    uint8_t t2b[2] = {0};
    feature_read_word(s, TAP_2, t2b);
    uint16_t t2   = (uint16_t)((t2b[1] << 8) | t2b[0]);
    uint16_t thr  = (uint16_t)(t2 & 0x03FFu);        // bits [9:0]
    // +50% (change to *2 for more filtering): new_thr = thr + thr/2;
    uint16_t new_thr = (uint16_t)(thr + thr/2);
    if (new_thr > 0x03FFu) new_thr = 0x03FFu;        // clamp to 10 bits
    t2 = (uint16_t)((t2 & ~0x03FFu) | new_thr);
    t2b[0] = (uint8_t)(t2 & 0xFF);
    t2b[1] = (uint8_t)(t2 >> 8);
    feature_write_word(s, TAP_2, t2b);

    /* 4) Read-back & print */
    feature_read_word(s, TAP_2, t2b);
    t2   = (uint16_t)((t2b[1] << 8) | t2b[0]);
    thr  = (uint16_t)(t2 & 0x03FFu);
    uint8_t dur_code = (uint8_t)((t2 >> 10) & 0x3F); // 40 ms/LSB
    uint16_t dur_ms  = (uint16_t)(dur_code * 40);

    snprintf(msg, sizeof(msg),
             "Shake-harden (keep single): thr=%u (0x%03X), window~%u ms, IO0.MSB=0x%02X\r\n",
             thr, thr, dur_ms, io0[1]);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

