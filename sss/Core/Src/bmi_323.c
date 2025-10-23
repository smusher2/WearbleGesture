/* =========================================================================
 *  bmi_323.c  —  BMI323 driver (SPI + feature engine + tap)  [tunable]
 *  - SPI helpers
 *  - Feature-engine mailbox (read/write words)
 *  - Init / read raw data
 *  - Tap configuration with ALL knobs at the top
 *  - Optional "shake gate" to ignore arm-pump false taps
 *  - Tap IRQ handler that prints "Single Tap"/"Double Tap"
 * ========================================================================= */

#include "main.h"
#include "bmi_323.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

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

/* ====================== Feature engine/mailbox regs ===================== */
/* These symbolic names are expected to be in bmi_323.h; kept here for clarity.
 * If they’re already defined in your header, you can remove this block.
 */
#ifndef FEATURE_DATA_ADDR
#define FEATURE_DATA_ADDR       0x41
#define FEATURE_DATA_TX         0x42
#define FEATURE_DATA_STATUS     0x43
#endif
#ifndef FE_ST_TX_READY
#define FE_ST_TX_READY          0x02u  /* bit1 == data_tx_ready */
#endif

/* Datasheet “extended words” — TAP_1/TAP_2/TAP_3 addresses.
 * Your bmi_323.h likely defines these; keep if not.
 */
#ifndef TAP_1
#define TAP_1                   0x1E
#define TAP_2                   0x1F
#define TAP_3                   0x20
#endif

/* Other BMI323 regs expected from your header:
 *   REG_CHIP_ID_BMI323, CHIP_ID_BMI323, REG_ERR, REG_SENSOR_STATUS
 *   REG_ACC_DATA_X (burst reads AX..GZ)
 *   FEATURE_IO0_REG, FEATURE_IO1_REG, FEATURE_IO2_REG, FEATURE_IO_STATUS_REG
 *   FEATURE_CTRL_REG, FEATURE_EVENT_EXT
 *   INT_MAP2, IO_INT_CTRL
 *   REG_GYR_CONF_BMI323, REG_ACC_CONF_BMI323  (used by your main.c)
 */

/* ========================= Externals & typedefs ======================== */
extern UART_HandleTypeDef huart1;

/* ========================= Local prototypes ============================ */
static void     bmi323_activate_spi(bmi323TypeDef* s);
static void     bmi323_deactivate_spi(bmi323TypeDef* s);
static void     bmi323_receive_spi(bmi323TypeDef* s, uint8_t reg, uint8_t* data, uint16_t len);

static void     fe_wait_ready(bmi323TypeDef* s, uint32_t to_ms);
static void     fe_set_addr(bmi323TypeDef* s, uint16_t addr11);
static void     feature_write_word(bmi323TypeDef* s, uint16_t addr, const uint8_t *val);
static void     feature_read_word (bmi323TypeDef* s, uint16_t addr, uint8_t *out);

static void     tap_write_TAP3(bmi323TypeDef* s,
                               uint8_t quite_after, uint8_t min_quiet_between,
                               uint8_t settle, uint8_t max_pk_gap);
static void     apply_tap_profile_from_knobs(bmi323TypeDef* s);
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
    fe_set_addr(s, addr);                       /* 1) set which word   */
    bmi323_write_spi(s, FEATURE_DATA_TX, (uint8_t*)val, 2); /* 2) LSB,MSB */
}

static void feature_read_word(bmi323TypeDef* s, uint16_t addr, uint8_t *out)
{
    if (!out) return;
    fe_wait_ready(s, 10);                       /* safe to poll before */
    fe_set_addr(s, addr);                       /* 1) set which word   */
    bmi323_receive_spi(s, FEATURE_DATA_TX, out, 2); /* 2) LSB,MSB */
}

/* ========================= Debug helper ================================ */
static void debug_feature_engine_status(bmi323TypeDef* s)
{
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

/* Programmable: set only the double-tap window (TAP_2[15:10] = 40 ms/LSB). */
static void bmi323_set_double_tap_window_ms(bmi323TypeDef* s, uint16_t window_ms)
{
    uint8_t t2[2] = {0};
    feature_read_word(s, TAP_2, t2);

    uint16_t t2u = (uint16_t)((t2[1] << 8) | t2[0]);
    uint8_t  dur = (uint8_t)(window_ms / 40u);   // 40 ms per LSB
    if (dur > 63u) dur = 63u;
    t2u = (uint16_t)((t2u & 0x03FFu) | ((uint16_t)dur << 10));

    t2[0] = (uint8_t)(t2u & 0xFFu);
    t2[1] = (uint8_t)(t2u >> 8);
    feature_write_word(s, TAP_2, t2);
}

/* Write TAP_3 from four nibbles. */
static void tap_write_TAP3(bmi323TypeDef* s, uint8_t quite_after,
                           uint8_t min_quiet_between, uint8_t settle, uint8_t max_pk_gap)
{
    if (quite_after       > 15) quite_after = 15;
    if (min_quiet_between > 15) min_quiet_between = 15;
    if (settle            > 15) settle = 15;
    if (max_pk_gap        > 15) max_pk_gap = 15;

    uint16_t v = ((uint16_t)quite_after << 12) |
                 ((uint16_t)min_quiet_between << 8) |
                 ((uint16_t)settle << 4) |
                 ((uint16_t)max_pk_gap);
    uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    feature_write_word(s, TAP_3, b);
}

/* Apply all tap fields based on the top-of-file knobs */
static void apply_tap_profile_from_knobs(bmi323TypeDef* s)
{
    char msg[160];

    /* --- TAP_1: mode + max_peaks_for_tap --- */
    uint8_t t1[2] = {0};
    feature_read_word(s, TAP_1, t1);
    /* mode bits [7:6] */
    t1[0] = (uint8_t)((t1[0] & ~(0x3u<<6)) | ((TAP1_MODE & 0x3u) << 6));
    /* max_peaks_for_tap bits [5:3] */
    t1[0] = (uint8_t)((t1[0] & ~(0x7u<<3)) | ((TAP1_MAX_PEAKS_FOR_TAP & 0x7u) << 3));
    feature_write_word(s, TAP_1, t1);

    /* --- TAP_2: scale threshold + set double-tap window --- */
    uint8_t t2b[2] = {0};
    feature_read_word(s, TAP_2, t2b);
    uint16_t t2 = (uint16_t)((t2b[1] << 8) | t2b[0]);

    uint16_t thr = (uint16_t)(t2 & 0x03FFu); /* lower 10 bits */
    uint32_t scaled = (uint32_t)thr * (uint32_t)TAP2_THRESH_SCALE_X100 / 100u;
    if (scaled > 0x03FFu) scaled = 0x03FFu;
    t2 = (uint16_t)((t2 & ~0x03FFu) | (uint16_t)scaled);
    t2b[0] = (uint8_t)(t2 & 0xFF);
    t2b[1] = (uint8_t)(t2 >> 8);
    feature_write_word(s, TAP_2, t2b);

    bmi323_set_double_tap_window_ms(s, TAP2_MAX_GESTURE_DUR_MS);

    /* --- TAP_3: pulse shape + quiet requirements --- */
    tap_write_TAP3(s,
        TAP3_QUIET_TIME_AFTER_GESTURE,
        TAP3_MIN_QUIET_BETWEEN_TAPS,
        TAP3_TAP_SHOCK_SETTLING_DUR,
        TAP3_MAX_DUR_BETWEEN_PEAKS
    );

    /* --- FEATURE_IO0: enable single/double/triple bits (MSB: bit4=single, bit5=double) --- */
    uint8_t io0[2] = {0};
    bmi323_receive_spi(s, FEATURE_IO0_REG, io0, 2);
    if (TAP_ENABLE_SINGLE) io0[1] |=  (1u<<4); else io0[1] &= ~(1u<<4);
    if (TAP_ENABLE_DOUBLE) io0[1] |=  (1u<<5); else io0[1] &= ~(1u<<5);
    if (TAP_ENABLE_TRIPLE) io0[1] |=  (1u<<6); else io0[1] &= ~(1u<<6);
    bmi323_write_spi(s, FEATURE_IO0_REG, io0, 2);

    /* --- Print summary --- */
    uint8_t t3b[2] = {0}; feature_read_word(s, TAP_3, t3b);
    feature_read_word(s, TAP_2, t2b);
    t2 = (uint16_t)((t2b[1] << 8) | t2b[0]);
    uint16_t thr_out = (uint16_t)(t2 & 0x03FFu);
    uint8_t  dur_out = (uint8_t)((t2 >> 10) & 0x3Fu);
    uint16_t win_ms  = (uint16_t)(dur_out * 40u);

    snprintf(msg, sizeof(msg),
        "TapCfg: T1=%02X%02X T2=%02X%02X(thr=%u) T3=%02X%02X win=%ums IO0.MSB=0x%02X\r\n",
        t1[1], t1[0], t2b[1], t2b[0], thr_out, t3b[1], t3b[0], win_ms, io0[1]);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/* =========== Public entry: enable and tune the tap detector =========== */
void bmi323_enable_tap(bmi323TypeDef* s)
{
    /* Ensure feature engine is on */
    {
        uint8_t fctrl[2] = {0};
        bmi323_receive_spi(s, FEATURE_CTRL_REG, fctrl, 2);
        fctrl[0] |= 0x01u;
        bmi323_write_spi(s, FEATURE_CTRL_REG, fctrl, 2);
    }

    /* Map tap to INT1 via INT_MAP2; set INT1 push-pull, active-high */
    {
        uint8_t map2[2] = {0};
        bmi323_receive_spi(s, INT_MAP2, map2, 2);
        map2[0] = (uint8_t)((map2[0] & ~0x03u) | 0x01u);  /* route tap to INT1 */
        bmi323_write_spi(s, INT_MAP2, map2, 2);

        uint8_t io[2] = {0};
        bmi323_receive_spi(s, IO_INT_CTRL, io, 2);
        io[0] |=  (1u<<2);          // int1_output_en
        io[0] &= ~(1u<<1);          // push-pull
        io[0] |=  (1u<<0);          // active high
        bmi323_write_spi(s, IO_INT_CTRL, io, 2);
    }

    /* Apply tuning from the top-of-file knobs */
    apply_tap_profile_from_knobs(s);

    /* Clear any latched feature status (optional) */
    {
        uint8_t dummy[2];
        bmi323_receive_spi(s, FEATURE_IO_STATUS_REG, dummy, 2);
    }
}



const char* BMI323_HandleTapEvent(bmi323TypeDef* sensor, uint32_t* haptic_deadline, uint8_t* haptic_active)
{
    (void)haptic_deadline;
    (void)haptic_active;

    // If you have the shake gate enabled, ignore and return NULL
    uint8_t tap_evt[2] = {0};
    bmi323_receive_spi(sensor, FEATURE_EVENT_EXT, tap_evt, 2); // [LSB, MSB]

    if (tap_evt[0] & (1u<<5)) {
        return "Triple Tap";
    } else if (tap_evt[0] & (1u<<4)) {
        return "Double Tap";
    } else if (tap_evt[0] & (1u<<3)) {
        return "Single Tap";
    }

    return NULL; // nothing recognized
}

