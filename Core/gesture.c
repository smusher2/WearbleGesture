/* ========================= gesture.c (no simulation) =========================
   - Gyro-based twist detector tuned for BMI323 (~100 Hz, ±2000 dps path)
   - Returns a gesture label + confidence; does not print itself
   - Refractory is valley/peak-aware AND adds a short same-direction “grace”
     window so two LEFTs (or two RIGHTs) can register back-to-back
   - All tunables are at the top for easy tweaking
   ========================================================================== */

#include "gesture.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "stm32wbxx_hal.h"

extern UART_HandleTypeDef huart1;
void MX_USART1_UART_Init(void);

/* ============================================================================
 * Build-time toggles
 * ========================================================================== */
#ifndef DETECT_DEBUG
#define DETECT_DEBUG 0   // 1 = print ACCEPT/REJECT debug lines over UART
#endif

/* ============================================================================
 * IMU / signal-scale assumptions
 * - Your main converts BMI323 raw to:
 *     accel: ~1000 == 1 g (legacy scale)
 *     gyro : integer dps
 * ========================================================================== */
#define ACC_1G                 1000
#define NOISE_ACC_DEV_MAX      0.25f   // normalized accel mag deviation guard

/* ============================================================================
 * Detector timing (ms)
 * ========================================================================== */
#define PRE_QUIET_MS           20u     // needed quiet before normal candidate
#define MIN_CANDIDATE_MS       20u     // collect at least this long
#define MAX_WINDOW_MS          160u    // stop evaluating candidate after this

/* Refractory (valley/peak-aware so two LEFTs can register back-to-back) */
#define REFRACTORY_MS_MIN     250u     // earliest re-arm after an accept
#define REFRACTORY_MS_MAX     410u     // hard cap (always re-arm by this time)
#define REFRACTORY_VALLEY_MS   10u     // time |gy| must stay below rearm_th

/* === NEW: same-direction “grace” window after re-arm =======================
 * Within this window, a new candidate in the SAME sign as the last event can
 * start as soon as it crosses th_hi (no PRE_QUIET needed).
 * This is what enables RIGHT+RIGHT or LEFT+LEFT in quick succession.
 * ========================================================================== */
#define SAME_DIR_GRACE_MS       70u     // grace window length
#define SAME_DIR_START_MULT    1.00f    // th multiplier during grace (1.0 = th_hi)

/* ============================================================================
 * Threshold band (dynamic high adapts during quiet)
 * ========================================================================== */
#define BASE_HI_THRESH        350.0f   // floor for high threshold
#define TH_LO_FRACTION          0.55f  // low threshold ratio
#define STD_MULT_HI              3.0f  // high = max(BASE, k * std)
#define TH_HI_MIN             340.0f   // clamp bounds for high threshold
#define TH_HI_MAX             570.0f
#define TH_HI_SMOOTH            0.75f  // IIR toward dyn_hi (higher = slower)

/* ============================================================================
 * Geometry / quality gates
 * ========================================================================== */
#define MIN_HI_COUNT              3
#define PEAK_MARGIN_X           1.15f  // peak must exceed this * th_hi
#define MIN_AREA_NORM           0.035f // ∫(|gy|/th)*dt area gate
#define MIN_AXIS_DOM            0.08f  // prefer Y axis
#define CONSISTENCY_MIN         0.70f  // same-sign consistency
#define MAX_SIGN_FLIPS            12
#define MAX_LIN_ACC_DEV          1.15f

/* ============================================================================
 * Noise/adaptation
 * ========================================================================== */
#define NOISE_GY_QUIET          45.0f  // only adapt when below this
#define BETA_NOISE               0.03f // running-mean/var update for gy
#define ALPHA_ACC_BASE           0.02f // accel baseline IIR

/* ============================================================================
 * Valley/peak aware re-arm
 * - If |gy| dips below max( VALLEY_TH_FRAC*th_hi, REARM_FRAC_OF_PEAK*last_peak )
 *   for REFRACTORY_VALLEY_MS, we re-arm after REFRACTORY_MS_MIN
 *   (values relaxed a bit to help same-direction chaining)
 * ========================================================================== */
#define VALLEY_TH_FRAC           0.55f
#define REARM_FRAC_OF_PEAK       0.30f

/* ========================= Internals ========================= */
typedef enum { ST_IDLE=0, ST_CANDIDATE, ST_REFRACT } fsm_t;

typedef struct {
    uint32_t last_ms;
    uint32_t quiet_ms;

    float mean_gy, var_gy;
    float th_hi, th_lo;

    float acc_base;

    fsm_t   state;
    uint32_t t_enter;
    int last_sign;

    int sign0;
    int total_cnt, same_sign_cnt;
    int hi_cnt;
    int sign_flips;
    float peak_abs_gy;
    float area_norm;
    float axis_dom_sum;

    /* Re-arm helpers */
    uint32_t ref_quiet_ms;
    float    last_peak_abs_gy;
    int      last_event_sign;
    uint32_t last_rearm_ms;     // when we left REFRACT → IDLE
} detector_t;

static detector_t D;

/* ========================= Utilities ========================= */
static inline float f_abs(float x){ return x < 0 ? -x : x; }
static inline float clampf(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }
static float vec_norm3(float x, float y, float z){ return sqrtf(x*x + y*y + z*z); }

/* Debug printing (compile-time gated) */
static inline void dbg_print_scaled(const char* tag,
                                    uint32_t age_ms,
                                    float area_norm,
                                    float avg_axis,
                                    float consistency,
                                    int hi_cnt,
                                    int flips,
                                    float acc_dev,
                                    float peak,
                                    float th_hi,
                                    int conf)
{
#if DETECT_DEBUG
    int area_m = (int)(area_norm * 1000.0f + 0.5f);
    int axis_p = (int)(avg_axis  * 100.0f + 0.5f);
    int cons_p = (int)(consistency*100.0f + 0.5f);
    int acc_p  = (int)(acc_dev   *100.0f + 0.5f);
    int peak_i = (int)(peak + 0.5f);
    int th_i   = (int)(th_hi + 0.5f);

    char b[200];
    int n = snprintf(b, sizeof(b),
        "{dbg:%s age:%u area_m:%d axis_p:%d cons_p:%d hi:%d flips:%d acc_p:%d peak:%d th:%d conf:%d}\r\n",
        tag, (unsigned)age_ms, area_m, axis_p, cons_p, hi_cnt, flips, acc_p, peak_i, th_i, conf);
    HAL_UART_Transmit(&huart1, (uint8_t*)b, (uint16_t)n, HAL_MAX_DELAY);
#else
    (void)tag; (void)age_ms; (void)area_norm; (void)avg_axis; (void)consistency;
    (void)hi_cnt; (void)flips; (void)acc_dev; (void)peak; (void)th_hi; (void)conf;
#endif
}

/* ========================= Gesture detection ========================= */
gesture_event_t gesture_process(int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz)
{
    gesture_event_t evt;
    memset(&evt, 0, sizeof(evt));
    /* Return nice-cased labels directly so main can print them as-is */
    strncpy(evt.gesture, "none", sizeof(evt.gesture));
    evt.timestamp = HAL_GetTick();

    /* pass-through for optional logging */
    evt.ax = ax; evt.ay = ay; evt.az = az;
    evt.gx = gx; evt.gy = gy; evt.gz = gz;

    /* init on first run */
    if (D.last_ms == 0) {
        D.last_ms = evt.timestamp;
        D.th_hi   = BASE_HI_THRESH;
        D.th_lo   = D.th_hi * TH_LO_FRACTION;

        float acc_mag0 = vec_norm3((float)ax, (float)ay, (float)az);
        D.acc_base = (acc_mag0 > 1.0f) ? acc_mag0 : (float)ACC_1G;

        D.state = ST_IDLE;
        D.quiet_ms = 1000;
        D.last_sign = 0;

        D.ref_quiet_ms = 0;
        D.last_peak_abs_gy = 0.f;
        D.last_event_sign  = 0;
        D.last_rearm_ms    = evt.timestamp;
    }

    uint32_t now = evt.timestamp;
    uint32_t dms = now - D.last_ms;
    D.last_ms = now;
    float dt = clampf(dms / 1000.0f, 0.001f, 0.050f); // clamp 1–50 ms

    /* accel baseline & deviation */
    float acc_mag = vec_norm3((float)ax, (float)ay, (float)az);
    if (D.state != ST_CANDIDATE) {
        D.acc_base = (1.0f - ALPHA_ACC_BASE)*D.acc_base + ALPHA_ACC_BASE*acc_mag;
    }
    float acc_dev = f_abs(acc_mag - D.acc_base) / (D.acc_base + 1e-3f);

    /* quiet tracker in IDLE */
    if (D.state == ST_IDLE) {
        float abs_gy0 = f_abs((float)gy);
        if (abs_gy0 >= D.th_hi) D.quiet_ms = 0;
        else                    D.quiet_ms += dms;
    }

    /* adapt threshold only in IDLE and under quietness */
    if (D.state == ST_IDLE) {
        float gy_f = (float)gy;
        float abs_gy = f_abs(gy_f);

        if (abs_gy <= NOISE_GY_QUIET && acc_dev <= NOISE_ACC_DEV_MAX) {
            float e = gy_f - D.mean_gy;
            D.mean_gy += BETA_NOISE * e;
            D.var_gy   = (1.0f - BETA_NOISE)*D.var_gy + BETA_NOISE * e*e;
        }

        float std_gy = sqrtf(fmaxf(D.var_gy, 1.0f));
        float dyn_hi = fmaxf(BASE_HI_THRESH, STD_MULT_HI * std_gy);
        dyn_hi = clampf(dyn_hi, TH_HI_MIN, TH_HI_MAX);

        D.th_hi = TH_HI_SMOOTH*D.th_hi + (1.0f - TH_HI_SMOOTH)*dyn_hi;
        D.th_lo = D.th_hi * TH_LO_FRACTION;
    }

    /* axis dominance (prefer Y) */
    float sum_abs = f_abs((float)gx) + f_abs((float)gy) + f_abs((float)gz) + 1e-3f;
    float axis_dom = f_abs((float)gy) / sum_abs;
    int sign = (gy > 0) ? +1 : (gy < 0 ? -1 : 0);

    switch (D.state) {
    case ST_IDLE: {
        float abs_gy = f_abs((float)gy);

        /* ---- SNAP-ACCEPT: very strong Y-dominant burst ---- */
        if (abs_gy >= 2.0f * D.th_hi && axis_dom >= 0.80f) {
            float conf_f = clampf((abs_gy / (2.0f * D.th_hi) - 1.0f), 0.f, 1.f);
            evt.confidence = 60 + (int)(40.0f * conf_f + 0.5f);
            strncpy(evt.gesture, (gy > 0) ? "Rotate Right" : "Rotate Left", sizeof(evt.gesture));

            D.state = ST_REFRACT;
            D.t_enter = now;
            D.quiet_ms = 0;

            D.ref_quiet_ms = 0;
            D.last_event_sign = (gy > 0) ? +1 : -1;
            D.last_peak_abs_gy = abs_gy;
            break;
        }

        /* ---- SAME-DIRECTION GRACE START -------------------------------
         * If we just re-armed, for a short window allow a new candidate
         * in the SAME sign to start once it crosses th_hi (no PRE_QUIET).
         * --------------------------------------------------------------- */
        int in_grace = ((now - D.last_rearm_ms) <= SAME_DIR_GRACE_MS) ? 1 : 0;
        int same_dir = (sign != 0 && sign == D.last_event_sign) ? 1 : 0;
        float grace_th = SAME_DIR_START_MULT * D.th_hi;

        if (in_grace && same_dir &&
            abs_gy >= grace_th &&
            acc_dev <= MAX_LIN_ACC_DEV)
        {
            D.state = ST_CANDIDATE;
            D.t_enter = now;
            D.sign0 = sign;
            D.last_sign = sign;
            D.total_cnt = 1;
            D.same_sign_cnt = 1;
            D.hi_cnt = (abs_gy >= D.th_hi) ? 1 : 0;
            D.sign_flips = 0;
            D.peak_abs_gy = abs_gy;
            D.area_norm = (abs_gy / (D.th_hi + 1e-3f)) * dt;
            D.axis_dom_sum = axis_dom;
            break;
        }

        /* ---- NORMAL CANDIDATE START ----
         * Either a clear burst or sufficient pre-quiet + above th_hi
         * (kept as-is to preserve robustness) */
        if (((abs_gy >= 1.50f * D.th_hi) ||
             (D.quiet_ms >= PRE_QUIET_MS && abs_gy >= D.th_hi)) &&
            acc_dev <= MAX_LIN_ACC_DEV)
        {
            D.state = ST_CANDIDATE;
            D.t_enter = now;
            D.sign0 = (gy > 0) ? +1 : -1;
            D.last_sign = D.sign0;
            D.total_cnt = 1;
            D.same_sign_cnt = 1;
            D.hi_cnt = 1;
            D.sign_flips = 0;
            D.peak_abs_gy = abs_gy;
            D.area_norm = (abs_gy / (D.th_hi + 1e-3f)) * dt;
            D.axis_dom_sum = axis_dom;
        }
        break;
    }

    case ST_CANDIDATE: {
        float abs_gy = f_abs((float)gy);
        D.total_cnt++;

        if (sign != 0 && sign != D.last_sign) { D.sign_flips++; D.last_sign = sign; }
        if (sign == D.sign0) D.same_sign_cnt++;
        if (abs_gy >= D.th_hi) D.hi_cnt++;
        if (abs_gy > D.peak_abs_gy) D.peak_abs_gy = abs_gy;

        D.area_norm    += (abs_gy / (D.th_hi + 1e-3f)) * dt;
        D.axis_dom_sum += axis_dom;

        uint32_t age = now - D.t_enter;

        if (abs_gy < D.th_lo && age < MIN_CANDIDATE_MS) {
            dbg_print_scaled("ABORT_EARLY", age, D.area_norm,
                             D.axis_dom_sum/(float)D.total_cnt,
                             D.same_sign_cnt/(float)D.total_cnt,
                             D.hi_cnt, D.sign_flips, acc_dev,
                             D.peak_abs_gy, D.th_hi, 0);
            D.state = ST_IDLE; D.quiet_ms = 0;
            break;
        }

        if (age >= MIN_CANDIDATE_MS) {
            float avg_axis    = D.axis_dom_sum / (float)D.total_cnt;
            float consistency = (float)D.same_sign_cnt / (float)D.total_cnt;
            int   peak_ok     = (D.peak_abs_gy >= PEAK_MARGIN_X * D.th_hi);
            float cross_ratio = (f_abs((float)gx) + f_abs((float)gz)) / (f_abs((float)gy) + 1e-3f);

            /* Fast accept: clean, strong twist */
            if (abs_gy >= (1.75f * D.th_hi) &&
                peak_ok &&
                avg_axis   >= 0.85f &&
                cross_ratio <= 0.55f &&
                consistency >= 0.90f &&
                D.hi_cnt   >= 3 &&
                D.sign_flips == 0 &&
                acc_dev    <= 0.50f)
            {
                float c_peak = clampf(D.peak_abs_gy / (2.0f*D.th_hi), 0.f, 1.f);
                float c_area = clampf(D.area_norm / (MIN_AREA_NORM*2.0f), 0.f, 1.f);
                float c_axis = clampf(avg_axis, 0.f, 1.f);
                float c_cons = clampf(consistency, 0.f, 1.f);
                float conf = 0.45f*c_peak + 0.25f*c_area + 0.20f*c_axis + 0.10f*c_cons;

                evt.confidence = (int)(100.0f * conf + 0.5f);
                strncpy(evt.gesture, (D.sign0 > 0) ? "Rotate Right" : "Rotate Left", sizeof(evt.gesture));

                dbg_print_scaled("ACCEPT_FAST", age, D.area_norm, avg_axis, consistency,
                                 D.hi_cnt, D.sign_flips, acc_dev, D.peak_abs_gy, D.th_hi, evt.confidence);

                D.state = ST_REFRACT; D.t_enter = now; D.quiet_ms = 0;

                D.ref_quiet_ms = 0;
                D.last_event_sign = D.sign0;
                D.last_peak_abs_gy = D.peak_abs_gy;

                break;
            }

            /* Standard accept */
            if (D.area_norm >= MIN_AREA_NORM &&
                avg_axis    >= MIN_AXIS_DOM &&
                cross_ratio <= 0.70f &&
                consistency >= CONSISTENCY_MIN &&
                D.hi_cnt    >= MIN_HI_COUNT &&
                D.sign_flips <= MAX_SIGN_FLIPS &&
                peak_ok &&
                acc_dev     <= MAX_LIN_ACC_DEV)
            {
                float c_peak = clampf(D.peak_abs_gy / (1.8f*D.th_hi), 0.f, 1.f);
                float c_area = clampf(D.area_norm / (MIN_AREA_NORM*2.0f), 0.f, 1.f);
                float c_axis = clampf(avg_axis, 0.f, 1.f);
                float c_cons = clampf(consistency, 0.f, 1.f);
                float conf = 0.40f*c_peak + 0.30f*c_area + 0.20f*c_axis + 0.10f*c_cons;

                evt.confidence = (int)(100.0f * conf + 0.5f);
                strncpy(evt.gesture, (D.sign0 > 0) ? "Rotate Right" : "Rotate Left", sizeof(evt.gesture));

                dbg_print_scaled("ACCEPT", age, D.area_norm, avg_axis, consistency,
                                 D.hi_cnt, D.sign_flips, acc_dev, D.peak_abs_gy, D.th_hi, evt.confidence);

                D.state = ST_REFRACT; D.t_enter = now; D.quiet_ms = 0;

                D.ref_quiet_ms = 0;
                D.last_event_sign = D.sign0;
                D.last_peak_abs_gy = D.peak_abs_gy;

                break;
            }

            if (age >= MAX_WINDOW_MS || acc_dev > MAX_LIN_ACC_DEV) {
                dbg_print_scaled("REJECT", age, D.area_norm,
                                 D.axis_dom_sum/(float)D.total_cnt,
                                 D.same_sign_cnt/(float)D.total_cnt,
                                 D.hi_cnt, D.sign_flips, acc_dev,
                                 D.peak_abs_gy, D.th_hi, 0);
                D.state = ST_IDLE; D.quiet_ms = 0;
            }
        }
        break;
    }

    case ST_REFRACT: {
        /* valley/peak-aware re-arm so two LEFTs (or two RIGHTs) are possible */
        float th1 = VALLEY_TH_FRAC * D.th_hi;
        float lp  = (D.last_peak_abs_gy > 1.f) ? D.last_peak_abs_gy : D.th_hi;
        float th2 = REARM_FRAC_OF_PEAK * lp;
        float rearm_th = (th1 > th2) ? th1 : th2;

        float abs_gy = f_abs((float)gy);

        if (abs_gy <= rearm_th) D.ref_quiet_ms += dms;
        else                    D.ref_quiet_ms  = 0;

        if ((now - D.t_enter) >= REFRACTORY_MS_MIN && D.ref_quiet_ms >= REFRACTORY_VALLEY_MS) {
            D.state = ST_IDLE;
            D.quiet_ms = 0;
            D.last_rearm_ms = now;   // mark start of SAME_DIR_GRACE window
            break;
        }

        if ((now - D.t_enter) >= REFRACTORY_MS_MAX) {
            D.state = ST_IDLE;
            D.quiet_ms = 0;
            D.last_rearm_ms = now;   // also mark grace start
        }
        break;
    }

    default:
        D.state = ST_IDLE;
        break;
    }

    return evt;
}

/* Force quiet/idle between tests (helper) */
void gesture_force_quiet(uint32_t ms)
{
    D.quiet_ms = ms;
    D.state = ST_IDLE;
    D.total_cnt = 0;
    D.same_sign_cnt = 0;
    D.hi_cnt = 0;
    D.sign_flips = 0;
    D.area_norm = 0.0f;
    D.axis_dom_sum = 0.0f;
    D.peak_abs_gy = 0.0f;
    D.ref_quiet_ms = 0;
    D.last_rearm_ms = HAL_GetTick();
}

/* Live high-threshold accessor (for logging) */
float gesture_dbg_th_hi(void) { return D.th_hi; }
