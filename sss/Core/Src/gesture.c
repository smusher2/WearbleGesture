#include "gesture.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "stm32wbxx_hal.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;

/* =========================
   Build-time toggles
   ========================= */
#ifndef DETECT_DEBUG
#define DETECT_DEBUG 0   // 1 = print ACCEPT/REJECT debug lines
#endif

/* =========================
   Simulation realism (BMI323)
   Assumes gyro full-scale ±1000 dps, ODR ~100 Hz
   ========================= */
#define GY_RANGE_DPS           2000.0f
#define SIM_ODR_HZ             100.0f
#define SIM_DT                 (1.0f / SIM_ODR_HZ)

/* Gyro noise & bias (per-sample @100 Hz) */
#define GY_NOISE_STD_DPS       3.0f     // white noise
#define GY_BIAS_RW_STD_DPS     0.04f    // bias random walk

/* BMI-like low-pass (simple 1st-order IIR ~ ODR/2) */
#define GY_LPF_BETA            0.35f

/* Cross-axis / misalignment */
#define GY_MISALIGN_DEG        20.0f    // ~20° X–Y misalignment
#define GY_CROSS_COEFF         0.06f    // small soft coupling

/* Accel “1g” legacy scale (~1000 ≈ 1 g in your codebase) */
#define ACC_1G                 1000
#define ACC_NOISE_MG           20

/* Scenario peak ranges (in dps) */

#define INTENT_MEAN_DPS     1600.0f
#define INTENT_SIGMA_DPS    120.0f   // spread; try 80–140
#define INTENT_MIN_DPS      1200.0f
#define INTENT_MAX_DPS      2000.0f   // keep headroom below ±1000 FS
#define SHAKE_MIN_DPS          60
#define SHAKE_MAX_DPS          140
#define SWING_MAX_DPS          80
#define OTHER_MIN_DPS          300
#define OTHER_MAX_DPS          500

/* =========================
   Detector (tuned for ±1000 dps)
   ========================= */
/* ================= Tunables for ±2000 dps, ~100 Hz ================= */
// timing
// timing
// timing (ensure we see the peak)
#define PRE_QUIET_MS        45u      // need ~60 ms calm before burst
#define MIN_CANDIDATE_MS    45u      // ~3 samples at 60 Hz (or 5 at 100 Hz)
#define MAX_WINDOW_MS       250u
#define REFRACTORY_MS       250u

// threshold band
#define BASE_HI_THRESH      400.0f   // strong twist gate (raise to 900–1000 if still chatty)
#define TH_LO_FRACTION      0.60f
#define STD_MULT_HI         4.0f
#define TH_HI_MIN           350.0f
#define TH_HI_MAX           600.0f

// shape/geometry
#define MIN_HI_COUNT        3        // don’t finalize with only 1–2 hits
#define PEAK_MARGIN_X       1.30f
#define MIN_AREA_NORM       0.055f   // enough integrated energy at 60–100 Hz
#define MIN_AXIS_DOM        0.10f
#define CONSISTENCY_MIN     0.8f
#define MAX_SIGN_FLIPS      10
#define MAX_LIN_ACC_DEV     0.95f


// noise/adaptation
#define NOISE_GY_QUIET      60.0f    // treat ≤60 dps as “quiet noise”
#define NOISE_ACC_DEV_MAX   0.20f
#define TH_HI_SMOOTH        0.80f
#define BETA_NOISE          0.02f
#define ALPHA_ACC_BASE      0.02f






/* =========================
   Internals
   ========================= */
typedef enum { ST_IDLE=0, ST_CANDIDATE, ST_REFRACT } fsm_t;

typedef struct {
    uint32_t last_ms;
    uint32_t quiet_ms;

    float mean_gy, var_gy;
    float th_hi, th_lo;

    float acc_base;

    fsm_t state;
    uint32_t t_enter;
    int last_sign;

    int sign0;
    int total_cnt, same_sign_cnt;
    int hi_cnt;
    int sign_flips;
    float peak_abs_gy;
    float area_norm;
    float axis_dom_sum;
} detector_t;

static detector_t D;

/* ========== Utilities ========== */
static inline float f_abs(float x){ return x < 0 ? -x : x; }
static inline float clampf(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }
static float vec_norm3(float x, float y, float z){ return sqrtf(x*x + y*y + z*z); }
static inline float deg2rad(float d){ return d * 3.1415926535f / 180.0f; }

/* Gaussian noise (Box–Muller) */
static float randn(void) {
    float u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    float u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    float r = sqrtf(-2.0f * logf(u1));
    float th = 6.2831853f * u2;
    return r * cosf(th);
}

/* Debug printing */
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

/* =========================
   Manual Scenario Simulator
   (paused by default; one-shot per command)
   ========================= */
static sim_case_t S_case = SC_NONE;
static int        S_left = 0;   // samples remaining in current window
static int        S_sign = +1;
static float      S_phase = 0.f;
static const char* S_label = "paused";

/* Gyro state for realism */
static float g_true_x=0.f, g_true_y=0.f, g_true_z=0.f;  // true (dps)
static float g_meas_x=0.f, g_meas_y=0.f, g_meas_z=0.f;  // filtered output (dps)
static float g_bias_x=0.f, g_bias_y=0.f, g_bias_z=0.f;  // bias (dps)

/* Start a single simulation window (matches gesture.h prototype) */
void sim_set_case(sim_case_t c)
{
    S_case = c;
    switch (c) {
    case SC_NONE:              S_left = 0;   S_label = "paused";       break;
    case SC_QUIET:             S_left = 40;  S_label = "quiet";        break;   // ~0.4 s
    case SC_INTENT_TWIST:      S_left = 18;  S_label = "intent_twist"; S_sign = (rand()&1)?+1:-1; break; // ~180 ms
    case SC_ARM_SWING:         S_left = 100; S_label = "arm_swing";    break;   // ~1 s
    case SC_BUMP:              S_left = 5;   S_label = "bump";         break;
    case SC_OTHER_AXIS_TWIST:  S_left = 20;  S_label = "other_axis";   S_sign = (rand()&1)?+1:-1; break;
    case SC_SHAKE:             S_left = 40;  S_label = "shake";        break;
    default:                   S_left = 0;   S_label = "paused";       break;
    }
    S_phase = 0.f;
}

/* Map a key to a one-shot window */
void sim_set_case_by_char(char c)
{
    switch (c) {
    case 'q': sim_set_case(SC_QUIET);            break;
    case 't': sim_set_case(SC_INTENT_TWIST);     break;
    case 'a': sim_set_case(SC_ARM_SWING);        break;
    case 'b': sim_set_case(SC_BUMP);             break;
    case 'o': sim_set_case(SC_OTHER_AXIS_TWIST); break;
    case 's': sim_set_case(SC_SHAKE);            break;
    default:  sim_set_case(SC_NONE);             break;
    }
}

const char* imu_sim_label(void){ return S_label; }
int sim_is_active(void){ return (S_case != SC_NONE); }

/* Core simulator. Outputs gyro in dps (cast to int16), accel in ~1000=1g units. */
void imu_generate_sim(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz)
{
    /* Base accel ~1g + noise */
    int16_t nax = (rand()%(2*ACC_NOISE_MG+1)) - ACC_NOISE_MG;
    int16_t nay = (rand()%(2*ACC_NOISE_MG+1)) - ACC_NOISE_MG;
    int16_t naz = ACC_1G + ((rand()%(2*ACC_NOISE_MG+1)) - ACC_NOISE_MG);

    /* If paused or finished, hold near-zero with tiny noise */
    if (S_case == SC_NONE || S_left <= 0) {
        S_case = SC_NONE;
        S_label = "paused";
        g_true_x = g_true_y = g_true_z = 0.f;

        g_meas_x += GY_LPF_BETA*(g_true_x - g_meas_x);
        g_meas_y += GY_LPF_BETA*(g_true_y - g_meas_y);
        g_meas_z += GY_LPF_BETA*(g_true_z - g_meas_z);

        *ax = nax; *ay = nay; *az = naz;
        *gx = (int16_t)(g_meas_x + randn()*1.0f);
        *gy = (int16_t)(g_meas_y + randn()*1.0f);
        *gz = (int16_t)(g_meas_z + randn()*1.0f);
        return;
    }

    S_left--;

    /* Bias random-walk (slow drift) */
    g_bias_x += randn() * GY_BIAS_RW_STD_DPS;
    g_bias_y += randn() * GY_BIAS_RW_STD_DPS;
    g_bias_z += randn() * GY_BIAS_RW_STD_DPS;

    /* Generate “true” motion for the active scenario */
    float tx=0.f, ty=0.f, tz=0.f;

    switch (S_case) {
    case SC_QUIET:
        tx = randn()*2.0f; ty = randn()*2.0f; tz = randn()*2.0f;
        break;

    case SC_INTENT_TWIST: {
        // Raised-cosine envelope (~180 ms), peak 250–700 dps
    	// --- inside case SC_INTENT_TWIST: replace the amplitude selection ---
    	float A = INTENT_MEAN_DPS + randn()*INTENT_SIGMA_DPS;
    	A = clampf(A, INTENT_MIN_DPS, INTENT_MAX_DPS);

    	float n = (float)S_left;
    	float N = 18.0f; // ~180 ms at 100 Hz
    	float env = 0.5f - 0.5f*cosf(3.1415926535f * ((N - n)/N)); // 0..1..0
    	ty = S_sign * (A * env);
    	// small cross components remain:
    	tx = randn()*6.0f; tz = randn()*6.0f;

        break;
    }

    case SC_ARM_SWING: {
        // ~1.2 Hz slow swing; Gy small; accel swings
        S_phase += 0.075f;
        float s = sinf(S_phase);
        nax += (int16_t)(220*s);
        nay += (int16_t)(140*s);
        naz += (int16_t)(100*(-s));
        ty = (SWING_MAX_DPS * s) + randn()*3.0f;
        tx = randn()*3.0f; tz = randn()*3.0f;
        break;
    }

    case SC_BUMP:
        // Translational bump; gyro small
        nax = (rand()%900 - 450);
        nay = (rand()%900 - 450);
        naz = ACC_1G + (rand()%1000 - 500);
        ty = randn()*12.0f; tx = randn()*6.0f; tz = randn()*6.0f;
        break;

    case SC_OTHER_AXIS_TWIST: {
        // Large rotation on X or Z instead of Y
        int A = OTHER_MIN_DPS + (rand()%(OTHER_MAX_DPS - OTHER_MIN_DPS + 1));
        float n = (float)S_left, N = 20.0f;
        float env = 0.5f - 0.5f*cosf(3.1415926535f * ( (N - n) / N ));
        if (rand() & 1) tx = S_sign * (A * env);
        else            tz = S_sign * (A * env);
        ty = randn()*10.0f;
        break;
    }

    case SC_SHAKE: {
        // Moderate Gy with frequent sign flips and jitter
        int base = SHAKE_MIN_DPS + (rand()%(SHAKE_MAX_DPS - SHAKE_MIN_DPS + 1));
        if (rand() & 1) S_sign = -S_sign;
        ty = S_sign * (float)base + randn()*6.0f;
        tx = randn()*25.0f; tz = randn()*25.0f;
        break;
    }

    default: break;
    }

    /* Apply fixed misalignment (rotate X–Y by ~20°) */
    {
        float ca = cosf(deg2rad(GY_MISALIGN_DEG));
        float sa = sinf(deg2rad(GY_MISALIGN_DEG));
        float x2 =  ca*tx - sa*ty;
        float y2 =  sa*tx + ca*ty;
        tx = x2; ty = y2;
    }

    /* Add bias + sensor noise, clamp to range */
    tx = clampf(tx + g_bias_x + randn()*GY_NOISE_STD_DPS, -GY_RANGE_DPS, GY_RANGE_DPS);
    ty = clampf(ty + g_bias_y + randn()*GY_NOISE_STD_DPS, -GY_RANGE_DPS, GY_RANGE_DPS);
    tz = clampf(tz + g_bias_z + randn()*GY_NOISE_STD_DPS, -GY_RANGE_DPS, GY_RANGE_DPS);

    /* Soft cross-axis coupling */
    float cx = tx + GY_CROSS_COEFF*(ty + tz);
    float cy = ty + GY_CROSS_COEFF*(tx + tz);
    float cz = tz + GY_CROSS_COEFF*(tx + ty);

    /* BMI-like LPF */
    g_meas_x += GY_LPF_BETA * (cx - g_meas_x);
    g_meas_y += GY_LPF_BETA * (cy - g_meas_y);
    g_meas_z += GY_LPF_BETA * (cz - g_meas_z);

    /* Output */
    *ax = nax; *ay = nay; *az = naz;
    *gx = (int16_t)(g_meas_x);
    *gy = (int16_t)(g_meas_y);
    *gz = (int16_t)(g_meas_z);
}

/* =========================
   Gesture detection (Gy)
   ========================= */
gesture_event_t gesture_process(int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz)
{
    gesture_event_t evt;
    memset(&evt, 0, sizeof(evt));
    strncpy(evt.gesture, "none", sizeof(evt.gesture));
    evt.timestamp = HAL_GetTick();

    /* pass-through */
    evt.ax = ax; evt.ay = ay; evt.az = az;
    evt.gx = gx; evt.gy = gy; evt.gz = gz;

    /* init */
    if (D.last_ms == 0) {
        D.last_ms = evt.timestamp;
        D.th_hi = BASE_HI_THRESH;
        D.th_lo = D.th_hi * TH_LO_FRACTION;
        float acc_mag0 = vec_norm3((float)ax, (float)ay, (float)az);
        D.acc_base = (acc_mag0 > 1.0f) ? acc_mag0 : (float)ACC_1G;
        D.state = ST_IDLE;
        D.quiet_ms = 1000;
        D.last_sign = 0;
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

    /* quiet tracker */
    if (D.state == ST_IDLE) {
        float abs_gy0 = f_abs((float)gy);
        if (abs_gy0 >= D.th_hi) {
            D.quiet_ms = 0;                 // big activity resets quiet
        } else {
            D.quiet_ms += dms;              // anything below th_hi is "quiet enough"
        }
    }

    /* adapt threshold during quiet */
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

        // ---- SNAP-ACCEPT FOR VERY STRONG Y-DOMINANT BURSTS ----
        // strict: needs > 2x threshold AND clear Y dominance
        if (abs_gy >= 2.0f * D.th_hi && axis_dom >= 0.80f) {
            // confidence biased by how far above 2.0*th_hi you are
            float conf_f = clampf((abs_gy / (2.0f * D.th_hi) - 1.0f), 0.f, 1.f);
            // 60..100, never super low if we snap-accept
            evt.confidence = 60 + (int)(40.0f * conf_f + 0.5f);
            strncpy(evt.gesture, (gy > 0) ? "rotate_right" : "rotate_left", sizeof(evt.gesture));

            D.state = ST_REFRACT;
            D.t_enter = now;
            D.quiet_ms = 0;
            break;
        }

        // ---- NORMAL CANDIDATE START (still strict) ----
        if (((abs_gy >= 1.50f * D.th_hi) ||                  // clear burst -> bypass pre-quiet
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

            /* Fast accept: very strong, clean twist */
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
                strncpy(evt.gesture, (D.sign0 > 0) ? "rotate_right" : "rotate_left", sizeof(evt.gesture));

                dbg_print_scaled("ACCEPT_FAST", age, D.area_norm, avg_axis, consistency,
                                 D.hi_cnt, D.sign_flips, acc_dev, D.peak_abs_gy, D.th_hi, evt.confidence);

                D.state = ST_REFRACT; D.t_enter = now; D.quiet_ms = 0;
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
                strncpy(evt.gesture, (D.sign0 > 0) ? "rotate_right" : "rotate_left", sizeof(evt.gesture));

                dbg_print_scaled("ACCEPT", age, D.area_norm, avg_axis, consistency,
                                 D.hi_cnt, D.sign_flips, acc_dev, D.peak_abs_gy, D.th_hi, evt.confidence);

                D.state = ST_REFRACT; D.t_enter = now; D.quiet_ms = 0;
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

    case ST_REFRACT:
        if ((now - D.t_enter) >= REFRACTORY_MS) {
            D.state = ST_IDLE;
            D.quiet_ms = 0;
        }
        break;

    default:
        D.state = ST_IDLE;
        break;
    }

    return evt;
}

/* Force quiet/idle between tests */
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
}

/* Live high-threshold accessor (for logging) */
float gesture_dbg_th_hi(void) { return D.th_hi; }
