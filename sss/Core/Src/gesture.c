#include "gesture.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32wbxx_hal.h"

// ----------------- Tunables (100 Hz-friendly) -----------------
#define BASE_HI_THRESH      300.0f
#define STD_MULT_HI         5.0f
#define TH_LO_FRACTION      0.60f

#define BETA_NOISE          0.02f
#define ALPHA_ACC_BASE      0.02f

#define MIN_CANDIDATE_MS    30u
#define MAX_WINDOW_MS       200u
#define REFRACTORY_MS       250u


#define MIN_AREA_NORM       0.10f
#define MIN_AXIS_DOM        0.50f
#define CONSISTENCY_MIN     0.75f
#define MAX_LIN_ACC_DEV     2.00f


typedef enum { ST_IDLE=0, ST_CANDIDATE, ST_REFRACT } fsm_t;

typedef struct {
    // time base
    uint32_t last_ms;

    // adaptive noise on Gy
    float mean_gy, var_gy;
    float th_hi, th_lo;

    // accel magnitude baseline (approx "1g" in raw units)
    float acc_base;

    // FSM
    fsm_t state;
    uint32_t t_enter;

    // candidate accumulators
    int sign0;
    int total_cnt, same_sign_cnt;
    float peak_abs_gy;
    float area_norm;
    float axis_dom_sum;
} detector_t;

static detector_t D;

// ----------------- Helpers -----------------
static inline float f_abs(float x){ return x < 0 ? -x : x; }
static inline float clampf(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }
static float vec_norm3(float x, float y, float z){ return sqrtf(x*x + y*y + z*z); }



void imu_generate_sim(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz) {
    static int burst_left = 0;
    static int burst_sgn  = +1;
    static int burst_amp  = 900; // counts

    *ax = (rand()%100 - 50);
    *ay = (rand()%100 - 50);
    *az = 1000 + (rand()%60 - 30);

    *gx = (rand()%50 - 25);
    *gz = (rand()%50 - 25);

    if (burst_left > 0) {

        *gy = (int16_t)(burst_sgn * burst_amp + (rand()%60 - 30)); // add tiny jitter
        burst_left--;
    } else {
        *gy = (rand()%50 - 25);
        if ((rand() % 25) == 0) {
            burst_left = 6 + (rand()%3);
            burst_sgn  = (rand()&1) ? +1 : -1;
            burst_amp  = 800 + (rand()%600);
        }
    }
}

// ----------------- Core detection -----------------
gesture_event_t gesture_process(int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz) {
    gesture_event_t evt;
    memset(&evt, 0, sizeof(evt));
    strncpy(evt.gesture, "none", sizeof(evt.gesture));
    evt.timestamp = HAL_GetTick();

    // pass-through raw values
    evt.ax = ax; evt.ay = ay; evt.az = az;
    evt.gx = gx; evt.gy = gy; evt.gz = gz;

    // init on first run
    if (D.last_ms == 0) {
        D.last_ms = evt.timestamp;
        D.th_hi = BASE_HI_THRESH;
        D.th_lo = D.th_hi * TH_LO_FRACTION;
        float acc_mag0 = vec_norm3((float)ax, (float)ay, (float)az);
        D.acc_base = (acc_mag0 > 1.0f) ? acc_mag0 : 1000.0f;
        D.state = ST_IDLE;
    }

    // time delta
    uint32_t now = evt.timestamp;
    uint32_t dms = now - D.last_ms;
    D.last_ms = now;
    float dt = clampf(dms / 1000.0f, 0.001f, 0.050f); // 1–50 ms

    // live accel baseline (~1g in raw units)
    float acc_mag = vec_norm3((float)ax, (float)ay, (float)az);
    if (D.state != ST_CANDIDATE) {
        D.acc_base = (1.0f - ALPHA_ACC_BASE)*D.acc_base + ALPHA_ACC_BASE*acc_mag;
    }
    float acc_dev = f_abs(acc_mag - D.acc_base) / (D.acc_base + 1e-3f);

    // adaptive noise / thresholds on Gy (update when idle to avoid contamination)
    if (D.state == ST_IDLE) {
        float gy_f = (float)gy;
        float e = gy_f - D.mean_gy;
        D.mean_gy += BETA_NOISE * e;
        D.var_gy   = (1.0f - BETA_NOISE)*D.var_gy + BETA_NOISE * e*e;
        float std_gy = sqrtf(fmaxf(D.var_gy, 1.0f));
        float dyn_hi = fmaxf(BASE_HI_THRESH, STD_MULT_HI * std_gy);
        // Smooth threshold to avoid jitter
        D.th_hi = 0.6f*D.th_hi + 0.4f*dyn_hi;
        D.th_lo = D.th_hi * TH_LO_FRACTION;
    }

    // axis dominance for Gy
    float sum_abs = f_abs((float)gx) + f_abs((float)gy) + f_abs((float)gz) + 1e-3f;
    float axis_dom = f_abs((float)gy) / sum_abs;

    // FSM
    switch (D.state) {
    case ST_IDLE: {
        float abs_gy = f_abs((float)gy);
        if (abs_gy >= D.th_hi && acc_dev <= MAX_LIN_ACC_DEV) {
            // enter candidate window
            D.state = ST_CANDIDATE;
            D.t_enter = now;
            D.sign0 = (gy > 0) ? +1 : -1;
            D.total_cnt = 0;
            D.same_sign_cnt = 0;
            D.peak_abs_gy = abs_gy;
            D.area_norm = 0.0f;
            D.axis_dom_sum = 0.0f;

            // include first sample
            D.total_cnt = 1;
            if (((gy > 0) ? +1 : -1) == D.sign0) D.same_sign_cnt = 1;
            D.area_norm += (abs_gy / (D.th_hi + 1e-3f)) * dt;
            D.axis_dom_sum += axis_dom;
        }
        break;
    }

    case ST_CANDIDATE: {
        float abs_gy = f_abs((float)gy);
        D.total_cnt++;
        if (((gy > 0) ? +1 : -1) == D.sign0) D.same_sign_cnt++;
        if (abs_gy > D.peak_abs_gy) D.peak_abs_gy = abs_gy;
        D.area_norm += (abs_gy / (D.th_hi + 1e-3f)) * dt;
        D.axis_dom_sum += axis_dom;

        uint32_t age = now - D.t_enter;

        // if it collapses below low threshold too early, abort
        if (abs_gy < D.th_lo && age < MIN_CANDIDATE_MS) {
            D.state = ST_IDLE;
            break;
        }

        // once we have enough time, evaluate confirmation
        if (age >= MIN_CANDIDATE_MS) {
            float avg_axis = D.axis_dom_sum / (float)D.total_cnt;
            float consistency = (float)D.same_sign_cnt / (float)D.total_cnt;

            if (D.area_norm >= MIN_AREA_NORM &&
                avg_axis >= MIN_AXIS_DOM &&
                consistency >= CONSISTENCY_MIN &&
                acc_dev <= MAX_LIN_ACC_DEV)
            {
                // compute dynamic confidence (0..100)
                float c_peak = clampf(D.peak_abs_gy / (1.6f*D.th_hi), 0.f, 1.f);
                float c_area = clampf(D.area_norm / (MIN_AREA_NORM*1.8f), 0.f, 1.f);
                float c_axis = clampf(avg_axis, 0.f, 1.f);
                float c_cons = clampf(consistency, 0.f, 1.f);

                float conf = 0.35f*c_peak + 0.30f*c_area + 0.20f*c_axis + 0.15f*c_cons;
                evt.confidence = (int)(100.0f * conf + 0.5f);

                // label based on sign of Gy
                if (D.sign0 > 0) {
                    strncpy(evt.gesture, "rotate_right", sizeof(evt.gesture));
                } else {
                    strncpy(evt.gesture, "rotate_left", sizeof(evt.gesture));
                }

                // enter refractory
                D.state = ST_REFRACT;
                D.t_enter = now;
            }
            else if (age >= MAX_WINDOW_MS || acc_dev > MAX_LIN_ACC_DEV) {
                // give up
                D.state = ST_IDLE;
            }
        }
        break;
    }

    case ST_REFRACT:
        if ((now - D.t_enter) >= REFRACTORY_MS) {
            D.state = ST_IDLE;
        }
        break;

    default:
        D.state = ST_IDLE;
        break;
    }

    return evt;
}
