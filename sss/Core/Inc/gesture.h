#ifndef GESTURE_H
#define GESTURE_H

#include <stdint.h>
// gesture.h
void gesture_force_quiet(uint32_t ms);


// Structured event (API contract)
typedef struct {
    char gesture[16];     // "rotate_left", "rotate_right", or "none"
    int  confidence;      // 0–100
    int16_t gx, gy, gz;   // raw gyro values
    int16_t ax, ay, az;   // raw accel values
    uint32_t timestamp;   // ms (detector uses HAL_GetTick internally)
} gesture_event_t;

// Simulation + processing API
void imu_generate_sim(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz);

gesture_event_t gesture_process(int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz);

// Scenario control / debug
typedef enum {
    SC_NONE=0,
    SC_QUIET,
    SC_INTENT_TWIST,
    SC_ARM_SWING,
    SC_BUMP,
    SC_OTHER_AXIS_TWIST,
    SC_SHAKE
} sim_case_t;

void sim_set_case(sim_case_t c);
void sim_set_case_by_char(char c);
const char* imu_sim_label(void);
int  sim_is_active(void);          // non-zero if a scenario is currently running
float gesture_dbg_th_hi(void);     // current high threshold

#endif // GESTURE_H
