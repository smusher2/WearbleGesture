#ifndef GESTURE_H
#define GESTURE_H

#include <stdint.h>
#include <stdbool.h>

// Structured event format (API contract for BLE)
typedef struct {
    char gesture[16];     // e.g. "rotate_left", "rotate_right", "none"
    int  confidence;      // 0–100
    int16_t gx, gy, gz;   // raw gyro values
    int16_t ax, ay, az;   // raw accel values
    uint32_t timestamp;   // in ms (HAL_GetTick)
} gesture_event_t;

// ---- Lifecycle / config ----
void gesture_init(uint32_t sample_hz);
bool  gesture_is_calibrating(void);

// ---- Simulation (optional for host tests) ----
void imu_generate_sim(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz);

// ---- Core processing ----
gesture_event_t gesture_process(int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz);

#endif // GESTURE_H
