#ifndef MATH_FILTER_FILTER_H_
#define MATH_FILTER_FILTER_H_

#include <stdbool.h>
#include "drivers/icm45686/icm45686.h"

typedef struct {
    float b0, b1, b2, a1, a2;
    float s1, s2;
} biquad2p_filter_t;

void  biquad2p_filter_init(biquad2p_filter_t *f, float cutoff_hz, float sample_hz);
float biquad2p_filter_apply(biquad2p_filter_t *f, float input);

/*
 * Ban chat he so Kalman trong bai toan Sensor Fusion (IMU):
 * - Q (Process Noise): Ly thuyet la nhieu qua trinh / moi truong -> vao bai toan nay la Nhieu Gyro.
 * - R (Measurement Noise): Ly thuyet la nhieu cam bien do luong -> vao bai toan nay la Nhieu Accel.
 */
typedef struct {
    float angle, bias;
    float P[2][2];
    float Q_angle, Q_bias, R_measure;
    bool  initialized;
} kalman_angle_t;

void  kalman_angle_init(kalman_angle_t *k, float q_angle, float q_bias, float r_measure);
float kalman_angle_update(kalman_angle_t *k, float new_angle_acc, float new_rate_gyro, float dt);

typedef struct {
    float roll;        // Goc Roll (deg)
    float pitch;       // Goc Pitch (deg)
    float yaw;         // Goc Yaw (deg)
    float roll_rate;   // Toc do goc Roll sau loc (deg/s)
    float pitch_rate;  // Toc do goc Pitch sau loc (deg/s)
    float yaw_rate;    // Toc do goc Yaw sau loc (deg/s)
} attitude_t;

void imu_calibrate_gyro(icm45686_t *imu, float bias[3]);

#endif /* MATH_FILTER_FILTER_H_ */
