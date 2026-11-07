#ifndef MATH_FILTER_FILTER_H_
#define MATH_FILTER_FILTER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
   1. Biquad 2-Pole Butterworth Low-Pass Filter (Gyro & Baro)
   ========================================================================= */
typedef struct {
    float b0, b1, b2, a1, a2;
    float s1, s2;
} biquad2p_filter_t;

void  biquad2p_filter_init(biquad2p_filter_t *f, float cutoff_hz, float sample_hz);
float biquad2p_filter_apply(biquad2p_filter_t *f, float input);
void  biquad2p_filter_reset(biquad2p_filter_t *f, float value);

/* =========================================================================
   2. 2-State Linear Kalman Filter for Attitude (Roll / Pitch)
   - Q (Process Noise): Nhieu Gyro
   - R (Measurement Noise): Nhieu Accel
   ========================================================================= */
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

/* =========================================================================
   3. Median Filter (Co the cau hinh kich thuoc cua so: 3, 5, 9... mau)
   ========================================================================= */
#define MEDIAN_FILTER_MAX_SIZE 15

typedef struct {
    float buf[MEDIAN_FILTER_MAX_SIZE];
    uint8_t size;     // Kich thuoc cua so (vi du: 3, 5, 9)
    uint8_t count;    // So mau hop le hien co
    uint8_t idx;      // Vi tri ghi xoay vong
} median_filter_t;

typedef median_filter_t medianfilter_t; // Alias theo yeu cau

void  median_filter_init(median_filter_t *f, uint8_t size);
float median_filter_apply(median_filter_t *f, float input);
void  median_filter_reset(median_filter_t *f, float value);

/* =========================================================================
   4. PX4 ECL EKF 3-State Altitude Filter (IMU Accel Z + Baro)
   Trang thai: [Do cao z (m), Van toc leo vz (m/s), Accel Z bias (m/s^2)]
   - Predict: 500 Hz voi gia toc thang dung he quy chieu Trai Dat
   - Update: 50 Hz voi do cao Barometer
   ========================================================================= */
typedef struct {
    float z;            // Do cao uoc luong (m)
    float vz;           // Van toc leo uoc luong (m/s)
    float accel_bias;   // Bias gia toc truc Z (m/s^2)
    float P[3][3];      // Ma tran hiep phuong sai sai so
    float Q_accel;      // Phuong sai nhieu gia toc
    float Q_bias;       // Phuong sai nhieu bias gia toc
    float R_baro;       // Phuong sai nhieu do luong Baro
    bool  initialized;
} altitude_ekf_t;

void altitude_ekf_init(altitude_ekf_t *ekf, float acc_noise, float bias_noise, float baro_noise);
void altitude_ekf_reset(altitude_ekf_t *ekf, float initial_alt);
void altitude_ekf_predict(altitude_ekf_t *ekf, float accel_z_world, float dt);
void altitude_ekf_update_baro(altitude_ekf_t *ekf, float baro_alt);

/* =========================================================================
   5. Outlier Rejection Filter (Khu nhieu gai bien thien dot bien)
   ========================================================================= */
typedef struct {
    float   max_delta;          // Nguong bien thien toi da cho phep giua 2 mau lien tiep
    float   last_valid;         // Gia tri hop le gan nhat
    uint8_t count;              // Dem so mau vuot nguong lien tiep
    uint8_t max_consecutive;    // So mau vuot nguong lien tiep de chap nhan dong bo lai
    bool    initialized;
} outlier_filter_t;

void  outlier_filter_init(outlier_filter_t *f, float max_delta, uint8_t max_consecutive);
void  outlier_filter_reset(outlier_filter_t *f, float initial_val);
float outlier_filter_apply(outlier_filter_t *f, float input);

#ifdef __cplusplus
}
#endif

#endif /* MATH_FILTER_FILTER_H_ */
