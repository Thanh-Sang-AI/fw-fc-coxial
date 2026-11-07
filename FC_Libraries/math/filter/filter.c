#include "math/filter/filter.h"
#include <math.h>
#include <stddef.h>

/* =========================================================================
   1. Biquad 2-Pole Butterworth Low-Pass Filter
   ========================================================================= */
void biquad2p_filter_init(biquad2p_filter_t *f, float cutoff_hz, float sample_hz)
{
    if (f == NULL) return;
    float c = 1.0f / tanf(3.14159265f * cutoff_hz / sample_hz);
    float c2 = c * c;
    float a0 = 1.0f + 1.41421356f * c + c2;
    *f = (biquad2p_filter_t){
        .b0 = 1.0f / a0,
        .b1 = 2.0f / a0,
        .b2 = 1.0f / a0,
        .a1 = 2.0f * (1.0f - c2) / a0,
        .a2 = (1.0f - 1.41421356f * c + c2) / a0,
        .s1 = 0.0f,
        .s2 = 0.0f
    };
}

float biquad2p_filter_apply(biquad2p_filter_t *f, float input)
{
    if (f == NULL) return input;
    float output = f->b0 * input + f->s1;
    f->s1 = f->b1 * input - f->a1 * output + f->s2;
    f->s2 = f->b2 * input - f->a2 * output;
    return output;
}

void biquad2p_filter_reset(biquad2p_filter_t *f, float value)
{
    if (f == NULL) return;
    f->s1 = value * (f->b1 - f->a1);
    f->s2 = value * (f->b2 - f->a2);
}

/* =========================================================================
   2. 2-State Linear Kalman Filter for Attitude
   - Q (Process Noise): Nhieu Gyro
   - R (Measurement Noise): Nhieu Accel
   ========================================================================= */
void kalman_angle_init(kalman_angle_t *k, float q_angle, float q_bias, float r_measure)
{
    if (k == NULL) return;
    *k = (kalman_angle_t){ .Q_angle = q_angle, .Q_bias = q_bias, .R_measure = r_measure };
}

float kalman_angle_update(kalman_angle_t *k, float new_angle_acc, float new_rate_gyro, float dt)
{
    if (k == NULL) return new_angle_acc;
    if (!k->initialized) {
        k->angle = new_angle_acc;
        k->initialized = true;
        return k->angle;
    }

    k->angle += dt * (new_rate_gyro - k->bias);
    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += dt * k->Q_bias;

    float y = new_angle_acc - k->angle;
    float S = k->P[0][0] + k->R_measure;
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    k->angle += K0 * y;
    k->bias  += K1 * y;

    float p00 = k->P[0][0], p01 = k->P[0][1];
    k->P[0][0] -= K0 * p00;
    k->P[0][1] -= K0 * p01;
    k->P[1][0] -= K1 * p00;
    k->P[1][1] -= K1 * p01;

    return k->angle;
}

/* =========================================================================
   3. Median Filter (Co the cau hinh kich thuoc cua so: 3, 5, 9... mau)
   ========================================================================= */
void median_filter_init(median_filter_t *f, uint8_t size)
{
    if (f == NULL) return;
    if (size == 0) size = 3;
    if (size > MEDIAN_FILTER_MAX_SIZE) size = MEDIAN_FILTER_MAX_SIZE;
    f->size  = size;
    f->count = 0;
    f->idx   = 0;
    for (uint8_t i = 0; i < MEDIAN_FILTER_MAX_SIZE; i++) {
        f->buf[i] = 0.0f;
    }
}

void median_filter_reset(median_filter_t *f, float value)
{
    if (f == NULL) return;
    f->count = f->size;
    f->idx   = 0;
    for (uint8_t i = 0; i < f->size; i++) {
        f->buf[i] = value;
    }
}

float median_filter_apply(median_filter_t *f, float input)
{
    if (f == NULL || f->size == 0) return input;

    f->buf[f->idx] = input;
    f->idx = (f->idx + 1) % f->size;
    if (f->count < f->size) f->count++;

    if (f->count == 1) return f->buf[0];

    // Sao chep sang mang tam de sap xep
    float temp[MEDIAN_FILTER_MAX_SIZE];
    for (uint8_t i = 0; i < f->count; i++) {
        temp[i] = f->buf[i];
    }

    // Insertion sort (cuc nhanh va nhe cho so mau nho <= 15)
    for (uint8_t i = 1; i < f->count; i++) {
        float key = temp[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }

    // Tra ve gia tri o giua (trung vi)
    return temp[f->count / 2];
}

/* =========================================================================
   4. PX4 ECL EKF 3-State Altitude Filter (IMU Accel Z + Baro)
   ========================================================================= */
void altitude_ekf_init(altitude_ekf_t *ekf, float acc_noise, float bias_noise, float baro_noise)
{
    if (ekf == NULL) return;
    ekf->z          = 0.0f;
    ekf->vz         = 0.0f;
    ekf->accel_bias = 0.0f;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ekf->P[i][j] = (i == j) ? ((i == 2) ? 0.1f : 1.0f) : 0.0f;
        }
    }

    ekf->Q_accel     = acc_noise * acc_noise;
    ekf->Q_bias      = bias_noise * bias_noise;
    ekf->R_baro      = baro_noise * baro_noise;
    ekf->initialized = false;
}

void altitude_ekf_reset(altitude_ekf_t *ekf, float initial_alt)
{
    if (ekf == NULL) return;
    ekf->z  = initial_alt;
    ekf->vz = 0.0f;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ekf->P[i][j] = (i == j) ? ((i == 2) ? 0.1f : 0.1f) : 0.0f;
        }
    }
    ekf->initialized = true;
}

void altitude_ekf_predict(altitude_ekf_t *ekf, float accel_z_world, float dt)
{
    if (ekf == NULL || !ekf->initialized || dt <= 0.0f) return;

    /* 1. Predict state: dong hoc quan tinh strapdown */
    float a = accel_z_world - ekf->accel_bias;
    ekf->z  += ekf->vz * dt + 0.5f * a * dt * dt;
    ekf->vz += a * dt;

    /* 2. Predict covariance: P = F * P * F^T + Q */
    float P00 = ekf->P[0][0], P01 = ekf->P[0][1], P02 = ekf->P[0][2];
    float P11 = ekf->P[1][1], P12 = ekf->P[1][2], P22 = ekf->P[2][2];

    ekf->P[0][0] = P00 + dt * (2.0f * P01 + dt * P11);
    ekf->P[0][1] = P01 + dt * (P11 - P02 - dt * P12);
    ekf->P[0][2] = P02 + dt * P12;
    ekf->P[1][1] = P11 + dt * (-2.0f * P12 + dt * P22) + ekf->Q_accel * dt;
    ekf->P[1][2] = P12 - dt * P22;
    ekf->P[2][2] = P22 + ekf->Q_bias * dt;

    /* Dam bao tinh doi xung cua ma tran hiep phuong sai P */
    ekf->P[1][0] = ekf->P[0][1];
    ekf->P[2][0] = ekf->P[0][2];
    ekf->P[2][1] = ekf->P[1][2];
}

void altitude_ekf_update_baro(altitude_ekf_t *ekf, float baro_alt)
{
    if (ekf == NULL) return;
    if (!ekf->initialized) {
        altitude_ekf_reset(ekf, baro_alt);
        return;
    }

    float y = baro_alt - ekf->z;

    float S = ekf->P[0][0] + ekf->R_baro;
    if (S <= 0.0f) return;

    /* Innovation consistency gate: 3 * sqrt(S) (chuan PX4) */
    float gate = 3.0f * sqrtf(S);
    if (gate < 0.5f) gate = 0.5f;
    if (y > gate)  y = gate;
    if (y < -gate) y = -gate;

    /* Tinh Kalman Gain */
    float K0 = ekf->P[0][0] / S;
    float K1 = ekf->P[1][0] / S;
    float K2 = ekf->P[2][0] / S;

    /* Cap nhat trang thai */
    ekf->z          += K0 * y;
    ekf->vz         += K1 * y;
    ekf->accel_bias += K2 * y;

    /* Gioi han bias gia toc trong pham vi vat ly +-2.0 m/s^2 */
    if (ekf->accel_bias > 2.0f)  ekf->accel_bias = 2.0f;
    if (ekf->accel_bias < -2.0f) ekf->accel_bias = -2.0f;

    /* Cap nhat hiep phuong sai: P = (I - K*H) * P */
    float P00 = ekf->P[0][0];
    float P01 = ekf->P[0][1];
    float P02 = ekf->P[0][2];

    ekf->P[0][0] -= K0 * P00;
    ekf->P[0][1] -= K0 * P01;
    ekf->P[0][2] -= K0 * P02;

    ekf->P[1][0] = ekf->P[0][1];
    ekf->P[1][1] -= K1 * P01;
    ekf->P[1][2] -= K1 * P02;

    ekf->P[2][0] = ekf->P[0][2];
    ekf->P[2][1] = ekf->P[1][2];
    ekf->P[2][2] -= K2 * P02;
}

/* =========================================================================
   5. Outlier Rejection Filter
   ========================================================================= */
void outlier_filter_init(outlier_filter_t *f, float max_delta, uint8_t max_consecutive)
{
    if (f == NULL) return;
    f->max_delta        = max_delta;
    f->max_consecutive  = (max_consecutive == 0) ? 10 : max_consecutive;
    f->last_valid       = 0.0f;
    f->count            = 0;
    f->initialized      = false;
}

void outlier_filter_reset(outlier_filter_t *f, float initial_val)
{
    if (f == NULL) return;
    f->last_valid   = initial_val;
    f->count        = 0;
    f->initialized  = true;
}

float outlier_filter_apply(outlier_filter_t *f, float input)
{
    if (f == NULL) return input;
    if (!f->initialized) {
        f->last_valid   = input;
        f->initialized  = true;
        f->count        = 0;
        return input;
    }

    if (fabsf(input - f->last_valid) > f->max_delta) {
        f->count++;
        if (f->count < f->max_consecutive) {
            return f->last_valid; // Loai bo spike, tra ve gia tri hop le truoc do
        } else {
            // Vuot nguong qua nhieu lan lien tiep -> chap nhan va dong bo lai
            f->last_valid = input;
            f->count      = 0;
            return input;
        }
    }

    f->count      = 0;
    f->last_valid = input;
    return input;
}
