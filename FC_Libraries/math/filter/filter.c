#include "math/filter/filter.h"
#include <math.h>

void biquad2p_filter_init(biquad2p_filter_t *f, float cutoff_hz, float sample_hz)
{
    float c = 1.0f / tanf(3.14159265f * cutoff_hz / sample_hz);
    float c2 = c * c;
    float a0 = 1.0f + 1.41421356f * c + c2;
    *f = (biquad2p_filter_t){
        .b0 = 1.0f / a0,
        .b1 = 2.0f / a0,
        .b2 = 1.0f / a0,
        .a1 = 2.0f * (1.0f - c2) / a0,
        .a2 = (1.0f - 1.41421356f * c + c2) / a0
    };
}

float biquad2p_filter_apply(biquad2p_filter_t *f, float input)
{
    float output = f->b0 * input + f->s1;
    f->s1 = f->b1 * input - f->a1 * output + f->s2;
    f->s2 = f->b2 * input - f->a2 * output;
    return output;
}

/*
 * Q (Process Noise): Ly thuyet la nhieu qua trinh / moi truong -> vao bai toan nay la Nhieu Gyro.
 * R (Measurement Noise): Ly thuyet la nhieu cam bien do luong -> vao bai toan nay la Nhieu Accel.
 */
void kalman_angle_init(kalman_angle_t *k, float q_angle, float q_bias, float r_measure)
{
    *k = (kalman_angle_t){ .Q_angle = q_angle, .Q_bias = q_bias, .R_measure = r_measure };
}

float kalman_angle_update(kalman_angle_t *k, float new_angle_acc, float new_rate_gyro, float dt)
{
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
