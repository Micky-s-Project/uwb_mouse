#ifndef _KALMAN_3_H_
#define _KALMAN_3_H_

#include "algo_config.h"
#include "my_mat.h"

typedef struct Kalman_3
{
    uint8_t init;
    float P_k_1_INIT[9];
    float Q_INIT[9];
    float R_INIT[9];
    float x_k_1_INIT[3];
    mat_f32 P_k_1;
    mat_f32 Q;
    mat_f32 R;
    mat_f32 x_k_1;
} Kalman_3;
void kalman3_init(Kalman_3 *kalman, float x0[3], float p0[9], float q[9], float r[9]);
void kalman3_next(Kalman_3 *kalman, float F_k_k_1_INIT[9], float Z_k_INIT[3], uint8_t only_integral);

typedef struct Kalman_2
{
    uint8_t init;
    float P_k_1_INIT[4];
    float Q_INIT[4];
    float R_INIT[4];
    float x_k_1_INIT[2];
    mat_f32 P_k_1;
    mat_f32 Q;
    mat_f32 R;
    mat_f32 x_k_1;
} Kalman_2;
void kalman2_init(Kalman_2 *kalman, float x0[2], float p0[4], float q[4], float r[4]);
void kalman2_next(Kalman_2 *kalman, float F_k_k_1_INIT[4], float Z_k_INIT[2], float U_k_INIT[2], uint8_t only_integral);

#endif // _KALMAN_3_H_