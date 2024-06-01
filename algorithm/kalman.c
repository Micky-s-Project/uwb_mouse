/*
 * @Author: Jhaoz Jhao_Z@163.com
 * @Date: 2024-01-29 10:06:33
 * @LastEditors: Jhaoz Jhao_Z@163.com
 * @LastEditTime: 2024-01-31 09:48:54
 * @FilePath: \pd3_bt\kalman.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */

#include "kalman.h"
#include "my_mat.h"
#include <string.h>

void kalman3_init(Kalman_3 *kalman, float x0[3], float p0[9], float q[9], float r[9])
{
    copy_f32(kalman->Q_INIT, q, 9);
    copy_f32(kalman->R_INIT, r, 9);
    copy_f32(kalman->P_k_1_INIT, p0, 9);
    copy_f32(kalman->x_k_1_INIT, x0, 3);
    mat_init_f32(&kalman->Q, 3, 3, kalman->Q_INIT);
    mat_init_f32(&kalman->R, 3, 3, kalman->R_INIT);
    mat_init_f32(&kalman->P_k_1, 3, 3, kalman->P_k_1_INIT);
    mat_init_f32(&kalman->x_k_1, 3, 1, kalman->x_k_1_INIT);
    kalman->init = 1;
}

void kalman3_next(Kalman_3 *kalman, float F_k_k_1_INIT[9], float Z_k_INIT[3], uint8_t only_integral)
{
    mat_f32 F_k_k_1, Z_k;
    mat_init_f32(&F_k_k_1, 3, 3, F_k_k_1_INIT);
    mat_init_f32(&Z_k, 3, 1, Z_k_INIT);

    if (!only_integral) {
        float ans311_INIT[3], ans312_INIT[3], ans331_INIT[9], ans332_INIT[9], eye33_INIT[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        mat_f32 ans311, ans312, ans331, ans332, eye33;

        mat_init_f32(&eye33, 3, 3, eye33_INIT);
        mat_init_f32(&ans331, 3, 3, ans331_INIT);
        mat_init_f32(&ans332, 3, 3, ans332_INIT);
        mat_init_f32(&ans311, 3, 1, ans311_INIT);
        mat_init_f32(&ans312, 3, 1, ans312_INIT);

        float x_k_k_1_INIT[3];
        mat_f32 x_k_k_1;
        mat_init_f32(&x_k_k_1, 3, 1, x_k_k_1_INIT);

        float P_k_k_1_INIT[9];
        mat_f32 P_k_k_1;
        mat_init_f32(&P_k_k_1, 3, 3, P_k_k_1_INIT);

        float K_k_INIT[9];
        mat_f32 K_k;
        mat_init_f32(&K_k, 3, 3, K_k_INIT);

        // X_k_k-1 = F_k_k_1 * X_k-1 + U_k
        mat_mult_f32(&F_k_k_1, &kalman->x_k_1, &x_k_k_1);

        mat_mult_f32(&F_k_k_1, &kalman->P_k_1, &ans331);
        mat_trans_f32(&F_k_k_1, &ans332);
        mat_mult_f32(&ans331, &ans332, &P_k_k_1);
        mat_add_f32(&P_k_k_1, &kalman->Q, &P_k_k_1);

        mat_add_f32(&P_k_k_1, &kalman->R, &ans331);
        mat_inv_f32(&ans331, &ans332);
        mat_mult_f32(&P_k_k_1, &ans332, &K_k);

        mat_sub_f32(&eye33, &K_k, &ans331);
        mat_mult_f32(&ans331, &P_k_k_1, &kalman->P_k_1);

        mat_sub_f32(&Z_k, &x_k_k_1, &ans311);
        mat_mult_f32(&K_k, &ans311, &ans312);
        mat_add_f32(&x_k_k_1, &ans312, &kalman->x_k_1);
    } else {
        float ans31_INIT[3];
        mat_f32 ans31;
        mat_init_f32(&ans31, 3, 1, ans31_INIT);
        // X_k_k-1 = F_k_k_1 * X_k-1 + U_k
        mat_mult_f32(&F_k_k_1, &kalman->x_k_1, &ans31);
        copy_f32(kalman->x_k_1_INIT, ans31_INIT, 3);
    }
}

void kalman2_init(Kalman_2 *kalman, float x0[2], float p0[4], float q[4], float r[4])
{
    copy_f32(kalman->Q_INIT, q, 4);
    copy_f32(kalman->R_INIT, r, 4);
    copy_f32(kalman->P_k_1_INIT, p0, 4);
    copy_f32(kalman->x_k_1_INIT, x0, 2);
    mat_init_f32(&kalman->Q, 2, 2, kalman->Q_INIT);
    mat_init_f32(&kalman->R, 2, 2, kalman->R_INIT);
    mat_init_f32(&kalman->P_k_1, 2, 2, kalman->P_k_1_INIT);
    mat_init_f32(&kalman->x_k_1, 2, 1, kalman->x_k_1_INIT);
    kalman->init = 1;
}

void kalman2_next(Kalman_2 *kalman, float F_k_k_1_INIT[4], float Z_k_INIT[2], float U_k_INIT[2], uint8_t only_integral)
{
    mat_f32 F_k_k_1, Z_k, U_k;
    mat_init_f32(&F_k_k_1, 2, 2, F_k_k_1_INIT);
    mat_init_f32(&Z_k, 2, 1, Z_k_INIT);
    mat_init_f32(&U_k, 2, 1, U_k_INIT);

    if (!only_integral) {
        float ans211_INIT[2], ans212_INIT[2], ans221_INIT[4], ans222_INIT[4], eye22_INIT[4] = {1, 0, 0, 1};
        mat_f32 ans211, ans212, ans221, ans222, eye22;

        mat_init_f32(&eye22, 2, 2, eye22_INIT);
        mat_init_f32(&ans221, 2, 2, ans221_INIT);
        mat_init_f32(&ans222, 2, 2, ans222_INIT);
        mat_init_f32(&ans211, 2, 1, ans211_INIT);
        mat_init_f32(&ans212, 2, 1, ans212_INIT);

        float x_k_k_1_INIT[2];
        mat_f32 x_k_k_1;
        mat_init_f32(&x_k_k_1, 2, 1, x_k_k_1_INIT);

        float P_k_k_1_INIT[4];
        mat_f32 P_k_k_1;
        mat_init_f32(&P_k_k_1, 2, 2, P_k_k_1_INIT);

        float K_k_INIT[4];
        mat_f32 K_k;
        mat_init_f32(&K_k, 2, 2, K_k_INIT);

        // X_k_k-1 = F_k_k_1 * X_k-1 + U_k
        mat_mult_f32(&F_k_k_1, &kalman->x_k_1, &ans211);
        mat_add_f32(&ans211, &U_k, &x_k_k_1);

        mat_mult_f32(&F_k_k_1, &kalman->P_k_1, &ans221);
        mat_trans_f32(&F_k_k_1, &ans222);
        mat_mult_f32(&ans221, &ans222, &P_k_k_1);
        mat_add_f32(&P_k_k_1, &kalman->Q, &P_k_k_1);

        mat_add_f32(&P_k_k_1, &kalman->R, &ans221);
        mat_inv_f32(&ans221, &ans222);
        mat_mult_f32(&P_k_k_1, &ans222, &K_k);

        mat_sub_f32(&eye22, &K_k, &ans221);
        mat_mult_f32(&ans221, &P_k_k_1, &kalman->P_k_1);

        mat_sub_f32(&Z_k, &x_k_k_1, &ans211);
        mat_mult_f32(&K_k, &ans211, &ans212);
        mat_add_f32(&x_k_k_1, &ans212, &kalman->x_k_1);
        // printf("%f,%f,%f,%f\n",  K_k.pData[0], K_k.pData[1], K_k.pData[2], K_k.pData[3]);
    } else {
        float ans21_INIT[2];
        mat_f32 ans21;

        mat_init_f32(&ans21, 2, 1, ans21_INIT);

        // X_k_k-1 = F_k_k_1 * X_k-1 + U_k
        mat_mult_f32(&F_k_k_1, &kalman->x_k_1, &ans21);
        mat_add_f32(&ans21, &U_k, &kalman->x_k_1);
    }
}