/*
 * @Author: Jhaoz Jhao_Z@163.com
 * @Date: 2024-01-29 10:06:33
 * @LastEditors: Jhaoz0133 JHao_Z@163.com
 * @LastEditTime: 2024-06-22 16:44:09
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
    copy_f32(kalman->Q_pdata, q, 9);
    copy_f32(kalman->R_pdata, r, 9);
    copy_f32(kalman->P_pdata, p0, 9);
    copy_f32(kalman->x_pdata, x0, 3);
    mat_init_f32(&kalman->Q, 3, 3, kalman->Q_pdata);
    mat_init_f32(&kalman->R, 3, 3, kalman->R_pdata);
    mat_init_f32(&kalman->P, 3, 3, kalman->P_pdata);
    mat_init_f32(&kalman->x, 3, 1, kalman->x_pdata);
    kalman->init = 1;
}

void kalman3_predict(Kalman_3 *kalman, float F_pdata[9])
{
    mat_f32 F;
    mat_init_f32(&F, 3, 3, F_pdata);

    float ans31_pdata[3];
    mat_f32 ans31;
    mat_init_f32(&ans31, 3, 1, ans31_pdata);
    mat_mult_f32(&F, &kalman->x, &ans31);
    copy_f32(kalman->x_pdata, ans31_pdata, 3);

    float ans331_pdata[9], ans332_pdata[9];
    mat_f32 ans331, ans332;
    mat_init_f32(&ans331, 3, 3, ans331_pdata);
    mat_init_f32(&ans332, 3, 3, ans332_pdata);

    mat_mult_f32(&F, &kalman->P, &ans331);
    mat_trans_f32(&F, &ans332);
    mat_mult_f32(&ans331, &ans332, &kalman->P);
    mat_add_f32(&kalman->P, &kalman->Q, &kalman->P);
}

void kalman3_update(Kalman_3 *kalman, float z_pdata[3])
{
    float ans311_pdata[3], ans312_pdata[3], k_pdata[9], ans331_pdata[9], ans332_pdata[9], eye33_pdata[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    mat_f32 z, K, ans311, ans312, ans331, ans332, eye33;

    mat_init_f32(&eye33, 3, 3, eye33_pdata);
    mat_init_f32(&ans331, 3, 3, ans331_pdata);
    mat_init_f32(&ans332, 3, 3, ans332_pdata);
    mat_init_f32(&ans311, 3, 1, ans311_pdata);
    mat_init_f32(&ans312, 3, 1, ans312_pdata);
    mat_init_f32(&z, 3, 1, z_pdata);
    mat_init_f32(&K, 3, 3, k_pdata);

    mat_add_f32(&kalman->P, &kalman->R, &ans331);
    mat_inv_f32(&ans331, &ans332);
    mat_mult_f32(&kalman->P, &ans332, &K);

    mat_sub_f32(&eye33, &K, &ans331);
    mat_mult_f32(&ans331, &kalman->P, &ans332);
    copy_f32(kalman->P_pdata, ans332_pdata, 9);

    mat_sub_f32(&z, &kalman->x, &ans311);
    mat_mult_f32(&K, &ans311, &ans312);
    mat_add_f32(&kalman->x, &ans312, &kalman->x);
}
