/*
 * @Author: Jhaoz0133 JHao_Z@163.com
 * @Date: 2024-06-01 00:09:11
 * @LastEditors: Jhaoz0133 JHao_Z@163.com
 * @LastEditTime: 2024-06-22 16:44:38
 * @FilePath: \uwb_mouse\algorithm\kalman.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _KALMAN_3_H_
#define _KALMAN_3_H_

#include "algo_config.h"
#include "my_mat.h"

typedef struct Kalman_3
{
    uint8_t init;
    float P_pdata[9];
    float Q_pdata[9];
    float R_pdata[9];
    float x_pdata[3];
    mat_f32 P;
    mat_f32 Q;
    mat_f32 R;
    mat_f32 x;
} Kalman_3;

void kalman3_init(Kalman_3 *kalman, float x0[3], float p0[9], float q[9], float r[9]);
void kalman3_predict(Kalman_3 *kalman, float F_pdata[9]);
void kalman3_update(Kalman_3 *kalman, float z_pdata[3]);

#endif // _KALMAN_3_H_