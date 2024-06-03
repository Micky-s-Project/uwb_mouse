#include "kalman.h"
#include "my_math.h"
#include "my_mat.h"
#include "my_queue.h"
#include "attitude_calculator.h"
#include <math.h>
#include "algo_interfaces.h"
#include "platform_api.h"

#define queue_len 20

Kalman_2 x_pos_kalman;
Kalman_2 y_pos_kalman;

float position[2] = {0};

float qx_data[queue_len] = {0}, qy_data[queue_len] = {0};
Queue qx, qy;

void cal_v_by_uwb(float x[3], float v[3], float t);

void position_calculator_get_pos(float pdata[2])
{
    pdata[0] = position[0];
    pdata[1] = position[1];
}

void position_calculate(float t, uint8_t uwb_ready)
{
    static float x[2] = {0};
    algo_get_uwb_data_xy(x);

    ALGO_DEBUG("x:%f,%f\n", x[0], x[1]);

    if (x_pos_kalman.init == 0 || y_pos_kalman.init == 0)
    {
        if (uwb_ready)
        {
            float P0[4] = {1, 0, 0, 1};
            float Q[4] = {0.01, 0, 0, 0.01};
            float R[4] = {1, 0, 0, 0.1};
            float x_x0[2] = {x[0], 0};
            float y_x0[2] = {x[1], 0};
            kalman2_init(&x_pos_kalman, x_x0, P0, Q, R);
            kalman2_init(&y_pos_kalman, y_x0, P0, Q, R);
            platform_printf("\nposition init!\n\n");
        }
    }
    else
    {
        float v[2] = {0};
        float tt2 = t * t * 0.5f;

        static float a[3] = {0};
        attitude_calculator_get_a(a);

        float z_k_x[2] = {x[0], v[0]}, z_k_y[2] = {x[1], v[1]};
        float u_k_x[2] = {a[0] * tt2, a[0] * t},
              u_k_y[2] = {a[1] * tt2, a[1] * t};
        float F_k_k_1[4] = {1, t, 0, 1};

        if (uwb_ready)
        {
            cal_v_by_uwb(x, v, t);
            z_k_x[0] = qx.mean;
            z_k_y[0] = qy.mean;
            float R_t[4] = {1, 0, 0, 0.001};
            R_t[3] = fabsf(v[0]);
            copy_f32(x_pos_kalman.R_INIT, R_t, 4);
            R_t[3] = fabsf(v[1]);
            copy_f32(y_pos_kalman.R_INIT, R_t, 4);
            kalman2_next(&x_pos_kalman, F_k_k_1, z_k_x, u_k_x, 0);
            kalman2_next(&y_pos_kalman, F_k_k_1, z_k_y, u_k_y, 0);
            position[0] = x_pos_kalman.x_k_1.pData[0];
            position[1] = y_pos_kalman.x_k_1.pData[0];
            platform_printf("x:%f,y:%f,x:%f,y:%f\n", position[0], position[1],z_k_x[0],z_k_y[0]);
        }
        else
        {

            kalman2_next(&x_pos_kalman, F_k_k_1, z_k_x, u_k_x, 1);
            kalman2_next(&y_pos_kalman, F_k_k_1, z_k_y, u_k_y, 1);
            position[0] = x_pos_kalman.x_k_1.pData[0];
            position[1] = y_pos_kalman.x_k_1.pData[0];
        }
    }
}

void cal_v_by_uwb(float x[2], float v[2], float t)
{
    queue_init(&qx, qx_data, queue_len, QUENE_ANALYZE_OPEN);
    queue_init(&qy, qy_data, queue_len, QUENE_ANALYZE_OPEN);
    float tmp;
    tmp = qx.mean;
    queue_input(&qx, x[0]);
    v[0] = (qx.mean - tmp) / (1 * t);
    tmp = qy.mean;
    queue_input(&qy, x[1]);
    v[1] = (qy.mean - tmp) / (1 * t);
}