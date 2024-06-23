#include "algo_config.h"
#include "algo_interfaces.h"
#include "uwb_data_parser.h"
#include "kalman.h"
#include "my_math.h"
#include "my_mat.h"
#include <math.h>
#include <stdlib.h>
#include "my_queue.h"
float cbn[9] = {0};
float euler[3] = {0};
float a[3] = {0};

Kalman_3 g_n_b_kalman;
Kalman_3 m_n_b_kalman;

void cal_cbn(float g_n_b_pdata[3], float m_n_b_pdata[3], float c_b_n_pdata[9]);
void cal_aln(float c_b_n_INIT[9], float f_n_k_INIT[3], float a_l_n_INIT[3]);
void cal_euler(float cnb[9]);
void cal_so3(float w[3], float t, float result[9]);
void cal_ynb(float pdata[3], float yaw);

void attitude_calculator_get_a(float *pDst)
{
    pDst[0] = a[0];
    pDst[1] = a[1];
    pDst[2] = a[2];
}

void attitude_calculator_get_euler(float *pDst)
{
    pDst[0] = euler[0];
    pDst[1] = euler[1];
    pDst[2] = euler[2];
}
void attitude_predict(float t)
{

    if (g_n_b_kalman.init == 1 && m_n_b_kalman.init == 1)
    {
        float w[3] = {0};
        algo_get_gyro_data(w);

        // 滤掉传感器错误值
        if (fabsf(w[0]) > 30 || fabsf(w[1]) > 30 || fabsf(w[2]) > 30)
        {
            platform_printf("error_w:%f,%f,%f\n", w[0], w[1], w[2]);
            return;
        }

        float so3[9] = {0};
        cal_so3(w, t, so3);

        kalman3_predict(&g_n_b_kalman, so3);
        kalman3_predict(&m_n_b_kalman, so3);

        cal_cbn(g_n_b_kalman.x_pdata, m_n_b_kalman.x_pdata, cbn);
        cal_euler(cbn);
    }
    // else
    // {
    //     float f[3] = {0};
    //     algo_get_acc_data(f);

    //     if (f[0] == 0 && f[1] == 0 && f[2] == 0)
    //     {
    //         return;
    //     }
    //     float P0[9] = {3.161519, 0, 0, 0, 3.161519, 0, 0, 0, 3.161519};
    //     float Q[9] = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};
    //     float R[9] = {10000, 0, 0, 0, 10000, 0, 0, 0, 10000};
    //     float g_x0[3] = {f[0], f[1], f[2]};
    //     float m_x0[3] = {0, 1, 0};
    //     kalman3_init(&m_n_b_kalman, m_x0, P0, Q, R);
    //     kalman3_init(&g_n_b_kalman, g_x0, P0, Q, R);
    //     platform_printf("init:%f,%f,%f,%f,%f,%f\n", m_x0[0], m_x0[1], m_x0[2], f[0], f[1], f[2]);
    // }
}

void attitude_uwb_update()
{
    if (g_n_b_kalman.init == 0 || m_n_b_kalman.init == 0)
    {
        float f[3] = {0};
        algo_get_acc_data(f);

        if (f[0] == 0 && f[1] == 0 && f[2] == 0)
        {
            return;
        }

        float P0[9] = {3.0f, 0, 0, 0, 3.0f, 0, 0, 0, 3.0f};
        float Q[9] = {KALMAN_Q, 0, 0, 0, KALMAN_Q, 0, 0, 0, KALMAN_Q};
        float R[9] = {ACC_R, 0, 0, 0, ACC_R, 0, 0, 0, ACC_R};
        float g_x0[3] = {f[0], f[1], f[2]};
        float m_x0[3] = {0, 1, 0};

        cal_cbn(g_x0, m_x0, cbn);
        cal_euler(cbn);

        float aoa = 0;
        algo_get_uwb_data_aoa(&aoa);
        cal_ynb(m_x0, aoa);
#if UWB_DYNAMIC_R
        float weight = aoa * aoa * UWB_R;
        float R_t[9] = {0};
        R_t[0] = weight;
        R_t[4] = weight;
        R_t[8] = weight;
        copy_f32(m_n_b_kalman.R_pdata, R_t, 9);
#endif
        kalman3_init(&m_n_b_kalman, m_x0, P0, Q, R);
        kalman3_init(&g_n_b_kalman, g_x0, P0, Q, R);

        cal_cbn(g_n_b_kalman.x_pdata, m_n_b_kalman.x_pdata, cbn);
        cal_euler(cbn);
        // platform_printf("init:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", aoa * 57.3, euler[0] * 57.3, euler[1] * 57.3, euler[2] * 57.3, m_x0[0], m_x0[1], m_x0[2], f[0], f[1], f[2]);
        platform_printf("init:");
    }
    else
    {

        float aoa = 0, ynb[3] = {0};
        algo_get_uwb_data_aoa(&aoa);
        cal_ynb(ynb, aoa);
        cal_cbn(g_n_b_kalman.x_pdata, m_n_b_kalman.x_pdata, cbn);
        cal_euler(cbn);
    }
}

void attitude_acc_update()
{
    if (g_n_b_kalman.init == 1 && m_n_b_kalman.init == 1)
    {
        float f[3] = {0};
        algo_get_acc_data(f);

        if (f[0] == 0 && f[1] == 0 && f[2] == 0)
        {
            return;
        }

#if ACC_DYNAMIC_R
        float tmp = fabsf(sqrt_carmack(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]) - G_CONST);
        float weight = tmp * tmp * ACC_R;
        float R_t[9] = {0};
        R_t[0] = weight;
        R_t[4] = weight;
        R_t[8] = weight;
        copy_f32(g_n_b_kalman.R_pdata, R_t, 9);
#endif

        kalman3_update(&g_n_b_kalman, f);
        // platform_printf("p = %f\n", g_n_b_kalman.P_pdata[0]);
        cal_cbn(g_n_b_kalman.x_pdata, m_n_b_kalman.x_pdata, cbn);
        cal_euler(cbn);
    }
}

void cal_cbn(float g_n_b_pdata[3], float m_n_b_pdata[3], float c_b_n_pdata[9])
{
    float x_n_b_pdata[3], y_n_b_pdata[3], z_n_b_pdata[3];
    normalize3(m_n_b_pdata, y_n_b_pdata);
    normalize3(g_n_b_pdata, z_n_b_pdata);
    cross3(y_n_b_pdata, z_n_b_pdata, x_n_b_pdata);
    cross3(z_n_b_pdata, x_n_b_pdata, y_n_b_pdata);

    copy_f32(m_n_b_pdata, y_n_b_pdata, 3);
    c_b_n_pdata[0] = x_n_b_pdata[0];
    c_b_n_pdata[1] = x_n_b_pdata[1];
    c_b_n_pdata[2] = x_n_b_pdata[2];
    c_b_n_pdata[3] = y_n_b_pdata[0];
    c_b_n_pdata[4] = y_n_b_pdata[1];
    c_b_n_pdata[5] = y_n_b_pdata[2];
    c_b_n_pdata[6] = z_n_b_pdata[0];
    c_b_n_pdata[7] = z_n_b_pdata[1];
    c_b_n_pdata[8] = z_n_b_pdata[2];
}

void cal_euler(float cnb[9])
{
    euler[0] = atan2_piecewise(-cnb[6], cnb[8]); // y
    euler[1] = asin_piecewise(cnb[7]);           // b
    euler[2] = atan2_piecewise(-cnb[1], cnb[4]); // a
}

void cal_so3(float w[3], float t, float result[9])
{
    float theta_square = 0;
    float mu[3] = {0};
    scale3(w, t, mu);
    dot3(mu, mu, &theta_square);
    float theta = sqrt_carmack(theta_square);
    double kA;
    double kB;
    if (theta_square < 1.0E-8)
    {
        kA = 1.0 - 0.1666666716337204 * theta_square;
        kB = 0.5;
    }
    else if (theta_square < 1.0E-6)
    {
        kB = 0.5 - 0.0416666679084301 * theta_square;
        kA = 1.0 - theta_square * 0.1666666716337204 * (1.0 - 0.1666666716337204 * theta_square);
    }
    else
    {
        float invTheta = 1.0 / theta;
        kA = sinf(theta) * invTheta;
        kB = (1.0 - cosf(theta)) * (invTheta * invTheta);
    }

    float wx2 = mu[0] * mu[0];
    float wy2 = mu[1] * mu[1];
    float wz2 = mu[2] * mu[2];
    result[0] = 1.0 - kB * (wy2 + wz2);
    result[4] = 1.0 - kB * (wx2 + wz2);
    result[8] = 1.0 - kB * (wx2 + wy2);
    float a = -kA * mu[2];
    float b = -kB * (mu[0] * mu[1]);
    result[1] = b - a;
    result[3] = b + a;
    a = -kA * mu[1];
    b = -kB * (mu[0] * mu[2]);
    result[2] = b + a;
    result[6] = b - a;
    a = -kA * mu[0];
    b = -kB * (mu[1] * mu[2]);
    result[5] = b - a;
    result[7] = b + a;
}

void cal_ynb(float pdata[3], float yaw)
{
    static float yaw_mean_filt_data[25] = {0};
    static Queue yaw_mean_filter;
    queue_init(&yaw_mean_filter, yaw_mean_filt_data, 25, QUENE_ANALYZE_OPEN);
    queue_input(&yaw_mean_filter, yaw);
    float ca, cb, cy, sa, sb, sy;
    ca = cosf(yaw_mean_filter.mean);
    cb = cosf(euler[1]);
    cy = cosf(euler[0]);
    sa = sinf(yaw_mean_filter.mean);
    sb = sinf(euler[1]);
    sy = sinf(euler[0]);
    pdata[0] = ca * sb * sy + sa * cy;
    pdata[1] = ca * cb;
    pdata[2] = -ca * sb * cy + sa * sy;
}