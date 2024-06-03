#include "algo_config.h"
#include "algo_interfaces.h"
#include "kalman.h"
#include "my_math.h"
#include "my_mat.h"
#include <math.h>
#include <stdlib.h>

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

void attitude_calculate(float t, uint8_t uwb_data_ready)
{
    float w[3] = {0};
    float f[3] = {0};
    algo_get_gyro_data(w);
    algo_get_acc_data(f);
    if (uwb_data_ready == 1)
    {
        // platform_printf("w:%f,%f,%f\n", w[0], w[1], w[2]);
        // platform_printf("f:%f,%f,%f\n", f[0], f[1], f[2]);
    }

    if (g_n_b_kalman.init == 0 || m_n_b_kalman.init == 0)
    {
        float P0[9] = {3.161519, 0, 0, 0, 3.161519, 0, 0, 0, 3.161519};
        float Q[9] = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};
        float R[9] = {10000, 0, 0, 0, 10000, 0, 0, 0, 10000};
        float g_x0[3] = {f[0], f[1], f[2]};
        float m_x0[3] = {0, 1, 0};
        if (uwb_data_ready && (f[0] != 0 || f[1] != 0 || f[2] != 0))
        {
            cal_cbn(g_x0, m_x0, cbn);
            ALGO_DEBUG("cbn:%f,%f,%f\n", cbn[0], cbn[1], cbn[2]);
            ALGO_DEBUG("cbn:%f,%f,%f\n", cbn[3], cbn[4], cbn[5]);
            ALGO_DEBUG("cbn:%f,%f,%f\n", cbn[6], cbn[7], cbn[8]);
            cal_euler(cbn);
            ALGO_DEBUG("e:%f,%f,%f\n", euler[0] * 57.3, euler[1] * 57.3, euler[2] * 57.3);
            float aoa = 0;
            algo_get_uwb_data_aoa(&aoa);
            cal_ynb(m_x0, aoa);
            ALGO_DEBUG("ynb:%f,%f,%f\n", m_x0[0], m_x0[1], m_x0[2]);

            kalman3_init(&g_n_b_kalman, g_x0, P0, Q, R);
            kalman3_init(&m_n_b_kalman, m_x0, P0, Q, R);
            platform_printf("\nattitude init!\n\n");

            cal_cbn(g_n_b_kalman.x_k_1.pData, m_n_b_kalman.x_k_1.pData, cbn);
            ALGO_DEBUG("cbn:%f,%f,%f\n", cbn[0], cbn[1], cbn[2]);
            ALGO_DEBUG("cbn:%f,%f,%f\n", cbn[3], cbn[4], cbn[5]);
            ALGO_DEBUG("cbn:%f,%f,%f\n", cbn[6], cbn[7], cbn[8]);
            cal_euler(cbn);
        }
    }
    else
    {
        uint8_t only_predict = 1;
        float ynb[3] = {0};
        if (uwb_data_ready == 1)
        {
            only_predict = 0;
            float aoa = 0;
            algo_get_uwb_data_aoa(&aoa);
            cal_ynb(ynb, aoa);

            float weight = abs(sqrt_carmack(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]) - G_CONST) * 25000;
            float R_t[9] = {10000, 0, 0, 0, 10000, 0, 0, 0, 10000};
            R_t[0] = weight;
            R_t[4] = weight;
            R_t[8] = weight;
            copy_f32(g_n_b_kalman.R_INIT, R_t, 9);
            // copy_f32(m_n_b_kalman.R_INIT, R_t, 9);
            // ALGO_DEBUG("ynb:%f,%f,%f\n", ynb[0], ynb[1], ynb[2]);
        }
        float so3[9] = {0};
        cal_so3(w, t, so3);

        kalman3_next(&g_n_b_kalman, so3, f, only_predict);
        kalman3_next(&m_n_b_kalman, so3, ynb, only_predict);

        cal_cbn(g_n_b_kalman.x_k_1.pData, m_n_b_kalman.x_k_1.pData, cbn);
        cal_aln(cbn, f, a);
        // ALGO_DEBUG("cbn:%f,%f,%f\n", cbn[0], cbn[1], cbn[2]);
        // ALGO_DEBUG("cbn:%f,%f,%f\n", cbn[3], cbn[4], cbn[5]);
        // ALGO_DEBUG("cbn:%f,%f,%f\n", cbn[6], cbn[7], cbn[8]);
        // ALGO_DEBUG("a:%f,%f,%f\n", a[0], a[1], a[2]);
        cal_euler(cbn);
        if (uwb_data_ready == 1)
        {
            // platform_printf("a:%f,%f,%f\n", a[0], a[1], a[2]);
            platform_printf("e:%f,%f,%f\n", euler[0] * 57.3, euler[1] * 57.3, euler[2] * 57.3);
        }
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

void cal_aln(float c_b_n_INIT[9], float f_n_k_INIT[3], float a_l_n_INIT[3])
{
    float ans_31_INIT[3] = {0}, g_const_INIT[3] = {0, 0, G_CONST};
    mat_f32 ans_31, g_const, c_b_n, f_n_k, a_l_n;
    mat_init_f32(&ans_31, 3, 1, ans_31_INIT);
    mat_init_f32(&g_const, 3, 1, g_const_INIT);
    mat_init_f32(&c_b_n, 3, 3, c_b_n_INIT);
    mat_init_f32(&f_n_k, 3, 1, f_n_k_INIT);
    mat_init_f32(&a_l_n, 3, 1, a_l_n_INIT);
    mat_mult_f32(&c_b_n, &f_n_k, &ans_31);
    mat_sub_f32(&ans_31, &g_const, &a_l_n);
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

void gyro_data_zero_cali(float gyro_data[3], int16_t acc_data_raw[3])
{
    static float gyro_bias[3] = {0};
    static uint8_t cali_if = 0;
    static uint8_t start_if = 0;
    static uint16_t freeze_count = 0;

    if (freeze_count > 0)
        freeze_count--;
    if (cali_if == 0 && freeze_count == 0)
    {
        static uint16_t acc_static_count = 0;
        static int16_t acc_static_first[3];

        static float gyro_bias_sum[3] = {0};
        static uint16_t gyro_bias_count = 0;
        if (start_if == 0)
        {
            acc_static_first[0] = acc_data_raw[0];
            acc_static_first[1] = acc_data_raw[1];
            acc_static_first[2] = acc_data_raw[2];
            start_if = 1;
            gyro_bias_count = 0;
            gyro_bias_sum[0] = 0;
            gyro_bias_sum[1] = 0;
            gyro_bias_sum[2] = 0;
        }
        else if (++acc_static_count == 8)
        {
            acc_static_count = 0;

            int16_t ans[3];
            ans[0] = abs(acc_static_first[0] - acc_data_raw[0]);
            ans[1] = abs(acc_static_first[1] - acc_data_raw[1]);
            ans[2] = abs(acc_static_first[2] - acc_data_raw[2]);

            if (ans[0] < 20 && ans[1] < 20 && ans[2] < 20)
            {
                gyro_bias_sum[0] += gyro_data[0];
                gyro_bias_sum[1] += gyro_data[1];
                gyro_bias_sum[2] += gyro_data[2];
                if (++gyro_bias_count == 25)
                {
                    gyro_bias[0] = gyro_bias_sum[0] / 25;
                    gyro_bias[1] = gyro_bias_sum[1] / 25;
                    gyro_bias[2] = gyro_bias_sum[2] / 25;
                    freeze_count = 40000;
                    start_if = 0;
                }
            }
            else
            {
                start_if = 0;
            }
        }
    }

    gyro_data[0] -= gyro_bias[0];
    gyro_data[1] -= gyro_bias[1];
    gyro_data[2] -= gyro_bias[2];
}

void cal_ynb(float pdata[3], float yaw)
{
    float ca, cb, cy, sa, sb, sy;
    ca = cosf(yaw);
    cb = cosf(euler[1]);
    cy = cosf(euler[0]);
    sa = sinf(yaw);
    sb = sinf(euler[1]);
    sy = sinf(euler[0]);
    pdata[0] = ca * sb * sy + sa * cy;
    pdata[1] = ca * cb;
    pdata[2] = -ca * sb * cy + sa * sy;
}