#include "imu_data_parser.h"
#include "algo_config.h"

IMU_DATA_t imu_data = {0};
void imu_data_parse(int16_t gyro_data_raw[3], int16_t acc_data_raw[3])
{
    for (uint8_t i = 0; i < 3; i++)
    {
        imu_data.gyro_data_raw[i] = gyro_data_raw[i];
        imu_data.gyro_data[i] = (float)gyro_data_raw[i] * GYRO_DATA_SCALE;
        imu_data.acc_data_raw[i] = acc_data_raw[i];
        imu_data.acc_data[i] = (float)acc_data_raw[i] * ACC_DATA_SCALE;
    }
    gyro_data_zero_cali(imu_data.gyro_data, acc_data_raw);
}

void algo_get_gyro_data(float *pdata)
{
    pdata[0] = imu_data.gyro_data[0];
    pdata[1] = imu_data.gyro_data[1];
    pdata[2] = imu_data.gyro_data[2];
}

void algo_get_acc_data(float *pdata)
{
    pdata[0] = imu_data.acc_data[0];
    pdata[1] = imu_data.acc_data[1];
    pdata[2] = imu_data.acc_data[2];
}

void gyro_data_zero_cali(float gyro_data[3], int16_t acc_data_raw[3])
{
    static float gyro_bias[3] = {0.015936f, -0.004048f, 0.013848f};
    static uint8_t cali_if = 0;
    static uint8_t start_if = 0;
    static uint16_t freeze_count = 0;
    if (cali_if == 0)
    {
        // platform_printf("gyro_bias:%f,%f,%f,%d,%d,%d\n", gyro_data[0], gyro_data[1], gyro_data[2], acc_data_raw[0], acc_data_raw[1], acc_data_raw[2]);
    }

    if (freeze_count > 0)
        freeze_count--;
    if (freeze_count == 0)
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
                    platform_printf("gyro_bias:%f,%f,%f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
                    freeze_count = 40000;
                    start_if = 0;
                    cali_if = 1;
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