#include <stdint.h>

int16_t gyro_data_raw_list[][3] = {
    {-33, 14, -9},
    {-33, 14, -9},
    {-33, 14, -9},
    {-33, 14, -9},
    {-33, 14, -9},
    {-33, 14, -9},
    {-33, 14, -9},
    {-33, 14, -9},
    {-33, 14, -9},
    {-33, 14, -9},
    {-33, 14, -9},
    {-33, 14, -9},
};
int16_t acc_data_raw_list[][3] = {
    {-1417, 139, 1440}, 
    {-1418, 139, 1441},
    {-1417, 139, 1440},
    {-1418, 139, 1441},
    {-1417, 139, 1440},
    {-1418, 139, 1441},
    {-1417, 139, 1440},
    {-1418, 139, 1441},
    {-1417, 139, 1440},
    {-1418, 139, 1441},
    {-1417, 139, 1440},
    {-1418, 139, 1441},
    {-1417, 139, 1440},
    {-1418, 139, 1441},
};
char uwb_data_raw_list[][50] = {
    "NO(47756). D: 0.442, A: -17, *",
    "NO(47757). D: 0.447, A: -17, *",
    "NO(47758). D: 0.424, A: -14, *",
    "NO(47759). D: 0.461, A: -11, *",
    "NO(47760). D: 0.438, A: -12, *",
    "NO(47762). D: 0.480, A: -10, *",
    "NO(47763). D: 0.428, A: -11, *",
    "NO(47764). D: 0.447, A: -13, *",
    "NO(47766). D: 0.466, A: -8, *",
    "NO(47767). D: 0.466, A: -6, *",
    "NO(47769). D: 0.447, A: -6, *",
    "NO(47771). D: 0.358, A: -2, *",
    "NO(47776). D: 0.358, A: -2, *",
    "NO(47779). D :0.391, A: 0, *",
    "NO(47780). D: 0.391, A: 0, *",
    "NO(47781). D: 0.381, A: -2, *",
    "NO(47783). D: 0.410, A: -6, *",
    "NO(47784). D: 0.363, A: -3, *",
    "NO(47785). D: 0.306, A: -4, ",
    "NO(47787). D: 0.283, A: -5, ",
    "NO(47788). D: 0.316, A: -2, *",
    "NO(47793). D: 0.419, A: -5, *",
};
uint16_t count = 0;
void main()
{  
    while (count < 11) {
        printf("%d,1\n", count);
        algo_imu_data_update_event_handler(gyro_data_raw_list[count], acc_data_raw_list[count]);
        printf("%d,2\n", count);
        algo_uwb_data_update_event_handler(uwb_data_raw_list[count]);
        count++;
    }


}