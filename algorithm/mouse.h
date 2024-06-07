#ifndef MOUSE_H
#define MOUSE_H

#include <stdint.h>

void mouse_init();
void mouse_cal_pix(float pitch, float yaw);
void mouse_control_init();
#endif