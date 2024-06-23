/*
 * @Author: Jhaoz0133 JHao_Z@163.com
 * @Date: 2024-06-03 00:53:14
 * @LastEditors: Jhaoz0133 JHao_Z@163.com
 * @LastEditTime: 2024-06-22 21:39:46
 * @FilePath: \uwb_mouse\algorithm\mouse.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MOUSE_H
#define MOUSE_H

#include <stdint.h>
#include "wireless.h"

void mouse_init();
void mouse_cal_pix(float tag_pitch, float tag_yaw, float anchor_aoa, float dis);
void mouse_control_init(float tag_pitch, float tag_yaw, float anchor_aoa, float dis);
void set_btn_state(uint8_t state);

void mouse_data_send(uint8_t btn, float pitch, float yaw);

#if WIRELESS_SLAVE
void btn_down_event_handler(uint8_t btn_index);
#endif
#endif