#ifndef MY_MATH_H
#define MY_MATH_H

#include <stdint.h>

void get_so3_by_w(float w[3], float t, float result[9]);
float asin_piecewise(float x);
float atan2_piecewise(float y, float x);

void cross3(float pSrcA[3], float pSrcB[3], float pDst[3]);
void dot3(float pSrcA[3], float pSrcB[3], float pDst[1]);
void scale3(float pSrc[3], float scale, float pDst[3]);
void normalize3(float pSrc[3], float pDst[3]);

float sqrt_carmack(float number);
float inv_sqrt_carmack(float number);

#endif // MY_MATH_H