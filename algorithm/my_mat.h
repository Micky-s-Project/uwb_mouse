#ifndef MY_MAT_H
#define MY_MAT_H

#include <stdint.h>
#include <string.h>

typedef struct {
    uint16_t numRows;
    uint16_t numCols;
    float *pData;
} mat_f32;


void mat_init_f32(mat_f32 *S, uint16_t nRows, uint16_t nCols, float *pData);
uint8_t mat_add_f32(const mat_f32 *pSrcA, const mat_f32 *pSrcB, mat_f32 *pDst);
uint8_t mat_sub_f32(const mat_f32 *pSrcA, const mat_f32 *pSrcB, mat_f32 *pDst);
uint8_t mat_mult_f32(const mat_f32 *pSrcA, const mat_f32 *pSrcB, mat_f32 *pDst);
void mat_scale_f32(const mat_f32 *pSrc, float scale, mat_f32 *pDst);
void mat_trans_f32(const mat_f32 *pSrc, mat_f32 *pDst);
uint8_t mat_inv_f32(mat_f32 *pSrc, mat_f32 *pDst);
void mat_norm_f32(const mat_f32 *pSrc, mat_f32 *pDst);
void mat_copy_f32(const mat_f32 *pSrc, mat_f32 *pDst);
#define copy_f32(dst, src, len) memcpy(dst, src, len * sizeof(float))


#endif // MY_MAT_H