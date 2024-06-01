/*
 * @Author: Jhaoz Jhao_Z@163.com
 * @Date: 2024-01-29 09:13:58
 * @LastEditors: Jhaoz Jhao_Z@163.com
 * @LastEditTime: 2024-01-29 11:27:16
 * @FilePath: \代码移植\algorithm\mat.c
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "algo_config.h"
#include "my_mat.h"
#define DATA(mat, i, j) (mat)->pData[(i) * (mat)->numCols + (j)]

// 初始化矩阵
void mat_init_f32(mat_f32 *pSrc, uint16_t nRows, uint16_t nCols, float *pData)
{
    pSrc->numRows = nRows;
    pSrc->numCols = nCols;
    pSrc->pData = pData;
}

// 矩阵相加
uint8_t mat_add_f32(const mat_f32 *pSrcA, const mat_f32 *pSrcB, mat_f32 *pDst)
{
    if (pSrcA->numRows != pSrcB->numRows || pSrcA->numCols != pSrcB->numCols || pSrcA->numRows != pDst->numRows || pSrcA->numCols != pDst->numCols) {
        return 1;
    }

    for (uint16_t i = 0; i < pSrcA->numRows; i++) {
        for (uint16_t j = 0; j < pSrcA->numCols; j++) {
            pDst->pData[i * pSrcA->numCols + j] = pSrcA->pData[i * pSrcA->numCols + j] + pSrcB->pData[i * pSrcA->numCols + j];
            if (isnan(pDst->pData[i * pSrcA->numCols + j])) {
                ALGO_DEBUG("mat cal add error!\n");
            }
        }
    }

    return 0;
}

// 矩阵相减
uint8_t mat_sub_f32(const mat_f32 *pSrcA, const mat_f32 *pSrcB, mat_f32 *pDst)
{
    if (pSrcA->numRows != pSrcB->numRows || pSrcA->numCols != pSrcB->numCols || pSrcA->numRows != pDst->numRows || pSrcA->numCols != pDst->numCols) {
        return 1;
    }

    for (uint16_t i = 0; i < pSrcA->numRows; i++) {
        for (uint16_t j = 0; j < pSrcA->numCols; j++) {
            pDst->pData[i * pSrcA->numCols + j] = pSrcA->pData[i * pSrcA->numCols + j] - pSrcB->pData[i * pSrcA->numCols + j];
            if (isnan(pDst->pData[i * pSrcA->numCols + j])) {
                ALGO_DEBUG("mat cal sub error!\n");
            }
        }
    }

    return 0;
}

void mat_copy_f32(const mat_f32 *pSrc, mat_f32 *pDst)
{
    for (uint16_t i = 0; i < pSrc->numRows; i++) {
        for (uint16_t j = 0; j < pSrc->numCols; j++) {
            DATA(pDst, i, j) = DATA(pSrc, i, j);
        }
    }
}
// 矩阵相乘
uint8_t mat_mult_f32(const mat_f32 *pSrcA, const mat_f32 *pSrcB, mat_f32 *pDst)
{
    if (pSrcA->numCols != pSrcB->numRows || pSrcA->numRows != pDst->numRows || pSrcB->numCols != pDst->numCols) {
        return 1;
    }

    for (uint16_t i = 0; i < pSrcA->numRows; i++) {
        for (uint16_t j = 0; j < pSrcB->numCols; j++) {
            float sum = 0.0;
            for (uint16_t k = 0; k < pSrcA->numCols; k++) {
                sum += pSrcA->pData[i * pSrcA->numCols + k] * pSrcB->pData[k * pSrcB->numCols + j];
                if (isnan(sum)) {
                    ALGO_DEBUG("mat cal mult error!\n");
                    ALGO_DEBUG("pSrcA, row:%d, col:%d, data:\n", pSrcA->numRows, pSrcA->numCols);
                    for (uint16_t i = 0; i < pSrcA->numRows; i++) {
                        for (uint16_t j = 0; j < pSrcA->numCols; j++) {
                            ALGO_DEBUG("%f,", pSrcA->pData[i * pSrcA->numCols + j]);
                        }
                        ALGO_DEBUG("\n");
                    }
                    ALGO_DEBUG("pSrcB, row:%d, col:%d, data:\n", pSrcB->numRows, pSrcB->numCols);
                    for (uint16_t i = 0; i < pSrcB->numRows; i++) {
                        for (uint16_t j = 0; j < pSrcB->numCols; j++) {
                            ALGO_DEBUG("%f,", pSrcB->pData[i * pSrcB->numCols + j]);
                        }
                        ALGO_DEBUG("\n");
                    }
                    ALGO_DEBUG("pDst, row:%d, col:%d, data:\n", pDst->numRows, pDst->numCols);
                    for (uint16_t i = 0; i < pDst->numRows; i++) {
                        for (uint16_t j = 0; j < pDst->numCols; j++) {
                            ALGO_DEBUG("%f,", pDst->pData[i * pDst->numCols + j]);
                        }
                        ALGO_DEBUG("\n");
                    }
                }
            }
            pDst->pData[i * pSrcB->numCols + j] = sum;
        }
    }

    return 0;
}

// 矩阵缩放
void mat_scale_f32(const mat_f32 *pSrc, float scale, mat_f32 *pDst)
{
    for (uint16_t i = 0; i < pSrc->numRows * pSrc->numCols; i++) {
        pDst->pData[i] = pSrc->pData[i] * scale;
        if (isnan(pDst->pData[i])) {
            ALGO_DEBUG("mat cal scale error!\n");
        }
    }
}

// 矩阵转置
void mat_trans_f32(const mat_f32 *pSrc, mat_f32 *pDst)
{
    for (uint16_t i = 0; i < pSrc->numRows; i++) {
        for (uint16_t j = 0; j < pSrc->numCols; j++) {
            pDst->pData[j * pSrc->numRows + i] = pSrc->pData[i * pSrc->numCols + j];
            if (isnan(pDst->pData[j * pSrc->numRows + i])) {
                ALGO_DEBUG("mat cal trans error!\n");
            }
        }
    }
}

void mat_power_f32(const mat_f32 *pSrc, mat_f32 *pDst)
{
    if (pSrc->numRows != pDst->numRows || pSrc->numCols != pDst->numCols) {
        // 返回错误码或进行适当处理
        return;
    }

    for (uint16_t i = 0; i < pSrc->numRows; i++) {
        for (uint16_t j = 0; j < pSrc->numCols; j++) {
            pDst->pData[i * pSrc->numCols + j] = pSrc->pData[i * pSrc->numCols + j] * pSrc->pData[i * pSrc->numCols + j];
        }
    }
}

void mat_sqrt_f32(const mat_f32 *pSrc, mat_f32 *pDst)
{
    if (pSrc->numRows != pDst->numRows || pSrc->numCols != pDst->numCols) {
        // 返回错误码或进行适当处理
        return;
    }

    for (uint16_t i = 0; i < pSrc->numRows; i++) {
        for (uint16_t j = 0; j < pSrc->numCols; j++) {
            pDst->pData[i * pSrc->numCols + j] = sqrtf(pSrc->pData[i * pSrc->numCols + j]);
        }
    }
}

void mat_norm_f32(const mat_f32 *pSrc, mat_f32 *pDst)
{
    if (pSrc->numRows != pDst->numRows || pSrc->numCols != pDst->numCols) {
        // 返回错误码或进行适当处理
        return;
    }

    float sum = 0.0;

    for (uint16_t i = 0; i < pSrc->numRows; i++) {
        for (uint16_t j = 0; j < pSrc->numCols; j++) {
            sum += pSrc->pData[i * pSrc->numCols + j] * pSrc->pData[i * pSrc->numCols + j];
        }
    }

    for (uint16_t i = 0; i < pSrc->numRows; i++) {
        for (uint16_t j = 0; j < pSrc->numCols; j++) {
            pDst->pData[i * pSrc->numCols + j] = pSrc->pData[i * pSrc->numCols + j] / sqrtf(sum);
        }
    }
}

// 交换两行
void _swapRows(mat_f32 *matrix, int row1, int row2)
{
    for (int i = 0; i < matrix->numCols; i++) {
        float temp = matrix->pData[row1 * matrix->numCols + i];
        matrix->pData[row1 * matrix->numCols + i] = matrix->pData[row2 * matrix->numCols + i];
        matrix->pData[row2 * matrix->numCols + i] = temp;
    }
}

// 将矩阵的某一行乘以一个标量
void _scaleRow(mat_f32 *matrix, int row, float scalar)
{
    for (int i = 0; i < matrix->numCols; i++) {
        matrix->pData[row * matrix->numCols + i] *= scalar;
    }
}

// 将矩阵的一行加到另一行上（可以乘以一个标量再相加）
void _addScaledRow(mat_f32 *matrix, int row1, int row2, float scalar)
{
    for (int i = 0; i < matrix->numCols; i++) {
        matrix->pData[row2 * matrix->numCols + i] += scalar * matrix->pData[row1 * matrix->numCols + i];
    }
}

float inv_buffer[9];
// 高斯-约当消元法求逆矩阵
uint8_t mat_inv_f32(mat_f32 *pSrc, mat_f32 *pDst)
{
    mat_f32 tempMat;
    tempMat.numRows = pSrc->numRows;
    tempMat.numCols = pSrc->numCols;
    tempMat.pData = inv_buffer;

    for (int i = 0; i < pSrc->numRows; i++) {
        for (int j = 0; j < pSrc->numCols; j++) {
            DATA(&tempMat, i, j) = DATA(pSrc, i, j);
        }
    }

    // 初始化单位矩阵
    for (int i = 0; i < pSrc->numCols; i++) {
        for (int j = 0; j < pSrc->numRows; j++) {
            pDst->pData[i * pSrc->numCols + j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // 高斯消元
    for (int i = 0; i < tempMat.numCols; i++) {
        float scale = 1.0 / DATA(&tempMat, i, i);
        _scaleRow(&tempMat, i, scale);
        _scaleRow(pDst, i, scale);

        for (int j = 0; j < tempMat.numRows; j++) {
            if (j != i) {
                float factor = -DATA(&tempMat, j, i);
                _addScaledRow(&tempMat, i, j, factor);
                _addScaledRow(pDst, i, j, factor);
            }
        }
    }
    return 0; // 成功
}

// // 示例用法
// int main() {
//     // 假设有两个3x3的矩阵 A 和 B
//     float dataA[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
//     float dataB[] = {9, 0, 0, 0, 9, 0, 0, 0, 9};
//     float dataC[] = {9, 8, 7, 6, 5, 4, 3, 2, 1};
//     mat_f32 A, B, C;

//     mat_init_f32(&A, 3, 3, dataA);
//     mat_init_f32(&B, 3, 3, dataB);
//     mat_init_f32(&C, 3, 3, dataC);
//     // 矩阵相加
//     mat_inv_f32(&B, &C);

//     // 输出结果
//     for (uint16_t i = 0; i < C.numRows; i++) {
//         for (uint16_t j = 0; j < C.numCols; j++) {
//             printf("%f ", B.pData[i * C.numCols + j]);
//         }
//         printf("\n");
//     }

//     return 0;
// }
