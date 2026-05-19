/**
 * @file    mahony.h
 * @brief   Mahony 互补滤波姿态解算（6轴/9轴自适应）。
 *
 * 算法来源：Mahony et al., "Nonlinear Complementary Filters on the Special
 * Orthogonal Group", IEEE TAC 2008.
 *
 * 使用方式：
 *   1. 调用 Mahony_Init 初始化参数；
 *   2. 每个控制周期调用 Mahony_Update（9轴）或 Mahony_Update6（6轴）；
 *   3. 调用 Mahony_GetEuler 获取欧拉角（弧度）。
 *
 * 磁力计可靠性由调用方判断后传入 mag_reliability（0~1）：
 *   - 1.0 → 完全信任磁力计（九轴）
 *   - 0.0 → 完全忽略磁力计（六轴）
 *   - 中间值 → 平滑过渡
 */
#ifndef MAHONY_H
#define MAHONY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float q0, q1, q2, q3;  /* 姿态四元数，初始化为 [1,0,0,0] */
    float ix, iy, iz;       /* 积分误差（Ki 项） */
    float kp;               /* 比例增益 */
    float ki;               /* 积分增益 */
} Mahony_t;

/**
 * @brief 初始化 Mahony 滤波器。
 * @param kp  比例增益，典型值 2.0
 * @param ki  积分增益，典型值 0.005
 */
void Mahony_Init(Mahony_t *m, float kp, float ki);

/**
 * @brief 9轴更新（加速度计 + 陀螺仪 + 磁力计）。
 *
 * @param gx/gy/gz   陀螺仪，rad/s
 * @param ax/ay/az   加速度计，任意单位（内部归一化）
 * @param mx/my/mz   磁力计，任意单位（内部归一化）
 * @param mag_rel    磁力计可靠性 [0,1]，0 退化为六轴
 * @param dt         时间步长，秒
 */
void Mahony_Update(Mahony_t *m,
                   float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz,
                   float mag_rel, float dt);

/**
 * @brief 从四元数提取欧拉角（ZYX 顺序，弧度）。
 * @param roll   绕 X 轴，[-π, π]
 * @param pitch  绕 Y 轴，[-π/2, π/2]
 * @param yaw    绕 Z 轴，[-π, π]
 */
void Mahony_GetEuler(const Mahony_t *m, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* MAHONY_H */
