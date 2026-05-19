/**
 * @file  ekf.h
 * @brief 四元数 + 陀螺仪 bias 的 7 维 EKF 姿态融合。
 *
 * 状态向量 x = [q0, q1, q2, q3, bgx, bgy, bgz]^T
 *   - q   : 机体相对导航系的姿态四元数（单位四元数约束）
 *   - bg  : 陀螺仪零偏，rad/s
 *
 * 预测：四元数运动学积分 + bias 随机游走
 * 更新：
 *   - 加速度计观测（Roll/Pitch，3 维残差）
 *   - 磁力计观测（Yaw，1 维残差，按 mag_reliability 加权）
 */
#ifndef EKF_H
#define EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 状态维度 */
#define EKF_N  7

typedef struct {
    float x[EKF_N];          /* 状态向量 */
    float P[EKF_N][EKF_N];   /* 误差协方差矩阵 */

    /* 过程噪声标准差 */
    float q_gyro;    /* 陀螺仪角速度噪声，rad/s/sqrt(Hz) */
    float q_bias;    /* 陀螺仪 bias 随机游走，rad/s^2/sqrt(Hz) */

    /* 观测噪声标准差 */
    float r_accel;   /* 加速度计观测噪声 */
    float r_mag;     /* 磁力计观测噪声 */
} EKF_t;

/**
 * @brief 初始化 EKF，四元数置 [1,0,0,0]，P 置对角阵。
 * @param q_gyro   陀螺仪过程噪声，典型值 0.01
 * @param q_bias   bias 随机游走，典型值 0.001
 * @param r_accel  加速度计观测噪声，典型值 0.1
 * @param r_mag    磁力计观测噪声，典型值 0.3
 */
void EKF_Init(EKF_t *ekf,
              float q_gyro, float q_bias,
              float r_accel, float r_mag);

/**
 * @brief 预测步：用陀螺仪数据积分四元数，传播协方差。
 * @param gx/gy/gz  陀螺仪原始值，rad/s（未减 bias，内部自动减）
 * @param dt        时间步长，秒
 */
void EKF_Predict(EKF_t *ekf,
                 float gx, float gy, float gz,
                 float dt);

/**
 * @brief 更新步：用加速度计修正 Roll/Pitch，用磁力计修正 Yaw。
 * @param ax/ay/az   加速度计，任意单位（内部归一化）
 * @param mx/my/mz   磁力计，任意单位（内部归一化）
 * @param mag_rel    磁力计可靠性 [0,1]；0 时跳过磁力计更新
 */
void EKF_Update(EKF_t *ekf,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float mag_rel);

/**
 * @brief 从当前四元数提取欧拉角（ZYX，弧度）。
 */
void EKF_GetEuler(const EKF_t *ekf,
                  float *roll, float *pitch, float *yaw);

/**
 * @brief 读取当前估计的陀螺仪 bias（rad/s）。
 */
void EKF_GetBias(const EKF_t *ekf,
                 float *bgx, float *bgy, float *bgz);

#ifdef __cplusplus
}
#endif

#endif /* EKF_H */
