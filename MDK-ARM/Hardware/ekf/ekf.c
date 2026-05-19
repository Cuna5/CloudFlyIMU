#include "ekf.h"
#include <math.h>
#include <string.h>

static float inv_sqrt(float x)
{
    float y = x;
    long  i = *(long *)&y;
    i = 0x5F3759DFL - (i >> 1);
    y = *(float *)&i;
    return y * (1.5f - 0.5f * x * y * y);
}

/* P = A*P*A^T + Q */
static void cov_predict(float P[][EKF_N], const float A[][EKF_N],
                        const float Q[][EKF_N])
{
    float AP[EKF_N][EKF_N], tmp[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++) {
            AP[i][j] = 0.0f;
            for (int k = 0; k < EKF_N; k++)
                AP[i][j] += A[i][k] * P[k][j];
        }
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++) {
            tmp[i][j] = Q[i][j];
            for (int k = 0; k < EKF_N; k++)
                tmp[i][j] += AP[i][k] * A[j][k];
        }
    memcpy(P, tmp, sizeof(tmp));
}

/* 标量卡尔曼更新：h 为 1×N 观测行向量，z 为残差，r 为观测噪声方差 */
static void scalar_update(EKF_t *ekf, const float h[EKF_N], float z, float r)
{
    float Ph[EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        Ph[i] = 0.0f;
        for (int j = 0; j < EKF_N; j++)
            Ph[i] += ekf->P[i][j] * h[j];
    }
    float S = r;
    for (int j = 0; j < EKF_N; j++) S += h[j] * Ph[j];
    if (S < 1e-9f) return;

    float K[EKF_N];
    for (int i = 0; i < EKF_N; i++) K[i] = Ph[i] / S;

    for (int i = 0; i < EKF_N; i++) ekf->x[i] += K[i] * z;

    /* P = (I - K*h)*P，利用已算好的 Ph */
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            ekf->P[i][j] -= K[i] * Ph[j];

    float n = inv_sqrt(ekf->x[0]*ekf->x[0] + ekf->x[1]*ekf->x[1] +
                       ekf->x[2]*ekf->x[2] + ekf->x[3]*ekf->x[3]);
    ekf->x[0] *= n; ekf->x[1] *= n;
    ekf->x[2] *= n; ekf->x[3] *= n;
}

/* ------------------------------------------------------------------ */

void EKF_Init(EKF_t *ekf,
              float q_gyro, float q_bias,
              float r_accel, float r_mag)
{
    memset(ekf, 0, sizeof(*ekf));
    ekf->x[0] = 1.0f;
    for (int i = 0; i < 4; i++) ekf->P[i][i] = 0.01f;
    for (int i = 4; i < 7; i++) ekf->P[i][i] = 0.1f;
    ekf->q_gyro  = q_gyro;
    ekf->q_bias  = q_bias;
    ekf->r_accel = r_accel;
    ekf->r_mag   = r_mag;
}

void EKF_Predict(EKF_t *ekf,
                 float gx, float gy, float gz,
                 float dt)
{
    float q0 = ekf->x[0], q1 = ekf->x[1];
    float q2 = ekf->x[2], q3 = ekf->x[3];
    float bgx = ekf->x[4], bgy = ekf->x[5], bgz = ekf->x[6];

    float wx = gx - bgx;
    float wy = gy - bgy;
    float wz = gz - bgz;

    float h = 0.5f * dt;
    ekf->x[0] = q0 + h*(-q1*wx - q2*wy - q3*wz);
    ekf->x[1] = q1 + h*( q0*wx + q2*wz - q3*wy);
    ekf->x[2] = q2 + h*( q0*wy - q1*wz + q3*wx);
    ekf->x[3] = q3 + h*( q0*wz + q1*wy - q2*wx);

    float n = inv_sqrt(ekf->x[0]*ekf->x[0] + ekf->x[1]*ekf->x[1] +
                       ekf->x[2]*ekf->x[2] + ekf->x[3]*ekf->x[3]);
    ekf->x[0] *= n; ekf->x[1] *= n;
    ekf->x[2] *= n; ekf->x[3] *= n;

    /* 状态转移矩阵 F = I + dF*dt */
    float F[EKF_N][EKF_N];
    memset(F, 0, sizeof(F));
    for (int i = 0; i < EKF_N; i++) F[i][i] = 1.0f;

    /* ∂(dq)/∂q */
    F[0][1] = -h*wx; F[0][2] = -h*wy; F[0][3] = -h*wz;
    F[1][0] =  h*wx; F[1][2] =  h*wz; F[1][3] = -h*wy;
    F[2][0] =  h*wy; F[2][1] = -h*wz; F[2][3] =  h*wx;
    F[3][0] =  h*wz; F[3][1] =  h*wy; F[3][2] = -h*wx;

    /* ∂(dq)/∂bg（符号与 ∂(dq)/∂w 相反） */
    F[0][4] =  h*q1; F[0][5] =  h*q2; F[0][6] =  h*q3;
    F[1][4] = -h*q0; F[1][5] =  h*q3; F[1][6] = -h*q2;
    F[2][4] = -h*q3; F[2][5] = -h*q0; F[2][6] =  h*q1;
    F[3][4] =  h*q2; F[3][5] = -h*q1; F[3][6] = -h*q0;

    float Q[EKF_N][EKF_N];
    memset(Q, 0, sizeof(Q));
    float sq = ekf->q_gyro * ekf->q_gyro * dt;
    float sb = ekf->q_bias * ekf->q_bias * dt;
    for (int i = 0; i < 4; i++) Q[i][i] = sq;
    for (int i = 4; i < 7; i++) Q[i][i] = sb;

    cov_predict(ekf->P, F, Q);
}

void EKF_Update(EKF_t *ekf,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float mag_rel)
{
    float q0 = ekf->x[0], q1 = ekf->x[1];
    float q2 = ekf->x[2], q3 = ekf->x[3];

    /* ---- 加速度计更新（Roll/Pitch，3 维，逐轴标量更新）------------- */
    float an = inv_sqrt(ax*ax + ay*ay + az*az);
    if (an > 0.0f && an < 1e6f) {
        ax *= an; ay *= an; az *= an;

        /* 由四元数预测的重力方向（机体系） */
        float gx_hat = 2.0f*(q1*q3 - q0*q2);
        float gy_hat = 2.0f*(q0*q1 + q2*q3);
        float gz_hat = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        /* 残差 */
        float zx = ax - gx_hat;
        float zy = ay - gy_hat;
        float zz = az - gz_hat;

        float ra = ekf->r_accel * ekf->r_accel;

        /* ∂gx_hat/∂q */
        float hx[EKF_N] = {-2.0f*q2,  2.0f*q3, -2.0f*q0,  2.0f*q1, 0,0,0};
        float hy[EKF_N] = { 2.0f*q1,  2.0f*q0,  2.0f*q3,  2.0f*q2, 0,0,0};
        float hz[EKF_N] = { 2.0f*q0, -2.0f*q1, -2.0f*q2,  2.0f*q3, 0,0,0};

        scalar_update(ekf, hx, zx, ra);
        /* 更新后重新读取四元数 */
        q0=ekf->x[0]; q1=ekf->x[1]; q2=ekf->x[2]; q3=ekf->x[3];
        hy[0]= 2.0f*q1; hy[1]= 2.0f*q0; hy[2]= 2.0f*q3; hy[3]= 2.0f*q2;
        scalar_update(ekf, hy, zy, ra);
        q0=ekf->x[0]; q1=ekf->x[1]; q2=ekf->x[2]; q3=ekf->x[3];
        hz[0]= 2.0f*q0; hz[1]=-2.0f*q1; hz[2]=-2.0f*q2; hz[3]= 2.0f*q3;
        scalar_update(ekf, hz, zz, ra);
        q0=ekf->x[0]; q1=ekf->x[1]; q2=ekf->x[2]; q3=ekf->x[3];
    }

    /* ---- 磁力计更新（Yaw，1 维）------------------------------------ */
    if (mag_rel > 0.0f) {
        float mn = inv_sqrt(mx*mx + my*my + mz*mz);
        if (mn > 0.0f && mn < 1e6f) {
            mx *= mn; my *= mn; mz *= mn;

            /* 将磁场旋转到导航系，取水平分量 bx */
            float hx_n = 2.0f*(mx*(0.5f - q2*q2 - q3*q3) +
                               my*(q1*q2 - q0*q3) +
                               mz*(q1*q3 + q0*q2));
            float hy_n = 2.0f*(mx*(q1*q2 + q0*q3) +
                               my*(0.5f - q1*q1 - q3*q3) +
                               mz*(q2*q3 - q0*q1));
            float bx = sqrtf(hx_n*hx_n + hy_n*hy_n);
            float bz = 2.0f*(mx*(q1*q3 - q0*q2) +
                             my*(q2*q3 + q0*q1) +
                             mz*(0.5f - q1*q1 - q2*q2));

            /* 预测的磁场 x 分量（机体系） */
            float mx_hat = 2.0f*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2));
            float zm = mx - mx_hat;

            /* ∂mx_hat/∂q */
            float hm[EKF_N];
            hm[0] = -2.0f*bz*q2;
            hm[1] =  2.0f*bz*q3;
            hm[2] = -4.0f*bx*q2 - 2.0f*bz*q0;
            hm[3] = -4.0f*bx*q3 + 2.0f*bz*q1;
            hm[4] = hm[5] = hm[6] = 0.0f;

            float rm = ekf->r_mag * ekf->r_mag / (mag_rel + 1e-6f);
            scalar_update(ekf, hm, zm, rm);
        }
    }
}

void EKF_GetEuler(const EKF_t *ekf,
                  float *roll, float *pitch, float *yaw)
{
    float q0=ekf->x[0], q1=ekf->x[1], q2=ekf->x[2], q3=ekf->x[3];
    if (roll)  *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    if (pitch) *pitch = asinf( 2.0f*(q0*q2 - q3*q1));
    if (yaw)   *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}

void EKF_GetBias(const EKF_t *ekf,
                 float *bgx, float *bgy, float *bgz)
{
    if (bgx) *bgx = ekf->x[4];
    if (bgy) *bgy = ekf->x[5];
    if (bgz) *bgz = ekf->x[6];
}
