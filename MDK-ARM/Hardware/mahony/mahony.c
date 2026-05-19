#include "mahony.h"
#include <math.h>
#include <string.h>

/* 快速平方根倒数（Quake III 算法） */
static float inv_sqrt(float x)
{
    float y = x;
    long  i = *(long *)&y;
    i = 0x5F3759DFL - (i >> 1);
    y = *(float *)&i;
    return y * (1.5f - 0.5f * x * y * y);
}

void Mahony_Init(Mahony_t *m, float kp, float ki)
{
    memset(m, 0, sizeof(*m));
    m->q0 = 1.0f;
    m->kp = kp;
    m->ki = ki;
}

void Mahony_Update(Mahony_t *m,
                   float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz,
                   float mag_rel, float dt)
{
    float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    /* --- 加速度计修正 Roll/Pitch ---------------------------------------- */
    float norm = inv_sqrt(ax*ax + ay*ay + az*az);
    if (norm > 0.0f) {
        ax *= norm; ay *= norm; az *= norm;

        /* 由四元数估计重力方向（机体系） */
        float vx = 2.0f*(q1*q3 - q0*q2);
        float vy = 2.0f*(q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        /* 叉积误差 */
        ex += ay*vz - az*vy;
        ey += az*vx - ax*vz;
        ez += ax*vy - ay*vx;
    }

    /* --- 磁力计修正 Yaw（按可靠性加权）---------------------------------- */
    if (mag_rel > 0.0f) {
        norm = inv_sqrt(mx*mx + my*my + mz*mz);
        if (norm > 0.0f) {
            mx *= norm; my *= norm; mz *= norm;

            /* 将磁场旋转到导航系，取水平分量 */
            float hx = 2.0f*(mx*(0.5f - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2));
            float hy = 2.0f*(mx*(q1*q2 + q0*q3) + my*(0.5f - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));
            float bx = sqrtf(hx*hx + hy*hy);
            float bz = 2.0f*(mx*(q1*q3 - q0*q2) + my*(q2*q3 + q0*q1) + mz*(0.5f - q1*q1 - q2*q2));

            /* 由四元数估计磁场方向（机体系） */
            float wx = 2.0f*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2));
            float wy = 2.0f*(bx*(q1*q2 - q0*q3)        + bz*(q0*q1 + q2*q3));
            float wz = 2.0f*(bx*(q0*q2 + q1*q3)        + bz*(0.5f - q1*q1 - q2*q2));

            ex += mag_rel * (my*wz - mz*wy);
            ey += mag_rel * (mz*wx - mx*wz);
            ez += mag_rel * (mx*wy - my*wx);
        }
    }

    /* --- 积分项 ---------------------------------------------------------- */
    m->ix += m->ki * ex * dt;
    m->iy += m->ki * ey * dt;
    m->iz += m->ki * ez * dt;

    /* --- 陀螺仪 + 误差修正 ----------------------------------------------- */
    gx += m->kp * ex + m->ix;
    gy += m->kp * ey + m->iy;
    gz += m->kp * ez + m->iz;

    /* --- 四元数积分 ------------------------------------------------------- */
    float dq0 = 0.5f*(-q1*gx - q2*gy - q3*gz) * dt;
    float dq1 = 0.5f*( q0*gx + q2*gz - q3*gy) * dt;
    float dq2 = 0.5f*( q0*gy - q1*gz + q3*gx) * dt;
    float dq3 = 0.5f*( q0*gz + q1*gy - q2*gx) * dt;

    q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

    /* --- 归一化 ---------------------------------------------------------- */
    norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    m->q0 = q0 * norm;
    m->q1 = q1 * norm;
    m->q2 = q2 * norm;
    m->q3 = q3 * norm;
}

void Mahony_GetEuler(const Mahony_t *m, float *roll, float *pitch, float *yaw)
{
    float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;
    if (roll)  *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    if (pitch) *pitch = asinf( 2.0f*(q0*q2 - q3*q1));
    if (yaw)   *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}
