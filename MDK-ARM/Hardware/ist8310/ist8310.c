/**
 * @file    ist8310.c
 * @brief   IST8310 磁力计驱动（I2C1，7-bit 地址 0x0E）实现。
 *
 * I2C 帧格式：
 *   - 从机 7-bit 地址      = 0x0E
 *   - HAL_I2C_Mem_* devAddress = 0x0E << 1 = 0x1C
 *   - 内存地址大小          = 1 字节（I2C_MEMADD_SIZE_8BIT）
 *
 * 所有 I2C 传输在调度器运行时通过 I2CMutexHandle 加锁（与 bmp280.c 共享 I2C1）；
 * 调度器未启动时跳过加锁，直接调用 HAL API。
 * HAL 返回码通过 Driver_MapHalStatus 映射为统一的 Driver_Status。
 */
/* 先包含 hardware.h：它拥有 Driver_Status 枚举和 HAL_TIMEOUT_MS，
 * 并通过聚合门面重新导出 ist8310.h。 */
#include "hardware.h"   /* Driver_Status, HAL_TIMEOUT_MS, Driver_MapHalStatus */
#include "i2c.h"        /* extern hi2c1 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* 共享 I2C 总线互斥锁（由 CubeMX 在 Core/Src/freertos.c 中创建）     */
/* ------------------------------------------------------------------ */
extern osMutexId_t I2CMutexHandle;

/* ------------------------------------------------------------------ */
/* 设备特定常量                                                        */
/* ------------------------------------------------------------------ */

/** 7-bit 从机地址（数据手册 §7.1，CAD0/CAD1 接地）。 */
#define IST8310_I2C_ADDR_7BIT       0x0Eu

/** HAL_I2C_* 调用需要左移一位的地址（R/W 标志位）。 */
#define IST8310_I2C_ADDR_HAL        ((uint16_t)(IST8310_I2C_ADDR_7BIT << 1))   /* 0x1C */

/** 期望的 Chip ID（WAI）值——数据手册 §8.1。 */
#define IST8310_CHIP_ID             0x10u

/* 寄存器映射——仅列出本驱动使用的地址 */
#define IST8310_REG_WAI             0x00u   /* Who-Am-I（Chip ID） */
#define IST8310_REG_DATA_X_L        0x03u   /* 6 字节突发读取数据起始地址 */
#define IST8310_REG_CTRL1           0x0Au   /* 控制寄存器 1——写 0x01 启动单次测量 */
#define IST8310_REG_CTRL2           0x0Bu   /* 控制寄存器 2——DRDY/中断使能 */
#define IST8310_REG_AVG_CTRL        0x41u   /* 平均控制 */
#define IST8310_REG_PD_CTRL         0x42u   /* 脉冲宽度/性能控制 */

/* 初始化和读取时使用的控制值 */
#define IST8310_CTRL2_INIT_VALUE    0xC0u
#define IST8310_AVG_INIT_VALUE      0x09u
#define IST8310_PD_INIT_VALUE       0xC0u
#define IST8310_CTRL1_SINGLE_MEAS   0x01u

/** 单次测量稳定时间（数据手册 §6.1，典型值 6 ms）。 */
#define IST8310_MEAS_DELAY_MS       6u

/** 灵敏度：0.3 µT/LSB（数据手册 §6.1）。 */
#define IST8310_LSB_TO_UT           (0.3f)

/* ------------------------------------------------------------------ */
/* 模块状态                                                            */
/* ------------------------------------------------------------------ */

/** IST8310_Init 成功完成后置 true，用于门控所有读取操作。 */
static bool s_initialized = false;

/* ------------------------------------------------------------------ */
/* 内部辅助函数                                                        */
/* ------------------------------------------------------------------ */

static inline bool ist8310_scheduler_running(void)
{
    return osKernelGetState() == osKernelRunning;
}

/**
 * @brief 调度器运行时获取共享 I2C 总线互斥锁。
 *
 * @return DRV_OK 已加锁或调度器未运行；DRV_ERR_TIMEOUT 超时。
 */
static Driver_Status ist8310_bus_lock(void)
{
    if (!ist8310_scheduler_running()) {
        return DRV_OK;
    }
    if (osMutexAcquire(I2CMutexHandle, HAL_TIMEOUT_MS) != osOK) {
        return DRV_ERR_TIMEOUT;
    }
    return DRV_OK;
}

/** 调度器运行时释放 I2C 总线互斥锁；调度器未运行时为空操作。 */
static void ist8310_bus_unlock(void)
{
    if (ist8310_scheduler_running()) {
        (void)osMutexRelease(I2CMutexHandle);
    }
}

/** 从寄存器 reg 开始突发读取 len 字节到 buf。 */
static Driver_Status ist8310_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hs = HAL_I2C_Mem_Read(&hi2c1,
                                            IST8310_I2C_ADDR_HAL,
                                            (uint16_t)reg,
                                            I2C_MEMADD_SIZE_8BIT,
                                            buf,
                                            len,
                                            HAL_TIMEOUT_MS);
    return Driver_MapHalStatus(hs);
}

/** 向寄存器 reg 写入单字节 value。 */
static Driver_Status ist8310_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t payload = value;
    HAL_StatusTypeDef hs = HAL_I2C_Mem_Write(&hi2c1,
                                             IST8310_I2C_ADDR_HAL,
                                             (uint16_t)reg,
                                             I2C_MEMADD_SIZE_8BIT,
                                             &payload,
                                             1u,
                                             HAL_TIMEOUT_MS);
    return Driver_MapHalStatus(hs);
}

/** 调度器感知延时，用于触发和突发读取之间的等待。 */
static void ist8310_delay_ms(uint32_t ms)
{
    if (ist8310_scheduler_running()) {
        (void)osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}

/* ------------------------------------------------------------------ */
/* 公共 API                                                            */
/* ------------------------------------------------------------------ */

Driver_Status IST8310_Init(void)
{
    Driver_Status st = ist8310_bus_lock();
    if (st != DRV_OK) {
        return st;
    }

    uint8_t chip_id = 0;
    st = ist8310_read(IST8310_REG_WAI, &chip_id, 1u);
    if (st != DRV_OK) {
        ist8310_bus_unlock();
        return st;
    }
    if (chip_id != IST8310_CHIP_ID) {
        ist8310_bus_unlock();
        return DRV_ERR_ID;
    }

    st = ist8310_write_reg(IST8310_REG_CTRL2, IST8310_CTRL2_INIT_VALUE);
    if (st != DRV_OK) {
        ist8310_bus_unlock();
        return st;
    }

    st = ist8310_write_reg(IST8310_REG_AVG_CTRL, IST8310_AVG_INIT_VALUE);
    if (st != DRV_OK) {
        ist8310_bus_unlock();
        return st;
    }

    st = ist8310_write_reg(IST8310_REG_PD_CTRL, IST8310_PD_INIT_VALUE);
    if (st != DRV_OK) {
        ist8310_bus_unlock();
        return st;
    }

    s_initialized = true;
    ist8310_bus_unlock();
    return DRV_OK;
}

Driver_Status IST8310_ReadMag(float *mx, float *my, float *mz)
{
    if (mx == NULL || my == NULL || mz == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = ist8310_bus_lock();
    if (st != DRV_OK) {
        return st;
    }

    /* 1) 触发单次转换 */
    st = ist8310_write_reg(IST8310_REG_CTRL1, IST8310_CTRL1_SINGLE_MEAS);
    if (st != DRV_OK) {
        ist8310_bus_unlock();
        return st;
    }

    /* 2) 等待至少 6 ms 转换完成 */
    ist8310_delay_ms(IST8310_MEAS_DELAY_MS);

    /* 3) 突发读取 0x03..0x08 共 6 字节（X、Y、Z，小端格式） */
    uint8_t buf[6] = { 0 };
    st = ist8310_read(IST8310_REG_DATA_X_L, buf, sizeof(buf));
    ist8310_bus_unlock();
    if (st != DRV_OK) {
        return st;
    }

    /* 4) 重建有符号 16-bit 值，然后换算为微特斯拉 */
    int16_t raw_x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    int16_t raw_y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    int16_t raw_z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    *mx = (float)raw_x * IST8310_LSB_TO_UT;
    *my = (float)raw_y * IST8310_LSB_TO_UT;
    *mz = (float)raw_z * IST8310_LSB_TO_UT;
    return DRV_OK;
}

Driver_Status IST8310_GetChipID(uint8_t *id)
{
    if (id == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = ist8310_bus_lock();
    if (st != DRV_OK) {
        return st;
    }

    st = ist8310_read(IST8310_REG_WAI, id, 1u);
    ist8310_bus_unlock();
    return st;
}
