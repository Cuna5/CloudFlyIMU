/**
 * @file    bmp280.c
 * @brief   BMP280 温度/气压传感器驱动（I2C3）实现。
 *
 * 总线：I2C3（hi2c3），HAL 接受左移一位的 7-bit 地址。
 * 互斥锁：所有公共 API 在调度器运行时通过 I2C3MutexHandle 加锁；
 *         调度器未启动时跳过加锁（单线程启动阶段）。
 */

#include "hardware.h"      /* Driver_Status, HAL_TIMEOUT_MS, 子模块头文件 */
#include "i2c.h"           /* hi2c1                                       */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>


/* ================================================================== */
/* 模块私有常量                                                        */
/* ================================================================== */

/** 默认 7-bit I2C 地址（SDO 接 GND）。 */
#define BMP280_ADDR7_DEFAULT    0x76u
/** 备用 7-bit I2C 地址（SDO 接 VDDIO）。 */
#define BMP280_ADDR7_ALT        0x77u

/** 寄存器 0xD0 返回的期望 ChipID。 */
#define BMP280_CHIP_ID          0x58u

/** 寄存器地址（数据手册表 18）。 */
#define BMP280_REG_CALIB_T1_LSB 0x88u   /* 24 字节校准块起始地址 */
#define BMP280_REG_ID           0xD0u
#define BMP280_REG_RESET        0xE0u
#define BMP280_REG_CTRL_MEAS    0xF4u
#define BMP280_REG_CONFIG       0xF5u
#define BMP280_REG_PRES_MSB     0xF7u   /* 0xF7..0xF9 存放 20-bit 气压原始值 */
#define BMP280_REG_TEMP_MSB     0xFAu   /* 0xFA..0xFC 存放 20-bit 温度原始值 */

#define BMP280_RESET_KEY        0xB6u
#define BMP280_CTRL_MEAS_VALUE  0x57u   /* osrs_t=×2, osrs_p=×16, mode=Normal */
#define BMP280_CONFIG_VALUE     0x90u   /* t_sb=62.5 ms, filter=16            */

#define BMP280_RESET_WAIT_MS    5u


/* ================================================================== */
/* 模块私有状态                                                        */
/* ================================================================== */

/**
 * @brief BMP280 calibration coefficients parsed from registers 0x88..0x9F.
 *
 * Per datasheet table 17, dig_T1 and dig_P1 are unsigned 16-bit; the rest
 * are signed 16-bit. The block is little-endian on the wire.
 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_t;

static uint8_t        s_addr7        = BMP280_ADDR7_DEFAULT;
static bool           s_initialized  = false;
static bmp280_calib_t s_calib;
static int32_t        s_t_fine       = 0;
static bool           s_t_fine_valid = false;


/* ================================================================== */
/* 互斥锁 / 内核辅助函数                                              */
/* ================================================================== */

/* 共享 I2C3 总线互斥锁由 Core/Src/freertos.c 中的 MX_FREERTOS_Init 创建 */
extern osMutexId_t I2C3MutexHandle;

static inline bool bmp280_scheduler_running(void)
{
    return osKernelGetState() == osKernelRunning;
}

/**
 * @brief 调度器运行时获取 I2C 总线互斥锁。
 *
 * @return DRV_OK 已加锁或调度器未运行；DRV_ERR_TIMEOUT 超时。
 */
static Driver_Status bmp280_lock(void)
{
    if (!bmp280_scheduler_running()) {
        return DRV_OK;
    }
    if (osMutexAcquire(I2C3MutexHandle, HAL_TIMEOUT_MS) != osOK) {
        return DRV_ERR_TIMEOUT;
    }
    return DRV_OK;
}

static void bmp280_unlock(void)
{
    if (bmp280_scheduler_running()) {
        (void)osMutexRelease(I2C3MutexHandle);
    }
}

#if defined(DWT) && defined(CoreDebug) && \
    defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
static void bmp280_busy_delay_ms(uint32_t ms)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t cycles_per_ms = SystemCoreClock / 1000u;
    if (cycles_per_ms == 0u) cycles_per_ms = 1u;
    while (ms-- > 0u) {
        uint32_t start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < cycles_per_ms) { __NOP(); }
    }
}
#define BMP280_HAS_DWT_DELAY 1
#endif

static void bmp280_delay_ms(uint32_t ms)
{
    if (bmp280_scheduler_running()) {
        (void)osDelay(ms);
    } else {
#if defined(BMP280_HAS_DWT_DELAY)
        bmp280_busy_delay_ms(ms);
#else
        HAL_Delay(ms);
#endif
    }
}


/* ================================================================== */
/* 底层 I2C 封装（调用方已持有总线互斥锁）                            */
/* ================================================================== */

static inline uint16_t bmp280_dev_addr(void)
{
    /* HAL 需要 7-bit 地址左移一位（R/W 标志位位置） */
    return (uint16_t)((uint16_t)s_addr7 << 1);
}

static Driver_Status bmp280_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hs = HAL_I2C_Mem_Read(&hi2c3, bmp280_dev_addr(),
                                            reg, I2C_MEMADD_SIZE_8BIT,
                                            buf, len, HAL_TIMEOUT_MS);
    return Driver_MapHalStatus(hs);
}

static Driver_Status bmp280_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t v = value;
    HAL_StatusTypeDef hs = HAL_I2C_Mem_Write(&hi2c3, bmp280_dev_addr(),
                                             reg, I2C_MEMADD_SIZE_8BIT,
                                             &v, 1u, HAL_TIMEOUT_MS);
    return Driver_MapHalStatus(hs);
}


/* ================================================================== */
/* 校准块解析                                                          */
/* ================================================================== */

/**
 * @brief 将寄存器 0x88..0x9F 的 24 字节原始数据解析到 s_calib。
 *
 * 数据为小端格式；有符号/无符号宽度遵循数据手册表 17。
 */
static void bmp280_parse_calibration(const uint8_t *raw)
{
    s_calib.dig_T1 = (uint16_t)(raw[0]  | ((uint16_t)raw[1]  << 8));
    s_calib.dig_T2 = (int16_t) (raw[2]  | ((uint16_t)raw[3]  << 8));
    s_calib.dig_T3 = (int16_t) (raw[4]  | ((uint16_t)raw[5]  << 8));
    s_calib.dig_P1 = (uint16_t)(raw[6]  | ((uint16_t)raw[7]  << 8));
    s_calib.dig_P2 = (int16_t) (raw[8]  | ((uint16_t)raw[9]  << 8));
    s_calib.dig_P3 = (int16_t) (raw[10] | ((uint16_t)raw[11] << 8));
    s_calib.dig_P4 = (int16_t) (raw[12] | ((uint16_t)raw[13] << 8));
    s_calib.dig_P5 = (int16_t) (raw[14] | ((uint16_t)raw[15] << 8));
    s_calib.dig_P6 = (int16_t) (raw[16] | ((uint16_t)raw[17] << 8));
    s_calib.dig_P7 = (int16_t) (raw[18] | ((uint16_t)raw[19] << 8));
    s_calib.dig_P8 = (int16_t) (raw[20] | ((uint16_t)raw[21] << 8));
    s_calib.dig_P9 = (int16_t) (raw[22] | ((uint16_t)raw[23] << 8));
}


/* ================================================================== */
/* 补偿公式（BMP280 数据手册 3.11.3 节）                              */
/* ================================================================== */

/**
 * @brief 从 3 字节寄存器读取中组装 20-bit 无符号 ADC 值。
 *
 * 布局：[MSB] [LSB] [XLSB]，20-bit 值占 XLSB 的高 4 位。
 */
static uint32_t bmp280_pack_adc20(const uint8_t raw[3])
{
    return ((uint32_t)raw[0] << 12) |
           ((uint32_t)raw[1] << 4)  |
           ((uint32_t)raw[2] >> 4);
}

/**
 * @brief 应用整数温度补偿公式并更新 t_fine。
 *
 * @return 温度，单位 0.01 ℃（如 5123 表示 51.23 ℃）。
 */
static int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) *
                    ((int32_t)s_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) *
                    ((int32_t)s_calib.dig_T3)) >> 14;
    s_t_fine = var1 + var2;
    s_t_fine_valid = true;
    return (s_t_fine * 5 + 128) >> 8;
}

/**
 * @brief 应用 int64 气压补偿公式。
 *
 * @return 气压，Q24.8 定点 Pa（即 value / 256 = Pa）。
 *         校准数据导致除零时返回 0。
 */
static uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
    int64_t var1 = ((int64_t)s_t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)s_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)s_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)s_calib.dig_P2) << 12);
    var1 = ((((int64_t)1 << 47) + var1)) * ((int64_t)s_calib.dig_P1) >> 33;
    if (var1 == 0) {
        return 0u;
    }
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);
    return (uint32_t)p;
}


/* ================================================================== */
/* 公共 API                                                            */
/* ================================================================== */

Driver_Status BMP280_SetAddress(uint8_t addr7)
{
    /* Init 成功后地址锁定，防止运行中更改总线地址 */
    if (s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if (addr7 != BMP280_ADDR7_DEFAULT && addr7 != BMP280_ADDR7_ALT) {
        return DRV_ERR_PARAM;
    }
    s_addr7 = addr7;
    return DRV_OK;
}

Driver_Status BMP280_Init(void)
{
    Driver_Status st;

    st = bmp280_lock();
    if (st != DRV_OK) {
        return st;
    }

    /* 1. ChipID 检查 */
    uint8_t id = 0;
    st = bmp280_read_regs(BMP280_REG_ID, &id, 1u);
    if (st != DRV_OK) {
        goto fail;
    }
    if (id != BMP280_CHIP_ID) {
        st = DRV_ERR_ID;
        goto fail;
    }

    /* 2. 软复位 + 等待 ≥ 5 ms */
    st = bmp280_write_reg(BMP280_REG_RESET, BMP280_RESET_KEY);
    if (st != DRV_OK) {
        goto fail;
    }
    bmp280_delay_ms(BMP280_RESET_WAIT_MS);

    /* 3. 突发读取 24 字节校准块 */
    uint8_t calib_raw[24];
    st = bmp280_read_regs(BMP280_REG_CALIB_T1_LSB, calib_raw, sizeof(calib_raw));
    if (st != DRV_OK) {
        goto fail;
    }
    bmp280_parse_calibration(calib_raw);

    /* 4. 配置 ctrl_meas + config */
    st = bmp280_write_reg(BMP280_REG_CTRL_MEAS, BMP280_CTRL_MEAS_VALUE);
    if (st != DRV_OK) {
        goto fail;
    }
    st = bmp280_write_reg(BMP280_REG_CONFIG, BMP280_CONFIG_VALUE);
    if (st != DRV_OK) {
        goto fail;
    }

    /* 初始化成功——重置 t_fine 缓存，首次 ReadPressure 需先读温度 */
    s_t_fine_valid = false;
    s_t_fine       = 0;
    s_initialized  = true;

    bmp280_unlock();
    return DRV_OK;

fail:
    bmp280_unlock();
    return st;
}

Driver_Status BMP280_GetChipID(uint8_t *id)
{
    if (id == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = bmp280_lock();
    if (st != DRV_OK) {
        return st;
    }
    st = bmp280_read_regs(BMP280_REG_ID, id, 1u);
    bmp280_unlock();
    return st;
}

/**
 * @brief 内部温度读取；调用方已持有总线互斥锁。
 */
static Driver_Status bmp280_read_temperature_locked(float *temp_c)
{
    uint8_t raw[3];
    Driver_Status st = bmp280_read_regs(BMP280_REG_TEMP_MSB, raw, sizeof(raw));
    if (st != DRV_OK) {
        return st;
    }
    int32_t adc_T = (int32_t)bmp280_pack_adc20(raw);
    int32_t T_x100 = bmp280_compensate_T_int32(adc_T);
    *temp_c = (float)T_x100 / 100.0f;
    return DRV_OK;
}

Driver_Status BMP280_ReadTemperature(float *temp_c)
{
    if (temp_c == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = bmp280_lock();
    if (st != DRV_OK) {
        return st;
    }
    st = bmp280_read_temperature_locked(temp_c);
    bmp280_unlock();
    return st;
}

Driver_Status BMP280_ReadPressure(float *pressure_pa)
{
    if (pressure_pa == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = bmp280_lock();
    if (st != DRV_OK) {
        return st;
    }

    /* 气压补偿依赖 t_fine；若初始化后尚未读取温度，先内部触发一次以获得有效结果 */
    if (!s_t_fine_valid) {
        float dummy_temp;
        st = bmp280_read_temperature_locked(&dummy_temp);
        if (st != DRV_OK) {
            bmp280_unlock();
            return st;
        }
    }

    uint8_t raw[3];
    st = bmp280_read_regs(BMP280_REG_PRES_MSB, raw, sizeof(raw));
    if (st != DRV_OK) {
        bmp280_unlock();
        return st;
    }

    int32_t adc_P = (int32_t)bmp280_pack_adc20(raw);
    uint32_t p_q248 = bmp280_compensate_P_int64(adc_P);
    *pressure_pa = (float)p_q248 / 256.0f;

    bmp280_unlock();
    return DRV_OK;
}
