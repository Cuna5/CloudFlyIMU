# 恒温 AI IMU 姿态测量系统：接线与 FreeRTOS 任务分配总结

## 1. 当前系统方案

本项目暂定为：

> 基于恒温控制、AI 零漂预测与自适应 EKF 的智能 IMU 姿态测量系统

当前硬件组合如下：

| 模块 | 器件 | 作用 |
|---|---|---|
| 主控 | STM32H743IIT6 | 传感器采集、PID 恒温控制、AI 推理、EKF 姿态融合、通信 |
| 六轴 IMU | BMI088 | 加速度、角速度采集 |
| 磁力计 | IST8310 | 九轴融合、Yaw 修正、磁干扰检测 |
| 温度/气压 | BMP280 | 第一版用于温度反馈或环境参考 |
| 加热器 | 10Ω 5W 功率电阻 | 给 IMU 模块加热，实现恒温 |
| MOSFET | YJL3400A | PWM 控制加热电阻 |
| 显示，可选 | OLED | 显示温度、姿态、模式、磁干扰状态 |
| 通信 | USB CDC / UART | 上传数据到上位机，用于记录、训练和调试 |
| 参数存储 | W25Q64 (QSPI) | 存储 PID 参数、EKF 初始值等可持久化配置 |

---

## 2. 总体接线结构

```text
STM32H743IIT6
├── SPI1 → BMI088
│   ├── CS_ACC  → GPIO 输出
│   └── CS_GYRO → GPIO 输出
│
├── I2C1 → IST8310 + BMP280
│
├── TIMx_CHx PWM → YJL3400A Gate
│   └── 控制 10Ω 5W 加热电阻
│
├── QSPI → W25Q64 (8 MB NOR Flash)
│   └── 存储 PID 参数等可持久化配置
│
├── USB CDC / UART → 上位机
│
└── OLED，可选 → I2C / SPI
```

---

## 3. BMI088 接线

BMI088 建议使用 SPI 通信，因为 IMU 数据采样频率较高，SPI 比 I2C 更适合高频稳定采集。

| BMI088 引脚 | STM32H743 连接 | 说明 |
|---|---|---|
| VCC | 3.3V | 传感器供电 |
| GND | GND | 共地 |
| SCK | SPI1_SCK | SPI 时钟 |
| MISO | SPI1_MISO | BMI088 输出，STM32 输入 |
| MOSI | SPI1_MOSI | STM32 输出，BMI088 输入 |
| CS_ACC | GPIO_Output | 加速度计片选 |
| CS_GYRO | GPIO_Output | 陀螺仪片选 |
| INT_ACC | 可不接 / EXTI | 第一版可不接 |
| INT_GYRO | 可不接 / EXTI | 第一版可不接 |

### 注意事项

- BMI088 的加速度计和陀螺仪通常各有一个 CS 引脚。
- 两个 CS 默认都应拉高。
- 读取加速度计时，拉低 `CS_ACC`。
- 读取陀螺仪时，拉低 `CS_GYRO`。
- 第一版建议用任务周期轮询读取，不必一开始使用数据就绪中断。

---

## 4. IST8310 接线

IST8310 使用 I2C 通信，用于磁场数据采集、九轴融合和磁干扰判断。

| IST8310 引脚 | STM32H743 连接 | 说明 |
|---|---|---|
| VCC | 3.3V | 供电 |
| GND | GND | 共地 |
| SCL | I2C1_SCL | I2C 时钟 |
| SDA | I2C1_SDA | I2C 数据 |
| DRDY / INT | 可不接 / EXTI | 第一版可不接 |

### 注意事项

IST8310 对磁干扰比较敏感，安装时要注意：

- 远离加热电阻；
- 远离 MOSFET；
- 远离加热电流线；
- 远离电机、磁铁、铁螺丝；
- 尽量放在恒温仓边缘或外侧；
- 加热线正负线尽量并排走线，减小电流环路磁场。

---

## 5. BMP280 接线

BMP280 使用 I2C，可与 IST8310 共用 I2C1。

| BMP280 引脚 | STM32H743 连接 | 说明 |
|---|---|---|
| VCC | 3.3V | 供电 |
| GND | GND | 共地 |
| SCL | I2C1_SCL | 与 IST8310 共用 |
| SDA | I2C1_SDA | 与 IST8310 共用 |
| SDO | GND 或 VCC | 决定地址，一般为 0x76 或 0x77 |
| CSB | VCC | 使用 I2C 模式 |

### 注意事项

- BMP280 第一版可用于温度反馈或环境温度参考。
- 如果作为恒温反馈，应尽量靠近 BMI088。
- 最好让 BMI088 和 BMP280 处于同一热区，例如共用一块小铜片或铝片。
- BMP280 不是专用高精度温度传感器，后续可替换为 NTC 或 TMP117。

---

## 6. 加热电路接线

### 6.1 推荐加热方案

```text
5V 电源
10Ω 5W 功率电阻
YJL3400A N-MOSFET
STM32 PWM 控制
```

最大加热功率：

```text
P = V² / R = 5² / 10 = 2.5W
```

对一个小型 IMU 恒温仓来说，2.5W 通常已经足够。

---

### 6.2 YJL3400A 低边开关接法

```text
5V+
  ↓
10Ω 5W 加热电阻
  ↓
YJL3400A D 极
YJL3400A S 极
  ↓
GND
```

控制端：

| YJL3400A 引脚 | 连接方式 |
|---|---|
| G 极 | STM32 PWM 引脚，经 100Ω 电阻 |
| G 极 | 通过 10k 下拉电阻接 GND |
| D 极 | 加热电阻下端 |
| S 极 | GND |

### 必须注意

```text
STM32 GND、5V 加热电源 GND、传感器 GND 必须共地。
```

---

### 6.3 加热电路保护

建议软件中加入以下保护：

```c
if (temperature > 55.0f)
{
    heater_pwm = 0;
    heater_fault = 1;
}

if (bmp280_read_failed)
{
    heater_pwm = 0;
    heater_fault = 1;
}
```

建议恒温目标：

| 参数 | 建议值 |
|---|---|
| 目标温度 | 40℃ 或 45℃ |
| 允许波动 | ±0.5℃ 左右 |
| 过温保护 | 55℃ |
| PWM 频率 | 1Hz ~ 10Hz |

---

## 7. 结构布局建议

建议做一个小型恒温 IMU 模块：

```text
┌──────────────────────────┐
│        保温外壳           │
│                          │
│  IST8310 磁力计            │  ← 放边缘，远离加热电流
│                          │
│  BMI088 + BMP280          │  ← 放一起，处于同一热区
│        ↓                 │
│  小铜片 / 铝片均热层        │
│        ↓                 │
│  10Ω 5W 功率电阻           │
│                          │
└──────────────────────────┘

MOSFET、主控板、电源模块放在恒温仓外部。
```

### 布局原则

- BMI088 和 BMP280 尽量靠近；
- 功率电阻不要直接贴 BMI088 芯片；
- 功率电阻和 IMU 之间加铜片或铝片均热；
- IST8310 远离加热电阻和大电流线；
- 加热线正负线并排走，避免形成大电流环路；
- MOSFET 放在恒温仓外，降低热干扰和磁干扰。

---

## 8. CubeMX 外设分配总结

| 功能 | CubeMX 外设 | 说明 |
|---|---|---|
| BMI088 | SPI1 | 主 IMU 采集 |
| BMI088 CS_ACC | GPIO_Output | 加速度计片选 |
| BMI088 CS_GYRO | GPIO_Output | 陀螺仪片选 |
| IST8310 | I2C1 | 磁力计 |
| BMP280 | I2C1 | 温度/气压 |
| 加热 PWM | TIM3_CH1 或其他 TIM 通道 | 控制 YJL3400A |
| OLED，可选 | I2C1 / I2C2 / SPI | 状态显示 |
| 上位机通信 | USB CDC / UART | 上传 CSV 数据 |
| HAL Timebase | TIM6 | 避免和 FreeRTOS SysTick 冲突 |

### 推荐外设配置

```text
SPI1：Full-Duplex Master，Software NSS，8-bit
I2C1：100kHz 起步，稳定后可改 400kHz
TIM PWM：1Hz ~ 10Hz
FreeRTOS：CMSIS-RTOS V2
USB CDC 或 UART：用于数据记录
```

---

## 9. FreeRTOS 任务分配

第一版建议设计 5 个任务：

| 任务 | 周期 | 优先级 | 主要功能 |
|---|---:|---|---|
| sensorTask | 5ms | High | 读取 BMI088、IST8310、BMP280 |
| heatTask | 100ms | Normal | PID 恒温控制 |
| fusionTask | 5ms 或 10ms | High | AI 零漂预测 + EKF 姿态融合 |
| logTask | 20ms 或 50ms | Low | 上传 CSV 数据到上位机 |
| uiTask | 200ms | Low | OLED 显示，可选 |

---

## 10. sensorTask 任务

### 周期

```text
5ms，即 200Hz
```

### 功能

```text
读取 BMI088 加速度 ax ay az
读取 BMI088 角速度 gx gy gz
按较低频率读取 IST8310 磁场 mx my mz
按较低频率读取 BMP280 温度 T 和气压 P
更新时间戳
保存到全局数据结构或队列
```

### 建议采样频率

| 数据 | 建议频率 |
|---|---:|
| BMI088 加速度/角速度 | 200Hz |
| IST8310 磁力计 | 50Hz |
| BMP280 温度/气压 | 10Hz |

### 伪代码

```c
void StartSensorTask(void *argument)
{
    for (;;)
    {
        read_bmi088_accel_gyro();

        if (mag_time_due())
        {
            read_ist8310_mag();
        }

        if (temp_time_due())
        {
            read_bmp280_temp_pressure();
        }

        update_sensor_data();
        osDelay(5);
    }
}
```

---

## 11. heatTask 任务

### 周期

```text
100ms，即 10Hz
```

### 功能

```text
读取当前温度
计算目标温度误差
执行 PID 控制
更新 PWM 占空比
进行过温保护和传感器异常保护
```

### 伪代码

```c
void StartHeatTask(void *argument)
{
    for (;;)
    {
        float T = get_current_temperature();

        if (T > 55.0f || temperature_sensor_error())
        {
            set_heater_pwm(0.0f);
            heater_fault = 1;
        }
        else
        {
            float duty = pid_update(heater_target_temp, T);
            set_heater_pwm(duty);
        }

        osDelay(100);
    }
}
```

---

## 12. fusionTask 任务

### 周期

```text
5ms 或 10ms
```

建议第一版用 10ms，即 100Hz；调通后可以提升到 200Hz。

### 功能

```text
获取最新传感器数据
判断是否静止/低动态
AI 预测陀螺仪零漂 bgx bgy bgz
补偿 gx gy gz
计算磁场可靠性 S_mag
根据 S_mag 调整 EKF 中磁力计权重
执行 EKF 姿态更新
输出 Roll Pitch Yaw
```

### 模式逻辑

```text
磁场正常：接近九轴 EKF
磁场异常：降低磁力计权重，接近六轴 EKF
```

### 伪代码

```c
void StartFusionTask(void *argument)
{
    for (;;)
    {
        SensorData_t sensor = get_latest_sensor_data();

        Bias_t bias_ai = ai_predict_gyro_bias(sensor);

        sensor.gx -= bias_ai.bgx;
        sensor.gy -= bias_ai.bgy;
        sensor.gz -= bias_ai.bgz;

        float mag_score = calc_mag_reliability(sensor.mx, sensor.my, sensor.mz);

        ekf_set_mag_weight(mag_score);
        ekf_update(sensor);

        update_attitude_data();

        osDelay(5);
    }
}
```

---

## 13. logTask 任务

### 周期

```text
20ms 或 50ms
```

### 功能

```text
打包 CSV 数据
通过 USB CDC 或 UART 发送
用于 Python / MATLAB 记录、训练和画图
```

### 推荐 CSV 格式

```text
time,T,pwm,ax,ay,az,gx,gy,gz,mx,my,mz,bgx,bgy,bgz,roll,pitch,yaw,mag_score,ekf_mode
```

### 伪代码

```c
void StartLogTask(void *argument)
{
    for (;;)
    {
        SensorData_t sensor = get_latest_sensor_data();
        AttitudeData_t att = get_latest_attitude_data();

        pack_csv_line(sensor, att);
        send_to_pc();

        osDelay(20);
    }
}
```

---

## 14. uiTask 任务

### 周期

```text
200ms
```

### 功能

```text
OLED 显示当前温度
OLED 显示 PWM 占空比
OLED 显示 Roll / Pitch / Yaw
OLED 显示磁场状态
OLED 显示 EKF 模式
```

### 显示内容示例

```text
Temp: 45.0 C
PWM : 35%
Mode: Adaptive EKF
Mag : Normal
Roll: 1.2
Pitch: -0.4
Yaw: 83.5
```

如果第一版不加 OLED，可以暂时不创建 uiTask。

---

## 15. 数据结构建议

### 15.1 传感器数据结构

```c
typedef struct
{
    uint32_t timestamp_ms;

    float ax, ay, az;
    float gx, gy, gz;

    float mx, my, mz;

    float temperature;
    float pressure;

    float pwm_duty;
} SensorData_t;
```

### 15.2 姿态输出结构

```c
typedef struct
{
    uint32_t timestamp_ms;

    float roll;
    float pitch;
    float yaw;

    float bgx;
    float bgy;
    float bgz;

    float mag_reliability;
    uint8_t ekf_mode;
} AttitudeData_t;
```

---

## 16. 互斥锁建议

| Mutex | 保护对象 | 说明 |
|---|---|---|
| spiMutex | BMI088 SPI 总线 | 防止多个任务同时访问 SPI |
| i2cMutex | IST8310 + BMP280 I2C 总线 | 两个传感器共用 I2C1，必须保护 |
| dataMutex | 全局 SensorData / AttitudeData | 防止读写冲突 |
| logMutex，可选 | 日志发送缓冲区 | 防止 USB/UART 输出冲突 |

### I2C 互斥锁使用示例

```c
osMutexAcquire(i2cMutexHandle, osWaitForever);
read_bmp280();
osMutexRelease(i2cMutexHandle);
```

### SPI 互斥锁使用示例

```c
osMutexAcquire(spiMutexHandle, osWaitForever);
read_bmi088();
osMutexRelease(spiMutexHandle);
```

---

## 17. 任务关系图

```text
sensorTask
  ↓
采集 BMI088 / IST8310 / BMP280 数据
  ↓
SensorData_t

heatTask
  ↓
读取温度 → PID → PWM 加热

fusionTask
  ↓
读取 SensorData_t
  ↓
AI 零漂预测
  ↓
磁干扰判断
  ↓
自适应 EKF
  ↓
AttitudeData_t

logTask
  ↓
上传 CSV 数据到上位机

uiTask
  ↓
OLED 显示系统状态
```

---

## 18. 第一版最小可运行任务

如果希望先快速跑通系统，不建议一开始就把所有算法都加进去。

第一阶段只保留：

```text
sensorTask：采集 BMI088 + IST8310 + BMP280
heatTask：PID 恒温控制
logTask：上传数据
```

第二阶段再加入：

```text
fusionTask：AI 零漂预测 + EKF
uiTask：OLED 显示
```

这样开发风险最低。

---

## 19. 推荐开发顺序

```text
1. 调通 BMI088 SPI 通信，读取芯片 ID
2. 调通 IST8310 I2C 通信，读取磁场数据
3. 调通 BMP280 I2C 通信，读取温度和气压
4. 调通 TIM PWM，控制 YJL3400A 开关加热电阻
5. 实现 heatTask 的 PID 恒温控制
6. 实现 logTask，上传 CSV 数据
7. 采集无恒温/恒温静止零漂数据
8. 在电脑端训练 MLP 零漂预测模型
9. 将 MLP 权重移植到 STM32
10. 实现 fusionTask 中的 AI 零漂补偿
11. 实现六轴 EKF
12. 加入 IST8310，扩展为九轴 EKF
13. 加入磁干扰检测和自适应磁力计权重
14. 完成实验对比和报告图表
```

---

## 20. 最终总结

当前硬件接线和任务分配可以确定为：

```text
STM32H743IIT6
+
BMI088，SPI1
+
IST8310，I2C1
+
BMP280，I2C1
+
10Ω 5W 电阻加热
+
YJL3400A MOSFET PWM 控制
+
FreeRTOS 多任务调度
```

FreeRTOS 任务建议为：

```text
sensorTask：传感器采集
heatTask：恒温 PID 控制
fusionTask：AI 零漂预测 + 自适应 EKF
logTask：数据上传
uiTask：OLED 显示，可选
```

该结构能够支撑项目的三个核心目标：

```text
1. 恒温控制降低 IMU 温漂
2. AI 预测并补偿陀螺仪零漂
3. 自适应 EKF 在磁干扰环境下自动调整六轴/九轴融合权重
```

---

## 21. 可能需要注意的问题

```text
1. 时钟配置需要检查一遍有没有存在什么问题 [√]
2. 框架主要由OPUS4.7搭建，需要检查一下有无冗余的功能 [ ]
3. 目前还没添加AI以及EKF，需要进一步处理相关代码，这里只做基础驱动 [ ]
4. 考虑是否需要DMA的参与，并且需要严格注意各传感器的频率问题 [ ]
5. 检查互斥锁、队列、任务相关内容是否都正确配置了 [ ]
6. 代码总体框架精简优化 [ ]
7. 确定一遍所有任务的频率 [ ]
8. 确定所有东西没有冲突CUBE [ ]

```


---

## 22. 版本说明（V1.4）

```text
1. RGB灯光DEBUG
2. uart1调试
3. 检查时钟配置
```