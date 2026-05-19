# MDK-ARM/Hardware

`MDK-ARM/Hardware` 是 CloudFlyIMU 的基础硬件驱动层目录，负责把 CubeMX 生成的 HAL 外设句柄封装成统一的驱动 API。上层任务建议只包含 `hardware.h`，不要直接包含各子模块头文件。

详细需求与设计见 `.kiro/specs/hardware-base-drivers/`。

## 目录结构

```text
Hardware/
├── common/
│   ├── hardware.h
│   └── hardware.c
├── debug_uart/
│   ├── debug_uart.h
│   └── debug_uart.c
├── bmp280/
│   ├── bmp280.h
│   └── bmp280.c
├── ist8310/
│   ├── ist8310.h
│   └── ist8310.c
├── bmi088/
│   ├── bmi088.h
│   └── bmi088.c
└── heater/
    ├── heater.h
    └── heater.c
```

## common/

### `common/hardware.h`

硬件驱动层的统一入口头文件。

- 定义统一返回码 `Driver_Status`：`DRV_OK`、`DRV_ERR_BUS`、`DRV_ERR_ID`、`DRV_ERR_TIMEOUT`、`DRV_ERR_PARAM`、`DRV_ERR_NOT_INIT`。
- 定义公共参数：`HAL_TIMEOUT_MS`、`HEATER_OVERHEAT_THRESHOLD_C`、`DEBUG_LEVEL_MIN`。
- 定义共享数据结构：`SensorData_t` 和 `AttitudeData_t`。
- 声明共享数据互斥锁 `DataMutexHandle` 以及 `DataMutex_Create()`。
- 声明顶层 API：`Hardware_Init()`、`Hardware_SelfTest()`、`Sensor_SampleOnce()`、`Sensor_GetErrorCounters()`、`Heater_ApplyDuty()`、`SensorData_Set/Get()`、`AttitudeData_Set/Get()`、`Driver_MapHalStatus()`。
- 在文件末尾重新导出 `debug_uart.h`、`bmp280.h`、`ist8310.h`、`bmi088.h`、`heater.h`，让应用层只需要 `#include "hardware.h"`。

### `common/hardware.c`

驱动层聚合实现文件，不直接操作具体传感器寄存器，主要负责编排和共享数据管理。

- `Driver_MapHalStatus()`：将 HAL 返回值映射为 `Driver_Status`。
- `SensorData_Set/Get()`、`AttitudeData_Set/Get()`：通过 `DataMutexHandle` 保护全局传感器数据和姿态数据；调度器未运行时跳过加锁。
- `Hardware_Init()`：按 `DebugUART -> BMP280 -> IST8310 -> BMI088 -> Heater` 顺序初始化子模块；遇到首个错误立即返回，并在调试串口可用时输出错误日志。
- `Hardware_SelfTest()`：读取各传感器 Chip ID 并输出 PASS/FAIL 日志。
- `Sensor_SampleOnce()`：采样 BMI088 加速度/角速度，按节流周期采样 IST8310 和 BMP280，并维护传感器错误计数。
- `Sensor_GetErrorCounters()`：读取 BMI088、IST8310、BMP280 的累计错误计数。
- `Heater_ApplyDuty()`：调用加热驱动设置 PWM 占空比，并把实际占空比写回 `SensorData_t.pwm_duty`。

## debug_uart/

### `debug_uart/debug_uart.h`

USART1 调试串口的公共 API。

- `DebugUART_Init()`：检查 `huart1.Instance == USART1`，创建内部串行化互斥锁，标记调试串口可用。
- `Debug_Log()`：以 `DBG_INFO` 级别输出格式化日志。
- `Debug_Log_Level()`：按 `DBG_ERR`、`DBG_WARN`、`DBG_INFO`、`DBG_DEBUG` 输出带前缀的格式化日志。
- `Debug_GetDroppedCount()`：读取因未初始化、互斥锁失败或 HAL 发送失败而丢弃的日志计数。

### `debug_uart/debug_uart.c`

USART1 调试串口实现。

- 使用 256 字节静态缓冲区和 `vsnprintf` 生成日志文本，超长内容会截断。
- 调度器运行后使用内部 mutex 避免多个任务同时写串口导致日志交错。
- 通过 `fputc()` 和弱定义 `_write()` 重定向 `printf` 到 USART1。
- 发送失败时只累计丢弃计数，不递归调用日志接口，避免错误路径再次触发串口输出。

## bmp280/

### `bmp280/bmp280.h`

BMP280 温度/气压传感器公共 API，使用 I2C1。

- `BMP280_SetAddress()`：初始化前选择 7-bit I2C 地址 `0x76` 或 `0x77`。
- `BMP280_Init()`：校验 Chip ID、软复位、读取 24 字节校准参数，并配置温度/气压采样参数。
- `BMP280_ReadTemperature()`：读取原始温度并按数据手册补偿为摄氏度。
- `BMP280_ReadPressure()`：读取原始气压并按数据手册补偿为 Pa；必要时内部先读取温度以更新 `t_fine`。
- `BMP280_GetChipID()`：读取 `0xD0` Chip ID。

### `bmp280/bmp280.c`

BMP280 驱动实现。

- 默认地址为 `0x76`，HAL 访问时转换为左移一位的设备地址。
- 使用 `I2CMutexHandle` 保护 I2C1 访问；调度器未运行时跳过加锁。
- 解析 `dig_T1..dig_T3`、`dig_P1..dig_P9` 校准参数。
- 使用 BMP280 数据手册的整数/64 位补偿公式计算温度和气压。
- 维护 `s_initialized`、`s_t_fine`、`s_t_fine_valid` 等模块私有状态。

## ist8310/

### `ist8310/ist8310.h`

IST8310 三轴磁力计公共 API，使用 I2C1，7-bit 地址 `0x0E`。

- `IST8310_Init()`：读取 WAI/Chip ID，配置 CTRL2、AVG_CTRL、PD_CTRL。
- `IST8310_ReadMag()`：触发单次测量，等待转换完成后读取 X/Y/Z 磁场数据，单位为微特斯拉。
- `IST8310_GetChipID()`：读取 `0x00` Chip ID。

### `ist8310/ist8310.c`

IST8310 驱动实现。

- 使用 `I2CMutexHandle` 保护 I2C1 访问；调度器未运行时跳过加锁。
- 初始化时校验 Chip ID `0x10`。
- 单次测量流程为写 CTRL1 启动转换、延时约 6 ms、突发读取 6 字节数据。
- 按小端 `int16_t` 重建三轴原始值，并按 `0.3 uT/LSB` 转换为物理量。

## bmi088/

### `bmi088/bmi088.h`

BMI088 六轴 IMU 公共 API，使用 SPI1，PC4 为加速度计 CS，PC5 为陀螺仪 CS。

- `BMI088_Init()`：修正 SPI 数据位宽、软复位加速度计、读取并校验两个 Chip ID，然后配置加速度计和陀螺仪量程/ODR/带宽。
- `BMI088_ReadAccel()`：读取加速度计 X/Y/Z，输出单位为 `m/s^2`。
- `BMI088_ReadGyro()`：读取陀螺仪 X/Y/Z，输出单位为 `rad/s`。
- `BMI088_GetChipID()`：读取加速度计和陀螺仪 Chip ID。

### `bmi088/bmi088.c`

BMI088 驱动实现。

- 使用 `SPIMutexHandle` 保护 SPI1 访问；调度器未运行时跳过加锁。
- 在初始化阶段将 CubeMX 生成的 `SPI_DATASIZE_4BIT` 修正为 `SPI_DATASIZE_8BIT` 并重新初始化 SPI1。
- 正确处理加速度计 SPI 读操作的额外 dummy byte，以及陀螺仪读操作无 dummy byte 的差异。
- 通过 `BMI_ACC_CS_Pin` / `BMI_GYRO_CS_Pin` 控制双片选，所有退出路径都会释放片选。
- 加速度使用 `5460 LSB/g` 转换为 `m/s^2`，陀螺仪使用 `16.384 LSB/(deg/s)` 转换为 `rad/s`。

## heater/

### `heater/heater.h`

加热 PWM 驱动公共 API，使用 TIM3_CH1/PC6。

- `Heater_Init()`：初始化 PWM 输出，并在频率不在预期范围时修正为约 1 Hz。
- `Heater_SetDuty()`：设置 0.0 到 1.0 的 PWM 占空比；越界值会夹紧，NaN 返回 `DRV_ERR_PARAM`。
- `Heater_GetDuty()`：读取最近一次成功应用的占空比。
- `Heater_EmergencyStop()`：立即将 PWM 比较值置 0，并锁存故障。
- `Heater_ClearFault()`：清除故障锁存；不会自动恢复加热输出。
- `Heater_OverheatCheck()`：温度达到 `HEATER_OVERHEAT_THRESHOLD_C` 时触发急停。
- `Heater_IsFaultLatched()`：查询故障锁存状态。

### `heater/heater.c`

加热驱动实现。

- 依据 `htim3.Init.Prescaler` 和 `htim3.Init.Period` 估算 PWM 频率。
- 当频率不在 `[0.9, 1.1] Hz` 时，强制设置 `PSC = 24000-1`、`ARR = 10000-1`。
- `Heater_SetDuty()` 根据 `ARR + 1` 计算 CCR，占空比为 1.0 时输出满周期。
- 故障锁存后拒绝继续设置占空比，直到调用 `Heater_ClearFault()`。

## 与 Core/CubeMX 的关系

- 本目录源码独立于 `Core/`，不会被 CubeMX 重新生成覆盖。
- 依赖 CubeMX 生成的外设句柄：`hi2c1`、`hspi1`、`htim3`、`huart1`。
- 依赖 CubeMX 生成的 RTOS 互斥锁句柄：`I2CMutexHandle`、`SPIMutexHandle`。
- 自己创建驱动层共享数据锁：`DataMutexHandle`。
- `Core/Src/main.c` 在外设初始化后调用 `Hardware_Init()`。
- `Core/Src/freertos.c` 创建 SensorTask、HeatTask、FusionTask、DebugTask 等任务框架；这些任务后续通过 `hardware.h` 使用本驱动层。

## Keil/MDK-ARM 工程集成

`MDK-ARM/CloudFlyIMU.uvprojx` 已包含：

- C/C++ IncludePath：
  ```text
  Hardware/common;Hardware/bmi088;Hardware/bmp280;Hardware/ist8310;Hardware/heater;Hardware/debug_uart
  ```
- 汇编 IncludePath 同步追加上述目录。
- 源文件分组 `Hardware/Driver_Layer`，包含：
  ```text
  Hardware/common/hardware.c
  Hardware/debug_uart/debug_uart.c
  Hardware/bmp280/bmp280.c
  Hardware/ist8310/ist8310.c
  Hardware/bmi088/bmi088.c
  Hardware/heater/heater.c
  ```

## 使用约定

- 应用层统一包含：
  ```c
  #include "hardware.h"
  ```
- 不建议应用层直接包含 `bmi088.h`、`bmp280.h`、`ist8310.h`、`heater.h`、`debug_uart.h`。
- `Hardware_Init()` 成功后再调用各读取/控制 API。
- 传感器共享数据通过 `SensorData_Set/Get()` 和 `AttitudeData_Set/Get()` 访问。
- 加热占空比建议通过 `Heater_ApplyDuty()` 设置，这样 `SensorData_t.pwm_duty` 会同步更新。
