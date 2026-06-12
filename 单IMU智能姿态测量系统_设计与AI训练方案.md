# 单 IMU 智能姿态测量系统设计与 AI 训练方案

## 1. 项目名称

**基于恒温控制、AI 零漂预测与自适应 EKF 的单 IMU 智能姿态测量系统**

本方案采用单颗 **BMI088** 作为核心 IMU，结合 **IST8310 磁力计、BMP280 温度/气压传感器、电阻加热模块、YJL3400A MOSFET、SD 卡数据记录和 STM32H743 控制器**，构建一个面向课程设计的智能姿态测量系统。

系统重点不是直接替代成熟工业 AHRS，而是搭建一个可观测、可调试、可验证的嵌入式传感器实验平台，用于研究：

1. 温度变化对 MEMS IMU 陀螺仪零漂的影响；
2. 恒温控制对 IMU 零漂稳定性的改善；
3. 神经网络对陀螺仪零漂的预测与补偿；
4. 磁力计干扰下自适应 EKF 姿态融合的稳定性；
5. SD 卡数据记录与 AI 训练数据采集流程。

---

## 2. 系统总体目标

低成本 MEMS IMU 在长期运行中主要存在三个问题：

```
1. 温度变化导致陀螺仪零偏漂移；
2. 陀螺仪积分导致姿态角，尤其是 Yaw 角长期漂移；
3. 磁力计在磁场异常环境下容易导致九轴融合偏航角突变。
```

系统设计思路：

```
电阻恒温控制 → 降低温度变化
AI 零漂预测  → 补偿陀螺仪 bias（EKF 前端）
自适应 EKF   → 融合 IMU 和磁力计，内部同时估计残余 bias
SD 卡记录    → 采集训练数据和实验日志
```

最终输出：

```
Roll / Pitch / Yaw 姿态角
IMU 温度
加热 PWM 占空比
AI 预测零偏
磁场可靠性评分
EKF 工作模式
系统运行状态
```

---

## 3. 硬件组成

| 模块 | 器件 | 作用 |
|---|---|---|
| 主控 | STM32H743IIT6 | 运行 FreeRTOS、PID、AI 推理、EKF |
| 单 IMU | BMI088 | 采集三轴加速度和三轴角速度 |
| 磁力计 | IST8310 | 提供地磁方向，用于 Yaw 修正和磁干扰检测 |
| 温度/气压 | BMP280 | 采集温度，作为恒温反馈 |
| 加热器 | 5V 10Ω 5W 功率电阻 | 对 IMU 模块进行局部加热 |
| 加热驱动 | YJL3400A MOSFET | 通过 PWM 控制加热电阻功率 |
| 参数存储 | W25Q64 QSPI Flash | 存储 PID 参数等可持久化配置（备用） |
| 数据存储 | SD 卡（SDMMC1） | 保存 CSV 日志、训练数据、模型权重、配置文件 |
| 显示 | SPI OLED（可选） | 显示温度、姿态、CPU 占用率、系统状态 |
| 状态灯 | 3528 RGB（可选） | 显示正常、升温、磁干扰、故障等状态 |
| 通信 | USB CDC / UART | 实时输出数据到上位机 |

> **存储分工**：SD 卡负责 CSV 日志、AI 模型权重（`/model/bias_mlp.bin`）和应用参数（`CFIMUCFG.BIN`）；W25Q64 作为备用参数存储，SD 卡不可用时回退使用。

---

## 4. 外设接口分配

| 功能 | 外设 | 说明 |
|---|---|---|
| BMI088 | SPI1（PC4/PC5 双片选） | IMU 高频采样，200 Hz |
| IST8310 | I2C1 | 磁力计，50 Hz |
| BMP280 | I2C1 | 与 IST8310 共用 I2C，10 Hz |
| SD 卡 | SDMMC1 | 数据记录，FatFS |
| OLED | SPI2 | 状态显示，5 Hz |
| RGB LED | TIM4_CH1/CH2/CH3 | 三路 PWM 控制 RGB |
| 加热 MOSFET | TIM3_CH1（PC6） | PWM 控制电阻加热，约 1 Hz |
| 调试串口 | USART1 | 日志输出 |
| 上位机通信 | USB CDC / UART | 实时数据流 |

---

## 5. 系统数据流

```
BMI088 ──────────────────────────────────────────────────────┐
                                                              ↓
IST8310 ──────────────────────────────────────────────────→ EKF（7 维）
                                                              ↑
BMP280 → AI 零漂预测（MLP）→ bgx/bgy/bgz → gx_corrected ────┘
           ↑
      T, dT/dt, pwm, 滑动窗口统计特征

EKF 输出 → Roll / Pitch / Yaw
         → EKF 内部 bias（残余补偿）

BMP280 → PID 恒温控制 → PWM → 加热电阻

所有数据 → RAM 环形缓冲区 → sdLogTask → SD 卡 CSV
         → OLED / RGB 状态显示
```

**AI 与 EKF 的协作关系**：

- AI（MLP）在 EKF 前端预测温度相关的慢变零漂，补偿幅度较大（温度引起的系统性偏差）；
- EKF 内部 7 维状态（`[q0,q1,q2,q3,bgx,bgy,bgz]`）继续估计残余随机游走 bias；
- 两者互补：AI 处理温度相关的确定性漂移，EKF 处理随机游走和短期扰动。

---

## 6. 传感器采样频率设计

| 模块 | 频率 | 周期 | 说明 |
|---|---:|---:|---|
| BMI088 陀螺仪 + 加速度计 | 200 Hz | 5 ms | EKF 预测核心数据 |
| IST8310 磁力计 | 50 Hz | 20 ms | Yaw 修正，在 sensorTask 内分频 |
| BMP280 温度 | 10 Hz | 100 ms | 恒温控制反馈，在 sensorTask 内分频 |
| 恒温 PID | 10 Hz | 100 ms | 热系统变化慢 |
| AI 零漂预测 | 10 Hz | 100 ms | 零漂变化慢，在 fusionTask 内分频 |
| EKF 姿态融合 | 200 Hz | 5 ms | 与 BMI088 同步 |
| SD 卡日志 | 50 Hz | 20 ms | 批量写入，独立任务 |
| OLED 刷新 | 5 Hz | 200 ms | 人眼显示足够 |
| RGB 状态灯 | 10 Hz | 100 ms | 状态提示 |

---

## 7. 需要采集的数据

### 7.1 CSV 格式

SD 卡日志推荐保存为 CSV 文件：

```
time_ms,T,pwm,ax,ay,az,gx,gy,gz,mx,my,mz,bgx_ai,bgy_ai,bgz_ai,gx_c,gy_c,gz_c,mag_norm,mag_score,roll,pitch,yaw,ekf_mode
```

| 字段 | 来源 | 用途 |
|---|---|---|
| time_ms | HAL_GetTick() | 时间戳 |
| T | BMP280 | 恒温控制、AI 输入 |
| pwm | TIM3 占空比 | 记录加热功率 |
| ax/ay/az | BMI088 加速度计（m/s²） | 姿态融合、静止检测 |
| gx/gy/gz | BMI088 陀螺仪（rad/s） | 姿态预测、零漂标签 |
| mx/my/mz | IST8310（μT） | Yaw 修正、磁干扰判断 |
| bgx/bgy/bgz_ai | AI 预测零偏 | 补偿量记录 |
| gx_c/gy_c/gz_c | 补偿后角速度 | 送入 EKF 的实际值 |
| mag_norm | sqrt(mx²+my²+mz²) | 磁场模长 |
| mag_score | 磁场可靠性评分 [0,1] | 自适应 EKF 权重 |
| roll/pitch/yaw | EKF 输出（rad） | 姿态角 |
| ekf_mode | 0=六轴 1=九轴 2=自适应 | EKF 当前模式 |

### 7.2 文件结构

```
SD 卡根目录
├── CFIMUCFG.BIN          应用参数（PID 增益、目标温度）
├── model/
│   └── bias_mlp.bin      MLP 权重（W1,b1,W2,b2,W3,b3,x_mean,x_std,y_mean,y_std）
└── log/
    ├── no_heat_001.csv   无恒温静止实验
    ├── heat_up_001.csv   升温过程实验
    ├── hold_45c_001.csv  恒温保持实验
    ├── cool_down_001.csv 自然降温实验
    └── mag_test_001.csv  磁干扰实验
```

---

## 8. AI 零漂预测设计

### 8.1 AI 的作用

AI 不直接输出姿态角，只预测陀螺仪零漂：

```
输入：温度统计特征 + IMU 滑动窗口统计特征（10 维）
输出：bgx, bgy, bgz（rad/s）
```

补偿方式：

```c
gx_corrected = gx_raw - bgx_ai;
gy_corrected = gy_raw - bgy_ai;
gz_corrected = gz_raw - bgz_ai;
// 补偿后送入 EKF
```

### 8.2 输入特征（10 维，第一版）

每个样本由 2 秒滑动窗口（400 个点，步长 0.5 秒）生成：

| 特征 | 说明 |
|---|---|
| T_mean | 窗口内平均温度（°C） |
| T_std | 窗口内温度标准差 |
| dT_dt | 温度变化率（°C/s），用相邻窗口均值差估算 |
| pwm_mean | 平均加热占空比 [0,1] |
| gx_mean, gy_mean, gz_mean | 陀螺仪均值（rad/s） |
| gx_std, gy_std, gz_std | 陀螺仪标准差（rad/s） |

> 后续可扩展加入 ax_mean/ax_std 等加速度特征（16 维）。

### 8.3 标签生成

静止状态下真实角速度为 0，因此：

```
bgx_label = mean(gx, 2s 窗口)
bgy_label = mean(gy, 2s 窗口)
bgz_label = mean(gz, 2s 窗口)
```

使用低通滤波（或滑动均值）去除噪声，保留低频零偏分量。

### 8.4 静止检测（在线更新条件）

```c
float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz);
bool is_static = (fabsf(acc_norm - 9.81f) < 0.5f) && (gyro_norm < 0.00873f); // 0.5 dps
// 仅静止时更新 AI bias，运动时保持上一时刻值
```

---

## 9. 模型结构

```
Input: 10
Dense(32) + ReLU
Dense(32) + ReLU
Dense(3)
Output: bgx, bgy, bgz
```

- 损失函数：MSELoss
- 优化器：Adam（lr=1e-3）
- 训练轮数：200 epochs
- 参数量：约 1.4K，STM32H743 完全可以手写推理

### 9.1 STM32 端推理

不使用 TFLite Micro，手写 MLP 推理：

```c
// 1. 输入标准化
for (int i = 0; i < 10; i++)
    x_norm[i] = (x[i] - x_mean[i]) / x_std[i];

// 2. Dense1 + ReLU
mat_vec_mul(W1, x_norm, h1, 32, 10);
vec_add_bias(h1, b1, 32);
relu(h1, 32);

// 3. Dense2 + ReLU
mat_vec_mul(W2, h1, h2, 32, 32);
vec_add_bias(h2, b2, 32);
relu(h2, 32);

// 4. Dense3
mat_vec_mul(W3, h2, y_norm, 3, 32);
vec_add_bias(y_norm, b3, 3);

// 5. 输出反标准化
for (int i = 0; i < 3; i++)
    y[i] = y_norm[i] * y_std[i] + y_mean[i];
```

权重从 SD 卡 `/model/bias_mlp.bin` 加载，格式：

```
[W1: 32×10 float] [b1: 32 float]
[W2: 32×32 float] [b2: 32 float]
[W3: 3×32 float]  [b3: 3 float]
[x_mean: 10 float] [x_std: 10 float]
[y_mean: 3 float]  [y_std: 3 float]
```

总大小约 6.5 KB。

---

## 10. AI 训练数据采集方案

### 实验一：无恒温静止零漂

目的：观察自然温度变化下陀螺仪零偏变化。

```
1. 关闭加热（pwm=0）
2. PCB 固定静止，连续采集 15~20 分钟
3. 保存为 no_heat_001.csv、no_heat_002.csv（重复 3 次）
```

### 实验二：升温过程零漂

目的：学习温度上升与零漂的关系。

```
1. 开启加热，温度从室温升至 45°C
2. IMU 全程静止，记录 T、pwm、gx/gy/gz
3. 重复 3~5 次，保存为 heat_up_001.csv 等
```

### 实验三：恒温保持零漂

目的：验证恒温状态下零漂稳定性。

```
1. PID 控制温度稳定在 45°C
2. 恒温保持 15~20 分钟
3. 保存为 hold_45c_001.csv（重复 3 次）
```

### 实验四：自然降温零漂

目的：增加降温阶段训练样本。

```
1. 关闭加热，从 45°C 自然降温至室温
2. 全程静止，保存为 cool_down_001.csv（重复 3 次）
```

### 实验五：磁干扰验证

目的：验证磁场可靠性评分和自适应 EKF（不用于零漂训练）。

```
1. 正常环境运行九轴 EKF
2. 将磁铁靠近 IST8310，观察 mag_score 下降和 EKF 模式切换
3. 移除干扰，观察恢复九轴融合
4. 保存为 mag_test_001.csv
```

### 训练集划分

按实验轮次划分，不随机打乱：

```
第 1~3 轮（各实验类型）：训练集
第 4 轮：验证集
第 5 轮：测试集
```

---

## 11. Python 训练流程

```python
# 1. 读取并合并 CSV
# 2. 滑动窗口（2s，步长 0.5s）生成样本
# 3. 静止检测过滤（仅保留静止段）
# 4. 生成零漂标签（窗口内 gx/gy/gz 均值）
# 5. 特征标准化（保存 x_mean, x_std）
# 6. 标签标准化（保存 y_mean, y_std）
# 7. 按轮次划分训练/验证/测试集
# 8. PyTorch 训练 MLP，200 epochs
# 9. 评估 RMSE，对比补偿前后 Yaw 漂移
# 10. 导出权重为二进制文件 bias_mlp.bin
# 11. 将 bias_mlp.bin 复制到 SD 卡 /model/
```

---

## 12. EKF 姿态融合方案

### 12.1 状态向量

```
x = [q0, q1, q2, q3, bgx, bgy, bgz]  （7 维）
```

EKF 内部 bias（bgx/bgy/bgz）估计残余随机游走，与 AI 预测的温度相关 bias 互补。

### 12.2 EKF 输入

```c
// AI 补偿后的角速度送入 EKF
EKF_Update(&ekf,
    gx_raw - bgx_ai,   // gx_corrected
    gy_raw - bgy_ai,   // gy_corrected
    gz_raw - bgz_ai,   // gz_corrected
    ax, ay, az,
    mx, my, mz,
    mag_score,         // 磁场可靠性 [0,1]
    dt);
```

### 12.3 磁场可靠性评分

```c
float mag_norm = sqrtf(mx*mx + my*my + mz*mz);
// 正常地磁模长参考值（需现场标定，约 30~60 μT）
float norm_err = fabsf(mag_norm - MAG_NORM_REF) / MAG_NORM_REF;
float mag_score = 1.0f / (1.0f + 10.0f * norm_err);  // 软评分，偏离越大分越低
// 可叠加方向突变检测、与陀螺仪预测 Yaw 的差异等
```

EKF 内部根据 mag_score 动态调整磁力计观测噪声：

```c
float r_mag_adaptive = ekf->r_mag / (mag_score + 1e-6f);
// mag_score=1 时 r_mag_adaptive=r_mag（正常信任）
// mag_score→0 时 r_mag_adaptive→∞（忽略磁力计，退化为六轴）
```

---

## 13. FreeRTOS 任务分配

| 任务 | 周期 | 优先级 | 功能 |
|---|---:|---|---|
| sensorTask | 5 ms | osPriorityHigh | 读取 BMI088；分频读取 IST8310（每 4 次）和 BMP280（每 20 次） |
| fusionTask | 5 ms | osPriorityHigh | AI bias 补偿（每 20 次）+ 自适应 EKF（每次） |
| heatTask | 100 ms | osPriorityNormal | PID 恒温控制，更新 PWM |
| sdLogTask | 20 ms | osPriorityLow | 从环形缓冲区批量写入 SD 卡 |
| uiTask | 200 ms | osPriorityLow | OLED 刷新 + RGB 状态灯 |
| monitorTask | 1000 ms | osPriorityLow | CPU 占用率、堆栈水位监测 |

**数据流**：sensorTask / fusionTask → RAM 环形缓冲区 → sdLogTask 批量写入（每次 1~4 KB）。

---

## 14. RGB 状态灯设计

| 状态 | 颜色 | 触发条件 |
|---|---|---|
| 正常运行（九轴 EKF） | 绿色常亮 | mag_score > 0.7 |
| 升温中 | 黄色常亮 | T < target - 2°C |
| 恒温稳定 | 青色常亮 | \|T - target\| < 0.5°C |
| 磁干扰（自适应/六轴） | 紫色常亮 | mag_score < 0.3 |
| SD 卡写入 | 蓝色闪烁 | sdLogTask 写入中 |
| 过温保护 | 红色快闪 | T > 55°C（Heater_OverheatCheck） |
| 系统故障 | 红色常亮 | Hardware_Init 失败 |

---

## 15. OLED 显示内容

```
Temp: 45.0C  PWM:36%
Mode: ADAPT  Mag:OK
Roll: +1.2   CPU:42%
Pitch:-0.5
Yaw : 86.4
```

---

## 16. 实验验证方案

### 16.1 恒温控制实验

对比无恒温 vs 恒温控制，评价：
- 温度稳定时间
- 温度波动范围（目标 ±0.5°C）
- PWM 占空比变化曲线

### 16.2 零漂补偿实验

三组对比：

```
① 原始陀螺仪（无补偿）
② 恒温后陀螺仪（无 AI）
③ 恒温 + AI 补偿后陀螺仪
```

评价指标：
- gx/gy/gz 均值（越接近 0 越好）
- gx/gy/gz 标准差
- 静止 10 分钟后 Yaw 漂移量（°）

### 16.3 磁干扰实验

```
正常运行 → 靠近磁铁 → 观察 mag_score 下降 → EKF 降低磁力计权重
→ 移除磁铁 → 观察 mag_score 恢复 → EKF 恢复九轴融合
```

评价：磁干扰时自适应 EKF 的 Yaw 突变幅度 vs 固定九轴 EKF。

---

## 17. 预期结果

1. 恒温控制后，IMU 温度波动 < ±0.5°C；
2. 恒温后陀螺仪零漂标准差降低；
3. AI 补偿后，静止角速度均值更接近 0；
4. AI 补偿后，10 分钟 Yaw 漂移量明显减小；
5. 磁干扰时 mag_score 明显下降，EKF 自动降低磁力计权重；
6. 自适应 EKF 比固定九轴 EKF 在磁干扰时 Yaw 更稳定；
7. SD 卡完整记录实验数据，为 AI 训练提供数据来源。

---

## 18. 开发顺序

```
已完成（V1.3）：
  ✓ BMI088 驱动（SPI1）
  ✓ IST8310 驱动（I2C1）
  ✓ BMP280 驱动（I2C1）
  ✓ 加热器 PWM 驱动（TIM3_CH1）
  ✓ PID 控制器
  ✓ EKF 7 维实现
  ✓ SD 卡存储（FatFS，参数读写）
  ✓ OLED 驱动
  ✓ RGB LED 驱动
  ✓ 调试串口（USART1）

待完成：
  [ ] 确认所有任务频率和互斥锁配置
  [ ] 实现 CSV 数据记录（sdLogTask + 环形缓冲区）
  [ ] 实现 PID 恒温控制任务
  [ ] 采集四类实验数据（无恒温/升温/恒温/降温）
  [ ] Python 训练 MLP 零漂预测模型
  [ ] 导出 bias_mlp.bin 并部署到 STM32
  [ ] 实现 AI bias 补偿（fusionTask 内分频调用）
  [ ] 实现磁场可靠性评分
  [ ] 完善自适应 EKF（mag_score 动态调整 R_mag）
  [ ] 完善 RGB 状态机
  [ ] 完成实验对比和报告图表
```

---

## 19. 方案总结

本方案采用单颗 BMI088 作为核心 IMU，结合 IST8310 磁力计、BMP280 温度传感器、电阻恒温结构、SD 卡数据记录和 STM32H743 控制器，构建了一个可用于课程设计的智能姿态测量系统。

系统主要创新点：

```
1. 电阻加热 + PID 控制实现 IMU 局部恒温；
2. MLP 神经网络预测温度相关陀螺仪零漂（EKF 前端补偿）；
3. EKF 7 维状态同时估计残余随机游走 bias（与 AI 互补）；
4. 磁场可靠性评分实现六轴/九轴自适应融合；
5. SD 卡记录完整训练数据和实验日志；
6. 可观测、可调试、可验证的嵌入式传感器实验平台。
```