# MPU6050 → 航向 → PID（合一篇）

> 本文合并了原来的两份笔记：
> - `mpu6050_PID_learn.md`（传感器链路：I2C → MPU6050 → attitude）
> - `course_PID_Learn.md`（航向环 PID：从 P 到完整实现）
>
> 二者本来就是**同一条链路的两段**：陀螺读数 → 航向角 → PID 输出。
> 合并后按"数据从哪来 → 怎么算 → 怎么用"组织，配合工程的 `yaw_ctrl.c` 阅读。

**当前工程的模块对应**

| 文件 | 层 | 职责 |
| --- | --- | --- |
| `IIC.c/h` | Hardware | 软件 I2C 时序（PB8/PB9） |
| `mpu6050.c/h` | Hardware | 读原始数据、换算、零偏标定 |
| `attitude.c/h` | Software | 陀螺积分成航向 yaw |
| `yaw_ctrl.c/h` | Software | 航向环 PID，输出差速 |
| `question1.c/h` | Software | 用上面的东西完成 Q1 |

---

# 第一部分：从传感器到航向

## 0. 数据流

```
MPU6050 芯片
   │  I2C（PB8/PB9，软件模拟）
   ▼
IIC.c         Start/Stop/SendByte/ReadRegs
   │  14 字节原始数据
   ▼
mpu6050.c     GetRaw → GyroToDps → GetGyroZDps（扣零偏）
   │  gz (°/s)
   ▼
attitude.c    Update：dt + 死区 + 残差跟踪 + 积分
   │  yaw (°) / gz (°/s)
   ▼
yaw_ctrl.c    PID：e = target − yaw，输出 delta(%)
```

**一句话**：`mpu6050.c` 负责“读得准”，`attitude.c` 负责“算得稳”，`yaw_ctrl.c` 负责“控得住”。

---

## 1. 软件 I2C（`IIC.c`）

### 1.1 为什么用软件 I2C

| | 硬件 I2C1 | 软件 I2C |
| --- | --- | --- |
| 引脚 | PB6/PB7 或 PB8/PB9 | 任意两个 GPIO |
| 与 OLED 共用 | 需要 OLED 改成硬件驱动 | ✅ 同一条总线即可 |
| 调试 | 卡死难查 | 每根线都能用万用表量 |
| 速度 | 100k/400k | ≈150 kHz（2µs 延时） |

OLED（0x78）和 MPU6050（0xD0）挂在同一对线上，靠地址区分。

### 1.2 引脚必须“开漏 + 上拉”

I2C 是线与总线：设备只能**拉低**，不能主动拉高。

```c
#define IIC_SCL_H()  HAL_GPIO_WritePin(..., GPIO_PIN_SET)   /* 释放，靠上拉拉高 */
```

配成推挽时，一方输出高、一方输出低 → 电源短路。所以 CubeMX 里是 `GPIO_MODE_OUTPUT_OD`。

### 1.3 读寄存器的完整时序

```
START → 0xD0(W) → ACK → 0x47 → ACK → RESTART → 0xD1(R) → ACK → dataH → ACK → dataL → NACK → STOP
```

对应 `IIC_ReadRegs()` 的十个步骤；最后一字节必须回 **NACK**，告诉从机“够了”。

---

## 2. MPU6050 芯片

### 2.1 关键寄存器

| 地址 | 名称 | 作用 |
| --- | --- | --- |
| `0x6B` | PWR_MGMT_1 | 复位(bit7) / 睡眠 / 时钟源 |
| `0x19` | SMPLRT_DIV | 采样率分频 |
| `0x1A` | CONFIG | 数字低通 DLPF |
| `0x1B` | GYRO_CONFIG | 陀螺量程 |
| `0x1C` | ACCEL_CONFIG | 加速度量程 |
| `0x3B~0x48` | 数据区 | 加速度6 + 温度2 + 陀螺6 = **14 字节连续** |
| `0x75` | WHO_AM_I | 正常 = 0x68 |

### 2.2 初始化的每个值为什么这么设

| 配置 | 值 | 理由 |
| --- | --- | --- |
| 软复位 | `PWR_MGMT_1 = 0x80` | 上电寄存器状态不确定 |
| 时钟源 | `0x01`（PLL X 陀螺） | 内部 8 MHz 振荡器温漂大 |
| 采样率 | `SMPLRT_DIV = 9` → 100 Hz | 和 10 ms 控制周期**严格对齐** |
| 低通 | `CONFIG = 0x03` → 44 Hz | 5 Hz 延迟 19 ms，44 Hz 只延迟 4.9 ms |
| 陀螺量程 | `0x18` → ±2000 °/s | 转向 90 °/s 很常见，±250 会饱和 |
| 加速度量程 | `0x08` → ±4 g | 本工程不用，留合理范围 |

### 2.3 量程换算

```
±2000 °/s：16.4 LSB = 1 °/s   → 1 个 LSB = 0.061 °/s（分辨率下限）
```

**改量程必须同步改 `MPU6050_GYRO_LSB_PER_DPS`**，否则数值差 4 倍。

---

## 3. `mpu6050.c` 逐行

### 3.1 一次突发读 14 字节

```c
uint8_t MPU6050_GetRaw(s_mpu6050_raw *out)
{
    uint8_t buf[14];
    if (IIC_ReadRegs(MPU6050_ADDR_W, MPU6050_ACCEL_XOUT_H, buf, 14) != 0) return 1;

    out->acc[0]  = (int16_t)((buf[0]  << 8) | buf[1]);
    ...
    out->gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);   /* Z 轴，航向用这个 */
    return 0;
}
```

**为什么必须突发读**：

| 方式 | 耗时 |
| --- | --- |
| 逐字节读 14 次 | ≈ 14 ms |
| **一次突发读** | **≈ 1 ms** |

100 Hz 采样下，逐字节读会把 CPU 全耗在 I2C 上。

> `(int16_t)` 强转不能省：否则负角速度会变成 6 万多。

### 3.2 换算与扣零偏

```c
float MPU6050_GyroToDps(int16_t raw) { return (float)raw / MPU6050_GYRO_LSB_PER_DPS; }

float MPU6050_GetGyroZDps(void)
{
    s_mpu6050_raw raw;
    if (MPU6050_GetRaw(&raw) != 0) return 0.0f;      /* 读失败返回 0，不污染积分 */
    return (float)(raw.gyro[2] - m_gz_bias) / MPU6050_GYRO_LSB_PER_DPS;
}
```

### 3.3 零偏标定

| 参数 | 值 | 说明 |
| --- | --- | --- |
| 预热 | 300 ms | 等芯片内部振荡器稳定 |
| 丢弃 | 20 个样本 | 复位后的过渡数据 |
| 采样 | 300 次 | 平均后精度足够 |

**为什么必须标定**：静止时陀螺不是 0（数据手册 ±20 °/s，标定后 0.1~0.5 °/s）。0.5 °/s 不处理 → 1 分钟漂 30°。

**为什么还要“运行时残差跟踪”**：芯片自加热会让零偏继续变，标定只解决“上电那一刻”。

---

## 4. `attitude.c` 逐行

```c
void Attitude_Update(void)
{
    now = HAL_GetTick();
    dt  = (now - m_last_ms) * 0.001f;          /* ① 实际经过时间 */

    if (dt < 0.001f) return;                   /* ② 太快，跳过 */
    if (dt > 0.050f) dt = 0.050f;              /* ③ 上限 50ms */
    m_last_ms = now;

    m_gz = MPU6050_GetGyroZDps() - m_bias_res; /* ④ 标定零偏 + 残差 */

    if (|m_gz| < 0.5) m_bias_res += m_gz * 0.001f;   /* ⑤ 慢速残差跟踪 */

    if (|m_gz| < 0.15) m_gz = 0.0f;            /* ⑥ 死区 */

    m_yaw += m_gz * dt;                        /* ⑦ 积分 */

    while (m_yaw >  180) m_yaw -= 360;         /* ⑧ 归一化 */
    while (m_yaw < -180) m_yaw += 360;
}
```

### ①②③ 为什么用实际 dt

主循环周期不是严格 10 ms（会被串口/OLED 拉长）。写死 `yaw += gz*0.01` 时，实际 20 ms 却按 10 ms 积分 → **角度只有真值一半**。

dt 上限 50 ms 是防止某次长时间阻塞（如初始化）把 yaw 一次性跳几百度假值。

### ⑤ 残差跟踪（本工程最关键的一招）

```
|gz| < 0.5 °/s 时，每拍把残差吸收 0.1%（1 秒约 10%）
```

- 真实转向远大于 0.5 °/s（跑弧线约 43 °/s），**不会被误吃**；
- 相当于“只在静止时工作的自动零点校准”。

### ⑥ 死区

小于 0.15 °/s 直接当 0。噪声级抖动不进入积分，yaw 才能真正“停住”。

### 为什么不用加速度计/卡尔曼

| 方案 | 为什么没用 |
| --- | --- |
| 加速度计修正倾角 | 小车在水平面，倾角对 yaw 无帮助 |
| 卡尔曼/互补滤波 | 没有磁力计，加速度计**纠正不了 yaw** |
| DMP | 要移植固件，yaw 仍是相对量 |

**结论**：本题只需“4 秒内相对航向”，纯积分 + 每段清零就够。

---

## 5. 误差来源与对策

| 误差源 | 量级 | 对策 | 代码 |
| --- | --- | --- | --- |
| 零偏 | ±20 °/s（未标定） | 上电标定 | `MPU6050_CalibGyro` |
| 温漂 | 0.1~0.5 °/s | 残差跟踪 + 死区 | `attitude.c` ⑤⑥ |
| 量化 | 0.061 °/s/LSB | 死区吸收 | ⑥ |
| 振动 | 几十 °/s 毛刺 | DLPF 44Hz + 减振 | `CONFIG=0x03` |
| 积分误差 | dt 不准 | 用实际 dt | ① |
| 长期漂移 | 分钟级 | **每段 ResetYaw** | `YawCtrl_Reset` |

---

# 第二部分：航向环 PID

## 6. 为什么需要它

开环下左右轮不同速：

```
左轮 35% → 33 cm/s
右轮 35% → 37 cm/s
跑 1 米（3 秒）→ 差 12 cm
```

黑线只有 1.8 cm 宽 → **不走直线就切不到弧线**。

## 7. 符号约定（先定死，否则越调越乱）

```
航向 yaw：顺时针为正，−180 ~ +180
角速度 gz：顺时针为正，°/s
误差 e = target − yaw
    e > 0 → 车头偏左，需向右转
差速 delta > 0 → 左轮快、右轮慢 → 车向右转
```

自洽检查：`e > 0` 输出 `delta > 0` → 向右转 → 回到目标 ✅

## 8. 从 P 到 PID 的演进

### 8.1 只有 P

```c
delta = KP * (target - yaw);
```

| Kp | 现象 |
| --- | --- |
| 0.5 | 慢慢回，回不到 |
| 1.2 | 能回，但**来回摆** |
| 3.0 | 画龙甚至发散 |

**为什么纯 P 会摆**：车有惯性，`e=0` 时车还在转，会转过头，于是反向拉，周而复始。

### 8.2 加 D（差分版，不推荐）

```c
delta = KP * e + KD * (e - e_prev);
```

问题：`e` 是积分出来的带噪声，相邻相减会放大噪声 → 车轮哆嗦。

### 8.3 加 D（陀螺版，推荐）

```c
delta = KP * e - KD * gz;      /* KD 单位：%/(°/s) */
```

因为 `de/dt = −gz`，两种写法数学等价，但陀螺直接测导数：**噪声小、无延迟**。

> ⚠️ 单位变了：差分版 KD≈0.10（每拍），陀螺版 KD≈0.12（%/°/s）。换实现必须重调。

### 8.4 加 I

```c
integ += KI * e;  然后限幅
delta = KP*e + integ - KD*gz;
```

**为什么必须加**：左右电机差异需要一个恒定差速抵消，而 P 项只在“有误差”时输出。

```
1° 误差 → 1 米偏 17.5 mm  → 接近线宽
```

所以稳态误差必须压到 **0.5° 以内**。

---

## 9. 完整实现（工程里的 `yaw_ctrl.c/h`）

### 9.1 接口

```c
void  YawCtrl_Init(void);              /* 上电一次          */
void  YawCtrl_Reset(void);             /* 换段：清积分+归零 */
void  YawCtrl_SetTargetDeg(float deg); /* 设目标航向        */
float YawCtrl_Update10ms(void);        /* 每 10ms，返回 delta */
```

### 9.2 实现要点（逐段）

```c
e_deg = target − Attitude_GetYaw();            /* ① 误差 */

if (|e_deg| < YAW_DEADBAND_DEG) e_deg = 0;     /* ② 死区 */

m_integ_pct += YAW_KI * e_deg;                 /* ③ 积分 */
m_integ_pct = clamp(m_integ_pct, ±YAW_IMAX);   /*    抗饱和 */

delta = YAW_KP*e_deg + m_integ_pct
        - YAW_KD*Attitude_GetGz();             /* ④ P+I+D */

delta = clamp(delta, ±YAW_OUTMAX);             /* ⑤ 限幅 */
```

### 9.3 参数（`config.h`）

| 宏 | 值 | 单位 |
| --- | --- | --- |
| `YAW_KP` | 1.2 | %/° |
| `YAW_KD` | 0.12 | %/(°/s) |
| `YAW_KI` | 0.02 | 每拍 |
| `YAW_IMAX` | 15 | % |
| `YAW_OUTMAX` | 40 | % |
| `YAW_DEADBAND_DEG` | 0.3 | ° |

### 9.4 接到驱动

```c
float   d = YawCtrl_Update10ms();
int16_t l = base + d, r = base - d;      /* 限幅 ±100 */
Car_SetSpeed(l, r);
```

---

## 10. 整定步骤（每步都有判据）

### 10.1 前提

```
□ 架空，四轮同向
□ 静止 GZ ≈ 0（±0.2 °/s）
□ 10 ms 节拍已启动
```

### 10.2 Kp（只开 P）

```
0.5 → 0.8 → 1.2 → 1.5 → 2.0
手把车头拧偏 20°，松手看回正：
  回不到       → 太小
  回到 0 略过冲 → 合适
  摆 3 次才停   → 太大
```

### 10.3 Kd（压超调）

```
0.05 → 0.12 → 0.2
超调从 3° 降到 <1° → 有效
开始高频抖        → 太大
```

### 10.4 Ki（消稳态误差）

```
0.01 → 0.02 → 0.05
跑 1 米看停车时 yaw：
  一直偏 +3°  → 加倍
  慢周期来回摆 → 减半
  ±0.5° 内     → ✅
```

### 10.5 验收

```
□ 直线 1 米，横向偏差 < 1 cm
□ 连跑 5 次都在 ±1 cm
□ 起步不甩尾，停车不跑偏
```

换算：`偏差(mm) = 1000 × tan(航向误差)` → 1 cm 对应 0.57°。

---

## 11. 故障表

| 现象 | 曲线特征 | 根因 | 处理 |
| --- | --- | --- | --- |
| 越纠越偏，几秒打满舵 | 输出单调增大 | **符号反** | 架空手转车身，看轮子是否往纠正方向转；对调电机线或取反 delta |
| 低频画龙 | 正弦摆动 | Kp 大 / Kd 小 | Kp×0.8，Kd×1.3 |
| 高频抖 | 密集锯齿 | Kd 大，放大陀螺噪声 | Kd×0.5 |
| 一直偏一点不回 | 稳态误差 | 无 I | 加 Ki |
| 慢周期来回摆 | 长周期 | Ki 大 | Ki×0.5 |
| 起步甩一下 | 输出先满 | 积分饱和 | 加 IMAX；起步前 Reset |
| GZ 恒为 0 | — | IMU 没识别 | 看 `Attitude_Init()` 返回值 |
| 静止 YAW 一直涨 | — | 零偏没扣好 | 加长预热 / 检查标定时是否静止 |

---

# 第三部分：工程落地

## 12. Q1 是怎么用这两个模块的

```c
/* question1.c */
void Ques1_Start(void)
{
    Car_Stop();
    YawCtrl_Reset();          /* 航向归零，积分清零 */
}

uint8_t Ques1_Task10ms(void)   /* 每 10ms */
{
    switch (m_step)
    {
    case 0:  停 300ms 等人松手 → 再 Reset → 进 1
    case 1:  看到线 → 停车 + 提示，返回 1
             超时   → 停车 + 长鸣，返回 2
             否则   → DriveStraight()
    }
}
```

`DriveStraight()` 就是 `base ± YawCtrl_Update10ms()`。

**调用链**：

```
main 的 10ms 门控
   └─ Ques1_Task10ms()
        ├─ Line_ReadRaw()          ← Hardware
        ├─ YawCtrl_Update10ms()    ← Software
        │    └─ Attitude_GetYaw/Gz ← Software
        │         └─ MPU6050_GetGyroZDps ← Hardware
        └─ Car_SetSpeed()          ← Software → Motor → PWM
```

## 13. 时间预算（每 10 ms）

| 操作 | 耗时 |
| --- | --- |
| `MPU6050_GetRaw`（14 字节） | ≈ 1 ms |
| `Attitude_Update` 计算 | < 10 µs |
| `YawCtrl_Update10ms` | < 5 µs |
| 总计 | **≈ 1 ms = 10%** |

**这 1 ms 的 I2C 读必须在主循环**，不能放 TIM1 中断。

## 14. 参数速查

| 项 | 值 | 位置 |
| --- | --- | --- |
| 陀螺量程 | ±2000 °/s（0x18） | `mpu6050.c` |
| 换算系数 | 16.4 LSB/(°/s) | `mpu6050.h` |
| DLPF | 44 Hz（0x03） | `mpu6050.c` |
| 采样率 | 100 Hz | `mpu6050.c` |
| 标定 | 预热 300ms + 采样 300 | `mpu6050.c` |
| 死区 | 0.15 °/s | `attitude.c` |
| 航向环 Kp/Ki/Kd | 1.2 / 0.02 / 0.12 | `config.h` |
| 输出限幅 | ±40 % | `config.h` |
| 直线 base | 35 % | `config.h` |

## 15. 一页总结

```
① 软件 I2C 一次读 14 字节（1ms），比逐字节快 14 倍
② 100Hz 采样 + 44Hz 低通，与 10ms 控制周期对齐
③ 上电标定零偏（300ms 预热），运行时残差跟踪 + 死区
④ yaw 用实际 dt 积分，每段 Reset，误差不跨段
⑤ 航向环：delta = Kp·e + ΣKi·e − Kd·gz
⑥ yaw 喂 P 项，gz 喂 D 项（陀螺直接给导数，别用差分）
⑦ 整定顺序：Kp（略超调）→ Kd（压超调）→ Ki（消稳态）
⑧ 验收：直线 1 米，横向偏差 < 1 cm
```

---

# 第四部分：Q2 / Q3 / Q4 的解决思路

> 三问都是"多个段串起来"，区别只在**段表**和**速度策略**。
> 核心思路：**把所有路线写成一个数组，状态机只负责推进下标。**

## 16. 三条路线长什么样

场地几何：A(0,20)、B(100,20)、C(100,100)、D(0,100)，两段半圆弧半径 40 cm。

| 问 | 路线 | 段数 | 总长 | 限时 |
| :--: | --- | :--: | ---: | :--: |
| Q2 | A→B→弧→C→D→弧→A | 4 | 100+125.7+100+125.7 = **451 cm** | 30 s |
| Q3 | A→C→弧→B→D→弧→A | 4 | 128.1+125.7+128.1+125.7 = **507 cm** | 40 s |
| Q4 | 同 Q3 连续 4 圈，不停车 | 16 | 507×4 = **2029 cm** | 40 s |

**几何数据**：

```
直线 A→B、C→D        ：100 cm
半圆弧（半径 40）    ：πR = 125.7 cm
斜线 A→C、B→D        ：√(100² + 80²) = 128.1 cm
斜线倾角             ：atan(80/100) = 38.66°
```

## 17. 关键设计：段表 + 事件

### 17.1 每段只需要四个信息

```c
typedef enum { SEG_LINE, SEG_ARC }              s_seg_type;
typedef enum { END_ON_LINE, END_LOST }          s_seg_end;

typedef struct
{
    s_seg_type type;        /* 直线还是弧线          */
    s_seg_end  end;         /* 怎么算"这一段走完了"  */
    float      target_deg;  /* 直线段目标航向（度）  */
    uint16_t   timeout_ms;  /* 超时保护              */
} s_path_seg;
```

### 17.2 两条结束判据（就是 Q1 用过的）

| 段类型 | 结束条件 | 依据 |
| --- | --- | --- |
| `SEG_LINE` | `Line_ReadRaw() != 0` | 直线尽头一定有黑线（弧线起点） |
| `SEG_ARC` | `LineTrack_IsLost()` | 弧线走完自然没线 |

**不需要里程**，也不需要知道"走了多少厘米"——靠物理基准。

### 17.3 Q2 的段表

```c
static const s_path_seg m_path_q2[] =
{
    { SEG_LINE, END_ON_LINE, 0.0f, 8000 },   /* ① A→B       */
    { SEG_ARC,  END_LOST,    0.0f, 8000 },   /* ② B→C 半圆弧 */
    { SEG_LINE, END_ON_LINE, 0.0f, 8000 },   /* ③ C→D       */
    { SEG_ARC,  END_LOST,    0.0f, 8000 },   /* ④ D→A 半圆弧 */
};
```

### 17.4 状态机骨架（Q2/Q3/Q4 共用）

```c
static const s_path_seg *m_path;
static uint8_t  m_seg_cnt;
static uint8_t  m_seg;
static uint32_t m_seg_t0;

static void Path_Start(const s_path_seg *path, uint8_t n)
{
    m_path    = path;
    m_seg_cnt = n;
    m_seg     = 0;
    m_seg_t0  = HAL_GetTick();

    YawCtrl_Reset();          /* 航向以本段起点为 0 */
    LineTrack_Init();
}

uint8_t Path_Task10ms(void)   /* 0=进行中 1=完成 2=超时 */
{
    const s_path_seg *s = &m_path[m_seg];
    uint8_t ir = Line_ReadRaw();

    if ((HAL_GetTick() - m_seg_t0) > s->timeout_ms) { Car_Stop(); return 2; }

    if (s->type == SEG_LINE)
    {
        if (ir != 0U)                       /* 看到线 → 本段结束 */
        {
            goto seg_done;
        }
        DriveStraight(s->target_deg);
    }
    else
    {
        LineTrack_Update(ir, Attitude_GetGz());
        DriveTrack(CAR_BASE_ARC_PCT, LineTrack_GetDelta());

        if (LineTrack_IsLost())             /* 丢线 → 本段结束 */
        {
            goto seg_done;
        }
    }
    return 0;

seg_done:
    Reminder_Start(120);                    /* 每个点提示一次 */
    m_seg++;
    if (m_seg >= m_seg_cnt) { Car_Stop(); return 1; }

    m_seg_t0 = HAL_GetTick();
    YawCtrl_Reset();                        /* 新段：航向归零、积分清零 */
    LineTrack_Init();                       /* 清掉上一段残留的偏差历史 */
    return 0;
}
```

**注意**：`YawCtrl_Reset()` 在每段开头调用，让"保持当前朝向"成为默认行为——
**下一段该朝哪，由上一段结束时车头实际朝向决定**。

## 18. Q3：斜线的朝向怎么来

这是 Q3 唯一的新问题。两种做法：

### 做法 A：每段 Reset，target = 0（推荐）

```
A 点：人工把车摆成 A→C 方向（38.66°），按 Start
      → YawCtrl_Reset()，target=0，车"保持当前朝向"跑直线
→ 到 C 点看到线，进弧线（循迹，不用管朝向）
→ 弧线走完丢线，此时车头天然就是 C→B 的切线方向
      → YawCtrl_Reset()，target=0，继续跑直线
```

**优点**：永远不需要写 38.66° 这种绝对角度，也就不怕陀螺漂移。
**前提**：起跑时车要摆正（可做一条辅助对齐线，或用直尺）。

### 做法 B：绝对角度表

```c
{ SEG_LINE, END_ON_LINE,  38.66f, 8000 },   /* A→C */
{ SEG_LINE, END_ON_LINE, 141.34f, 8000 },   /* B→D */
```

**问题**：`yaw` 是相对量，每段 Reset 之后角度基准就变了，绝对角度只有"全程不 Reset"才有意义——而那样漂移会累积到十几度。
**结论：不要用 B。**

## 19. Q4：四圈，难点是"不能停"

### 19.1 三个变化

| 项 | Q3 | Q4 |
| --- | --- | --- |
| 段表 | 4 段 | **16 段**（4 圈 × 4 段） |
| 过点行为 | 停车 + 提示 | **不停车**，只提示（或干脆不提示） |
| 基础速度 | 直线 40 / 弧线 25 | **直线 70~90 / 弧线 45~55** |

### 19.2 圈计数

不用单独数圈：**段表里 16 段跑完自然就是 4 圈**。要显示圈数就：

```c
uint8_t lap = (uint8_t)(m_seg / 4U) + 1U;     /* 当前第几圈 */
```

### 19.3 时间账（这是 Q4 的真正难点）

```
总长 2029 cm，限时 40 s  →  平均 50.7 cm/s
```

电机开环最高速估算约 48 cm/s（要实测）。**余量几乎为 0**，意味着：

1. **不能停**——每次停车再起步损失约 0.5 s，4 圈 16 次就是 8 s，直接超时；
2. **弧线不能太慢**——弧线占全长 50%，弧线速度必须 ≥ 45 cm/s；
3. **段切换不能犹豫**——`LineTrack_IsLost()` 需要 100 ms 确认，这段"空跑"是浪费，
   可把 `LINE_LOST_MS` 降到 60~80 ms（代价是误判风险上升）。

### 19.4 提速的先后顺序

```
① 先把 Q3 跑稳（不撞、不切内道）
② 提速：直线 40 → 50 → 60 → 70，每次只加一档，看弧线入口有没有冲出去
③ 弧线单独提速：25 → 35 → 45
④ 最后再压 LINE_LOST_MS
```

**每提一档都要重新验证**：弧线入口是否压线、出口是否丢线、有没有切内道。

## 20. 实施顺序（建议）

```
Q2：先只跑"直线段 + 停车"，确认两个直线段的航向环没问题
    → 再加弧线段（循迹），确认丢线判据可靠
    → 拼成完整 4 段

Q3：把 Q2 的段表换成斜线表，只改数据和起点摆放
    → 验证：到达 C 点时是否真的看到线（偏多了就看不到）

Q4：Q3 段表 × 4，去掉停车，逐步提速
```

## 21. 风险清单

| 风险 | 表现 | 对策 |
| --- | --- | --- |
| 直线段跑偏，看不到 C 点 | 卡在段里直到超时 | 航向环 Kp 加大；起跑摆放要准 |
| 弧线丢线误判 | 弧线中途跳到下一段 | `LINE_LOST_MS` 别小于 60ms |
| 过点时车头没回正 | 下一段起始朝向错 | 靠弧线自然给出朝向（做法 A 的前提） |
| Q4 超时 | 40 s 跑不完 | 提速 + 减少停车 + 压缩丢线确认时间 |
| 电池掉压 | 后几圈变慢 | 每圈记录用时，掉太快就换电池 |
