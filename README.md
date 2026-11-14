# 2024 电赛 H 题：自动行驶小车

基于 **STM32F103C8T6** 的循迹小车，实现 2024 年全国大学生电子设计竞赛 H 题的四项要求。

## 一、硬件

| 模块 | 型号 / 说明 |
| --- | --- |
| 主控 | STM32F103C8T6（LQFP48，72 MHz，HSE 8 MHz ×9） |
| 电机驱动 | TB6612FNG（TIM2 双通道 PWM，20 kHz） |
| 电机 | TT 减速电机 ×4（左右各两个并联） |
| 循迹 | 四路灰度（数字量，PB11 / PB10 / PB1 / PB0） |
| 姿态 | MPU6050（软件 I2C，PB8 / PB9，与 OLED 共用总线） |
| 显示 | SSD1306 OLED（0x78） |
| 通信 | 蓝牙模块（USART1，9600 8N1） |
| 按键 | PB3 选题 / PB4 启停 / PB5 复位 |
| 提示 | 蜂鸣器 PB15、LED PC13 |

完整接线见 [Hardware.md](Hardware.md)。

## 二、软件架构

```
main.c                  调度（10ms 控制 / 20ms 按键 / 100ms 显示 / 200ms 上报）
├── 按键：Question_Choose（选题）、cur_Question（实际执行）
└── question.c          四个问题的段表状态机
    ├── yaw_ctrl.c      航向环（陀螺闭环）
    ├── line_track.c    循迹环（灰度 PD + 角速度前馈）
    └── Car.c           左右轮 PWM 输出
        ├── attitude.c  陀螺积分成航向
        └── line_sensor.c 四路灰度读取
```

分层原则：`Hardware`（外设驱动）→ `Software`（控制逻辑），依赖单向向下。

### 核心思想：路线 = 段表

一条路线 = 一串"段"，每段只有两种类型：

| 类型 | 结束判据 |
| --- | --- |
| `SEG_LINE` 直线 | 连续 2 拍（20ms）压线 |
| `SEG_ARC` 弧线 | 连续丢线 500ms |

段表结构：

```c
typedef struct{
    uint8_t type;         /* SEG_LINE / SEG_ARC              */
    int16_t turn_angle;   /* 进段前转角 ×10：+ 右转，- 左转   */
    uint16_t timeout_ms;  /* 本段超时                         */
}s_seg;
```

四道题共用一套执行代码，只换段表：

```c
static const s_seg m_tbl_q1[] = { {SEG_LINE, 0, 12000U} };
static const s_seg m_tbl_q2[] = { 直线, 弧, 直线, 弧 };
static const s_seg m_tbl_q3[] = { 斜线, 弧, 斜线, 弧 };   /* 每段带 ±38.66° 转向 */
static const s_seg m_tbl_q4[] = { Q3 × 4 };
```

### 两个控制环

| 环 | 输入 | 输出 | 用在 |
| --- | --- | --- | --- |
| 航向环 | 陀螺 yaw | 差速 % | 直线段 |
| 循迹环 | 四路灰度 | 差速 % | 弧线段 |

两者输出格式一致（`base ± delta`），所以驱动层只有两个对称函数。

## 三、编译

依赖：CMake ≥ 3.22、Ninja、arm-none-eabi-gcc、STM32CubeMX 生成的 HAL。

```bash
cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/Debug/Disnsai_24h.elf`

用 ST-Link 下载（SWD：PA13 / PA14）。

## 四、按键操作

| 按键 | 作用 |
| --- | --- |
| **PB3 KEY_CNT** | 选题 1→2→3→4→1（只选，不启动） |
| **PB4 KEY_START** | 启动 / 停止当前题 |
| **PB5 KEY_RESET** | 急停 + 清空姿态数据 |

## 五、数据上报（VOFA+）

帧格式（FireWater，9600）：

```
pwm_l,pwm_r,yaw\r\n
```

例：`45,-45,12.3`（左轮 45%、右轮 -45%、航向 12.3°）

发送走 USART1 中断，不阻塞控制循环。

## 六、参数速查

| 参数 | 值 | 位置 |
| --- | --- | --- |
| 直线 base | 60 % | `Software_Config.h` |
| 弧线 base | 40 % | `Software_Config.h` |
| 循迹 Kp / Kd / Kff | 6.0 / 2.0 / 0.35 | `Software_Config.h` |
| 差速限幅 | 32 % | `Software_Config.h` |
| 转向 PWM | 55 ± 25 | `question.c` |
| 入弧慢速 | 22 % / 500 ms | `question.c` |
| 弧线丢线判定 | 500 ms | `question.c` |

## 七、已知限制

| 项 | 说明 |
| --- | --- |
| **扭矩不足** | 四轮原地转向需要克服横向滑擦，6V 电池下地面转不动（架空正常）。建议换 2S 锂电或改两驱 + 万向轮 |
| **灰度精度** | 四路数字量，偏差只有 7 个离散值（每级 1.4cm），循迹会有轻微抖动 |
| **Q3/Q4** | 代码完整，受上述扭矩限制，实车未能完整验证 |

## 八、文档

| 文档 | 内容 |
| --- | --- |
| [Hardware.md](Hardware.md) | 接线、引脚、电源、故障对照 |
| [DESIGN.md](DESIGN.md) | 总体方案与几何 |
| [question_Learn.md](question_Learn.md) | 段表状态机详解 |
| [mpu6050_PID_Learn.md](mpu6050_PID_Learn.md) | 陀螺链路 + 航向环 PID |
| [PID_Learn.md](PID_Learn.md) | 问题驱动的调试记录 |
| [CODE_STYLE.md](CODE_STYLE.md) | 命名与代码规范 |
| `Copy/reference.c` | 四个算法模块的零依赖参考实现 |
