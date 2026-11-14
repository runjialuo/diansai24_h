# 代码规范（CODE_STYLE）

> 本文件是本工程的**命名与结构约定**，新代码必须遵守；改老代码时顺便对齐。
> 修改规则要先改这份文件，再改代码。

---

## 1. 命名总则

### 1.1 前缀按"作用域"分配

| 对象 | 前缀 | 例子 | 说明 |
| --- | --- | --- | --- |
| **类型**（struct / enum / union） | `s_` | `s_mpu6050_raw`、`s_pid_param` | 不用 `_t` 后缀 |
| **文件内静态变量** | `m_` | `m_yaw`、`m_gz_bias` | module 私有 |
| **跨文件全局变量** | `e_` | `e_ctrl_tick` | external，尽量少用 |
| **局部临时变量** | `t_` | `t_tmp`、`t_swap` | 只在"暂存/交换"时用，普通局部不必加 |
| **宏 / 常量** | 模块名大写 | `LINE_KP`、`YAW_IMAX`、`MPU6050_ADDR_W` | 全部大写 + 下划线 |

> **为什么不用 `_t`**：`_t` 是 POSIX 保留后缀（`pid_t`、`time_t`、`size_t`），
> 自定义类型用它与系统头文件有撞名风险。

### 1.2 函数：`模块_动词_对象`

```
Car_SetSpeed()          车 设 速度
MPU6050_WriteReg()      MPU6050 写 寄存器
LineTrack_Update()      循迹 更新
Yaw_Update10ms()        航向环 更新（周期 10ms）
Attitude_GetYaw()       姿态 取 航向
MPU6050_CalibGyro()     MPU6050 标定 陀螺
```

**动词白名单**（不要自创）：

| 动词 | 含义 |
| --- | --- |
| `Init` | 初始化（上电一次） |
| `Set` | 设置一个目标/参数 |
| `Get` | 读取一个值（无副作用） |
| `Read` | 从硬件读 |
| `Write` | 往硬件写 |
| `Update` | 周期性地计算/推进 |
| `Start` / `Stop` | 启动 / 停止某个过程 |
| `Reset` | 复位状态 |
| `Is` / `Has` | 返回布尔（`IsLost`、`HasLine`） |
| `Calc` | 纯计算，不改状态 |
| `Probe` | 探测设备是否存在 |

**反例**：

| ❌ 差 | ✅ 好 | 原因 |
| --- | --- | --- |
| `doSpeed()` | `Car_SetSpeed()` | 没归属 |
| `motor_speed_set_value()` | `Car_SetSpeed()` | 词序乱、冗长 |
| `handleIR()` | `Line_ReadRaw()` | 看不出动作 |
| `MPU6050()` | `MPU6050_Init()` | 函数名像类型 |

### 1.3 单位后缀（防止"厘米当毫米"这类事故）

变量名末尾带上单位：

| 后缀 | 单位 | 例子 |
| --- | --- | --- |
| `_ms` | 毫秒 | `timeout_ms` |
| `_us` | 微秒 | `delay_us` |
| `_deg` | 度 | `yaw_deg` |
| `_dps` | 度/秒 | `gz_dps` |
| `_mm` | 毫米 | `dist_mm` |
| `_cm` | 厘米 | `err_cm` |
| `_pct` | 百分比 | `pwm_pct` |
| `_cnt` | 计数 | `lost_cnt` |

> 约定：**内部计算一律用定点的“最小单位”**（0.1°、0.1 °/s、mm），
> 只在和硬件/人交互时换算成 °、cm、%。

---

## 2. 文件与目录

```
Core/                  CubeMX 生成，只写 USER CODE 区
User/
├── Hardware/          与芯片/外设直接打交道
│   ├── Inc/  Src/     IIC / OLED / MPU6050 / LineSensor / Motor / PWM / Beep / LED / Serial / Delay
└── Software/          控制逻辑，不知道寄存器
    ├── Inc/  Src/     attitude / line_track / ctrl_yaw / Q1 / Q2 / config
```

**依赖方向单向向下**：`Software` 可以调 `Hardware`，反之禁止。

| 规则 | 说明 |
| --- | --- |
| 每个 `.c` 第一行 include 自己的 `.h` | 强制头文件自足 |
| 头文件只放声明 | 不放变量定义（会多重定义） |
| 只 include 用到的东西 | 不要"万能头" |
| 一个 `.c` 只解决一个问题 | 超过 300 行考虑拆分 |

---

## 3. 数据类型

| 规则 | 说明 |
| --- | --- |
| 用 `<stdint.h>` 定长类型 | `int16_t` 而不是 `int` |
| 物理量用 int | `int16_t yaw_deg10`（0.1° 分辨率） |
| float 只用于换算/积分 | 避免在中断里大量浮点 |
| 结构体放"一帧数据" | 如 `s_mpu6050_raw` |

---

## 4. 注释

### 4.1 文件头：写"这个文件解决什么问题"

```c
/*==============================================================
 *  循迹控制（Software 层）
 *
 *  输入：四路灰度状态 + 当前 Z 轴角速度
 *  输出：差速 delta（%）
 *==============================================================*/
```

### 4.2 函数头：一句话说清"做什么 + 什么时候调"

```c
/* 每 10ms 调用一次；返回差速 % */
float Yaw_Update10ms(void);
```

### 4.3 禁止魔法数字

```c
/* ❌ */ if (err > 4.0f) err = 4.0f;
/* ✅ */ if (err > LINE_ERR_MAX) err = LINE_ERR_MAX;   /* config.h */
```

---

## 5. 变量与状态

| 规则 | 说明 |
| --- | --- |
| 默认 `static` | 只有跨文件才用 `e_`（extern 访问） |
| ISR 与主循环共享的变量加 `volatile` | 如 `e_flag_10ms` |
| 状态机用 `enum` 或明确的 step 变量 | 不要用一堆 bool 拼 |
| 每个模块提供 `Xxx_Init()` | 上电调用一次，清空所有状态 |

---

## 6. 迁移记录（本工程已按此规范改名）

| 原 | 现 | 原因 |
| --- | --- | --- |
| `mpu6050_raw_t` | `s_mpu6050_raw` | `_t` 是 POSIX 保留后缀 |
| `s_yaw` / `s_gz` / `s_ok` … （attitude.c） | `m_yaw` / `m_gz` / `m_ok` … | `s_` 让给类型 |
| `s_err` / `s_delta` / `s_weight` … （line_track.c） | `m_err` / `m_delta` / `m_weight` … | 同上 |
| `s_gz_bias` | `m_gz_bias` | 同上 |
| `reminder_ms` | `m_reminder_ms` | 补作用域前缀 |
| `g_ctrl_tick` / `g_ui_tick` | `e_ctrl_tick` / `e_ui_tick` | 全局前缀统一为 `e_`（external） |

---

## 7. 待办（建议但未执行）

| 项 | 说明 |
| --- | --- |
| `Sofaware_Config.c/h` → `Software_Config.c/h` | 文件名拼写错误（Sofaware → Software） |
| `MyReminder()` → `Reminder_Play()` | 动词不规范 |
| `Q1/Q2` 的段号用 `enum` 而非 `uint8_t` | 可读性 |
| 给 `e_ctrl_tick` 换成 TIM1 中断里的 `e_flag_10ms` | 节拍更稳 |

---

## 8. 一句话总结

> **类型 `s_`、静态 `m_`、全局 `e_`、临时 `t_`；
> 函数「模块_动词_对象」；变量名带单位；魔法数字进 config.h。**
