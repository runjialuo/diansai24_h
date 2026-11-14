# question.c 代码分析与讲解（完整逻辑链）

> 这份文件负责"每一道题怎么跑"。
> 四个问题共用**一套代码 + 四张段表**，差别只有数据。
>
> 本文按"数据从哪来 → 怎么判断 → 怎么输出"的顺序，把整条链路补完整。

---

# 一、它在项目里的位置

## 1.1 三层结构

```
┌─────────────────────────────────────────────────────┐
│  main.c                                             │
│    20ms：按键 → 改 Question_Choose / cur_Question    │
│    10ms：Control_10ms → Attitude_Update             │
│                        → QuesN_Task10ms()           │
└───────────────────────┬─────────────────────────────┘
                        ▼
┌─────────────────────────────────────────────────────┐
│  question.c（本文）                                  │
│    决定：这一拍直行 / 循迹 / 转向                     │
│    判断：这一段走完没有                              │
│    动作：切下一段                                    │
└───────┬───────────────────────────────┬─────────────┘
        ▼                               ▼
┌──────────────────┐          ┌──────────────────────┐
│ yaw_ctrl.c       │          │ line_track.c         │
│ 输入 yaw（度）    │          │ 输入四路灰度          │
│ 输出差速 delta %  │          │ 输出差速 delta %      │
└────────┬─────────┘          └──────────┬───────────┘
         ▼                               ▼
┌─────────────────────────────────────────────────────┐
│  Car.c → Motor.c → PWM.c（TIM2，20kHz）              │
└─────────────────────────────────────────────────────┘
```

## 1.2 question.c 不做什么

| 不做 | 谁做 |
| --- | --- |
| 读寄存器、算 I2C | mpu6050.c / IIC.c |
| 积分成航向 | attitude.c |
| PID 计算 | yaw_ctrl.c / line_track.c |
| 输出 PWM | Car.c → Motor.c |
| 按键扫描 | Key.c |

**它只做决策**：这一拍该"直行"还是"循迹"，这一段"结束没结束"。

---

# 二、核心模型：路线 = 段表

## 2.1 一条路线拆成若干"段"

```
Q2：A→B(直线) → B→C(弧) → C→D(直线) → D→A(弧)
     段0          段1         段2          段3
```

## 2.2 段只有两种

```c
#define SEG_LINE    0U      /* 直线：航向环保持方向 */
#define SEG_ARC     1U      /* 弧线：循迹环贴线走   */
```

## 2.3 段的结束判据是固定的

| 段类型 | 结束条件 | 物理依据 |
| --- | --- | --- |
| `SEG_LINE` | `Line_ReadRaw() != 0` 连续 2 拍 | 直线尽头是弧线的黑线起点 |
| `SEG_ARC` | `LineTrack_IsLost()` | 弧线走完自然没有线 |

**所以不需要里程、不需要编码器**——用场地的物理特征当判据。

## 2.4 段的数据结构

```c
typedef struct{
    uint8_t  type;          /* SEG_LINE / SEG_ARC      */
    int16_t  turn_angle;    /* 进本段先原地转多少度 ×10 */
    uint16_t timeout_ms;    /* 本段最多跑多久           */
}s_seg;
```

`turn_angle` 是最容易被忽略的字段——**直线与弧线的切线方向不一致时，必须先转**（详见第 6 节）。

---

# 三、状态变量

```c
static const s_seg* m_seg_tbl = 0;    // 当前在跑哪条路线（指向四张表之一）
static uint8_t  m_seg_cnt = 0;        // 这条路线共几段
static uint8_t  m_seg_idx = 0;        // 现在跑到第几段（从 0 开始）

static uint16_t m_seg_turnAngle;      // 本段要转的角度（×10）
static uint8_t  m_turnIsOk;           // 本段转向是否完成（1=完成/不需要转）
static uint32_t m_turn_t0;            // 转向开始时刻（单独计时）

static uint8_t  m_seg_hit = 0;        // 连续几拍压到线（两拍确认）
static uint8_t  m_seg_armed = 0;      // 本段是否已过"前 300ms 不判线"期
static uint32_t m_seg_t0 = 0;         // 本段开始时刻（超时判断）
```

## 3.1 分组理解

| 组 | 变量 | 回答的问题 |
| --- | --- | --- |
| **路线** | `m_seg_tbl` `m_seg_cnt` `m_seg_idx` | 跑哪条路、跑到哪一段 |
| **转向** | `m_seg_turnAngle` `m_turnIsOk` `m_turn_t0` | 这一段开头要不要转、转完没 |
| **判线** | `m_seg_hit` `m_seg_armed` | 现在压线算不算"到点" |
| **计时** | `m_seg_t0` | 这一段跑了多久、要不要超时 |

## 3.2 一次运行中它们怎么变

```
按 START
  Path_Start(m_tbl_q2, 4)
      m_seg_tbl = m_tbl_q2
      m_seg_cnt = 4
      m_seg_idx = 0
      m_seg_hit = 0，m_seg_armed = 0
      m_seg_t0  = now

每 10ms  Path_Run10ms()
  ├ 取 s = &m_seg_tbl[m_seg_idx]
  ├ 超时？      now - m_seg_t0 > s->timeout_ms
  ├ 要转？      m_turnIsOk == 0 → 走转向分支，转完置 1
  ├ 直线段？    armed 计时 → 压线计数 → hit>=2 结束
  ├ 弧线段？    LineTrack 更新 → 丢线结束
  └ 结束：
        m_seg_idx++
        m_seg_hit = 0，m_seg_armed = 0，m_seg_t0 = now
        m_seg_turnAngle = 下一段的角度
        m_turnIsOk = (角度 == 0)，m_turn_t0 = now
```

---

# 四、启动链：Path_Start 做了什么

```c
static void Path_Start(const s_seg *tbl, uint8_t cnt){
    Car_Stop();
    m_seg_tbl=tbl;  m_seg_cnt=cnt;  m_seg_idx=0U;
    m_seg_hit=0U;   m_seg_armed=0U;

    if(Attitude_IsOk()==0U){ m_seg_cnt=0; return; }   /* IMU 挂了 → 不跑 */

    Attitude_Enable();        /* ① 打开陀螺积分（RESET 之后是关闭的） */
    HAL_Delay(500);           /* ② 等 0.5s，让芯片温度稳一点 */
    MPU6050_CalibGyro(300);   /* ③ 静止采样 300 次求零偏 */
    Attitude_Reset();         /* ④ yaw / gz / 残差全部清零 */
    YawCtrl_Reset();          /* ⑤ 航向环积分、目标角清零 */
    LineTrack_Init();         /* ⑥ 循迹环状态清零 */

    m_seg_t0=HAL_GetTick();
}
```

## 4.1 为什么标定放在这里，而不是上电时

```
上电时：手可能还拿着板子 → 标出来的零偏是"手的运动"
按START：车已经放好在 A 点、静止 → 标定才准
```

这就是之前"静止时 G 显示 11.5°/s、车一直往右偏"的根因。

## 4.2 为什么每步都要 Reset

| 调用 | 清掉什么 | 不清会怎样 |
| --- | --- | --- |
| `Attitude_Reset()` | yaw、gz、残差补偿 | 上一题的角度残留，第一段方向就错 |
| `YawCtrl_Reset()` | 积分、目标角 | 上一题的积分顶着，起步就往一边偏 |
| `LineTrack_Init()` | 偏差历史、丢线计数 | 上一题判"丢线"，第一段弧线秒结束 |

> ⚠️ `Path_Start` **没有**初始化 `m_seg_turnAngle` / `m_turnIsOk` / `m_turn_t0`（见第 12 节），
> 会导致"上一题的角度残留到这一题"。

---

# 五、主循环：Path_Run10ms 全流程

```
                    ┌─────────────────┐
                    │ 取当前段 s       │
                    └────────┬────────┘
                             ▼
                 ┌───────────────────────┐
                 │ 段超时？ now-t0 > to  │──是──► Car_Stop，return 2
                 └───────────┬───────────┘
                             │否
                             ▼
                 ┌───────────────────────┐
                 │ 需要转向？ turnIsOk=0 │
                 └───────────┬───────────┘
                    是       │        否
             ┌───────────────┴──────────────┐
             ▼                              ▼
   ┌──────────────────────┐        ┌──────────────────┐
   │ 误差 |e|<2° ?        │        │ 按段类型执行      │
   │  是 → Reset，t0=now  │        └────────┬─────────┘
   │        turnIsOk=1    │                 │
   │  否 → 原地转，返回   │        ┌────────┴────────┐
   └──────────────────────┘        ▼                 ▼
                                SEG_LINE          SEG_ARC
                              武装300ms→判线      LineTrack_Update
                              连续2拍→结束        DriveLineTrack
                                                 丢线→结束
                                   └────────┬────────┘
                                            ▼
                                  ┌──────────────────┐
                                  │ 本段结束          │
                                  │ 提示一声          │
                                  │ idx++            │
                                  │ 取下一段角度      │
                                  │ 归零 + 清循迹     │
                                  └──────────────────┘
```

## 5.1 逐段代码解释

### ① 防呆

```c
if((m_seg_tbl==0)||(m_seg_idx>=m_seg_cnt)){ Car_Stop(); return 2U; }
```

没表（`Path_Start` 里 IMU 失败会把 `m_seg_cnt` 置 0）或越界 → 停车报错。

### ② 段超时

```c
if((now-m_seg_t0)>s->timeout_ms){ Car_Stop(); return 2U; }
```

这个判断在转向之前，所以"转向耗时"算进段超时里，但有 `TURN_TIMEOUT_MS`（2s）单独兜底。

### ③ 转向

```c
if(!m_turnIsOk){
    float e=((float)m_seg_turnAngle/10.0f)-Attitude_GetYaw();

    if((e>-TURN_TOL_DEG)&&e<TURN_TOL_DEG){     /* 到位 */
        m_turnIsOk=1;
        YawCtrl_Reset();                        /* 以当前朝向为新的 0° */
        m_seg_t0=now;                           /* 直行段重新计时 */
    }else{
        float d=YawCtrl_Updata10ms();           /* 用航向环算转速 */
        ...最小 PWM 处理...
        Car_SetSpeed((int8_t)d, (int8_t)(-d));  /* 原地转：左右反向 */
        if((now-m_turn_t0)>TURN_TIMEOUT_MS){ Car_Stop(); return 2U; }
        return 0U;
    }
}
```

### ④ 直线段

```c
if(m_seg_armed==0U){                            /* 武装期：不判线 */
    if((now-m_seg_t0)>=PATH_ARM_MS) m_seg_armed=1U;
    DriveStraight();
    return 0U;
}

if(ir!=0U){ if(m_seg_hit<PATH_HIT_N) m_seg_hit++; }
else      { m_seg_hit=0U; }

if(m_seg_hit<PATH_HIT_N){ DriveStraight(); return 0U; }
```

### ⑤ 弧线段

```c
LineTrack_Update(ir, Attitude_GetGz());   /* 先更新循迹环 */
DriveLineTrack();                         /* 再用最新差速  */
if(LineTrack_IsLost()==0U) return 0U;     /* 没丢线继续走  */
```

### ⑥ 换段

```c
Reminder_Start(120);                      /* 经过一个点：提示一声 */
m_seg_idx++;
if(m_seg_idx>=m_seg_cnt){ Car_Stop(); return 1U; }   /* 全部走完 */

m_seg_hit=0U;  m_seg_armed=0U;  m_seg_t0=now;

m_seg_turnAngle = m_seg_tbl[m_seg_idx].turn_angle;   /* 取下一段角度 */
m_turnIsOk = (m_seg_turnAngle == 0) ? 1U : 0U;
m_turn_t0  = now;

YawCtrl_Reset();
LineTrack_Init();
```

---

# 六、转向机制（Q3/Q4 的关键）

## 6.1 为什么需要转向

Q2 的直线与弧线**天然相切**：

```
A→B 方向 = +x，B 处弧的切线也是 +x   ✅ 不用转
C→D 方向 = −x，C 处弧的切线也是 −x   ✅ 不用转
```

Q3 的斜线与弧线**不相切**：

| 位置 | 车头实际方向 | 下一段需要 | 差 |
| --- | --- | --- | --- |
| A 出发 | +x（摆放） | A→C 斜线 38.66° | **38.66°** |
| 到 C | 38.66° | 弧 C→B 入口切线 0° | **−38.66°** |
| 到 B | 180°（正对 A） | B→D 斜线 141.34° | **−38.66°** |
| 到 D | 141.34° | 弧 D→A 入口切线 180° | **+38.66°** |

**"车到 B 点后一定对着 A"**——这是最容易忽略的一点：弧线出口方向由圆的几何决定，不一定是下一段要走的方向。

## 6.2 怎么转：原地转

```c
Car_SetSpeed(d, -d);      /* 左右轮反向 → 车绕中心旋转 */
```

- 不往前走，**转角准**，不会切到角外；
- 转 38.66° 约 0.4~0.6 秒；
- 转完 `YawCtrl_Reset()`，把当前朝向重定为 0°，**误差不累积**。

## 6.3 四个参数

| 宏 | 值 | 作用 |
| --- | --- | --- |
| `TURN_TOL_DEG` | 2.0 | 误差小于 2° 就算到位 |
| `TURN_MIN_PCT` | 12.0 | 最小转向 PWM（否则轮子不转） |
| `TURN_TIMEOUT_MS` | 2000 | 转向最长 2 秒，超时停车 |
| `turn_angle` 单位 | ×10 | 386 = 38.6° |

## 6.4 为什么需要 `TURN_MIN_PCT`

航向环是 PID：误差 2° 时输出只有 `Kp×2 = 2.4%`，**远低于轮子的启动死区**（约 10~12%）。

不补最小 PWM 的话，车会停在目标角前面 2~3° 处，永远"到位不了"。

```c
if(d>=0.0f && d < TURN_MIN_PCT)  d = TURN_MIN_PCT;
if(d<0.0f  && d > -TURN_MIN_PCT) d = -TURN_MIN_PCT;
```

---

# 七、四张段表

```c
/* Q1：A→B 直线，看到弧线起点停车 */
static const s_seg m_tbl_q1[]={
    {SEG_LINE, 0, 12000U},
};

/* Q2：直线 → 弧 → 直线 → 弧（弧出口天然对齐，不用转） */
static const s_seg m_tbl_q2[] = {
    {SEG_LINE, 0, 10000U},
    {SEG_ARC,  0,  9000U},
    {SEG_LINE, 0, 10000U},
    {SEG_ARC,  0,  9000U},
};

/* Q3：斜线 → 弧 → 斜线 → 弧（每段都要转） */
#define Q3_SEG_LIST                  \
    {SEG_LINE, 386, 12000U},         \
    {SEG_ARC,  -386, 9000U},         \
    {SEG_LINE, -386, 12000U},        \
    {SEG_ARC,  386, 9000U}

static const s_seg m_tbl_q3[]={ Q3_SEG_LIST };

/* Q4：Q3 跑 4 圈 */
static const s_seg m_tbl_q4[]={
    Q3_SEG_LIST, Q3_SEG_LIST, Q3_SEG_LIST, Q3_SEG_LIST,
};
```

## 7.1 角度符号怎么确定

约定：`yaw` 顺时针为正。

```
 386 = 顺时针转 38.6°
-386 = 逆时针转 38.6°
```

**实车验证**：架空 → 按 START → 看车往哪边转。转反了就把表里所有 386/−386 对调。

---

# 八、和 main.c 的配合

## 8.1 两个状态变量

| 变量 | 谁改 | 含义 |
| --- | --- | --- |
| `Question_Choose` | KEY_CNT | 选中的题目 1~4（屏幕显示它） |
| `cur_Question` | KEY_START | 正在执行哪一题，0 = 未启动 |

## 8.2 按键

```c
if(ev & KEY_CNT){  cur_Question=0U; Car_Stop(); Question_Choose=Question_Choose%4+1; }
if(ev & KEY_START){
    if(cur_Question==0U){
        cur_Question=Question_Choose;
        switch(cur_Question){ case 1: Ques1_Start(); ... }
    } else { cur_Question=0U; Car_Stop(); }
}
if(ev & KEY_RESET){ cur_Question=0U; Car_Stop(); Attitude_Disable(); Attitude_Reset(); }
```

## 8.3 10ms 推进

```c
switch(cur_Question){
  case 1: if(Ques1_Task10ms()!=0U){ cur_Question=0U; Car_Stop(); } break;
  ...
}
```

---

# 九、两条信号链

## 9.1 直线：陀螺 → 航向 → 差速 → PWM

```
MPU6050（14 字节 I2C 读）
   │ gz
   ▼
attitude.c：yaw += (gz − bias) × dt
   │ yaw、gz
   ▼
yaw_ctrl.c：delta = Kp·e + ΣKi·e − Kd·gz
   │ delta %
   ▼
DriveStraight：l = base + delta，r = base − delta
   ▼
Car_SetSpeed → Motor → TIM2 PWM
```

## 9.2 弧线：灰度 → 偏差 → 差速 → PWM

```
四路灰度（PB11/PB10/PB1/PB0）
   │ 4 位掩码
   ▼
line_sensor.c：Line_ReadRaw()
   ▼
line_track.c：err = Σ(w·压线)/数量
              delta = Kp·err + Kd·Δerr + Kff·gz
   │ delta %
   ▼
DriveLineTrack：l = base + delta，r = base − delta
   ▼
Car_SetSpeed → Motor → PWM
```

**两条链的输出格式一样**（都是"差速 %"），所以两个 Drive 函数结构完全对称，只换数据来源。

---

# 十、参数总表

| 参数 | 值 | 位置 | 作用 |
| --- | --- | --- | --- |
| `CAR_BASE_STRAIGHT_PCT` | 60 | Software_Config.h | 直线 PWM |
| `CAR_BASE_ARC_PCT` | 40 | Software_Config.h | 弧线 PWM |
| `PATH_ARM_MS` | 300 | question.c | 直线段开头不判线的时间 |
| `PATH_HIT_N` | 2 | question.c | 压线确认拍数（20ms） |
| `TURN_TOL_DEG` | 2.0 | question.c | 转向到位容差 |
| `TURN_MIN_PCT` | 12.0 | question.c | 转向最小 PWM |
| `TURN_TIMEOUT_MS` | 2000 | question.c | 转向超时 |

---

# 十一、现象 → 原因 → 处理

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 按 START 车不动 | `Attitude_IsOk()==0`，`m_seg_cnt` 被置 0 | 看屏幕 IMU 状态、查接线 |
| 起步 0.3 秒就"完成" | 起步位置压在弧线末端 | 加大 `PATH_ARM_MS` |
| 转反了 | `turn_angle` 符号与实车相反 | 表里正负对调 |
| 转不过去、2 秒超时 | 陀螺没数据 / `TURN_MIN_PCT` 太小 | 查 IMU；加大最小 PWM |
| 停在离目标角 2~3° 处不动 | 没补最小 PWM | 确认 `TURN_MIN_PCT` 生效 |
| 直线段跑飞 | 航向环没拉住 / 摆放歪 | 调 `YAW_KP`，摆正车 |
| 弧线提前结束 | 丢线误判 | 加大 `LINE_LOST_MS` |
| 弧线走不完 | 探头没压到线 / 速度太快 | 降 `CAR_BASE_ARC_PCT`，调探头高度 |
| 每段都超时 | 表的 `type` 写反 | 检查段表 |
| 上一题的角度残留 | `Path_Start` 没初始化转向变量 | 见第 12 节 |

---

# 十二、已知问题（要修）

| # | 位置 | 问题 | 后果 |
| :--: | --- | --- | --- |
| 1 | `Path_Start` | 没有初始化 `m_seg_turnAngle` / `m_turnIsOk` / `m_turn_t0` | 上一题的角度残留，新题第一段会乱转 |
| 2 | 变量定义 | `m_seg_turnAngle` 是 `uint16_t` | 存 −386 会变成 65150 → 6515°，转向直接跑飞 |
| 3 | 注释 | `m_seg_armed` 注释写成"mpu6050 的预热时间" | 误导，实际是"前 300ms 不判线" |
| 4 | 第 58~60 行 | `QUES1_*` 宏已无人使用 | 只是冗余 |

## 修法

**① `Path_Start` 补三行**

```c
    m_seg_turnAngle = tbl[0].turn_angle;
    m_turnIsOk      = (m_seg_turnAngle == 0) ? 1U : 0U;
    m_turn_t0       = HAL_GetTick();
```

**② `m_seg_turnAngle` 改 `int16_t`**

```c
static int16_t m_seg_turnAngle;
```

**③ 注释纠正**

```c
static uint8_t m_seg_armed=0;   //本段是否已过"前 300ms 不判线"期
```

---

# 十三、怎么扩展

## 13.1 加第 5 题

```c
static const s_seg m_tbl_q5[] = { ... };

void    Ques5_Start(void)    { Path_Start(m_tbl_q5, sizeof(m_tbl_q5)/sizeof(m_tbl_q5[0])); }
uint8_t Ques5_Task10ms(void) { return Path_Run10ms(); }
```

再在 `question.h` 加声明、`main.c` 两个 switch 各加一个 case、按键取模改成 `%5`。

## 13.2 改路线

只改表：

```c
static const s_seg m_tbl_q3[]={
    {SEG_ARC,  0, 9000U},        /* 先走弧 */
    {SEG_LINE, 386, 12000U},     /* 再走斜线 */
    ...
};
```

## 13.3 按段设速度

给 `s_seg` 加字段：

```c
typedef struct{
    uint8_t  type;
    int16_t  turn_angle;
    uint16_t timeout_ms;
    uint8_t  base_pct;      /* 本段基础速度 */
}s_seg;
```

`DriveStraight(base_pct)` / `DriveLineTrack(base_pct)` 改成带参数，Q4 就能"直线快、弧线慢"。

---

# 十四、一页总结

```
① 一条路线 = 一串"直线/弧线"的段
② 直线看到线结束，弧线丢线结束——不需要里程
③ 直线用航向环，弧线用循迹环，输出都是"差速 %"
④ 段与段方向不连续时，先原地转到目标角
⑤ 每段开头清状态：转向完成 → YawCtrl_Reset → 航向从 0 开始
⑥ 四个问题共用一套代码，只换段表
⑦ 关键判据：武装 300ms + 连续 2 拍压线 + 丢线 100ms
```
