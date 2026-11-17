# Racing Car — STC32G144K 摄像头循迹车

基于 STC32G144K246 与逐飞 MT9V03X 摄像头的竞速智能车工程，Keil C251（MCS-251）工具链，裸机前后台 + 5 ms 控制中断。

当前分支 `version-1.4`，基线提交 `57222a3`（`version-1.3` 末次提交）。

---

## 代码来源声明

**自 `version-1.4` 起，本工程 `project/` 目录下的应用层代码全部由本队自行编写与维护。**

具体范围：

| 模块 | 文件 | 来源 |
|---|---|---|
| 图像链路 | `project/code/Image.c/.h` | 本队自研 |
| 控制链路 | `project/code/Motor.c/.h` | 本队自研 |
| 元素状态机 | `Cross.c` / `LRound.c` / `RRound.c` / `BendEntry.c` | 本队自研 |
| 发车与安全 | `Stop.c` / `Track.c` | 本队自研 |
| 调度主循环 | `project/user/main.c` | 本队自研 |
| 离线校验 | `tests/*.py` | 本队自研 |

应用层不含任何外部现成赛车的算法实现片段：图像处理（局部阈值爬线、等弧长重采样、余弦拐点、双侧中线融合）、纯跟踪与速度规划、元素状态机、异常检测，均为本队按本项目硬件与赛道标定的实现，设计依据见 `docs/TRACK_RECOGNITION_DESIGN.md`。

（文件级的上游派生历史——中断入口、工程文件组织——已在 `NOTICE` 中逐条列明，不涉及上述算法实现。）

**必须同时说明的第三方部分**（见 `NOTICE` 与 `LICENSE`，GPL-3.0 要求保留，不得删除）：

| 组件 | 位置 | 来源 | 许可 |
|---|---|---|---|
| `zf_` 驱动库 / 设备库 | `libraries/` | 成都逐飞科技 STC32G144K 开源库 V3.2.5 | GPL-3.0，保留原文件头 |
| `sqrt_int()` / `atan2_int()` | `libraries/zf_common/zf_common_function.c` | 上游不提供，由本队追加（已在 NOTICE 列明） | GPL-3.0 |
| 中断入口与向量表 | `project/user/isr.c` / `isr_extended_*.c` / `isr_extended_vectors.A51` | 结构沿用逐飞中断模板，内容按 C251 向量号 0~31 限制重写 | GPL-3.0 |
| 工程文件与启动代码 | `project/mdk/seekfree.uvproj`、`libraries/zf_common/START251.A51` | 逐飞模板，仅增删文件列表 | 原样引用 |
| 芯片寄存器头 | `libraries/zf_common/stc32g144k246.h` | STC 官方 | 原样引用 |

本仓库整体仍以 **GNU General Public License v3.0** 分发。再分发须保留 `LICENSE` / `NOTICE` 全文、版权声明，并标明修改内容。

---

## 版本演进

| 版本 | 分支 | 主要内容 |
|---|---|---|
| 1.0 | `version1.1` 之前 | 从 C-model 模板起步：双缓冲采集、扩展中断向量处理、发车 / 里程 / 定点停车 |
| 1.1 | `version1.1` | 实车验证的稳定基线 |
| 1.2 | `version-1.2` / `version-1.2.1` | 出界保护与赛道识别完善 |
| 1.3 | `version-1.3` | 弯道入口识别、自适应转向增益、轨迹偏置标定 |
| **1.4** | `version-1.4` | 元素判据修复、前瞻取法可选、出界判据修正、编译产物与离线校验补齐（见第 12 节） |

---

## 1. 硬件

| 项目 | 型号 / 参数 |
|---|---|
| MCU | STC32G144K246，96 MHz，256 KB Flash / 128 KB XRAM |
| 摄像头 | 逐飞 MT9V03X，188 × 120 灰度，LCM_DMA 采集 |
| 舵机 | 200 Hz PWM，中值 `steerMid = 2975`，机械修正 `STEER_TRIM = -58` |
| 电机 | 双直流减速电机，17 kHz PWM，AB 相编码器正交解码 |
| 调试 | USB CDC 虚拟串口，`printf` 输出 |
| 安装 | 摄像头高度 20 cm，俯角 45° |

### 1.1 引脚

| 功能 | 引脚 | 外设 | 方向 |
|---|---|---|---|
| 右轮 PWM | P76 | PWMB CH3 | 输出 |
| 右轮 DIR | P77 | GPIO | 输出 |
| 左轮 PWM | P74 | PWMB CH1 | 输出 |
| 左轮 DIR | P75 | GPIO | 输出 |
| 右编码器 A / B | P60 / P62 | PWMA CH1P / CH2P | 输入 |
| 左编码器 A / B | P40 / P42 | PWMC CH1P / CH2P | 输入 |
| 舵机信号 | PA4 | PWME CH3P | 输出 |
| 状态灯 | P52 | GPIO | 输出，低电平点亮 |
| 拨码 SW1 / SW2 | PB0 / PB1 | GPIO 上拉 | 输入 |
| 按键 B2 / B3 / B4 | PB2 / PB3 / PB4 | GPIO 上拉 | 输入 |
| 摄像头 D0~D7 | P20~P27 | GPIO | 输入 |
| 摄像头 SCL / SDA | PB6 / PB7 | 软件 IIC | 双向 |
| FIFO VSY | P33 | INT1 | 输入 |
| FIFO 控制 | P37 / P41 / P73 / P72 / P43 | GPIO | 输出 |
| DMA 占用 | P45 / P36 | — | 不可复用 |
| USB | P30 / P31 | USB 全速设备 | 双向 |

## 2. 目录

```
project/
  code/
    Image.c/.h        快照、视野窗口、起点搜索、爬线、重采样、拐点
    Motor.c/.h        纯跟踪、舵机换算、差速、双轮 PID、故障检测
    Cross.c/.h        十字状态机与补线
    LRound.c/.h       左环岛五阶段状态机
    RRound.c/.h       右环岛五阶段状态机
    BendEntry.c/.h    弯道入口过渡区（出界宽容 + 转角保持）
    Stop.c/.h         发车、里程标定、定点停车
    Track.c/.h        赛道存在性判定与状态灯
  user/
    main.c            上电流程、按键、主循环、元素派发、中线融合
    isr.c             向量号 0~31 的中断入口
    isr_extended_0.c  P0~P9、DMA UART、DMA_LCM、TM11
    isr_extended_1.c  其余扩展中断
    isr_extended_vectors.A51  扩展中断跳转表
  mdk/                uVision 工程与编译产物
libraries/            逐飞库（禁止改算法；本队仅在函数式工具与头文件包含处有改动，见 NOTICE）
tests/                Python 离线校验脚本
docs/                 设计与评估文档
```

## 3. 构建与烧录

```powershell
$uv   = 'D:\Keil_v5\UV4\UV4.exe'
$proj = 'f:\项目文件夹\Racing Car C-model\main\project\mdk\seekfree.uvproj'
# -r 全量重建 / -b 增量构建；-j0 关闭并行
Start-Process $uv -ArgumentList "-r `"$proj`" -j0 -o `"$env:TEMP\build.log`"" -Wait
```

工程路径含空格，**命令行里必须给路径加引号**，否则 `UV4` 直接返回 15（读不到工程）。

产物：`project/mdk/out_file/SEEKFREE.hex`，同时生成 `SEEKFREE.map` 与 `.lnp`。

本版本基线结果：

```
Program Size: data=101.5  edata+hdata=8192  xdata=51892  const=3761  code=83514
".\out_file\SEEKFREE" - 0 Error(s), 1 Warning(s)
```

唯一警告是逐飞库自带的 `zf_device_mt9v03x.c(138): C183 dead assignment eliminated`，属第三方代码，不做修改。

## 4. 上电与操作

| 步骤 | 现象 |
|---|---|
| 上电 | 状态灯闪 3 下，USB CDC 开始输出 `ODO=` |
| 摄像头初始化 | 失败时状态灯 500 ms 慢闪并停住（最多重试 3 s） |
| 舵机自检 | 机械中位 → 一侧 30° → 另一侧 30° → 回中，约 1.5 s |
| 选档 | 读 PB0 / PB1，从 8 档里取 1 / 3 / 5 / 7 档 |
| 待发车 | 状态灯按编码器状态闪烁 |

| 按键 | 功能 |
|---|---|
| B2 | 发车（`RACE_IDLE` / `RACE_STOPPED` 时可发） |
| B3 | 任意状态紧急停车 |
| B4 短按 | 停车状态下舵机左右各摆 30° |
| B4 长按 2 s | 待发车状态下把当前里程记为一圈 |

`CALIB_REQUIRED = 0`：不标定也能发车，此时 `raceStart()` 把 `lapPulse` 设为 `0x7FFFFFFF`，不会因里程自动刹停，只能靠 B3 手动停。

## 5. 图像链路

```
DMA 写 mt9v03x_image
  → DMA 完成中断 mt9v03x_frame_snapshot() 搬 22.5 KB 到 mt9v03x_frame，置 frameBusy
  → 主循环 frameBusy == 1
      cutViewRegion()               视野窗口：只保留行 10~99、列 34~153，四周置暗
      getLStart / getRStart         行 95 → 13，列 36 → 151，车身行以下用窄列
      getLBorder / getRBorder       四邻域爬线，局部阈值每 2 步重算，turn < 8
      getGodRoadLine                当前旁路 reflection 查表（USE_REFLECTION = 0）
      filterRoad                    因果平均 (前 + 今 + 1) >> 1
      resampleRoad(dis = 3)         等弧长重采样
      findCorner                    夹角余弦找拐点，裁短点数
      buildCenterLine               双侧配对取中点 / 单侧按实测半宽外推
      filterRoad + resampleRoad(2)  中线平滑与等距化，首点锚到窗口下沿
```

设计细节见 `docs/TRACK_RECOGNITION_DESIGN.md`。

## 6. 控制链路

```
5 ms 控制中断 pitHandler
  编码器读数 → faultScan → 出界消抖
  → 差速分配 → 左右增量式 PID → pwm_set_duty
  → steerWrite(steerMid + STEER_TRIM + steerPIDCalculate(&steerPid, duty))
```

| 环节 | 实现 |
|---|---|
| 前瞻点 | `getAimPos()` = `c3 + currentSpeed / 40`，夹 `c3 ~ c3 + SPEED_LIMIT/40` |
| 取点方式 | `PP_ARC_MODE = 0`：按下标 `pos`，并夹 `count-1-12`；`= 1`：按弧长累计，线短时沿末端切线外推 |
| 纯跟踪 | `d = atan2_int(2·L·dy, dx² + dy²)`，`L = PP_LOOKAHEAD_PX = 18`，`dx = 99 - 点行 + 18`（下限 30），`dy = 94 - 点列` |
| 远端前馈 | `dy = dyAim + (dyFar - dyAim) / 3`，远端点在前瞻点之后 12 个点距（约 24 像素） |
| 舵机换算 | `currentDuty = d × c1 / 20`，夹 ±395；`d` 先夹 ±30 |
| 舵机环 | `steerPid` Kp 100 / Ki 0 / Kd 100，输出夹 ±395 |
| 曲率限速 | `r = 20000 / tan(abs(d))`，`v = √(r·c4)`，`target = min(v, c5)` |
| 弯道减速 | `bend = abs(dyFar - dyAim) + abs(d)`，比例 `100 - (bend-4)×3` 夹到 40~100，`bendHold` 记住本弯最严值 |
| 弯道增益 | 减速状态下角度放大 105%~155%（强度 15 起每单位 +5%） |
| 差速 | `diff = setSpeed × 8 / R`，夹 ±`setSpeed/2`；`R = 20000 / tan(abs(d))` 带符号 |
| 电机环 | 增量式，Kp 1000 / Ki 50 / Kd 0，输出 ±10000；刹车段负向限幅收窄到 -1500 |

## 7. 元素识别

| 元素 | 入口判据 | 状态机 |
|---|---|---|
| 十字 | 两侧点数都 < 14，任一侧有拐点，且中线点数 < 13 | `CROSS_READY` / `CROSS_IN`，补线打点后中线 > 24 退出 |
| 左环岛 | 右边界原始点数 > 16 且末点伸到行 ≤ 18，左边界点数 < 4 | 五阶段，`centerLineDistance` 13 / 27 / 11 / 18 |
| 右环岛 | 镜像（左边界 > 16，右边界 < 4） | 11 / 25 / 11 / 18 |
| 弯道入口 | 强度 ≥ 10 且转角绝对值 ≥ 2°，连续 2 帧同向 | `STRAIGHT` → `APPROACH` → `COMMITTED` → `RECOVER` |

环岛入口直接用 `LRawCount` / `RRawCount`（拐点裁短前的原始点数）：
`LRoadCount` 有拐点时被覆盖成 `LCorner`，没拐点时才等于原始点数，所以 `LRoadCount < 4`
同时覆盖“内侧被拐点裁短到 3 点”和“内侧整条消失只剩 0~2 点”两种工况，
不能再额外要求“拐点非零”——那会把后一种（主要工况）排除掉。

元素开关在 `main.c`：`ENABLE_CROSS`、`ENABLE_ROUND`，当前都为 1。

## 8. 安全保护

| 保护 | 触发条件 | 动作 |
|---|---|---|
| 出界 | 5 个采样点全暗 **且** 已 200 ms 没看到赛道，连续 3 帧成立后再经 3 拍消抖 | 停车，`ERR` 置 0x0400 |
| 编码器掉线 | 目标 > 60、输出 > 2000、该轮 300 ms 计数 < 3 | 停车，`ERR` 置 0x0002 / 0x0004 |
| PID 饱和 | 输出 ≥ 96% 上限且速度 < 目标一半，持续 1 s | 停车，`ERR` 置 0x0100 |
| 正反馈 | 正向输出 > 1500 且编码器 < -5，持续 500 ms | 停车，`ERR` 置 0x0200 |
| 帧看门狗 | 1 s 无新帧 | `raceStopNow()` |
| 元素超时 | 单个元素阶段超过 8 s，或整个元素超过 20 s | 强制回 `FORWARD` 并复位 |
| 环岛误判兜底 | `READY` 停留 1.2 s 仍等不到缺口 | 回 `FORWARD`，解除 50% 限速 |
| 发车宽限 | 发车后 300 ms | 不判出界 |

出界判据要求三条信号同时成立，且都要持续（`main.c` 的 `OUT_*` 宏）：

1. 5 个采样点全部偏暗（`outSamplePt`，每点取 3×3 块均值）；
2. 距最近一次“看到赛道”已超过 `OUT_BLIND_MS = 200` ms；
   “看到赛道”用**拐点裁短前**的 `LRawCount` / `RRawCount` 判定——用裁短值会在十字/环岛期间恒不成立；
3. 上述结论连续 `OUT_CONFIRM_FRAMES = 3` 帧，之后控制中断再做 3 拍（15 ms）消抖。

采样点全部放在车身顶边 `BODY_ROW = 80` 以上（行 66~76）并收进赛道内侧（列 74~114）：
摄像头视场远大于赛道，画面边缘与远端本来就是场地外暗区，早期版本把 3 个点放在行 84~88
（车身遮挡区）、2 个点放在列 47 / 141（常在线外），等于长期白送暗点，这是直道偶发停车的主因。

单侧还有边界时不停车：画面里仍有可循迹的线，交给纯跟踪把车拉回。

## 9. 参数表

`speedSet[8][4]`，列为 `{c1, c3, c4, c5}`：

| 档 | c1 | c3 | c4 | c5 |
|---|---|---|---|---|
| 1 | 260 | 26 | 2400 | 400 |
| 2 | 200 | 22 | 2000 | 300 |
| 3 | 210 | 23 | 2000 | 320 |
| 4 | 220 | 23 | 2000 | 340 |
| 5 | 230 | 24 | 2000 | 360 |
| 6 | 240 | 24 | 2000 | 380 |
| 7 | 250 | 25 | 2000 | 400 |
| 8 | 260 | 25 | 2000 | 400 |

拨码组合 `(SW2, SW1)` 映射到 1 / 3 / 5 / 7 档：`00 → 1 档`、`01 → 3 档`、`10 → 5 档`、`11 → 7 档`（0 为拨码断开）。

| 宏 | 值 | 说明 |
|---|---|---|
| `STEER_DUTY_LIMIT` | 395 | 舵机相对中值的最大偏移（1 duty = 0.5 µs） |
| `STEER_DUTY_DIV` | 20 | `duty = d × c1 / 20` |
| `STEER_TRIM` | -58 | 舵机机械中值修正，停车、自检、初始化三处统一叠加 |
| `TRACK_BIAS_PX` | 0 | 纯跟踪瞄准点横向偏置 |
| `MOTOR_DIFF_TRIM` | 0 | 左右轮目标速度差补偿 |
| `PP_LOOKAHEAD_PX` | 18 | 纯跟踪等效轴距（转角分母里的 L），唯一调参入口 |
| `PP_ARC_MODE` | 0 | 前瞻取法：0 按下标，1 按弧长（见第 12 节） |
| `PP_ARC_EXT_MAX` | 24 | 弧长模式下远端点沿切线外推的上限 |
| `PP_DX_MIN` | 30 | 纯跟踪纵向距离下限，防止贴脸点打满舵 |
| `STEER_REF_ROW` | `VIEW_NEAR_ROW`(99) | 纵向距离的参考行；改回 120 会拉长前瞻、转角变柔 |
| `SPEED_LIMIT` | 400 | 目标速度上限（脉冲 / 5 ms） |
| `MOTOR_DUTY_LIMIT` | 10000 | 双轮 PID 输出上限（满量程） |
| `MOTOR_BRAKE_DUTY` | 1500 | 刹车段允许的最大反向占空比 |
| `BEND_SPEED_CAP` | 200 | 减速状态的硬顶 |
| `BEND_SLOW_START` / `BEND_SLOW_GAIN` / `BEND_SLOW_MIN` | 4 / 3 / 40 | 弯道减速曲线 |
| `BEND_STEER_PCT` / `BEND_STEER_MAX` | 105 / 155 | 弯道转向增益区间 |
| `BORDER_IN_SPEED_RATIO` | 50 | 元素内速度比例 |
| `SERVO_TEST_SWING` / `SERVO_TEST_STEP` | 495 / 33 | 自检摆幅与步长（495 = 33×15，三段都精确落在端点） |
| `MOTOR_DIR_FORWARD` | 0 | 低电平前进 |
| `MOTOR_KICK_TICKS` | 0 | 起跑助推关闭 |
| `MOTOR_OPEN_LOOP` / `LOW_SPEED_TEST` | 0 / 0 | 闭环、比赛模式 |
| `USE_REFLECTION` | 0 | 旁路 reflection 查表（安装已变，旧表不适用） |
| `EDGE_DIFF` / `EDGE_RATIO` | 12 / 7 | 边界梯度双条件 |
| `HIDE_ROW` / `BODY_ROW` | 90 / 80 | 车身遮挡行 |
| `CROSS_MAX` | 14 | 十字入口两侧点数上限 |
| `CROSS_ENTER_COUNT` / `CROSS_EXIT_COUNT` | 13 / 24 | 十字进入 / 退出中线点数门槛 |
| `ROUND_MIN_PTS` / `ROUND_FAR_ROW` | 16 / `VIEW_FAR_ROW+8` | 环岛入口：外侧边界点数下限、末点必须伸到的行 |
| `ROUND_FAR_MAX` | 4 | 环岛入口：内侧边界点数上限 |
| `ROUND_READY_MS` | 1200 | 环岛误判兜底 |
| `VIEW_CUT_FAR` / `VIEW_CUT_NEAR` | 10 / 20 | 视野窗口上、下裁掉的行数 |
| `VIEW_CUT_LEFT` / `VIEW_CUT_RIGHT` | 34 / 34 | 视野窗口左、右裁掉的列数 |
| `VIEW_FAR_ROW` / `VIEW_NEAR_ROW` | 10 / 99 | 由上面四项推导 |
| `VIEW_COL_L` / `VIEW_COL_R` | 34 / 153 | 由上面四项推导 |
| `OUT_BLIND_MS` / `OUT_CONFIRM_FRAMES` | 200 / 3 | 出界盲区时间与确认帧数 |
| `OUT_START_GRACE_MS` | 300 | 发车宽限 |
| `OUT_ROW_NEAR` | 76 | 出界采样点近端行（`BODY_ROW - 4`） |
| `ELEM_TIMEOUT_MS` / `ELEM_TOTAL_MS` | 8000 / 20000 | 元素单阶段 / 全会话超时 |
| `BE_BEND_MIN` / `BE_BEND_EXIT` | 10 / 4 | 入弯强度进入 / 退出阈值 |
| `BE_BLANK_MS` / `BE_MAX_LOST_MS` | 250 / 900 | 连接处完全免判时长 / 最长容忍 |
| `BE_MAX_APPROACH_MS` | 400 | 预备入弯的超时 |
| `PULSE_PER_METER` | 34600 | 每米脉冲数，必须推车 1 m 复核 |
| `LAP_DISTANCE_CM` / `STOP_OFFSET_CM` / `BRAKE_DISTANCE_CM` | 1200 / 100 / 200 | 圈长 / 停车点 / 提前刹车距离 |

## 10. 串口协议

待发车每 250 ms 一行，跑车中每 1 s 一行：

```
ODO=%ld L=%d R=%d C=%d AIM=%d THR=%d DARK=%d DROP=%u MODE=%d AL=%d STA=%d ERR=%d SPD=%d DS=%d DUTY=%d
```

| 字段 | 含义 |
|---|---|
| `ODO` | 累计里程脉冲 |
| `L` / `R` | 左右边界点数（拐点裁短前） |
| `C` | 中线点数（融合后） |
| `AIM` | 本帧前瞻点所在行（对比两种取法用） |
| `THR` | 当前全局阈值 |
| `DARK` | 最近一帧的暗点数（0~5），直道正常行驶应为 0 |
| `DROP` | 被丢弃的帧数，异常增多说明 CPU/XRAM 到瓶颈 |
| `MODE` | 114 循迹、514 十字、810 左环岛、1919 右环岛 |
| `AL` | 88 左侧外推、99 右侧外推 |
| `STA` | `trackIsOn()`，赛道是否可跑 |
| `ERR` | 故障位，见下表 |
| `SPD` / `DS` | 实际速度 / 目标速度 |
| `DUTY` | 舵机偏移 |

| 位 | 值 | 含义 |
|---|---|---|
| `FAULT_STOPPED` | 0x0001 | 因异常被强制紧急停车 |
| `FAULT_ENC_L_DOWN` | 0x0002 | 左编码器无反馈 |
| `FAULT_ENC_R_DOWN` | 0x0004 | 右编码器无反馈 |
| `FAULT_PID_SAT` | 0x0100 | 输出饱和且速度上不去 |
| `FAULT_PID_REV` | 0x0200 | 正输出配反向计数 |
| `FAULT_OUT` | 0x0400 | 出界保护触发 |

## 11. 离线校验

`tests/` 下每个脚本独立可跑（纯 Python，无第三方依赖），数值实现与固件逐行对应：

| 脚本 | 校验内容 |
|---|---|
| `numeric_check.py` | `atan2_int` / `sqrt_int` / `tani` 表值、曲率限速曲线、舵机换算与饱和点 |
| `aim_arc_check.py` | 前瞻两种取法（`PP_ARC_MODE`）的前瞻距离、`dx`、转角、duty 对比 |
| `bend_slowdown_check.py` | 弯道减速曲线的分档与保持逻辑 |
| `bend_entry_check.py` | 入弯状态机迁移 |
| `bend_recognition_check.py` | 中线融合与弯道识别 |
| `out_of_track_check.py` | 出界判据的暗点 / 盲区 / 确认帧组合 |
| `body_row_check.py` | `STEER_REF_ROW` 与车身遮挡行的几何 |
| `track_bias_check.py` | 横向偏置对瞄准点的影响 |
| `reference_alignment_check.py` | 与参考实现的链路一致性 |

## 12. version-1.4 变更

1. **环岛入口判据**：去掉“内侧必须带拐点”的前提。`LRoadCount` 在 `loadRoad()` 里
   会被裁成 `LCorner`（取值只有 0 或 ≥3），再要求 `LCorner` 非零，就把“内侧线整条消失”
   这个主要入口工况排除掉了，实际只剩“拐点恰落在第 3 点”能进。
2. **出界判据**：“看到赛道”改用裁短前的 `LRawCount` / `RRawCount`，避免十字与环岛期间
   `LRoadCount` 恒 ≤3 导致盲区计时一直成立。
3. **前瞻取法可选**：新增 `PP_ARC_MODE`。旧取法为给远端点留 12 个点，前瞻距离被永久
   扣掉 24 像素，中线越短缩得越狠，同一横向偏差解出的打角可差 5 倍以上；弧长模式按
   累计弧长取点、线短时沿末端切线外推，默认仍为 0（保持现车标定）。
   新增 `PP_LOOKAHEAD_PX` 作为等效轴距的唯一调参入口——原来 `motorInit` 第 7 参数恒传 0，
   注释里的调参建议没有入口。
4. **舵机自检**：新增 `SERVO_TEST_STEP = 33`（495 = 33×15）。原步长 25 除不尽 495，
   三段摆动分别停在 +475 / -480 / -20，收尾离中位还差 20 duty。
5. **数值清理**：`tani` 表缩到 31 项（`atan2_int` 上限 ±30，原 31/32 两档永远取不到）；
   删除 `alongLine` / `bendEntryCalc` 中数学上不可达的 `ax <= 0` 分支；`getAimPos` 上界
   改为 `SPEED_LIMIT/40`（原 `c3+20` 需 800 脉冲/5 ms 才顶得到，恒不生效）。
6. **十字补线起点**：兜底改用本次扫描的 `min` / `max`。原来用 `minMaxH`，单调边界下它
   恒为 0，起点会落到画面最左侧的裁切暗区，爬线在暗区里爬出几十个假点。
7. **可观测性**：串口新增 `AIM` 字段；新增 `tests/aim_arc_check.py`；`numeric_check.py`
   同步 `STEER_REF_ROW = 99` 与 31 项 `tani` 表。
8. **交付**：`out_file/SEEKFREE.hex` 与 `.map` 随版本入库，Keil 全量重建 0 Error。

## 13. 当前状态

已在实车确认：

- 编码器方向与电机方向匹配，`MOTOR_DIR_FORWARD = 0` 对应前进；
- 双缓冲 16 位字搬运，有效帧率约 96 fps；
- 直道行走与起步正常；`DARK` 在直道正常行驶时为 0；
- `version-1.2` 为已实车验证的稳定基线。

仍需现场验证：

- 弯道以推头冲出为主，偶发能过；
- 环岛 / 十字的入口阈值只是初标（`ROUND_MIN_PTS` / `ROUND_FAR_MAX` / `CROSS_ENTER_COUNT`），
  需要用串口 `MODE` 与 `AIM` 观察实际迁移；
- 定点停车依赖 `lapPulse` 实推标定，`PULSE_PER_METER` 需推车 1 m 复核 `ODO`；
- `PP_ARC_MODE = 1` 尚未上车验证，切换后同一弯道打角会变小，需要重标 `c1` 或 `PP_LOOKAHEAD_PX`。

详细评估见 `EVALUATION.md`，逐项改动审查见 `REVIEW.md`，工程报告见 `docs/PROJECT_REPORT.md`。

## 14. 许可与致谢

- 本项目整体以 **GNU General Public License v3.0** 分发，全文见 `LICENSE`。
- 第三方组件与相对上游的修改清单见 `NOTICE`。
- 感谢成都逐飞科技提供 STC32G144K 开源库与硬件支持。
- 感谢浙江工商大学哈夫克自动智能车项目提供的原始工程结构（见 `NOTICE`）。
