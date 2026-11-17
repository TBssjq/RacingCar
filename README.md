# STC32G144K 摄像头循迹车

基于 STC32G144K246 与逐飞库的摄像头组智能车工程，Keil C251 工具链。当前分支 `version-1.3`，提交 `5291481`。

## 1. 硬件

| 项目 | 型号 / 参数 |
|---|---|
| MCU | STC32G144K246，96 MHz |
| 摄像头 | 逐飞 MT9V03X，120 × 188 灰度，DMA 采集 |
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
    Image.c/.h        图像采集快照、起点搜索、爬线、去畸变、重采样、拐点
    Motor.c/.h        纯跟踪、舵机换算、差速、双轮 PID、故障检测
    Cross.c/.h        十字状态机
    LRound.c/.h       左环岛五阶段状态机
    RRound.c/.h       右环岛五阶段状态机
    BendEntry.c/.h    弯道入口过渡区
    Stop.c/.h         发车、里程、定点停车
    Track.c/.h        赛道状态判定与状态灯
  user/
    main.c            上电流程、按键、主循环、元素派发
    isr.c             定时器与 INT1 入口
    isr_extended_0.c  DMA 完成、UART
    isr_extended_1.c  其余扩展中断
libraries/            逐飞库，禁止修改
tests/                Python 离线校验脚本
```

## 3. 构建与烧录

```powershell
$p = Start-Process -FilePath 'D:\Keil_v5\UV4\UV4.exe' `
     -ArgumentList '-r','project\mdk\seekfree.uvproj','-j0','-o','project\mdk\out_file\SEEKFREE.build_log.htm' `
     -WorkingDirectory 'f:\项目文件夹\Racing Car C-model\version1.0' -Wait -PassThru
$p.ExitCode
```

产物在 `project/mdk/out_file/SEEKFREE.hex`。编译只允许出现逐飞库的 `C183` 一条警告。

## 4. 上电与操作

| 步骤 | 现象 |
|---|---|
| 上电 | 状态灯闪 3 下，USB CDC 出现 `ODO=` 输出 |
| 摄像头初始化 | 失败时状态灯 500 ms 慢闪并停住 |
| 舵机自检 | 机械中位 → 左 30° → 右 30° → 回中，约 1.4 s |
| 选档 | 读 PB0 / PB1，从 8 档里取 1 / 3 / 5 / 7 档 |
| 待发车 | 状态灯按编码器状态闪烁 |

| 按键 | 功能 |
|---|---|
| B2 | 发车 |
| B3 | 任意状态紧急停车 |
| B4 短按 | 停车状态下舵机左右各摆 30° |
| B4 长按 2 s | 待发车状态下把当前里程记为一圈 |

`CALIB_REQUIRED = 0`，不标定也能发车；未标定时 `raceStart()` 把 `lapPulse` 设成 `0x7FFFFFFF`，不会因里程自动刹停，只能靠 B3 手动停。

## 5. 图像链路

```
DMA 写 mt9v03x_image
  → DMA 完成中断 mt9v03x_frame_snapshot() 搬 22.5 KB 到 mt9v03x_frame，置 frameBusy
  → 主循环 frameBusy == 1
      cutViewRegion()               视野窗口：只保留行 10~99、列 34~153，四周置暗
      getLStart / getRStart         行 95 → 13，列 36 → 151，车头区用窄列
      getLBorder / getRBorder       八邻域爬线，局部阈值每 2 步重算
      getRoundStart                 断点后二次爬线，补齐入环口
      getGodRoadLine                当前旁路 reflection 查表
      filterRoad                    因果平均
      resampleRoad(dis = 3)         弧长等距
      findCorner                    夹角余弦找拐点，裁短点数
      buildCenterLine               双侧配对取中点 / 单侧半宽外推
      filterRoad + resampleRoad(2)  中线平滑与等距化
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
| 前瞻点 | `getAimPos()` = `c3 + currentSpeed / 40`，夹 `c3 ~ c3+20` |
| 纯跟踪 | `d = atan2_int(2·18·dy, dx² + dy²)`，`dx = 120 - cl[pos].x + 18`，`dy = 94 - cl[pos].y` |
| 远端前馈 | 固定 1/4，`dy = dyAim + (dyFar - dyAim) / 4` |
| 舵机换算 | `currentDuty = d × c1 / 20`，夹 ±395 |
| 舵机环 | `steerPid` Kp 100 / Ki 0 / Kd 100 |
| 曲率限速 | `R = 20000 / tan(δ)`，`v = √(R·c4)`，`target = min(v, c5)` |
| 弯道减速 | `slowBend = |dySlow - dyAim| + |δ|`，比例 50%~100%，硬顶 `BEND_SPEED_CAP = 200` |
| 差速 | `diff = setSpeed × 8 / R`，夹 `setSpeed / 2` |
| 电机环 | 增量式，Kp 3000 / Ki 50 / Kd 50，输出 ±10000 |

## 7. 元素

| 元素 | 入口判据 | 状态机 |
|---|---|---|
| 十字 | 两侧点数都 < 14，且任一侧有拐点 | `CROSS_READY` / `CROSS_IN`，补线打点后退出 |
| 左环岛 | 右线延伸到远端（`RRoad[37].x < 20`）、左线有拐点、左线点数 < 11 | 五阶段，`centerLineDistance` 13 / 27 / 11 / 18 |
| 右环岛 | 镜像 | 11 / 25 / 11 / 18 |
| 弯道入口 | 强度 ≥ 10 连续 2 帧 | `STRAIGHT` → `APPROACH` → `COMMITTED` → `RECOVER` |

元素关闭开关在 `main.c`：`ENABLE_CROSS`、`ENABLE_ROUND`，当前都为 1。

## 8. 安全保护

| 保护 | 触发条件 | 动作 |
|---|---|---|
| 出界 | 5 个采样点全暗 **且** 已 200 ms 没看到赛道，连续 3 帧成立后再经 3 拍消抖 | 停车，`ERR` 置 0x0400 |
| 编码器掉线 | 目标 > 60、输出 > 2000、该轮 300 ms 计数 < 3 | 停车，`ERR` 置 0x0002 / 0x0004 |
| PID 饱和 | 输出 ≥ 96% 上限且速度 < 目标 1/4，持续 1 s | 停车，`ERR` 置 0x0100 |
| 正反馈 | 正向输出 > 1500 且编码器 < -5，持续 500 ms | 停车，`ERR` 置 0x0200 |
| 帧看门狗 | 1 s 无新帧 | `raceStopNow()` |
| 元素超时 | 离开 FORWARD 超过 8 s | 强制回 FORWARD |
| 环岛阶段超时 | 1.2 / 1.5 / 2 s | 进入下一阶段 |
| 发车宽限 | 发车后 800 ms | 不判出界 |

出界判据要求三条信号同时成立，且都要持续（`main.c` 的 `OUT_*` 宏）：

1. 5 个采样点全部偏暗（`outSamplePt`，每点取 3×3 块均值）；
2. 距最近一次“看到赛道”（任一侧边界点数 > 3）已超过 `OUT_BLIND_MS = 200` ms；
3. 上述结论连续 `OUT_CONFIRM_FRAMES = 3` 帧，之后控制中断再做 3 拍消抖。

采样点全部放在车身顶边 `BODY_ROW = 80` 以上（行 66~76）、且收进赛道内侧：摄像头视场远大于赛道，画面边缘与远端本来就是场地外暗区，原来的 84~88 行三点点在车身上、47 / 141 列常落在线外，等于长期白送 4~5 个暗点，这是直道偶发停车的主因。图像侧按现场草图只保留中间一块（上裁 10 行、下裁 20 行、左右各裁 34 列，即行 10~99、列 34~153），四周整块置暗；起点搜索、爬线、初始阈值取样与拐点判据的范围同步收紧到窗口内。远端比上一版多留 10 行，让中线点列更长、远端趋势前馈不退化，进弯前能更早打角。

单侧还有边界时不停车：画面里仍有可循迹的线，交给纯跟踪把车拉回，比停车更划算。

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

拨码组合 `(SW2, SW1)` 映射到 1 / 3 / 5 / 7 档。默认组合选中第 1 档。

| 宏 | 值 | 说明 |
|---|---|---|
| `STEER_DUTY_LIMIT` | 395 | 舵机相对中值的最大偏移 |
| `STEER_DUTY_DIV` | 20 | `duty = d × c1 / 20` |
| `STEER_TRIM` | -58 | 舵机机械中值修正 |
| `TRACK_BIAS_PX` | 0 | 纯跟踪瞄准点横向偏置 |
| `MOTOR_DUTY_LIMIT` | 10000 | 双轮 PID 输出上限（满量程） |
| `SPEED_LIMIT` | 400 | 目标速度上限 |
| `BEND_SLOW_START` | 4 | 弯道减速起始强度 |
| `BEND_SPEED_CAP` | 200 | 减速状态的硬上限 |
| `BORDER_IN_SPEED_RATIO` | 50 | 元素内速度比例 |
| `MOTOR_DIR_FORWARD` | 0 | 低电平前进 |
| `MOTOR_KICK_TICKS` | 0 | 起跑助推关闭 |
| `USE_REFLECTION` | 0 | 旁路 reflection 查表 |
| `EDGE_DIFF` / `EDGE_RATIO` | 12 / 7 | 边界梯度双条件 |
| `HIDE_ROW` / `BODY_ROW` | 90 / 80 | 车头遮挡行 |
| `CROSS_MAX` | 14 | 十字入口两侧点数上限 |
| `ROUND_MIN_PTS` / `ROUND_FAR_ROW` | 16 / `VIEW_FAR_ROW+8` | 环岛入口：外侧边界点数下限、末点必须伸到的行 |
| `ROUND_FAR_MAX` | 4 | 环岛入口：内侧边界点数上限（整条消失才算缺口） |
| `ROUND_READY_MS` | 1200 | 环岛 READY 误判兜底：超时回 FORWARD 并解除限速 |
| `PP_DX_MIN` | 30 | 纯跟踪纵向距离下限，防止贴脸点打满舵 |
| `VIEW_CUT_FAR` / `VIEW_CUT_NEAR` | 10 / 20 | 视野窗口上、下各裁掉的行数；调小 `VIEW_CUT_FAR` 可看到更远 |
| `VIEW_CUT_LEFT` / `VIEW_CUT_RIGHT` | 34 / 34 | 视野窗口左、右各裁掉的列数 |
| `VIEW_FAR_ROW` / `VIEW_NEAR_ROW` | 10 / 99 | 由上面四项推导的保留行区间 |
| `VIEW_COL_L` / `VIEW_COL_R` | 34 / 153 | 由上面四项推导的保留列区间 |
| `STEER_REF_ROW` | `VIEW_NEAR_ROW` | 纯跟踪纵向距离的参考行；改回 `MT9V03X_H` 会拉长前瞻、转角变柔 |
| `OUT_BLIND_MS` / `OUT_CONFIRM_FRAMES` | 200 / 3 | 出界判据的盲区时间与确认帧数 |
| `OUT_START_GRACE_MS` | 300 | 发车宽限：这段时间内不判出界 |
| `OUT_ROW_NEAR` | 76 | 出界采样点的近端行（`BODY_ROW - 4`） |
| `ELEM_TIMEOUT_MS` | 8000 | 单个元素阶段的最长停留时间（阶段一换就重置） |
| `BE_MAX_APPROACH_MS` | 400 | 预备入弯状态的最长停留时间 |
| `PULSE_PER_METER` | 34600 | 每米脉冲数，需按实测复核（推车 1 m 看 `ODO`） |
| `LAP_DISTANCE_CM` | 1200 | 赛道一圈长度 |

## 10. 串口字段

待发车每 250 ms 一行，跑车中每 1 s 一行：

```
ODO=%ld L=%d R=%d C=%d THR=%d DARK=%d DROP=%u MODE=%d AL=%d STA=%d ERR=%d SPD=%d DS=%d DUTY=%d
```

| 字段 | 含义 |
|---|---|
| `ODO` | 累计里程脉冲 |
| `L` / `R` | 左右边界点数（拐点裁短前的原始值） |
| `C` | 中线点数 |
| `THR` | 当前全局阈值 |
| `DARK` | 最近一帧的暗点数（0~5），直道正常行驶应为 0 |
| `DROP` | 被丢弃的帧数（主循环处理慢于帧率时累加），异常增多说明 XRAM/CPU 到瓶颈 |
| `MODE` | 114 循迹、514 十字、810 左环岛、1919 右环岛 |
| `AL` | 88 左侧外推、99 右侧外推 |
| `STA` | `trackIsOn()`，赛道是否可跑 |
| `ERR` | 故障位 |
| `SPD` / `DS` | 实际速度 / 目标速度 |
| `DUTY` | 舵机偏移 |
| `NOSE` | 车头顶行号 |

`ERR` 与原因：

| 位 | 值 | 含义 |
|---|---|---|
| `FAULT_ENC_L_DOWN` | 0x0002 | 左编码器无反馈 |
| `FAULT_ENC_R_DOWN` | 0x0004 | 右编码器无反馈 |
| `FAULT_PID_SAT` | 0x0100 | 输出饱和且速度上不去 |
| `FAULT_PID_REV` | 0x0200 | 正输出配反向计数 |
| `FAULT_OUT` | 0x0400 | 出界 |
| `FAULT_STOPPED` | 0x0800 | 故障停车标志 |
| `FAULT_BRAKE_DONE` | 0x1000 | 里程到点进入刹车段 |

## 11. 当前状态

已在实车确认：

- 编码器方向与电机方向匹配，`MOTOR_DIR_FORWARD = 0` 对应前进。
- 双缓冲搬运 16 位字，帧率约 96 fps。
- 直道行走、起步正常。

已知未解决：

- 弯道以推头冲出为主，偶发能过。
- 环岛 / 十字状态机已修掉三处"数学上不可达"的死态（见 REVIEW 第十节），
  但新判据的阈值只能算初标，仍需现场验证能否完整跑通。
- 定点停车依赖 `lapPulse` 实推标定；`PULSE_PER_METER` 已统一为 34600，
  但必须按实车推 1 m 复核 `ODO`（见 REVIEW 第十一节）。

已处理（直道偶发停车）：

- 原因是摄像头视场远大于赛道，场地外的暗区被当成了"车头前方全暗"。出界采样点
  原来有 3 个点落在车身遮挡区（行 84~88，车身顶边已到 80 行）、2 个点落在赛道外
  （列 47 / 141），等于长期白送暗点，远端点一变暗就凑齐"5 点全暗"。
- 现在三处一起改：采样点移到行 66~76 并收进赛道内侧、每点改 3×3 块均值；
  按现场草图保留中间一块（行 10~99、列 34~153），四周置暗；
  出界判据加"200 ms 内看到过赛道就不判"与连续 3 帧确认。
- 转弯幅度：远端多保留 10 行让前瞻点列更长；同时 `STEER_REF_ROW` 取窗口下沿
  （99）而不是画面底边（120），把近端裁掉的 20 行从纵向距离里扣掉，
  同一横向偏差算出的转角放大约 1.5~1.6 倍。
- 现场复验：直道正常行驶时串口 `DARK` 应为 0；若常年 ≥1，说明采样点仍需按实际画面内收。

详细状态与后续计划见 `docs/PROJECT_REPORT.md`，算法评估见 `EVALUATION.md`，本轮改动审查见 `REVIEW.md`。
