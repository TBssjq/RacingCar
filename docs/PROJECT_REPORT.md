# Racing Car C-model version1.0 项目技术报告

> **报告类型**：完整技术审计 / 硬件全量清单 / 软件架构说明
> **审计对象**：`F:\项目文件夹\Racing Car C-model\version1.0\`（以下简称"本仓库"）
> **生成日期**：2026-11-15
> **核验方式**：全量源码审读 + Keil 编译日志 / MAP 文件解析 + Git 状态与差异比对 + `tests/` 下 8 个脚本实际执行 + 芯片官方公开资料核对
> **证据口径**：凡标注「源码」「实测」者均以当前工作区文件为准；凡标注「文档」者引自仓库自带 README / REVIEW / EVALUATION / docs；凡标注「公开资料」者来自 STC 官方网页，不属于仓库内可核实内容
>
> **重要声明**：本仓库**没有原理图、BOM、机械图纸与实车测试数据**。本报告能给出的"所有硬件信息"= 工程源码与工程配置中可确证的引脚、外设、时序、参数，加上芯片厂商公开规格；缺失项已在附录 C 逐条列出，未作任何推测填充。

---

## 0. 摘要与关键结论

Racing Car C-model version1.0 是一套面向全国大学生智能汽车竞赛竞速组的 **STC32G144K246 单 MCU 摄像头循迹车**完整固件：以逐飞科技 STC32G144K 开源库（V3.2.5，GPL-3.0）为底层，在浙江工商大学"哈夫克"项目基础上二次开发。感知用 MT9V03X 灰度摄像头（188×120 / 170 fps / FIFO+DMA），执行用 1 路舵机 + 2 路直流电机 + 2 路正交编码器，人机用 3 按键 + 2 拨码 + 蓝色 LED + USB CDC 虚拟串口，屏幕（IPS200）已从硬件移除。

**核心结论**：

1. **可编译、可烧录**：最新一次重建 0 Error / 1 Warning，产物 `SEEKFREE.hex`（370,046 字节，8,246 行 Intel HEX），工具链 PK251 V5.60.0.0。
2. **硬件资源已接近上限**：`xdata = 51,893 B`，其中两块 22,560 B 图像缓冲占 45,120 B；`code = 82,662 B`、`const = 48,754 B`。README 自述 XDATA 余量约 13.4 KB（另一处写 13.8 KB，自相矛盾）。
3. **工作区存在 3 个未提交文件**（`Image.c` / `Image.h` / `main.c`），改动发生于 2026-11-15 11:11–11:13，晚于全部文档（2026-11-14 22:25）与最后一次提交（2026-11-14 15:26）。这批改动属于**现场标定**，且已使部分文档描述与对齐测试失效。
4. **透视去畸变当前未生效**：`Image.h` 新增 `USE_REFLECTION 0`，因摄像头安装改为 **20 cm 高 / 45° 俯角**，而 45 KB 的 `reflection[120][188][2]` 表是按旧安装 **25 cm / 39°** 离线标定的。README 第 2 节仍把"透视去畸变"列为生效功能。
5. **出界保护已被大幅放宽**：源码判据现为"5 个采样点全暗"，README / EVALUATION / REVIEW 描述的"两侧边界全丢且 ≥3 点暗"分支已删除。
6. **8 个核对脚本中 7 个输出符合预期，1 个判定失败**：`reference_alignment_check.py` 报「存在超出范围的差异」，原因正是上述未提交改动；且该脚本依赖**仓库目录之外**的 `backup/version1.3/project`，单独分发 `version1.0` 将无法复现。
7. **里程标定未完成**：`PULSE_PER_METER = 1000` 为占位值（EVALUATION 第 93 行明确"尚未实测"），而 README 的速度换算示例按 34,600 计算，两者差 34.6 倍；`CALIB_REQUIRED = 0` 时不会自动刹停，靠 B3 手动停车。
8. **闭源依赖**：摄像头 DMA 采集 `zf_device_mt9v03x_dma.LIB`（30,727 B）与 `zf_device_config.LIB`（8,815 B）为预编译库，内部行为无法从源码核实。

---

## 1. 项目标识与版本状态

| 项 | 值 | 依据 |
|---|---|---|
| 项目名 | Racing Car C-model（`version1.0`） | 目录名 / README 第 1 行 |
| 二次开发作者 | aMaz1ng | README 第 3 行 / NOTICE 第 4 行 |
| 上游项目 | 浙江工商大学 哈夫克自动智能车项目（非官方分支） | README 第 4 行 |
| 底层库 | 逐飞科技 STC32G144K 开源库 **V3.2.5**（2026-06-27） | `libraries/doc/version.txt` 第 1 行 |
| 许可证 | 整体 **GPL-3.0**；库修改按第 5(a) 条登记于 `NOTICE` | `LICENSE` / `NOTICE` |
| 开发环境 | Keil MDK FOR C251（uVision V5.25.3.0），目标 `stc32g144k246` | `SEEKFREE.build_log.htm` |
| 竞赛背景 | 全国大学生智能汽车竞赛（摄像头组 / 竞速组）；`Stop.h` 具体写「2026 南京工业大学智能汽车竞速组（传统赛道）」 | README 第 11 行 / `Stop.h:3` |
| 最后一次提交 | `5291481` 2026-11-14 15:26:42 `feat(chassis): 弯道入口识别与自适应转向增益` | `git log -1` |
| 文档最后更新 | 2026-11-14 22:25:54（README / REVIEW / EVALUATION / docs 四份同时） | 文件 mtime |
| **未提交改动** | `project/code/Image.c`、`project/code/Image.h`、`project/user/main.c`（+52 / −9 行） | `git status` / `git diff --stat` |
| 最新源码/产物时间 | 源码 11:11–11:13、构建 11:13:30（2026-11-15） | 文件 mtime / MAP 页眉 |

交付物清单（仓库内实际存在）：`README.md`、`REVIEW.md`、`EVALUATION.md`、`docs/TRACK_RECOGNITION_DESIGN.md`、`NOTICE`、`LICENSE`、`libraries/`（1.5 MB）、`project/`（13 MB，含 `mdk/out_file/` 编译产物）、`tests/`（8 个脚本）、`docs/`。

---

## 2. 适用范围与任务定义

| 项 | 内容 | 依据 |
|---|---|---|
| 车模 | C 车模（项目名 C-model），具体机械/型号参数**仓库未提供** | 目录名 |
| 赛道材质 | 蓝底白赛道（低对比度，故 `EDGE_DIFF`/`EDGE_RATIO` 调低） | `Image.h:13,20-21` |
| 支持元素 | 普通循迹、十字、左环岛、右环岛、里程定点停车 | README 第 55 行 |
| **不支持** | 斑马线、障碍块（已删除） | README 第 55 行 / NOTICE 第 50 行 |
| 比赛规则 | 从起跑线后出发，跑完 1 周，停在起跑线前方 2 m 内 | `Stop.h:5` |
| 圈长设定 | `LAP_DISTANCE_CM = 1200`（12 m）、`STOP_OFFSET_CM = 100`、`BRAKE_DISTANCE_CM = 200` | `Stop.h:21-23` |
| 选档方式 | 上电按 SW1/SW2 拨码自动选档，8 档参数表取奇数档 | `main.c:283-288` |

---

## 3. 硬件系统总览

### 3.1 系统拓扑

```
  ┌──────────────────────── 主控：STC32G144K246 (C251, 96 MHz) ────────────────────────┐
  │                                                                                  │
  │   MT9V03X 灰度摄像头 ──8bit 并口 P2──► FIFO(型号未提供) ──DMA(LCM)──► mt9v03x_image│
  │        └─ SCCB PB6/PB7(配置)            └─ VSYNC INT1/P33      │                 │
  │                                                                ▼ (DMA 完成 ISR)  │
  │                                                        mt9v03x_frame (快照)    │
  │                                                                │                │
  │   主循环：loadRoad() → 元素状态机 → Pure Pursuit → 曲率/弯道限速 → desiredSpeed │
  │   TIM1 5ms ISR：编码器 → 差速 → 双轮增量式 PID → PWM                            │
  │                                                                                  │
  │   PWME_CH3P_PA4 ──► 舵机(200 Hz)      PWMB_CH3_P76/P77 ──► 右电机+DIR(17 kHz) │
  │   PWMB_CH1_P74/P75 ──► 左电机+DIR      PWMA(P60/P62)/PWMC(P40/P42) ◄── 编码器 │
  │   PB0~PB4 ──► 拨码/按键                P52 ──► 蓝灯(低亮)                      │
  │   内置 USB ──► CDC 虚拟串口 ──► PC                                              │
  └──────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 硬件清单汇总

| 类别 | 器件 / 接口 | 数量 | 关键规格 | 状态 |
|---|---|---|---|---|
| 主控 | STC32G144K246 | 1 | C251 内核，96 MHz，LQFP100 系列 | 在用 |
| 视觉 | 逐飞「总钻风」MT9V03X 灰度摄像头 | 1 | 188×120，170 fps，8 bit 并行 + FIFO + DMA | 在用 |
| 转向 | 舵机（型号未提供） | 1 | PWM 200 Hz，电信号中值 2975 duty（修正后 2917） | 在用 |
| 驱动 | 直流减速电机 + 双路 H 桥（芯片型号未提供） | 2 | PWM 17 kHz，占空比满量程 10000 | 在用 |
| 反馈 | 正交编码器（线数未提供） | 2 | PWMA / PWMC 正交解码 | 在用 |
| 输入 | 独立按键 B2/B3/B4 | 3 | 低电平有效，内部上拉 | 在用 |
| 输入 | 拨码开关 SW1/SW2 | 2 | 低电平有效，内部上拉 | 在用 |
| 指示 | 核心板蓝色 LED | 1 | P52，**低电平点亮** | 在用 |
| 指示 | 蜂鸣器 | 1 | P90 | **本车未装**，代码仍驱动 |
| 显示 | IPS200 显示屏 | 1 | — | **硬件已移除**，工程仍编译其驱动 |
| 调试 | 芯片内置 USB → CDC | 1 | USB 2.0 Full-Speed，VID 34BF / PID FF02 | 在用 |
| 备用 | 示波器调试脚 | 1 | P04 | 注释状态，未使用 |

---

## 4. 主控单元（完整规格）

### 4.1 工程内可核实配置

| 项 | 值 | 来源 |
|---|---|---|
| 器件型号 | `STC32G144K246 Series`，Vendor `STC`，DeviceId `63697` | `seekfree.uvproj` `<TargetCommonOption>` |
| 工具集 | `MCS-251` / `ToolsetNumber 0x1`；`CPU MODE: 251 SOURCE MODE` | uvproj / MAP 第 12 行 |
| 寄存器头 | `STC32G.H` / `stc32g144k246.h` | uvproj |
| 系统主频 | `clock_init(SYSTEM_CLOCK_96M)` = **96 MHz** | `main.c:202` |
| 时钟源 | `USER_CRTSTAL = INTERNAL_CRYSTAL`（内部 IRC），可选 78M~124M | `zf_common_clock.h:46,49-63` |
| 仿真时钟 | `<Cpu> CLOCK(35000000)` = 35 MHz（仅 uVision 仿真/波特率计算用） | uvproj `<Cpu>` |
| 内存模型 | `MemoryModel 4` → MAP 显示 `LARGE WITH DOUBLE PRECISION FLOATING POINT ARITHMETIC` | uvproj / MAP 第 22 行 |
| IRAM / EDATA | `0x0–0x3FFF` = 16 KB | uvproj / MAP CLASSES |
| XRAM | `0x10000–0x2FFFF` = 128 KB（HDATA 类） | uvproj / MAP CLASSES |
| IROM / Flash | `0xFC2800–0xFFFFFF`，size `0x3D800` = **246 KB**；HCONST/ECODE `0xFC4800–0xFFFFFF` | uvproj / MAP CLASSES |
| 中断优先级 | 4 级（本工程实际使用 0 / 1 / 3） | 官方规格 / `main.c:236-237` |
| C251 编译 | `Optim 7`、`wLevel 3`、`uFlt64 1`、`uAlias 1`、`uSrcBin 1` | uvproj `<C251>` |
| 汇编 / 链接 | A251 V4.69.13.0；L251 `REMOVEUNUSED,NOOVERLAY`、`DISABLEWARNING(57,16,7,25)`、`CASE` | MAP 第 16–17 行 |
| 优化告警抑制 | `#pragma warning disable = 115 / 188` | `zf_common_headfile.h:40-41` |
| 应用层头文件注入 | `Cross.h / Image.h / LRound.h / RRound.h / Motor.h / Stop.h / Track.h` | `zf_common_headfile.h:111-117` |

### 4.2 芯片公开规格（STC 官方，非仓库内文件）

> 来源：`https://stcmicro.com/cn/stc/stc32g144k246.html`，用于补足仓库缺失的器件级参数，**不作为工程配置依据**。

| 项 | 公开规格 |
|---|---|
| 内核 | 超高速 32 位 8051（1T，兼容 8 位指令），CPU/DSP 100–120 MHz |
| 存储 | 246 KB Flash；144 KB SRAM（16 KB edata + 128 KB xdata）+ 4 KB 高端扩展 RAM；EEPROM 可配置 |
| 供电 | 1.9–5.5 V，内建 LDO；工作温度 −40~+85 ℃（另有 −40~+125 ℃ 档需用外部晶振/PLL） |
| GPIO | 最多 91 根（P0~P9、PA、PB），4 种模式，独立上下拉，全部支持中断与掉电唤醒 |
| 封装 | LQFP100 / LQFP64 / LQFP48 / LQFP44（本工程库名含 `100PIN`，推断为 **LQFP100**） |
| PWM | 24 通道高级 PWM：PWMA/PWMC/PWME 各 4 对互补，PWMB/PWMD/PWMF 各 4 路独立；PWM 时钟最高 240 MHz |
| 定时器 | 14 个（T0~T11、T17、T18）；RTC |
| 通信 | USB 2.0 全速（16 双向端点）、8 USART、3 SPI、2 I2C、2 CAN-FD、2 I2S、QSPI |
| 模拟 | 2×12 bit ADC（15 通道）、2×12 bit DAC、4 比较器、4 运放 |
| DMA | 外设到外设、LCM/TFT-i8080、QSPI、ADC/DAC、SPI、I2C、UART 等 |
| 其他 | 49+ 中断源 / 4 级优先级；硬件 SWD 仿真使用 P3.0 / P3.1；TFPU 浮点与三角函数单元 |
| 烧录 | 硬件 USB 直接下载或串口 ISP（STC-ISP） |

### 4.3 存储资源实测占用

实测来源：`project/mdk/out_file/SEEKFREE.build_log.htm` 与 `SEEKFREE.map`（L251，2026-11-15 11:13:30）。

| 段 | 实测字节 | 说明 |
|---|---:|---|
| `data` | 101.5 | 直接寻址内部 RAM |
| `edata+hdata` | 8,192 | 16 KB 内部 SRAM 中的已用部分 |
| `xdata` | **51,893** | 主要被两块图像缓冲占用 |
| `const` | 48,754 | 主要为 `reflection[120][188][2]` = 45,120 B + `tani[33]` 等 |
| `code` | **82,662** | 应用 + 逐飞库 + USB CDC 栈 |
| 产物 | `SEEKFREE.hex` 370,046 B / 8,246 行 | Intel HEX，起始扩展地址 `0x00FC` |

XDATA 大户（MAP 符号地址）：

| 对象 | 地址 | 大小 | 说明 |
|---|---|---|---|
| `mt9v03x_frame` | `0x0001019A` | 22,560 B | 算法只读快照 `uint8[120][188]` |
| `mt9v03x_image` | `0x00015AA8` | 22,560 B | DMA 写入的采集缓冲 |
| 合计 | — | **45,120 B ≈ 86.9 % 的 xdata** | 两缓冲之和 |

> **余量说明（严谨起见列出矛盾）**：README 第 30 行写「xdata 余量约 13.4 KB」，第 597 行写「13.8 KB」；按 65,536 − 51,893 = 13,643 B ≈ 13.3 KB 可支持前者，但 uvproj 又配置了 128 KB XRAM / HDATA 类。工程未说明链接器 XDATA 上限的实际取法，**故本报告不给出唯一余量结论**，只记录：不足以再放第三块 22.5 KB 帧缓冲（README 第 597 行据此否定 DMA 双缓冲方案）。

---

## 5. 视觉感知子系统（完整硬件信息）

### 5.1 MT9V03X 摄像头参数

| 参数 | 值 | 来源 |
|---|---|---|
| 器件 | 逐飞「总钻风」MT9V03X（灰度/黑白全局快门） | `zf_device_mt9v03x.h` |
| 分辨率 | `MT9V03X_W = 188`、`MT9V03X_H = 120` | `mt9v03x.h:65-66` |
| 单帧数据量 | `188 × 120 = 22,560 B`（22.5 KB，8 bit 灰度） | `mt9v03x.h:67` |
| 帧率 | `MT9V03X_FPS_DEF = 170`（约 5.88 ms/帧） | `mt9v03x.h:73` |
| 自动曝光 | `MT9V03X_AUTO_EXP_DEF = 0`（**关闭**，范围 0–63） | `mt9v03x.h:69` |
| 曝光时间 | `MT9V03X_EXP_TIME_DEF = 200` | `mt9v03x.h:72` |
| 图像增益 | `MT9V03X_GAIN_DEF = 20`（范围 16–64） | `mt9v03x.h:78` |
| 左右/上下偏移 | 均为 0 | `mt9v03x.h:74,76` |
| 初始化超时 | `MT9V03X_INIT_TIMEOUT = 400` | `mt9v03x.h:60` |
| 配置接口 | **软件 SCCB（IIC）**：SCL `IO_PB6`、SDA `IO_PB7`，`DELAY 50`（约 50 kHz） | `mt9v03x.h:46-48` / `mt9v03x.c` `soft_iic_init` |
| 配置命令 | 10 项：INIT / AUTO_EXP / EXP_TIME / FPS / SET_COL / SET_ROW / LR_OFFSET / UD_OFFSET / GAIN / CONFIG_FINISH | `mt9v03x.h:83-104` |
| 主程序重试 | `mt9v03x_init()` 最多 150 次 × 20 ms ≈ **3 s**，失败蓝灯 500/500 ms 慢闪并停机 | `main.c:216-229` |

### 5.2 采集链路与 FIFO 引脚（**禁止修改**）

`zf_device_mt9v03x_dma.h:8-15` 明确标注「FIFO 引脚禁止修改，除此之外摄像头的 DMA 还占用了 P45 P36 这两个引脚」。

| 信号 | 引脚 | GPIO 配置 | 来源 |
|---|---|---|---|
| 8 bit 数据 D0~D7 | **P20~P27** | GPI、上拉输入 | `mt9v03x.h:51-58` / `mt9v03x.c` init |
| SCCB SCL / SDA | **PB6 / PB7** | 软件 IIC，delay 50 | `mt9v03x.h:47-48` |
| FIFO VSY（场同步） | **P33** | GPI 上拉，同时作 `INT1_P33` 外部中断（BOTH 边沿） | `dma.h:11` / `mt9v03x.c` init |
| FIFO RCK（读时钟） | **P37** | GPO 推挽，初值 1 | `dma.h:12` |
| FIFO WE（写使能） | **P41** | GPO 推挽，初值 0 | `dma.h:13` |
| FIFO WRST（写复位） | **P73** | GPO 推挽，初值 1 | `dma.h:14` |
| FIFO OE（输出使能） | **P72** | GPO 推挽，初值 0 | `dma.h:15` |
| FIFO RRST（读复位） | **P43** | GPO 推挽，初值 1 | `dma.h:16` |
| DMA 额外占用 | **P45、P36** | 库内硬编码（`.LIB`，无源码） | `dma.h:8-10` 注释 |
| DMA 初始化 | `mt9v03x_dma_init(&mt9v03x_image, 22560, 0, 3, 3)` | 中断优先级 3、DMA 优先级 3（初始化时）；main 后续把 `LCM_DMA` 降到 1 | `mt9v03x.c` init / `main.c:237` |
| 单帧读取耗时 | 约 **3 ms**（库注释与 V3.2.5 版本说明） | `mt9v03x.c` / `version.txt:2` |
| VSYNC 中断 | `INT1_VECTOR`（向量 2），优先级 3，调用 `mt9v03x_vsync_handler()` | `isr.c:120-127` / `mt9v03x.c` |

### 5.3 图像双缓冲

| 项 | 值 |
|---|---|
| 采集缓冲 | `mt9v03x_image[120][188]`（库内固定地址，DMA 写入） |
| 算法快照 | `mt9v03x_frame[120][188]`（`Image.c:21`），算法层唯一读取对象 |
| 搬运函数 | `mt9v03x_frame_snapshot()`（`Image.c:24-47`），16 位字指针、每次搬 4 个字（8 B），共 2,820 次循环 |
| 执行上下文 | `DMA_LCM_IRQHandler()` 内（`isr_extended_0.c:282-288`），优先级 1 |
| 门控 | `frameBusy`：为 1 时 ISR 跳过搬运（可预期丢帧，杜绝半帧）；主循环消费完置 0 |
| 远端屏蔽 | `cutFarRegion()` 把行 0~19 全写 0（`FAR_CUT_ROW = 20`），**未提交的现场改动** |

> 摄像头配置上限 170 fps（README 第 53 行、EVALUATION 第 27 行）；`Image.c:31` 注释记录逐字节 memcpy 约 4–5 ms 时实测约 96 fps，改 16 位搬运后改善。当前实际帧率**未在实车上测量**。

### 5.4 安装与标定状态（当前工作区）

| 项 | 当前值 | 说明 |
|---|---|---|
| 摄像头安装高度 | **20 cm** | `Image.h:14`（未提交改动） |
| 俯角 | **45°** | `Image.h:14` |
| 旧标定安装 | 25 cm / 39° | `Image.h:15` |
| 透视查表开关 | `USE_REFLECTION = 0`（**旁路**） | `Image.h:18`；`Image.c:342-360` 走 `#else` 分支，保留原始斜视坐标 |
| 查找表 | `reflection[120][188][2]`，45,120 B，随 `Image.c:439` 起定义、来自参考实现 `version1.3` | `docs/TRACK_RECOGNITION_DESIGN.md:15-18` |
| 车身遮挡 | `HIDE_ROW = 90`、`HIDE_COL_L/R = 48/140`；起点搜索分界 `BODY_ROW = 80`；行 70~79 全宽兜底已改为行 **50** 以上 | `Image.h:25-32` / `Image.c`（未提交改动把 `i >= 70` 改为 `i >= 50`） |
| 起点扫描 | 行 `120−halfBlock−2` 向行 **50**，步长 3，二分找灰度跳变 | `Image.c` `getLStart()/getRStart()` |
| 阈值 | `EDGE_DIFF = 12`、`EDGE_RATIO = 7`、局部窗口 `block = 5`、`cap = 4`、初始兜底阈值 128 | `Image.h:9-10,20-21,33`；`imageInitThreshold()` |
| 出界采样点 | `[84][95]`、`[88][47]`、`[87][141]`、`[75][120]`、`[74][64]` | `main.c:190-198` |

---

## 6. 执行子系统

### 6.1 舵机

| 项 | 值 | 来源 |
|---|---|---|
| PWM 通道 / 引脚 | `PWME_CH3P_PA4`（**PA4**） | `Motor.c:48` / `Motor.c:453,487` |
| 频率 | **200 Hz**（周期 5 ms） | `Motor.c:453,487` |
| duty 单位 | 1 duty = **0.5 µs** | `Motor.h:17` |
| 电信号中值 | `steerMid = 2975` duty = 1487.5 µs | `Motor.c:10` |
| 机械中值修正 | `STEER_TRIM = −58` duty → 实际中心 **2917 duty = 1458.5 µs**，抵消机械右偏约 3.5°（舵机转角约 −3.52°） | `Motor.h:43` / `Motor.c:451` |
| 单边限幅 | `STEER_DUTY_LIMIT = 395` duty = 197.5 µs ≈ 24° 行程（相对修正中心限幅） | `Motor.h:21` |
| 换算 | `currentDuty = clamp(转角 × c1 / 20, ±395)`；`c1 = 260` 时 13 duty/度，30° → 390 duty | `Motor.h:25` / `Motor.c:387-388` |
| 转向环 PID | `Kp = 100, Ki = 0, Kd = 100`（直通 + 阻尼，增益由 `c1` 决定） | `Motor.c:478-480` |
| 上电 / B4 自检 | `SERVO_TEST_SWING = 495` duty ≈ 30°，以 2917 为中心左右摆动，范围 2422~3412 duty（1211~1706 µs），约 1.4 s，结束回机械 90° | `Motor.h:72` / `Motor.c:449-468` / `track_bias_check.py` 实测输出 |

### 6.2 双直流电机

| 项 | 右轮 | 左轮 | 来源 |
|---|---|---|---|
| PWM 通道 | `PWMB_CH3_P76`（**P76**） | `PWMB_CH1_P74`（**P74**） | `Motor.c:7,9` |
| PWM 频率 | 17 kHz | 17 kHz | `Motor.c:490-491` |
| 方向脚 DIR | `IO_P77` | `IO_P75` | `Motor.c:6,8` |
| DIR 初始电平 | 1（推挽输出） | 1 | `Motor.c:488-489` |
| 前进电平 | `MOTOR_DIR_FORWARD = 0`（实测结论） | 同 | `Motor.h:180` |
| 占空比上限 | `MOTOR_DUTY_LIMIT = 10000`（满量程 100 %） | 同 | `Motor.h:145` |
| 刹车反向限幅 | `MOTOR_BRAKE_DUTY = 1500`（刹车段 `min_out`，防倒车） | 同 | `Motor.h:150` / `Motor.c:269-275` |
| 速度环 PID | `Kp = 1000, Ki = 50, Kd = 0`（增量式，输入单位"脉冲 / 5 ms"） | 同 | `Motor.c:481-486` |
| 差速 | `diff = setSpeed × 8 / R`，限幅到 `setSpeed / 2`；`MOTOR_DIFF_TRIM = 0` | — | `Motor.c:257-265` |
| 起跑助推 | `MOTOR_KICK_TICKS = 0`（**已关闭**），`MOTOR_KICK_DUTY = 4500` 保留 | — | `Motor.h:185-186` |
| 开环/爬坡诊断 | `MOTOR_OPEN_LOOP = 0`、`MOTOR_DUTY_RAMP = 0`、`MOTOR_RUN_DUTY = 1800`、`MOTOR_DUTY_PER_UNIT = 8` 均为关闭状态的调试保留项 | — | `Motor.h:74-81,156-172` |

> **驱动芯片型号、电机型号/减速比、供电电压与电池规格：仓库未提供**（见附录 C）。

### 6.3 双路正交编码器

| 项 | 右轮 | 左轮 | 来源 |
|---|---|---|---|
| 解码外设 | `PWMA_ENCODER` | `PWMC_ENCODER` | `Motor.c:492-493` |
| A / B 相引脚 | `PWMA_ENCODER_CH1P_P60` / `CH2P_P62`（**P60 / P62**） | `PWMC_ENCODER_CH1P_P40` / `CH2P_P42`（**P40 / P42**） | 同上 |
| 计数符号 | `cntR = −count(PWMA) × 2` | `cntL = +count(PWMC) × 2` | `Motor.c:200-205` |
| 读取周期 | 5 ms（TIM1），读完立即 `encoder_clear_count()` | 同 | `Motor.c:200-203` |
| 车速 | `currentSpeed = (cntR + cntL) / 2`，限幅 ±32767 | — | `Motor.c:206-207` |
| 里程 | `odometer += avg`（IDLE 状态也累加，供推车标定） | — | `Motor.c:208` |
| 每米脉冲 | `PULSE_PER_METER = 1000`（**占位值，未实测**） | — | `Stop.h:20` |
| 掉线判据 | 有速度指令且 PID 输出 > 2000、该轮 300 ms 无计数 → `FAULT_ENC_L_DOWN / R_DOWN` | — | `Motor.c:81-91` |

---

## 7. 人机交互与调试接口

### 7.1 按键与拨码

| 器件 | 引脚 | 电气 | 功能 | 来源 |
|---|---|---|---|---|
| 拨码 SW1 | `IO_PB0` | GPI 上拉，低电平有效 | 上电选档基址 | `main.c:241,287` |
| 拨码 SW2 | `IO_PB1` | GPI 上拉，低电平有效 | 上电选档基址 | `main.c:240,286` |
| 按键 B2 | `IO_PB2` | GPI 上拉，低有效 | 短按发车（无赛道门槛） | `main.c:242,317-322` |
| 按键 B3 | `IO_PB3` | GPI 上拉，低有效 | 任意状态急停（清 PWM + 状态位） | `main.c:243,324-327` |
| 按键 B4 | `IO_PB4` | GPI 上拉，低有效 | 短按舵机自检；长按 2 s 标定一圈 | `main.c:244,329-338` |
| 时序基准 | `msTick`（TIM1 每 5 ms +5） | — | 消抖 20 ms、B4 长按 2000 ms、按键反馈 60 ms | `main.c:103-104,115-146` / `Motor.c:197` |

**强制约束**：按键与拨码必须在 `mt9v03x_init()` **之后**配置，否则摄像头初始化重配端口会导致按键全部失效（README 第 326 行，Git 提交 `25da8ae` 修复此问题）。

选档映射：`idx = ((SW2 ? 2 : 0) + (SW1 ? 1 : 0)) × 2 + 1` → 取 speedSet 索引 1/3/5/7（4 组奇数档）。

### 7.2 蓝色状态灯

| 项 | 值 |
|---|---|
| 引脚 | `STATUS_LED_PIN = IO_P52`，GPO 推挽 |
| 电平 | **低电平点亮**（`STATUS_LED_ON = 0` / `OFF = 1`：阴极接引脚、阳极经电阻接 3.3 V） |
| 上电 | 闪 3 下（亮/灭各 300 ms） |
| 摄像头初始化失败 | 500/500 ms 慢闪并停机 |
| 行驶指示 | 编码器正向计数 → 约 5 Hz 快闪（`LED_FAST_MS = 100`）；反向计数或故障 → 常亮；静止 → 灭 |
| 按键反馈 | 短亮 60 ms（非阻塞 `trackLedHold`） |
| 标定反馈 | 成功闪 2 下 / 失败闪 4 下 / 未标定拒绝发车闪 3 下 |

来源：`Motor.h:104-109`、`Track.c:14-16,55-95`、`main.c:206-215`、`Stop.c:88-103`。

### 7.3 蜂鸣器与屏幕

| 项 | 状态 |
|---|---|
| 蜂鸣器 `IO_P90` | 本车**未装**，`gpio_init(IO_P90, GPO, 0, PUSH_PULL)`；`Stop.c` / `Cross.c` / `LRound.c` / `RRound.c` 仍写该脚，蓝灯代替报警 |
| `BEEP_ON_LEVEL = 1` / `OFF = 0` | `Motor.h:98-99`（若报警无声音可对调） |
| IPS200 屏幕 | **硬件已移除**，`ips200_init()` 从未调用；`zf_device_ips200.c` 仍参与编译（链接 `REMOVEUNUSED` 剔除） |
| 残留屏幕调用 | 源码中仍有 11 处 `ips200_*`，全部位于 `#if DEBUG_MODE` / `#if ENABLE_SCREEN` / `#if CALIB_REQUIRED` 条件块内（`DEBUG_MODE=0`、`CALIB_REQUIRED=0`、`ENABLE_SCREEN` 未定义），**运行时均不生效** |

### 7.4 USB CDC 调试串口

| 项 | 值 | 来源 |
|---|---|---|
| 使能宏 | `USER_USB_CDC = 1`、`PRINTF_ENABLE = 1` | `zf_common_debug.h:42-43` |
| 初始化 | `debug_init()` → `usb_cdc_init()` → `usb_init()`；不启用 UART1 | `zf_common_debug.c:226-230` |
| USB 规格 | USB 2.0 Full-Speed，CDC ACM（bDeviceClass 0x02），2 接口，总线供电 100 mA | `usb_desc.c` |
| VID / PID | `0x34BF` / `0xFF02`，`bcdDevice 1.00` | `usb_desc.c:25-27` |
| 端点 | EP0 64 B；启用 EP1IN / EP2IN / EP1OUT，各 64 B | `config.h:4-47` |
| USB 时钟 | `FOSC 24 MHz`（配置宏）；`usb_init()` 中 `IRC48MCR` 启动段被注释，`USBCLK = 0x00`、`USBCON = 0x90` | `config.h:4` / `usb.c:38-45` |
| USB 中断优先级 | `IP2 \|= 0x80`、`IP2H \|= 0x80`（最高级），`EUSB = 1` | `usb.c:66-68` |
| 备用 UART1 | `UART_1 / 115200 / TX_P37 / RX_P36`（仅 `USER_USB_CDC=0` 时启用；注意 P36/P37 已被摄像头 DMA/FIFO 占用） | `zf_common_debug.h:47-50` |
| printf 重定向 | 逐字节 `usb_cdc_write_byte()`；未连接时 `usb_cdc_write_buffer()` 立即返回 | `zf_common_debug.c:166-174` |
| 输出周期 | 待发车 250 ms / 跑车中 1000 ms | `main.c:177` |
| 字段 | `ODO L R C THR MODE AL STA ERR SPD DS DUTY` | `main.c:182-184` |
| 元素码 | 114 循迹 / 514 十字 / 810 左环岛 / 1919 右环岛 | `Motor.h:114-117` |

---

## 8. 完整引脚分配表

> 本表汇总工程内**所有**出现的引脚占用。P0~PB 端口命名沿用逐飞库 `IO_Pxx` 宏。

| 引脚 | 功能 | 方向 / 模式 | 电气 / 时序 | 代码位置 | 备注 |
|---|---|---|---|---|---|
| P20~P27 | 摄像头 8 bit 数据 D0~D7 | GPI 上拉 | 并行，随 FIFO RCK | `mt9v03x.h:51-58` | **不可改** |
| PB6 | 摄像头 SCCB SCL | 软件 IIC 输出 | ~50 kHz | `mt9v03x.h:47` | |
| PB7 | 摄像头 SCCB SDA | 软件 IIC 双向 | ~50 kHz | `mt9v03x.h:48` | |
| P33 | FIFO VSY / `INT1_P33` | GPI 上拉 / 外部中断 | 双边沿，优先级 3 | `dma.h:11` / `mt9v03x.c` | **不可改** |
| P37 | FIFO RCK | GPO 推挽，初值 1 | — | `dma.h:12` | **不可改**；与 UART1_TX 复用宏冲突（未启用） |
| P41 | FIFO WE | GPO 推挽，初值 0 | — | `dma.h:13` | **不可改** |
| P73 | FIFO WRST | GPO 推挽，初值 1 | — | `dma.h:14` | **不可改** |
| P72 | FIFO OE | GPO 推挽，初值 0 | — | `dma.h:15` | **不可改** |
| P43 | FIFO RRST | GPO 推挽，初值 1 | — | `dma.h:16` | **不可改** |
| P45 | 摄像头 DMA 占用 | 库内硬编码 | — | `dma.h:8-10` 注释 | **不可改**，无源码 |
| P36 | 摄像头 DMA 占用 | 库内硬编码 | — | `dma.h:8-10` 注释 | **不可改**；与 UART1_RX 复用宏冲突（未启用） |
| PA4 | 舵机 PWM `PWME_CH3P` | PWM 输出 | 200 Hz，中心 2917 duty | `Motor.c:48,453,487` | |
| P76 | 右电机 PWM `PWMB_CH3` | PWM 输出 | 17 kHz，0~10000 | `Motor.c:7,490` | |
| P77 | 右电机 DIR | GPO 推挽 | 前进 = 0 | `Motor.c:6,488` | |
| P74 | 左电机 PWM `PWMB_CH1` | PWM 输出 | 17 kHz，0~10000 | `Motor.c:9,491` | |
| P75 | 左电机 DIR | GPO 推挽 | 前进 = 0 | `Motor.c:8,489` | |
| P60 | 右编码器 A 相 `PWMA_CH1P` | 正交解码输入 | — | `Motor.c:492` | 计数取负 |
| P62 | 右编码器 B 相 `PWMA_CH2P` | 正交解码输入 | — | `Motor.c:492` | |
| P40 | 左编码器 A 相 `PWMC_CH1P` | 正交解码输入 | — | `Motor.c:493` | 计数取正 |
| P42 | 左编码器 B 相 `PWMC_CH2P` | 正交解码输入 | — | `Motor.c:493` | |
| PB0 | 拨码 SW1 | GPI 上拉 | 低有效 | `main.c:241,287` | |
| PB1 | 拨码 SW2 | GPI 上拉 | 低有效 | `main.c:240,286` | |
| PB2 | 按键 B2 发车 | GPI 上拉 | 低有效，20 ms 消抖 | `main.c:242` | |
| PB3 | 按键 B3 急停 | GPI 上拉 | 低有效，20 ms 消抖 | `main.c:243` | |
| PB4 | 按键 B4 自检/标定 | GPI 上拉 | 低有效，长按 2 s | `main.c:244` | |
| P52 | 核心板蓝色 LED | GPO 推挽 | **低电平点亮** | `Motor.h:104-109` | |
| P90 | 蜂鸣器 | GPO 推挽 | `BEEP_ON = 1` | `main.c:245` / `Stop.c:111` | **本车未装** |
| P04 | 示波器调试脚 | — | — | `main.c:275` | 注释状态，未使用 |
| USB D+/D− | 内置 USB 全速 | USB PHY | 12 Mbps | `usb.c` / `usb_desc.c` | 具体引脚由芯片固定 |
| P3.0 / P3.1 | SWD 仿真（公开资料） | — | — | STC 官方 | 工程未配置，仅说明 |

**引脚使用注意**：
1. `P36/P37` 同时出现在摄像头 DMA/FIFO 占用与备用 `DEBUG_UART` 宏里；当前 `USER_USB_CDC=1`，UART1 分支不启用，不冲突，但改回 UART 调试前必须复核。
2. 按键 / 拨码 / 蓝灯的 `gpio_init` 必须在 `mt9v03x_init()` 之后；`main.c:246` 在摄像头初始化后又重配一次蓝灯。
3. README 第 7 节的引脚表未列出 **P45 / P36**（DMA 额外占用）与 **USB / SWD**，本表已补全。

---

## 9. 时钟、中断与并发

### 9.1 时钟与节拍

| 项 | 值 |
|---|---|
| 系统时钟 | 96 MHz（内部 IRC，由 `clock_init(SYSTEM_CLOCK_96M)` 设定） |
| 控制周期 | `pit_ms_init(TIM1_PIT, 5, pitHandler)` = **5 ms**，200 Hz，与舵机 PWM 同频 |
| 毫秒节拍 | `msTick += 5`（uint16，所有计时用差值，容忍回绕） |
| 摄像头帧间隔 | 170 fps → 约 5.88 ms |
| 帧看门狗 | 1000 ms 无新帧 → 紧急停车（`main.c:417-421`） |
| 元素超时 | 8000 ms 强制回 `FORWARD`（`main.c:409-412`） |
| 发车宽限期 | 出界判定 800 ms 内不生效；异常检测 2 s 内不判（400 × 5 ms） |

### 9.2 中断向量工程化

C251 的 `interrupt N` 只支持 0~31，而芯片扩展中断真实向量号 37~126。本工程做法：

- `isr.c`：仅保留 0~31 基础中断（INT0/2/3/4、TM0~TM4、INT1_VSYNC），编译器自动生成向量表。
- `isr_extended_0.c` / `isr_extended_1.c`：`#pragma noiv`，函数用局部编号 0~30 声明。
- `isr_extended_vectors.A51`：在真实地址 `CSEG AT 012BH … 031BH` 放 `EJMP` 远跳转，共 **26 个**扩展向量（P0~P9、DMA_UART1~8、DMA_LCM、TM11、PA/PB、TM5/6/7/9/10/17/18）。
- 向量地址公式：`3 + 8 × 向量号`；DMA_LCM 为向量 58 / 地址 `01D3H`。
- **禁止使能未实现的扩展中断**（其余向量无默认陷阱）。

### 9.3 中断优先级与上下文

| 中断 | 优先级 | 职责 |
|---|---|---|
| TIM1（5 ms 控制 ISR） | **3** | 编码器、异常检测、出界消抖、差速、双轮 PID、PWM、舵机、msTick |
| INT1 / VSYNC | **3** | 摄像头帧同步（`interrupt_set_priority(INT1_IRQn, 3)`，库内设置） |
| DMA_LCM | **1**（初始化时为 3，main 下调） | `mt9v03x_dma_handler()` + 22.5 KB 整帧搬运 |
| USB | 最高（IP2/IP2H） | CDC 传输 |

三个上下文：主循环（每帧，帧门控外仍跑按键与 `raceTask`）、TIM1 ISR（5 ms）、DMA_LCM ISR（每帧）。共享量保护：`desiredSpeed` 关中断写入 `setSpeedCommand()`；`odometer` 32 位读取前关中断取快照；`outOfTrack` 单字节；`faultFlags` 16 位只在状态变化时读；`msTick` 用差值比较。

> **未实测风险**：C251 高优先级中断能否抢占正在执行的 DMA_LCM ISR（22.5 KB 拷贝）决定 5 ms 控制周期的最坏延迟。README / EVALUATION 建议用 `IO_P04` 翻转 + 示波器确认，本报告未做板上验证。

---

## 10. 软件架构

### 10.1 源文件清单

| 分组 | 文件 | 行数 / 职责 |
|---|---|---|
| 入口 | `project/user/main.c` | 627：上电流程、按键、元素分派、`loadRoad()`、`buildCenterLine()`、出界判定、调试输出 |
| 中断 | `isr.c` 190 / `isr_extended_0.c` 298 / `isr_extended_1.c` 222 / `isr_extended_vectors.A51` / `isr.h` 40 | 基础中断、26 个扩展中断、向量跳转表 |
| 感知 | `project/code/Image.c` 2697（含 45 KB `reflection` 表）/ `Image.h` 60 | 双缓冲、阈值、起点搜索、八邻域爬线、去畸变（当前旁路）、滤波、重采样、拐点 |
| 决策 | `Motor.c` 513 / `Motor.h` 198 | 纯跟踪、弯道前馈/增益/减速、曲率限速、PID、差速、异常检测、舵机自检 |
| 元素 | `Cross.c` 149 / `LRound.c` 75 / `RRound.c` 79 / `BendEntry.c` 186 + 各自 .h | 十字 2 阶段、左右环岛各 5 阶段、入弯过渡区 4 状态 |
| 比赛 | `Stop.c` 135 / `Stop.h` 48 | 里程、发车、一圈标定、刹车段、定点停车 |
| 指示 | `Track.c` 95 / `Track.h` 16 | 赛道判定（50 ms 消抖）、蓝灯驱动 |
| 底层 | `libraries/` 四个目录 + `stc32g_usb_lib` + `seekfree_assistant` | 逐飞 V3.2.5，含 2 个预编译 `.LIB` |
| 工程 | `project/mdk/seekfree.uvproj`、`MDK删除临时文件.bat`、`out_file/` | Keil 工程与产物 |
| 测试 | `tests/*.py` 8 个 | Python 数值核算 / 行为复现（非板上运行） |

### 10.2 数据流（严格单向四层）

| 层 | 入口 | 产出 |
|---|---|---|
| 采集 | `DMA_LCM_IRQHandler()` | `mt9v03x_frame`、`frameBusy` |
| 感知 | `loadRoad()` | `LRoad/RRoad/centerLine`、拐点、阈值 |
| 决策 | `main.c` 主循环 | `RUN_MODE`、`ALONG`、`borderIn`、`desiredSpeed`、`currentDuty` |
| 执行 | `pitHandler()` | 左右轮 PWM、舵机脉宽、`faultFlags` |

元素识别读**去畸变前**的原始边界（`LBorder/RBorder`），控制读**融合后**的中线，两层不混用（EVALUATION 第 19 行）。

### 10.3 关键参数全集

**速度与舵机（`Motor.h` / `Motor.c`）**

| 参数 | 值 | 单位 / 含义 |
|---|---|---|
| `SPEED_LIMIT` | 400 | 脉冲 / 5 ms，唯一速度天花板 |
| `MOTOR_DUTY_LIMIT` | 10000 | 占空比满量程 |
| `MOTOR_BRAKE_DUTY` | 1500 | 刹车段反向占空比上限 |
| `STEER_DUTY_LIMIT` / `DIV` | 395 / 20 | duty / 换算分母 |
| `STEER_TRIM` / `TRACK_BIAS_PX` / `MOTOR_DIFF_TRIM` | −58 / 0 / 0 | 三项独立轨迹偏置 |
| `BORDER_IN_SPEED_RATIO` | 50 | %，与弯道减速比例取更严格值（不连乘） |
| `BEND_STEER_PCT / MAX / GAIN_BEND / GAIN_RATE` | 105 / 155 / 15 / 5 | %，弯道转向自适应增益 |
| `BEND_SLOW_START / GAIN / MIN` | 4 / 3 / 40 | 弯道减速触发、斜率、比例下限 |
| `BEND_SPEED_CAP / BEND_RELEASE` | 200 / 2 | 减速态硬上限 / 解除滞回 |
| `BEND_FAR_AHEAD / BEND_FEED_DIV` | 12 / 3 | 远端趋势采样点偏移 / 前馈分母 |
| `DEVICE_H` | 18 | 纯跟踪等效轴距（像素） |
| `SERVO_TEST_SWING` | 495 | duty ≈ 30° |
| `LOW_SPEED_TEST / CAP` | 0 / 150 | 试车模式（关闭） |

**八档整定参数（`main.c:76-85`）**：`{c1 舵机增益, c3 前瞻点序号, c4 侧向加速度上限, c5 直道限速}`

| 档 | c1 | c3 | c4 | c5 | | 档 | c1 | c3 | c4 | c5 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 260 | 26 | 2400 | 400 | | 5 | 230 | 24 | 2000 | 360 |
| 2 | 200 | 22 | 2000 | 300 | | 6 | 240 | 24 | 2000 | 380 |
| 3 | 210 | 23 | 2000 | 320 | | 7 | 250 | 25 | 2000 | 400 |
| 4 | 220 | 23 | 2000 | 340 | | 8 | 260 | 25 | 2000 | 400 |

`getAimPos()` 在 `c3` 基础上按 `currentSpeed / 40` 每 40 单位加 1 点，最多 +20。曲率限速 `R = 20000 / tan(δ)`，`v = min(√(R·c4), c5)`；`c4=2000, c5=400` 时 0–10° → 400、15° → 384、20° → 328、25° → 289、30° → 260（`numeric_check.py` 实测一致）。

**图像（`Image.h` / `Image.c` / `main.c` / `Motor.c`）**

| 参数 | 值 | | 参数 | 值 |
|---|---|---|---|---|
| `block` / `cap` | 5 / 4 | | `expRoadCount` | 100 |
| `EDGE_DIFF` / `EDGE_RATIO` | 12 / **7** | | `CORNER_COS100` | 50 |
| `HIDE_ROW` / `BODY_ROW` | 90 / 80 | | `HIDE_COL_L/R` | 48 / 140 |
| `FAR_CUT_ROW` | 20（未提交新增） | | 起点扫描下限 | 50（未提交，原 70） |
| `USE_REFLECTION` | **0**（未提交新增） | | `centerLineDistance` | 18（拐角 9，实测宽 8~60 限幅） |
| `MERGE_WIDTH_MIN/MAX` | 12 / 130 | | `MERGE_ROW_TOL` | 15 |
| 边界 / 中线重采样 | dis 3 / dis 2 | | 爬线步长 / 转向上限 | 2 px / 4 次 |

**元素判据与里程**

| 参数 | 值 | | 参数 | 值 |
|---|---|---|---|---|
| `ROUND_ROW` / `ROUND_NEAR_X` / `ROUND_FAR_MAX` / `CROSS_MAX` | 37 / 20 / 11 / 14 | | `ENABLE_CROSS` / `ENABLE_ROUND` | 1 / 1 |
| `TRACK_CONFIRM_MS` / `LED_FAST_MS` | 50 / 100 | | 出界消抖 | 3 拍 = 15 ms |
| `CALIB_REQUIRED` | **0**（不标定也发车、不自动刹停） | | `PULSE_PER_METER` | **1000（占位）** |
| `LAP_DISTANCE_CM` / `STOP_OFFSET_CM` / `BRAKE_DISTANCE_CM` | 1200 / 100 / 200 | | 标定门槛 | `LAP_PULSE_DEFAULT / 4` |

**弯道入口过渡区（`BendEntry.h`）**

| 参数 | 值 | | 参数 | 值 |
|---|---|---|---|---|
| `ENABLE_BEND_ENTRY` | 1 | | `BE_BEND_MIN / EXIT` | 10 / 4 |
| `BE_CONFIRM_FRAMES` | 2 | | `BE_BLANK_MS` | 250 |
| `BE_MAX_LOST_MS` | 900 | | `BE_RECOVER_POINTS / MIN_POINTS` | 15 / 8 |
| `BE_TURN_DECAY` | 1 | | 状态 | STRAIGHT / APPROACH / COMMITTED / RECOVER |

**故障位（`Motor.h:86-91`）**：`FAULT_STOPPED 0x0001`、`FAULT_ENC_L_DOWN 0x0002`、`FAULT_ENC_R_DOWN 0x0004`、`FAULT_PID_SAT 0x0100`、`FAULT_PID_REV 0x0200`、`FAULT_OUT 0x0400`。任一编码器/PID 异常 → `FAULT_STOPPED` + 停车；出界分支单独置 `FAULT_OUT`。

---

## 11. 编译、烧录与产物

### 11.1 工具链（实测，来自 build_log）

| 组件 | 版本 |
|---|---|
| uVision IDE | V5.25.3.0 |
| PK251 Prof. Developers Kit | 5.60.0.0 |
| C251.exe / A251.exe | V5.60.0.0 / V4.69.13.0 |
| L251.exe / LIB251.exe | V4.66.93.0 / V4.30.1.0 |
| OH251.exe | V1.47.0.0 |
| CPU / Target DLL | S251.DLL V3.129.0.0 / STCMON251.DLL |
| 构建耗时 | 00:00:05 |

### 11.2 编译结果：文档记录 vs 实际产物

| 来源 | data | xdata | const | code | 告警 |
|---|---:|---:|---:|---:|---|
| README 第 27 行 | 102.5 | 51893 | 48754 | 82777 | 1（C183） |
| REVIEW 第 11 行 | 102.5 | 51893 | 48754 | 82777 | 1（C183） |
| REVIEW 8.1「当前」列 | 102.5 | 51854 | 48754 | 81370 | 1（C183） |
| docs 7.3 节 | 102.5 | 51893 | 48754 | 82777 | 1（C183） |
| **实测 `build_log` / `MAP`（2026-11-15 11:13:30）** | **101.5** | **51893** | **48754** | **82662** | **1（C183）** |

唯一告警：`zf_device_mt9v03x.c(138): warning C183: dead assignment eliminated`（逐飞库内部，非应用层）。链接器 `L251 RUN COMPLETE. 0 WARNING(S), 0 ERROR(S)`。

> 四份文档给出三组不同数字，与最新产物又不一致。原因推测为不同时间点的重建结果，但**文档未标注构建时间**，本报告以 `SEEKFREE.map` / `build_log.htm` 为准。

### 11.3 构建与烧录

1. Keil MDK FOR C251 打开 `project/mdk/seekfree.uvproj`，目标 `stc32g144k246` → Rebuild。
2. 命令行：`UV4.exe -r project\mdk\seekfree.uvproj -j0 -o build.log`（需先关闭运行中的 uVision，否则编译被跳过）。
3. 产物 `project/mdk/out_file/SEEKFREE.hex`，用 **STC-ISP** 下载（芯片支持 USB 直接下载与串口 ISP）。
4. 清理：`project/mdk/MDK删除临时文件.bat`（`rd Out_File /s/q` 等）。
5. `.gitignore` 忽略 `out_file/`、`*.hex`、`*.map`、`*.obj`、`*.build_log.htm` 等；**当前本地存在这些产物但不在版本库中**。

---

## 12. 测试与验证（本次实际执行）

`tests/` 下 8 个脚本均为 **Python 数值核算 / 行为复现**，不编译也不运行 C 代码、不连接实车。执行环境 Python 3.13.14，全部以退出码 0 结束（脚本本身不使用失败退出码，故以输出判定为准）。

| 脚本 | 实测结论 | 判定 |
|---|---|---|
| `numeric_check.py` | `atan2_int` 在输入域返回 −30~30；`tani` 表偏差最大 3；`sqrt_int` 与真值一致（x=80,000,000 → 8944）；曲率限速曲线与文档一致 | 通过 |
| `bend_recognition_check.py` | 直道 0.00°；右弯前瞻仍在直道 −4.86°（含增益）/ 左弯提前转向；左侧 40 点右侧 8 点仍输出 40 点、中点列号 98（真值 100）；拐点 6 截断时中线仍 40 点 | 通过 |
| `bend_slowdown_check.py` | 入弯初期目标降到 200；弯心比例保持 40、速度 160/160/148/131/128/128；出弯回 400；`BEND_SPEED_CAP` 参数扫描符合预期 | 通过 |
| `body_row_check.py` | 车身遮住左边界时：参考起点 `(89,62)` 假边界，当前 `BODY_ROW=80` 得 `(77,31)` 真边界；无车身时两者均 `(116,31)` | 通过 |
| `out_of_track_check.py` | 7 组场景全部符合脚本内「上一版 / 当前」对照；发车 0/200/799 ms 不判、800 ms 起判；总判定「全部符合预期」 | **通过（但脚本判据已过时，见 13-A2）** |
| `bend_entry_check.py` | 完整入弯序列、直道误丢边、超时、转角保持衰减；关键对照：STRAIGHT 下 3 点暗停车、COMMITTED 前 250 ms 内 5 点暗都不停 | 通过 |
| `track_bias_check.py` | `STEER_TRIM −58` → 舵机 −3.52° / 图像 −4.46°；自检中心 2917 duty（1458.5 µs），范围 2422~3412 duty，约 30.1°，约 1.4 s | 通过 |
| **`reference_alignment_check.py`** | **「总判定: 存在超出范围的差异」**；`Image.c` 删除 4 行 / 新增 30 行、`Image.h` 删除 1 行 / 新增 18 行，均超出脚本「只允许 BODY_ROW 两行差异」的白名单；9 个关键宏、赛道判据 8 组组合、上电初值仍一致 | **失败** |

**失败根因**：脚本白名单按 2026-11-14 的基线编写，而工作区新增了 `USE_REFLECTION`、`cutFarRegion()`、起点扫描下限 70→50 等 4 类现场改动。这不是代码缺陷，是**改动未回归测试**。

**另一项可复现性缺陷**：该脚本 `REF = ROOT/backup/version1.3/project`，`ROOT` 是 `version1.0` 的父目录；参考实现位于本仓库**之外**的 `F:\项目文件夹\Racing Car C-model\backup\version1.3\`。只复制 `version1.0` 目录分发时，对齐核对将无法运行。

---

## 13. 文档—源码一致性审计

> 本节是"严格"报告的核心：以下均为本次核验发现的**文档与当前工作区不一致**，按影响程度排序。

| 编号 | 项目 | 文档说法 | 源码 / 产物实际 | 影响 | 建议 |
|---|---|---|---|---|---|
| **A1** | 透视去畸变 | README 第 2/6.1 节、docs 第 2 节把"查 `reflection` 表映射到俯视"列为生效步骤 | `Image.h:18` `USE_REFLECTION = 0`，`getGodRoadLine()` 走 `#else` 空实现，保留斜视坐标 | 几何基准变化，所有基于俯视坐标的调参经验与 `docs` 推导不再成立 | 文档标注"当前旁路"；重新标定时同步更新表 |
| **A2** | 出界保护 | README 6.5、EVALUATION 三、REVIEW C-14、docs 五：判据含「两侧边界全丢且 5 点中 ≥3 点暗」 | `main.c:367-372` 三分支均简化为 `dark >= 5`（guard=0 分支的 3 点暗条件已删） | 出界保护显著放宽，`out_of_track_check.py` 仍在验证旧判据，测试与实现脱节 | 更新判据文档并同步修改/新增测试用例 |
| **A3** | 图像链路与参考实现一致性 | README 第 2 节「与参考实现逐字节一致」；docs 第 1 节「差异仅 BODY_ROW 两行」 | 工作区 `Image.c` 另有 `cutFarRegion()`、扫描下限 50、`#if USE_REFLECTION`；`git diff` 共 52 行新增 | 对齐核对脚本判定失败 | 提交前先更新白名单或回退到受控差异 |
| **A4** | `EDGE_RATIO` | README 8.3 表格写 **10** | `Image.h:21` = **7**；`REVIEW` / `docs` / `reference_alignment_check.py` 均为 7 | 现场调参依据混淆 | 统一改 README |
| **A5** | 编译统计 | 四份文档给出 3 组数字（82777 / 81370 …） | 实测 `code=82662`、`data=101.5` | 无法据文档判断产物版本 | 以 build_log 为准并注明构建时间 |
| **A6** | XDATA 余量 | README 第 30 行 13.4 KB、第 597 行 13.8 KB | 实测 `xdata=51893`；65,536−51,893≈13.3 KB，但 uvproj 配置 128 KB XRAM | 余量口径不清，影响"能否加 DMA 双缓冲"的结论 | 明确链接器 XDATA 上限后统一 |
| **A7** | 速度量纲 | README 8.1/8.2 以 `PULSE_PER_METER = 34600` 换算（400 ≈ 2.31 m/s） | `Stop.h:20` = **1000**（占位），公式 `v = setSpeed × 200 / PPM` 代入得 80 m/s | 未标定前所有 m/s 数值无意义；EVALUATION 第 93 行已承认未实测 | 推车 1 m 实测后填回，文档标注占位 |
| **A8** | IPS200 调用 | `Motor.h:111`、README 第 19 行「工程内不再有任何 `ips200_*` 调用」 | 源码仍有 **11 处**（`Cross.c:82-84`、`Motor.c:371,390`、`Stop.c:33-36`、`Image.c:437` 注释），全部在条件编译块内；`zf_device_ips200.c` 仍参与编译 | 表述不严谨（应为"无生效调用"） | 改文档措辞或清理死代码 |
| **A9** | 舵机自检摆幅 | README 第 9 节 30°、`Motor.h` 30°、`main.c:333` 注释 15°、REVIEW 8.8 期望 15° | `SERVO_TEST_SWING = 495` duty ≈ **30.1°**（`track_bias_check.py` 实测） | 现场验收标准矛盾 | 统一为 30° |
| **A10** | 赛道状态用途 | README 第 49 行功能清单写"用于发车许可与出界保护"；而 README 6.4 节第 237 行又写"不阻断发车"（文档自相矛盾） | `main.c:317-322` B2 直接发车，`trackIsOn()` 仅用于串口 `STA` 与指示 | 功能边界误述 | 统一为"仅指示，不阻断发车" |
| **A11** | 弯道增益范围 | EVALUATION 三、REVIEW C-17 写 105%~130% | `Motor.h:51-54` 与 `Motor.c:363-367` 为 **105%~155%**（斜率 5，强度 25 到顶） | 参数表过时 | 统一为 155% |
| **A12** | 入弯连接处时长 | README 6.6、REVIEW C-16 写「600 ms 后交还」 | `BE_MAX_LOST_MS = 900`、`BE_BLANK_MS = 250`、`BE_TURN_DECAY = 1`；README 参数表本身写 900/250/1，正文自相矛盾 | 状态机时序描述错误 | 统一为 900 ms |
| **A13** | 按键注释 | `main.c:97-99` 注释仍写「B2 需已识别到赛道 / 长按 1 s 强制发车」 | 代码已取消赛道门槛与长按发车；`BTN_FORCE_MS` 已删除（REVIEW 第七节） | 注释误导维护者 | 清理残留注释 |
| **A14** | 外部依赖 | README 第 3 节文件表未列参考实现 | `reference_alignment_check.py` 依赖仓库外的 `backup/version1.3/project` | 单独分发后测试不可复现 | 把参考基线纳入仓库或改为可选 |
| **A15** | `CLOCK(35000000)` | — | uvproj 仿真 35 MHz vs 运行 96 MHz（README 第 96 行已说明只影响仿真/波特率） | 无运行影响，属已知项 | 保留说明 |
| **A16** | 蜂鸣器 | `Motor.h:91` 注释仍写「同时蜂鸣报警」 | 本车未装蜂鸣器，用蓝灯代替；但 4 个模块仍写 `IO_P90` | 硬件描述与实际装置不符 | 文档标注"未装/保留驱动" |
| **A17** | README 8.3 `BEND_FEED_DIV` | 表格写 3，正文 6.3 节公式写 `/2` | `Motor.c:303` = 3 | 公式与代码不一致 | 统一为 3 |

---

## 14. 风险与遗留问题

**P0（影响可交付性）**

1. **3 个源文件未提交**，且已使 `reference_alignment_check.py` 失败；源码、文档、测试脚本三者处于不同基线。
2. **`PULSE_PER_METER` 未实测**、`CALIB_REQUIRED = 0`：定点停车功能当前**不具备闭环里程保证**，只能靠 B3 手动停车。
3. **透视查表被旁路但 45 KB `const` 仍占用**；`reflection` 表与当前 20 cm/45° 安装不匹配，若误把 `USE_REFLECTION` 改回 1 会直接把中线拉偏。

**P1（实车风险）**

4. C251 高优先级中断能否抢占 DMA_LCM ISR 的 22.5 KB 搬运**未实测**，决定 5 ms 控制周期最坏延迟（建议 `IO_P04` + 示波器）。
5. `zf_device_mt9v03x_dma.LIB` 与 `zf_device_config.LIB` 为预编译库，采集行为与 P45/P36 占用无法从源码审计。
6. 出界判据放宽到「5 点全暗」后，EVALUATION 第 97 行指出的"低速贴边滑出、画面仍有亮区不触发"窗口进一步放大；`out_of_track_check.py` 仍在测旧判据。
7. 元素判据（`ROUND_ROW` 等）、`BODY_ROW`、`HIDE_ROW`、`EDGE_*` 都是像素量化值，与镜头安装强绑定；安装已从 25 cm/39° 改为 20 cm/45°，需重新标定。
8. 弯道减速较深（比例下限 40% + 硬限幅 200），长弯目标速度 128~160，接近失速（EVALUATION 第 96 行）。
9. 轨迹偏置为静态补偿，换电池/轮胎磨损后会漂；每场赛前应用 `DUTY` 稳态均值复核。

**P2（工程整洁度）**

10. 逐飞库上游现存缺陷：`zf_driver_encoder.c` 正交编码器清零语句被注释、`TIM6/8/10_ENCODER` 写成 `|= (1<<3)`；`zf_common_interrupt.c` 的 `IP/IPH` 分支只能调高不能调低、`P8~PB` 分支移位溢出。本工程只用 PWMA/PWMC 且优先级调整走置位方向，暂不受影响（README 第 601 行）。
11. 调试残留：`TEST_OPEN_LOOP`、`TEST_AUTO_RUN`、`MOTOR_DUTY_RAMP`、`LOW_SPEED_TEST`、`DEBUG_MODE`、`ENABLE_SCREEN`（未定义宏）等条件编译分支，上车前需确认全部为关闭值。
12. 上电第一帧无配对历史时中线仍退回固定半宽 18；`HIDE_ROW` 桌面调试需临时改 119，装车必须改回 90。

---

## 15. 结论与建议

**结论**：本仓库是一份结构清晰、文档详实、可在 Keil C251 下零错误构建的完整竞速固件，硬件接口定义完备且集中（绝大多数引脚可直接从 `Motor.c` / `mt9v03x*.h` / `main.c` 追溯到行号），双缓冲、中断向量工程化、控制量单一入口、异常检测等设计具备工程价值。

但**当前工作区不是可直接交付的发布状态**：存在未提交的现场标定改动，导致文档、测试脚本与实现三者失配；透视去畸变被旁路、出界判据被放宽、里程未标定三项属于影响实车行为的关键变更，均未在 README/REVIEW/EVALUATION 中体现；定点停车在未标定前不具备保证。

**建议按以下顺序处理**：

| 优先级 | 动作 |
|---|---|
| P0-1 | 冻结当前源码基线：要么提交 3 个未提交文件并同步全部文档，要么用 Git 明确建立"现场标定分支" |
| P0-2 | 更新 `reference_alignment_check.py` 白名单（或把新差异列入受控清单），并把 `backup/version1.3` 纳入可复现范围 |
| P0-3 | 按 `out_of_track_check.py` 的旧判据与当前 `dark>=5` 分支补一组新测试，覆盖"两侧丢边 + 3~4 点暗"场景 |
| P0-4 | 推车 1 m 实测 `PULSE_PER_METER`，完成一圈标定后再把 `CALIB_REQUIRED` 置 1 验证定点停车 |
| P1-1 | 用 `IO_P04` + 示波器实测 5 ms 控制周期抖动（目标 < 1 ms） |
| P1-2 | 对 20 cm/45° 安装重新标定 `reflection` 表，或正式确认永久旁路并删除 45 KB 常量 |
| P1-3 | 统一 13 节所列 17 项文档偏差，尤其是编译统计、增益范围、自检摆幅、速度量纲 |

---

## 附录 A 证据索引

| 主题 | 文件 : 行 |
|---|---|
| 项目定位 / 功能清单 | `README.md:9-55` |
| 文档版硬件引脚表 | `README.md:302-326` |
| 文档版参数全集 | `README.md:330-441` |
| 出界 / 异常 / 已知限制 | `README.md:239-299,572-601` |
| 评估与遗留风险 | `EVALUATION.md:23-36,87-98` |
| 修改记录与验证 | `REVIEW.md:16-305` |
| 图像链路设计 | `docs/TRACK_RECOGNITION_DESIGN.md:82-215,247-375` |
| 许可与修改登记 | `NOTICE:1-57`、`libraries/doc/version.txt:1-2` |
| 主循环 / 选档 / 按键 / 出界 | `project/user/main.c:201-425` |
| 八档参数 / 中线融合 / loadRoad | `project/user/main.c:68-89,428-604` |
| 电机舵机引脚与初始化 | `project/code/Motor.c:6-10,470-496` |
| 控制 ISR / 纯跟踪 / 曲率限速 | `project/code/Motor.c:194-296,311-426` |
| 全部控制宏 | `project/code/Motor.h:8-198` |
| 图像宏 / 双缓冲声明 | `project/code/Image.h:9-60` |
| 帧快照 / 远端裁剪 / 去畸变 | `project/code/Image.c:21-61,342-360` |
| 摄像头参数 | `libraries/zf_device/zf_device_mt9v03x.h:46-78,114-125` |
| FIFO 引脚 | `libraries/zf_device/zf_device_mt9v03x_dma.h:8-16` |
| 摄像头初始化流程 | `libraries/zf_device/zf_device_mt9v03x.c`（SCCB 配置表、`mt9v03x_init`） |
| 时钟枚举 | `libraries/zf_common/zf_common_clock.h:42-63` |
| 调试串口宏 | `libraries/zf_common/zf_common_debug.h:42-53` |
| USB 描述符 / 端点 | `libraries/zf_components/stc32g_usb_lib/usb_desc.c:17-35`、`config.h:4-47` |
| 工程配置 | `project/mdk/seekfree.uvproj`（`<TargetCommonOption>` / `<Target251>` / `<Groups>`） |
| 编译结果 | `project/mdk/out_file/SEEKFREE.build_log.htm`、`SEEKFREE.map`（末页） |
| 扩展中断向量 | `project/user/isr_extended_vectors.A51:1-90`、`isr_extended_0.c:282-288` |
| 元素状态机 | `Cross.c:78-150`、`LRound.c:20-76`、`RRound.c:21-79`、`BendEntry.c:41-186` |
| 里程与发车 | `Stop.h:14-48`、`Stop.c:12-135` |

## 附录 B 量纲与术语

| 量 | 单位 | 换算 |
|---|---|---|
| `setSpeed` / `currentSpeed` / `DS` / `SPD` | 脉冲 / 5 ms | `v(m/s) = setSpeed × 200 / PULSE_PER_METER`（PPM 待实测） |
| `c1` | duty / 图像角度 | `duty = 角度 × c1 / 20` |
| `c3` / `BEND_FAR_AHEAD` | 中线点序号 | 中线重采样间距 2 像素 |
| `c4` | (脉冲/5 ms)² / 像素 | 侧向加速度上限，`v = √(R·c4)` |
| `c5` | 脉冲 / 5 ms | 直道目标速度 |
| 舵机 duty | 0.5 µs | 200 Hz 下中心 2917 duty = 1458.5 µs |
| 图像角度 | `atan2_int` 索引 0~30 | 与舵机实际角度是两把尺子：395 duty ≈ 24° 舵机角 |
| `odometer` / `ODO` | 编码器脉冲（int32） | 里程标定基础 |
| `Point.x / .y` | 行号 0~119 / 列号 0~187 | 注意代码中 x 为行、y 为列 |

## 附录 C 本报告的信息边界（缺失的硬件资料）

以下内容**仓库中不存在**，本报告未作推测，若需完整硬件交付需另行补充：

1. 电路原理图、PCB、BOM、接线图。
2. 电机驱动芯片 / 模块型号、H 桥逻辑表、最大电流、保护特性。
3. 直流电机型号、额定电压、减速比、空载/堵转参数。
4. 舵机型号、额定扭矩、实际行程标定曲线（报告中的 24° 为按常见 0.5 ms/60° 舵机估算）。
5. 编码器类型与线数（每米脉冲数、`PULSE_PER_METER` 实测值）。
6. 电池 / 电源方案：电压、容量、稳压与各个 3.3 V / 5 V 域的分配。
7. 摄像头镜头焦距、视场角、安装支架尺寸（仅有高度 20 cm / 俯角 45° 与旧 25 cm / 39° 记录）。
8. 车模机械参数（轴距、轮距、轮胎、重心），`DEVICE_H = 18` 只是像素域等效轴距。
9. 实车测试数据：圈速、帧率、控制周期抖动、边界点数分布、功耗、故障日志。
10. 屏幕 / 蜂鸣器移除前后的硬件改板记录。
11. 摄像头 DMA `.LIB` 的源码与内部寄存器配置（闭源）。
12. 参考实现 `backup/version1.3/` 不在 `version1.0` 目录内，本报告只通过 `docs/` 与测试脚本的记录间接引用，未直接审计其源码。

---

*报告结束。所有数字若与源码冲突，以源码与 `SEEKFREE.map` 为准；所有「文档」表述均已标注来源章节。*
