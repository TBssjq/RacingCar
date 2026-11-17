# 图像处理链路与参考实现的对齐说明

> 参考实现：`backup/version1.3/project/code/Image.c`、`Image.h`（2671 行 / 43 行）
> 当前实现：`version1.0/project/code/Image.c`、`Image.h`
> 一致性手段：整文件替换，再用哈希与函数体比对复核

---

## 一、参考实现的来源

| 项 | 内容 |
|---|---|
| 路径 | `backup/version1.3/project/code/Image.c`、`Image.h` |
| 规模 | `Image.c` 2671 行（含 `reflection[120][188][2]` 常量表），`Image.h` 43 行 |
| 参考 SHA256（`Image.c`） | `337F28C872A86F323BEB808F6E438DFD814B079BEFD9F326A30FA4FEA9BC1BFA` |
| 参考 SHA256（`Image.h`） | `BD53F91F3CDE4B726D5524621D476655625C8D261B6AA26BC9EEF74CA8DC365A` |
| 当前 SHA256（`Image.c`） | `11BD628AF6FC397A96A92C2EDD5CD7B4950762F3779BE7E9BD8F227DE6933B52` |
| 当前 SHA256（`Image.h`） | `1B66D8E2290C241662756DEBA330F3064BB834B4752295C331F8EAEA8F5AA844` |
| 上游来源 | 浙江工商大学 哈夫克自动智能车项目，`HFK_STC32G144K-master/project/code/Image.c`，经本项目 `version1.3` 修订 |
| 同版本其他参考文件 | `backup/version1.3/project/user/main.c`（`loadRoad()` 与 `elementReset()`）、`project/code/Cross.c`（补线调用） |

`reflection` 标定表与参考实现相同。当前实现与参考的差异分两类：现场标定项 `BODY_ROW`，以及 `loadRoad()` 后半段与 `alongLine()` 为弯道识别做的改进，详见第 1.2 节与第八节。

## 1.1 车身入镜的现场约束

参考实现假定车身只出现在行 91 以下：`getLIndexStart()` / `getRIndexStart()` 以 `HIDE_ROW = 90` 为界，行 90 以下改用两侧窄列（左 3~48、右 140~186）绕开车身，行 70~89 仍搜全宽。

实车有部分车身出现在行 80~89，那里的中央暗块会被宽列二分当成赛道边界，产生假起点。参考实现里 `getImagePointFilterSelf()` 虽然会把 `i > HIDE_ROW && 48 <= j <= 140` 强制为 255，但它只被 `Cross.c` 的补线调用，起点搜索、`check()`、`getPointThreshold()` 与爬线全部直接读 `mt9v03x_frame` 原始像素，遮蔽对主链路不起作用。

处理方式沿用参考实现自己的思路：新增 `BODY_ROW = 80`，把起点搜索切换到窄列的界线上移，`HIDE_ROW` 保持 90 给补线遮挡使用。行 80~116 用窄列，行 70~79 仍用宽列兜底，保证车身遮住近处边界时还能在更远处找到真实起点。

## 1.2 弯道识别的局部改进

参考实现在弯道里来不及转向，根子在 `loadRoad()` 后半段：拐点序号直接把全局点数裁短，中线只剩 5~10 个点，`alongLine()` 的前瞻点被夹到车头附近；中线只由单侧边界沿法向平移固定 18 像素得到，弯道外侧边界被 `turn >= 4` 打断后整条中线偏向可见侧；纯跟踪只看单个前瞻点的偏差，车到达弯心之前该偏差仍是 0。

改动限定在 `main.c` 的 `loadRoad()` 后半段与 `Motor.c` 的 `alongLine()`：

| 改动 | 效果 |
|---|---|
| 拐点截断前复制 `midL` / `midR` 完整点列，中线改用副本，全局点数仍裁短供元素判据 | 中线点数不再随拐点骤降，前瞻点可取到 39 而非 4 |
| `buildCenterLine()` 以点数多的一侧为骨架，双侧同行取中点，缺侧按本帧实测半宽外推，半宽跨帧 1/4 权重平滑 | 单侧断裂时中线仍保持 40 点，中点列号 98 对真值 100 |
| `alongLine()` 增加远端趋势前馈 `dy = dyAim + (dyFar - dyAim)/3`，前瞻点保留 12 点远端余量，点数下限从 6 降到 3 | 前瞻点仍在直道段时即可给出转向角，右弯 -5.50°、左弯 +4.61°，直道保持 0.00° |
| `ALONG` 只在 `RUN_MODE == FORWARD` 时重算 | 元素状态机五个阶段的设置不再被覆盖 |

图像采集、起点搜索、爬线、去畸变、滤波、重采样、拐点检测保持参考实现。核对见 7.5 节。

## 1.3 弯道提前减速

原来的减速只在 `borderIn = 1` 时按 70% 生效，而 `borderIn` 靠边界列号越过 94 判定，车已经进弯才触发；曲率限速 `min(√(R·c4), c5)` 在转角 0~10 度一直是满速 400。

`Motor.c` 用同一个远端趋势信号提前触发减速：

```c
bend   = func_abs(dyFar - dyAim) + func_abs(d0);
ratio  = clamp(100 - (bend - BEND_SLOW_START) * BEND_SLOW_GAIN,
               BEND_SLOW_MIN, 100);
pct    = clamp(BEND_STEER_PCT + max(0, bend - BEND_GAIN_BEND) * BEND_GAIN_RATE,
               BEND_STEER_PCT, BEND_STEER_MAX);
d      = clamp(d0 * pct / 100, -30, 30);   // 仅减速状态
target = min(基础目标 * ratio / 100, BEND_SPEED_CAP);
```

`d0` 是纯跟踪原始转角，`bend` 用它计算，减速触发阈值不受转向增益影响。放大后的 `d` 同时进入 `currentDuty` 与曲率限速的 `tani` 查表，转向更狠的同时基础目标速度也再降一档。前馈分母 `BEND_FEED_DIV` 保持 3，远端趋势占三分之一；取 2 时实车反馈打角偏早。

| 参数 | 值 | 作用 |
|---|---|---|
| `BEND_STEER_PCT` | 105 | 缓弯的转角放大基准（%），强度 15 以下使用 |
| `BEND_STEER_MAX` | 155 | 急弯的放大上限（%），强度 25 以上达到 |
| `BEND_GAIN_BEND` | 15 | 超过该强度开始追加增益 |
| `BEND_GAIN_RATE` | 5 | 强度每多 1 单位增加的百分点 |
| `BEND_SLOW_START` | 4 | 弯道强度超过该值开始减速 |
| `BEND_SLOW_GAIN` | 3 | 每多 1 单位降低的百分点 |
| `BEND_SLOW_MIN` | 40 | 减速比例下限（%） |
| `BEND_SPEED_CAP` | 200 | 减速状态下的目标速度硬上限 |
| `BEND_RELEASE` | 2 | 强度回落到该值以下解除减速 |
| `BORDER_IN_SPEED_RATIO` | 50 | 车头进弯时的比例，与弯道比例取更严格值 |

`bendHold` 记住本次弯道出现过的最严比例，弯心两端都偏、强度收窄时不会中途加速；`controlReset()` 在发车时清掉保持值。核对见 7.6 节。

## 1.4 视野窗口与出界判据

摄像头高度 20 cm、俯角 45° 时装出来的视场明显大于赛道，画面四周拍到的是场地外的底布、反光、背景与车身。参考实现里这块区域只靠"远端若干行写暗"处理，其余三边完全没有约束；出界保护则用"5 个采样点全暗"，但采样点本身有一部分落在视场外与车身上，判据等于长期偏置在"暗"这一侧。直道上偶发停车就是这么来的。

现场草图给出的目标是**保留中间一整块、四周各裁一圈**：内框宽 120 列，上下按实车表现分别调。实车反馈"转弯幅度不够"后，把远端裁切量从 20 行减到 10 行、多保留 10 行远端：中线点列变长，前瞻点不再被夹到窗口末端，远端趋势前馈（1/3 权重）有真实远端数据可用，进弯前能更早打角。当前参数：上裁 10 行、下裁 20 行、左右各裁 34 列 → 行 10~99（90 行）、列 34~153（120 列）。四个 `VIEW_CUT_*` 独立可调，边界自动推导。

| 改动 | 位置 | 说明 |
|---|---|---|
| 视野窗口 | `Image.h` 的四个 `VIEW_CUT_*`（10 / 20 / 34 / 34） | 推导出 `VIEW_FAR_ROW = 10` / `VIEW_NEAR_ROW = 99` / `VIEW_COL_L = 34` / `VIEW_COL_R = 153`；`cutViewRegion()` 把四周写暗，起点扫描行（95 → 13）、二分列、爬线游走、初始阈值取样、拐点下界全部收紧到窗口内 |
| 采样点 | `main.c` 的 `outSamplePt[5][2]` | 全部落在 `BODY_ROW = 80` 以上（行 66~76）、收进赛道内侧，且都在窗口内（列 70~118）；每点取 3×3 块均值，单像素反光不足以判暗 |
| 判据 | `main.c` 的 `OUT_BLIND_MS = 200` / `OUT_CONFIRM_FRAMES = 3` | 5 点全暗 **且** 200 ms 未见赛道（任一侧点数 > 3 即刷新盲区计时）**且** 连续 3 帧成立，之后控制中断再做 3 拍消抖 |
| 转弯幅度 | `Motor.h` 的 `STEER_REF_ROW = VIEW_NEAR_ROW` | `dx` 从 `MT9V03X_H - x + DEVICE_H` 改为 `STEER_REF_ROW - x + DEVICE_H`，`BendEntry.c` 同步；见下面连带项 |

窗口带来的连带改动（不做就不自洽）：

| 连带项 | 原因 | 处理 |
|---|---|---|
| 中线首点锚定 | 原来钉在行 119（画面底边），近端裁掉 20 行后首点与第一个真实点之间会插进十个纯插值点 | 改钉在 `VIEW_NEAR_ROW`（99） |
| 初始阈值取样带 | 原来固定行 92~113，其中 100~113 已变成暗区，会把灰度最小值压成 0 | 改成窗口内缩 4 像素：行 14~95、列 38~149 |
| 纵向距离 `dx` | 近端裁掉 20 行后仍按画面底边 120 计算，`dx` 平白多出 20，同一横向偏差的转角只有裁切前的 3/4 左右，实车就是"转弯幅度不够" | `STEER_REF_ROW` 取窗口下沿 99，把被裁掉的近端从几何里扣掉；`tests/body_row_check.py` 第 5 节给出量级对照：典型前瞻点转角放大 1.5~1.6 倍 |

几点约束：

- 窗口仍覆盖画面中心列 94 与两侧边线的常见位置，`borderIn` 的分界列 94、`alongLine()` 的 `dy = 94 - y` 都保持不变。想让转向更柔就把 `STEER_REF_ROW` 改回 `MT9V03X_H`，再重调 `c1`；想更猛可把 `c1` 抬 10~20% 或 `DEVICE_H` 调到 20~22。
- 窗口高 90 行，直道单侧边界重采样后只有 21~28 点。原来按"第 37 个点"写的环岛判据在这里不可达，现已改为「外侧点数 > 16 且末点伸到 `VIEW_FAR_ROW+8`、内侧点数 < 4」并加 1.2 s 的 READY 误判兜底；换窗口后这三条要实测复标。
- 单侧仍有边界时不停车：画面里还有可循迹的线，纯跟踪能把车拉回，停车反而更亏。代价是"冲出赛道但残留一侧边线"这种情形不会触发出界保护。
- 现场标定采样点：直道正常行驶时串口 `DARK` 应为 0；若稳定 ≥1，说明有采样点落在场地外或车身上，按 `DARK` 变化方向内收或上移。

---

## 二、核心处理步骤

每帧一次，在主循环的 `if (frameBusy)` 分支内调用 `loadRoad()`。链路共 8 步，顺序固定。

| 步骤 | 函数 | 输入 | 输出 | 判据 |
|---|---|---|---|---|
| 1 | `getLStart()` / `getRStart()` | 快照、`LThreshold` / `RThreshold` | 起点 `Point` | 从窗口下沿行 95 向行 13 按 3 行步长扫描，二分（左 36~94、右 94~151）找第一处符合 `check()` 的灰度跳变 |
| 2 | `getLBorder()` / `getRBorder()` | 起点、点数上限 | 边界点数组、点数 | 八邻域爬行，步长 2 像素，`turn >= 4` 退出 |
| 3 | 拷贝 | 上一步结果 | `LBorder[]` / `RBorder[]`（原始坐标副本） | 供元素模块使用 |
| 4 | `getGodRoadLine()` | 边界点 | 俯视坐标边界点 | 查 `reflection[x][y]`，越界点跳过 |
| 5 | `filterRoad()` | 俯视边界点 | 平滑后的边界点 | 因果滤波 `(前一点 + 当前点 + 1) >> 1` |
| 6 | `resampleRoad(dis = 3)` | 平滑边界点 | 等弧长重采样点 | 按弧长累加，上限 `expRoadCount` |
| 7 | `findCorner()` | 重采样边界点 | 拐点序号 | 夹角余弦 `<= 0.50`，连续 3 点确认，只在前 40 像素以远搜索 |
| 8 | `buildCenterLine()` | 未裁短的完整左右边界点列 | 中线 | 双侧同行取中点，缺侧按实测半宽外推 |

中线生成后再做一次 `filterRoad()` 与 `resampleRoad(dis = 2)`，见 `loadRoad()`。

### 2.1 `loadRoad()` 的调用序列

```c
Point LStart = getLStart(), RStart = getRStart();
LRoadCount = expRoadCount, RRoadCount = expRoadCount;
getLBorder(LRoad, LStart, &LRoadCount);
getRBorder(RRoad, RStart, &RRoadCount);
LBorderCount = func_limit_ab(LRoadCount, 0, expRoadCount);
memcpy(LBorder, LRoad, LBorderCount * sizeof(Point));
RBorderCount = func_limit_ab(RRoadCount, 0, expRoadCount);
memcpy(RBorder, RRoad, RBorderCount * sizeof(Point));
getGodRoadLine(LRoad, LRoadCount);
getGodRoadLine(RRoad, RRoadCount);
filterRoad(LRoad, LRoadCount);
filterRoad(RRoad, RRoadCount);
resampleRoad(LRoad, &LRoadCount, 0, 3);
resampleRoad(RRoad, &RRoadCount, 0, 3);
LCorner = findCorner(LRoad, LRoadCount, 0);
RCorner = findCorner(RRoad, RRoadCount, 0);
/* 拐点截断前复制完整点列 → borderIn 判定 → 拐点截断 → 融合中线 → ALONG 门控投票 */
```

要点：

- 两侧点数在调用爬线前先置 `expRoadCount`，爬线函数以它作为写入上限。
- 从 `getLStart()` 到 `findCorner()` 的调用序列与参考实现逐行相同，核对见 7.1 节的 T3。
- 拐点截断作用于全局的 `LRoadCount` / `RRoadCount`，元素判据读这两个量；中线用截断前复制的 `midL` / `midR`。
- `ALONG` 只在 `RUN_MODE == FORWARD` 时由两侧点数重算，元素阶段保留状态机的设置。

---

## 三、关键参数与阈值

以下值来自 `Image.h` 与 `Image.c`，与参考实现完全相同。

| 符号 | 值 | 位置 | 作用 |
|---|---|---|---|
| `block` | 5 | `Image.h` | 局部阈值窗口边长 |
| `cap` | 4 | `Image.h` | 局部阈值去噪余量 |
| `expRoadCount` | 100 | `Image.h` | 边界点数组容量 |
| `EDGE_DIFF` | 12 | `Image.h` | 边界灰度跳变下限 |
| `EDGE_RATIO` | 7 | `Image.h` | `(亮+暗)/(亮-暗)` 上限 |
| `HIDE_ROW` | 90 | `Image.h` | 车身遮挡区起始行 |
| `HIDE_COL_L` / `HIDE_COL_R` | 48 / 140 | `Image.h` | 遮挡区列范围 |
| `CORNER_COS100` | 50 | `Image.h` | 拐点夹角余弦阈值 × 100 |
| 初始阈值对比度门槛 | 20 | `imageInitThreshold()` | 低于此值返回兜底值 128 |
| 初始阈值采样带 | 行 14~95、列 38~149，步长 2 | `imageInitThreshold()` | 窗口内缩 4 像素，取灰度最小最大值的中点 |
| 起点扫描范围 | 行 95 → 13，左列 36 → 94、右列 94 → 151，步长 3 | `getLStart()` / `getRStart()` | 近到远，范围由视野窗口决定；`BODY_ROW` 以上用窄列 |
| `BODY_ROW` | 80 | `Image.h` | 该行及以上改用全宽搜索，以下只用两侧窄列；参考实现的界是 `HIDE_ROW = 90` |
| `VIEW_CUT_FAR` / `VIEW_CUT_NEAR` | 10 / 20 | `Image.h` | 上、下各裁掉的行数；调小 `VIEW_CUT_FAR` 看到更远 |
| `VIEW_CUT_LEFT` / `VIEW_CUT_RIGHT` | 34 / 34 | `Image.h` | 左、右各裁掉的列数 |
| `VIEW_FAR_ROW` / `VIEW_NEAR_ROW` | 10 / 99 | `Image.h` | 由裁切量推导的保留行区间 |
| `VIEW_COL_L` / `VIEW_COL_R` | 34 / 153 | `Image.h` | 由裁切量推导的保留列区间 |
| `STEER_REF_ROW` | `VIEW_NEAR_ROW` = 99 | `Motor.h` | 纯跟踪纵向距离的参考行；改回 `MT9V03X_H` 则前瞻更长、转角更柔 |
| 爬线步长 | 2 像素 | `dirF` / `dirL` / `dirR` | 八邻域 |
| 爬线转向上限 | 4 次 | `getLBorder()` | 连续转向即退出 |
| 阈值滑动触发点 | 点数 > 25 | `getLBorder()` / `getRBorder()` | 局部阈值与上一帧阈值取算术平均 |
| `centerLineDistance` | 18，拐角处 9 | `main.c` / `Image.c` | 中线法向偏移；两侧边界都有效时改用本帧实测半宽（限幅 8~60），18 只作为单侧外推的初值与兜底 |
| 边界重采样间距 | 3 | `loadRoad()` | 等弧长 |
| 中线重采样间距 | 2 | `loadRoad()` | 等弧长 |

### 3.1 阈值更新公式

```c
LThreshold = (uint8) (((int16) tempThreshold + (int16) LThreshold) >> 1);
threshold  = (uint8) (((int16) LThreshold + (int16) RThreshold) >> 1);
```

C 的移位优先级低于加法，`temp + L >> 1` 本来就算 `(temp + L) >> 1`；源码里显式加括号只是把这一点写明白，语义与参考实现一致。

---

## 四、输入输出数据格式与精度

### 4.1 输入

| 数据 | 类型 | 范围 | 说明 |
|---|---|---|---|
| `mt9v03x_frame[120][188]` | `uint8` | 0~255 | 灰度快照，DMA 完成中断内搬入 |
| `threshold` / `LThreshold` / `RThreshold` | `uint8` | 0~255 | 全局与单侧阈值，128 表示对比度不足 |
| `banUpdateThreshold` | `uint8` | 0 / 1 | 为 1 时冻结阈值滑动 |
| `centerLineDistance` | `uint8` | 9 / 18 | 中线法向偏移 |

### 4.2 输出

| 数据 | 类型 | 精度 | 说明 |
|---|---|---|---|
| `Point.x` | `uint8` | 0~119 | 图像行号，去畸变后为俯视行号 |
| `Point.y` | `uint8` | 0~187 | 图像列号，去畸变后为俯视列号 |
| `LRoadCount` / `RRoadCount` | `int16` | 0~100 | 边界点数，0 表示该侧无效 |
| `LBorderCount` / `RBorderCount` | `int16` | 0~100 | 去畸变之前的原始边界点数 |
| `centerLineCount` | `int16` | 0~100 | 中线点数 |
| `LCorner` / `RCorner` | `int16` | 0~100 | 拐点序号，0 表示没有拐点 |

### 4.3 精度约束

- 爬线以 2 像素为一步，边界点的位置精度是 2 像素。
- 去畸变查表为整像素映射，不插值。
- `resampleRoad` 在段内做整数线性插值，`tx / len` 的整数除法会向下取整，最大误差 1 像素。
- 所有中间量用 `int16` 存储，平方运算前必须提升到 `int32`（`dy²` 可达 34969）。

---

## 五、异常与边界情况的处理

| 情况 | 处理方式 | 代码位置 |
|---|---|---|
| 起点搜索失败 | `getLStart()` 返回 `(0,0)`，`getLBorder()` 立即把点数置 0 返回 | `getLStart()` / `getLBorder()` |
| 快照读取越界 | `getImagePointFilterSelf()` 对 `i >= 120` 或 `j >= 188` 返回 0 | `getImagePointFilterSelf()` |
| 车身遮挡区 | `i > 90 && 48 <= j <= 140` 时返回 255（强制为白），只作用于 `Cross.c` 补线 | `getImagePointFilterSelf()` |
| 车身出现在行 80~89 | `getLIndexStart()` / `getRIndexStart()` 以 `BODY_ROW = 80` 为界改用两侧窄列，跳过车身暗块；行 70~79 仍用宽列兜底 | `Image.c` / `Image.h` |
| 局部阈值窗口越界 | 先把窗口中心夹到 `[halfBlock, 119-halfBlock]` 与 `[halfBlock, 187-halfBlock]` | `getPointThreshold()` |
| 画面没有明暗差 | `imageInitThreshold()` 返回 128 | `imageInitThreshold()` |
| 去畸变坐标越界 | 该点保持原值不发生映射，继续处理下一个点 | `getGodRoadLine()` |
| 重采样入口参数异常 | 负 `startPos`、`count > 100`、`count <= startPos + 1`、`dis == 0` 时把点数置 0 返回 | `resampleRoad()` |
| 重采样输出超出容量 | 结果点数达到 `expRoadCount` 时最后一点覆盖写入并收束 | `resampleRoad()` |
| 边界点写入超出容量 | `limit = func_limit_ab(*count, 0, expRoadCount)` 先夹取 | `getLBorder()` / `getRBorder()` |
| 爬线回到起点 | `x == start.x && y == start.y && !turn` 时跳出循环 | `getLBorder()` / `getRBorder()` |
| 边界点数不足以构造中线 | `centerLineCount < 2` 时把中线点数置 0，`alongLine()` 直接返回、控制量保持 | `loadRoad()` / `alongLine()` |
| 窗口外的场地与车身 | `cutViewRegion()` 把窗口外（行 < 10、行 > 99、列 < 34、列 > 153）整块写成暗区，起点搜索、爬线游走、阈值取样、拐点下界同步收紧到窗口内 | `Image.c` / `Image.h` |
| 画面全暗 | 5 个采样点全部低于阈值（3×3 块均值），**且**已 200 ms 没看到赛道（任一侧点数 > 3 即刷新），连续 3 帧成立才判出界 | `main.c` |
| 单侧仍有边界 | 不判出界，交给纯跟踪把车拉回；只有两侧全丢才可能成立 | `main.c` |
| 发车瞬间视觉未稳定 | `RACE_RUNNING` 进入后 800 ms 内不判出界，重新发车会重新计时 | `main.c` |

---

## 六、是否在赛道上的识别

参考实现没有独立的赛道判定函数：`main.c` 的 `updateStatusLed()` 注释写明"不含赛道检测"，`B2` 按键直接调用 `raceStart()`，不设门槛。参考实现里唯一与赛道状态相关的量是出界保护的条件：

```c
if ((!LRoadCount && !RRoadCount) || (5 个采样点全暗)) { 停车 }
```

当前实现在 `Track.c` 里取这条条件的补集作为"在赛道上"的判据：

```c
uint8 trackEvaluate(void) {
    rawOn = (uint8) ((LRoadCount > 0) || (RRoadCount > 0));
    return rawOn;
}
```

| `LRoadCount` | `RRoadCount` | 参考出界条件 | 当前赛道判定 |
|---|---|---|---|
| 0 | 0 | 成立 | 不在赛道 |
| 0 | 非 0 | 不成立 | 在赛道 |
| 非 0 | 0 | 不成立 | 在赛道 |
| 非 0 | 非 0 | 不成立 | 在赛道 |

判定的输入就是 `loadRoad()` 产出的边界点数，不额外读像素，所以相同输入必得相同结论。赛道状态只用于串口输出与蓝灯以外的指示，**不阻断发车**，与参考实现的行为一致。

---

## 七、测试用例与预期结果

### 7.1 对齐核对（`tests/reference_alignment_check.py`）

| 用例 | 输入 | 预期结果 | 实际结果 |
|---|---|---|---|
| T1 `Image.c` 差异限定 | 两份 `Image.c` 的行级差异 | 只有 2 行，都是 `i < HIDE_ROW` → `i < BODY_ROW` | 删除 2 行、新增 2 行，符合预期 |
| T2 `Image.h` 差异限定 | 两份 `Image.h` 的行级差异 | 只新增 `BODY_ROW` 及其注释 | 删除 0 行、新增 5 行，符合预期 |
| T3 感知前半段一致 | 两份 `main.c` 从爬线到拐点之前 | 去注释与空白后逐行相同 | 14 行全部相同 |
| T3a 中线改用完整点列 | 拐点前保存副本、元素判据仍用裁短的全局点数 | 三项检查全部通过 | 通过 |
| T4 关键参数一致 | 9 个宏 | 取值相同 | `block=5 cap=4 expRoadCount=100 EDGE_DIFF=12 EDGE_RATIO=7 HIDE_ROW=90 HIDE_COL_L=48 HIDE_COL_R=140 CORNER_COS100=50` |
| T5 赛道判据互补 | 8 组点数组合 | 两者互为补集 | 8/8 通过 |
| T6 上电初值一致 | `elementReset()` | 两侧点数都置 `expRoadCount` | 相同 |

### 7.1 视野窗口与车身入镜核对（`tests/body_row_check.py`）

模拟 `cutViewRegion()`、`getLIndexStart()` 的二分与 `check()`，图像 188 × 120，窗口行 10~99、列 34~153，赛道白 200 在列 40~145，底布 60，车身暗块 40，阈值 130。

第 1 节核对窗口本身：保留 90 × 120 = 10800 像素，其余 11760 像素全为 0；扫描行 95 → 13、左二分列 36 → 94 / 右 94 → 151；5 个出界采样点（行 66~76、列 70~118）都在窗口内。第 5 节给出转向量级对照（`STEER_REF_ROW = 99` 与 120 对比），典型前瞻点转角放大 1.5~1.6 倍。

| 场景 | 参考 `HIDE_ROW = 90` | 当前 `BODY_ROW = 80` | 期望 |
|---|---|---|---|
| 无车身 | 起点 `(95, 41)` | `(95, 41)` | 一致 |
| 车身只在中央 60~125，行 80~99 | `(95, 41)` | `(95, 41)` | 一致 |
| 车身遮住左边界 34~60，行 80~99 | `(89, 62)`，车身右缘被当成边界 | `(77, 41)`，真实赛道边界 | 当前正确 |
| 车身延伸到列 34~100，行 80~99 | `(77, 41)` | `(77, 41)` | 一致 |

逐行分析（车身 34~60，真实边界 40）：行 95~80 的窄列搜索返回列 48 但起点无效；参考实现在行 89~80 会把车身右缘 61 当成起点，当前实现跳过这些行，到行 77（< `BODY_ROW`，全宽搜索）才找到真实边界 40。窗口下沿是行 99，行 100 已在窗口外。

### 7.2 赛道判定真值表

| 输入 `(LRoadCount, RRoadCount)` | 参考判为出界 | 当前判为在赛道 | 期望 |
|---|---|---|---|
| (0, 0) | 是 | 否 | 通过 |
| (0, 1) | 否 | 是 | 通过 |
| (0, 40) | 否 | 是 | 通过 |
| (0, 100) | 否 | 是 | 通过 |
| (1, 0) | 否 | 是 | 通过 |
| (40, 0) | 否 | 是 | 通过 |
| (100, 0) | 否 | 是 | 通过 |
| (40, 40) | 否 | 是 | 通过 |

### 7.3 编译

```
0 Error(s), 1 Warning(s)        仅逐飞库 zf_device_mt9v03x.c(138) C183
data=102.5  edata+hdata=8192  xdata=51893  const=48754  code=82777
```

参考实现的 `README.md` 记录的条件编译告警是"3 条：`zf_device_mt9v03x.c(138)` 与 `Image.c(147/176)` 两条 `C173`"，并说明把 `step` 改为 `int16` 后两条 `C173` 应消失。当前只剩 1 条，与这条记录吻合。

### 7.4 相同输入产生相同输出的依据

`Image.c` 与参考的差异经 T1 验证严格限定为两行 `HIDE_ROW` → `BODY_ROW`，`Image.h` 经 T2 验证只新增 `BODY_ROW` 定义；`loadRoad()` 从爬线到拐点之前的调用序列经 T3 验证与参考逐行相同。因此：

- 同一帧 `mt9v03x_frame` 与同一组阈值进入后，边界点列 `LRoad[]` / `RRoad[]`、裁短后的全局点数 `LRoadCount` / `RRoadCount`、`LCorner` / `RCorner` 与参考实现完全相同。
- 车身进入行 70~89 时，当前实现跳过被车身污染的行，从更远处找真实起点，核对见 7.1 节。
- 中线生成与转向角有意偏离参考：中线改用未裁短的融合点列，转向角叠加远端趋势前馈，核对见 7.5 节。这两处改动只作用于决策与执行层，不改变感知层输出的边界数据。
- 赛道判定直接由 `LRoadCount` / `RRoadCount` 得出，同一组点数必得同一结论。

### 7.5 弯道识别核对（`tests/bend_recognition_check.py`）

| 场景 | 参考实现 | 当前实现 |
|---|---|---|
| 直道，前瞻点序号 14 | 0.00° | 0.00° |
| 右弯，前瞻点仍在直道段 | 0.00° | -5.50° |
| 左弯，前瞻点仍在直道段 | 0.00° | +4.61° |
| 弯道内序号 24 | -5.33° | -8.38° |
| 弯道内序号 36 | -7.71° | -9.00° |
| 左侧 40 点、右侧 8 点 | 单侧平移，中线偏 12 像素 | 输出 40 点，中点列号 98（真值 100） |
| 先双侧配对、下一帧只剩单侧 | 固定偏移 18，中线首点 78 | 跨帧半宽 30，中线首点 90（真值 90） |
| 上电第一帧无配对历史 | 固定偏移 18 | 保守值 18，中线首点 78 |
| 拐点序号 6 裁短全局点数 | 中线 6 点，前瞻点最大 4 | 中线 40 点，前瞻点可取到 39 |

### 7.6 弯道减速核对（`tests/bend_slowdown_check.py`）

`c4 = 2000`、`c5 = 400`，单位脉冲 / 5 ms。

| 场景 | 参考实现 | 上一版 | 当前 | 比上一版再降 |
|---|---|---|---|---|
| 直道 | 400 | 400 | 400 | 0% |
| 远端有弯，前瞻点仍在直道 | 400 | 260 | 200 | 23% |
| 更急的入弯（远端偏 28） | 400 | 220 | 160 | 27% |
| 弯中，前瞻点已偏 12 | 400 | 248 | 160 | 35% |
| 车头已进弯 `borderIn = 1` | 280 | 240 | 160 | 33% |
| 急弯，前瞻点偏 30 | 280 | 220 | 139 | 36% |

逐帧推进 14 帧：第 2 帧（远端偏 4）目标速度就落到 200，参考实现全程 400；第 5~10 帧弯心处比例保持 40，速度 160 / 160 / 160 / 144 / 139 / 139；第 13 帧强度回到 0，减速解除到 400。`BEND_SPEED_CAP = 200` 把入弯初期比例 52% 给出的 208 压到 200；cap 调到 240 或 300 时该阶段回到 208，急弯仍由比例与曲率限速决定为 139。

弯道转向增强（`tests/bend_recognition_check.py`）：

| 场景 | 参考实现 | 仅前馈 | 前馈 + 弯道增益 |
|---|---|---|---|
| 直道 | 0.00° | 0.00° | 0.00° |
| 右弯，前瞻点仍在直道段 | 0.00° | -5.50° | -7.42° |
| 左弯，前瞻点仍在直道段 | 0.00° | +5.50° | +5.99° |
| 弯道内序号 24 | -5.33° | -8.38° | -12.99° |
| 弯道内序号 30 | -7.22° | -9.01° | -13.97° |

前馈分母经历三轮：3 → 2 → 3。取 2 时远端趋势占一半，实车反馈转向时机偏早；回到 3 之后，前瞻点仍在直道段的转角从 -8.30° 收到 -6.43°。

增益从固定值改成随强度自适应。固定 105% 时急弯转不进去，固定 115% 时缓弯过冲，两端无法同时满足。现在强度 15 以下保持 105%，之后每单位加 5 个百分点，155% 封顶：入弯初期强度约 19 得 125%，弯内强度约 26 直接到上限。弯内角度 -12.99°，比参考实现大 144%。

斜率用 5 而非 2：弯内强度只到 26 左右，按每单位 2 个百分点算需要强度 40 才碰得到 155% 上限，实际永远用不满。

---

## 八、与参考实现保持一致的范围

| 文件 | 处理 |
|---|---|
| `project/code/Image.c` | 整文件来自参考实现，仅两处 `i < HIDE_ROW` 改为 `i < BODY_ROW` |
| `project/code/Image.h` | 整文件来自参考实现，仅新增 `BODY_ROW = 80` 及其注释 |
| `project/user/main.c` 的 `loadRoad()` | 从爬线到拐点之前与参考逐行相同；后半段改用完整点列融合中线，`ALONG` 加元素门控 |
| `project/code/Motor.c` 的 `alongLine()` | 增加远端趋势前馈与前瞻点远端余量，点数下限从 6 降到 3；计算弯道减速比例，并在减速状态按 `BEND_STEER_PCT` 放大转角 |
| `project/code/Motor.c` 的 `setServoDuty()` | 用减速比例与 `BEND_SPEED_CAP` 压低目标速度；`borderIn` 比例取更严格值 |
| `project/user/main.c` 的 `elementReset()` | `LRoadCount` / `RRoadCount` 置 `expRoadCount` |
| `project/code/Cross.c` | 补线调用恢复为 `getLBorder()` / `getRBorder()`，点数门槛恢复为 40 |
| `project/code/Track.c` | 判定改为参考出界条件的补集，蓝灯位掩码恢复为四个故障位 |
| `project/user/main.c` 的 B2 分支 | 取消赛道门槛，与参考一致直接发车 |

以下部分不在图像链路范围内，保留当前实现：电机与舵机的控制参数（`Motor.c` / `Motor.h`）、出界判定所在的执行上下文（主循环判定、控制中断消抖）、按键计时与调试串口输出。
