//
// 第一部分：连续 S 弯
//
// ============================ 输入 ============================
//   centerLine[] / centerLineCount   中线点列，0 号点锚在车头行 99，
//                                    相邻点弧长约 2 像素（resampleRoad dis=2）
//
// ========================== 识别条件 ==========================
//   先用短基线判方向为什么不行：
//     2 像素弧长 ≈ 0.3 cm，相邻 4 点只有 1.3 cm。半径 26 cm 的弯，这段弧上的
//     横向变化是 s²/(2R) ≈ 0.03 cm，不到 1 个像素——比量化噪声还小。
//     拿它算方向，测到的全是噪声：直道上噪声来回翻符号就会误触发。
//
//   改用"长窗横向速度"：
//     1. slope(i) = dy(i) - dy(i - SC_WINDOW)，窗口 12 点 = 24 像素弧长。
//        同样的弯在 24 像素上能攒出 3~7 像素，明显高于噪声；
//     2. 只统计 |slope| > SC_SLOPE_MIN 的显著值，在正负之间翻转 ≥ 1 次
//        —— S 形必然先往一侧走、再往另一侧走；单弯只有一个方向，翻转 0 次；
//     3. 全程 dy 的总摆幅必须 ≥ SC_P2P_MIN。
//        这一条专门挡直道：±2 像素的随机噪声也能让 slope 翻几次符号，
//        但摆幅只有 4 像素，过不了 8 像素的门槛。
//
//   实测（与固件同口径的整数算法）：
//     直道±1噪声 → 翻转 0、摆幅 2  → 不触发
//     直道±2噪声 → 翻转 1~4、摆幅 4 → 摆幅门槛挡掉，不触发
//     单弯 R=40/80、直道→急弯 → 翻转 0 → 不触发
//     大波浪 S（R=80 摆幅12 / R=40 摆幅24）→ 翻转 1 → 触发
//     平移 S（连接两条平行直道）→ 翻转 1 → 触发
//
// ========================== 触发后动作 ========================
//   只做一件事：把目标速度按 SC_SPEED_RATIO 压住，保持到 S 弯真正结束。
//   为什么不改前瞻：直觉上"前瞻收近、别瞄到下一个反向弯"是对的，但纯跟踪的
//   转角是 atan(2·L·dy / dx²)，收近前瞻等于把 dx 变小、增益变大，越收越抖。
//   实车已经验证过这条路走不通——从直道进入 S 弯那一刻开始摆。
//   连续 S 弯真正需要的是"不要在两个反向弯之间松油门"：原来的弯道减速靠
//   bendHold 记忆，弯心一过 bend 收窄就会松油，进下一个反向弯再刹一次，
//   于是全程一顿一顿。这里用一个跨整个 S 的恒速上限把它压平。
//
// ========================== 退出条件 ==========================
//   连续 SC_RELEASE 帧不再满足判据，且已占用至少 SC_MIN_MS（防止抖动式进出）。
//
// ========================== 与别处的边界 ======================
//   本模块只在"两侧边界都在、中线足够长"时成立。环岛（单侧丢线）与十字
//   （双侧丢线）的识别条件都不满足；仲裁器另有优先级兜底。
//

#include "SCurve.h"

/* 中线点列与点数在 main.c 里定义 */
extern int16 centerLineCount;
extern Point centerLine[expRoadCount];

/* ---- 判据参数 ---- */
#define SC_WINDOW      12   // 长窗：12 个点 = 24 像素弧长（短于此测不到真实曲率）
#define SC_SLOPE_MIN    3   // 长窗横向增量的显著门槛（像素）
#define SC_P2P_MIN      8   // 全程 dy 总摆幅门槛（像素），挡直道噪声
#define SC_RELEASE      5   // 连续不成立帧数才释放

/* ---- 动作参数 ---- */
#define SC_SPEED_RATIO 70   // S 弯内的目标速度比例（%）
#define SC_MIN_MS     200   // 最短占用时间，避免判据边缘抖动导致限速开关反复
#define SC_MAX_MS    3000   // 最长占用时间

static uint16 scMs;          // 进入时刻
static uint8 scLost;         // 连续不成立帧数
static uint8 scOn;           // 1 = 正在处理

/* 中线第 i 点的横向偏差，正 = 偏画面左侧 */
#define SC_DY(i)  ((int16) (94 - (int16) centerLine[i].y))

/* 本帧是否满足 S 弯形状。纯函数：不改任何状态，可以随便重复调用。 */
uint8 scDetect(void) {
    int16 i, n, side, flips, slope, hi, lo;

    n = centerLineCount;
    if (n < SC_WINDOW + 4) return 0;

    /* 先算总摆幅：不够就直接否掉，省掉后面的循环 */
    hi = lo = SC_DY(0);
    for (i = 1; i < n; i++) {
        if (SC_DY(i) > hi) hi = SC_DY(i);
        if (SC_DY(i) < lo) lo = SC_DY(i);
    }
    if ((int16) (hi - lo) < SC_P2P_MIN) return 0;

    /* 长窗横向增量，只在显著值上统计符号翻转 */
    side = 0;
    flips = 0;
    for (i = SC_WINDOW; i < n; i++) {
        slope = (int16) (SC_DY(i) - SC_DY((int16) (i - SC_WINDOW)));
        if (slope > SC_SLOPE_MIN) {
            if (side < 0) flips++;
            side = 1;
        } else if (slope < -SC_SLOPE_MIN) {
            if (side > 0) flips++;
            side = -1;
        }
    }

    return (uint8) (flips >= 1);
}

void scEnter(void) {
    scMs = msTick;
    scLost = 0;
    scOn = 1;
}

void scRun(void) {
    /* 只压速度，不动前瞻、不动打角：S 弯的修正交给正常纯跟踪 */
    elemSpeedRatio = SC_SPEED_RATIO;
    alongLine(centerLine, centerLineCount, getAimPos());

    if (scDetect()) scLost = 0;
    else if (scLost < 255) scLost++;
}

uint8 scBusy(void) {
    if (!scOn) return 0;
    if ((uint16) (msTick - scMs) > SC_MAX_MS) return 0;
    if ((uint16) (msTick - scMs) < SC_MIN_MS) return 1;   // 刚进来，先压一会儿
    return (uint8) (scLost < SC_RELEASE);
}

void scExit(void) {
    scOn = 0;
    scLost = 0;
    elemSpeedRatio = 100;
}

void scReset(void) {
    scOn = 0;
    scLost = 0;
    scMs = 0;
    elemSpeedRatio = 100;
}

uint8 scStage(void) {
    return (uint8) (scOn ? 1 : 0);
}
