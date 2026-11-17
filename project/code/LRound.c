//
// Created by 31663 on 2026/3/29.
//

#include "LRound.h"

#define ROUND_READY 1
#define ROUND_START 2
#define ROUND_IN 3
#define ROUND_END 4
#define ROUND_EXIT 5

/* ===== 左环岛状态机参数 =====
 * 左右环岛的数值是分别标定的，**不对称是有意的**（右环岛见 RRound.c）：
 *   READY→START / START→IN 的外推距离 13 / 27（右环岛 11 / 25）；
 *   START→IN 要求内侧点数 > 10（右环岛 > 20）；
 *   EXIT 用固定前瞻 40（右环岛用 getAimPos 自适应）。
 * 改一侧必须同步验证另一侧，不要按"镜像应该相等"直接拉平。 */
#define LR_READY_DIST     13    // READY 阶段的外推距离
#define LR_IN_DIST        27    // 进入环内后的外推距离
#define LR_END_DIST       11    // 出口阶段的外推距离
#define LR_EXIT_DIST      18    // 出环阶段的外推距离
#define LR_IN_MIN_PTS     10    // 内侧点数门槛（READY→START、EXIT 完成）
#define LR_GAP_MIN_PTS    20    // 缺口恢复点数门槛（START→IN、IN→END）
#define LR_END_MIN_PTS    25    // 出口边界长度门槛
#define LR_EXIT_AIM       40    // EXIT 阶段的固定前瞻点
/* READY 停留上限：入口判据是"外侧长边界 + 内侧短边界"的几何特征，急弯也可能命中。
 * 若 1.2 s 内始终等不到"内侧边界整条消失"（真正的环岛入弯缺口），
 * 就认定是误判并立即回 FORWARD——否则会带着 borderIn=1（50% 限速）跑满 8 s。 */
#define ROUND_READY_MS  1200

static uint8 ROUND_MODE = ROUND_READY, canStart = 0, canIn = 0, canOut = 0, canFinish = 0;
static uint16 readyMs = 0;   // 进入 READY 的时刻，用于误判兜底
extern int16 centerLineCount, RUN_MODE;
extern volatile int16 RRoadCount, LRoadCount;
extern int16 LRawCount, RRawCount;      // 拐点裁短前的原始点数，状态机只信它
extern uint8 ALONG;
extern volatile uint8 borderIn;
extern Point centerLine[expRoadCount], RRoad[expRoadCount];

/* 判据都用 LRawCount / RRawCount（裁短前的原始值）：
 * LRoadCount / RRoadCount 在 loadRoad() 里会被裁成 LCorner / RCorner（3~10），
 * ">10 且无拐点" 在裁短值上永远不能同时成立，READY/START/EXIT 三个迁移都会卡死。 */
void runLRound(int16 LCorner, int16 RCorner) {
    if (ROUND_MODE == ROUND_READY) {
        borderIn = 1;
        gpio_set_level(IO_P90, 1);
        alongLine(centerLine, centerLineCount, getAimPos());
        /* 误判兜底：1.2 s 内等不到"内侧边界整条消失"就退出环岛模式 */
        if (!readyMs)
            readyMs = msTick;
        else if ((uint16) (msTick - readyMs) > ROUND_READY_MS) {
            readyMs = 0;
            canStart = canIn = canOut = canFinish = 0;
            borderIn = 0;              // 解除 50% 限速
            centerLineDistance = 18;
            RUN_MODE = FORWARD;
            return;
        }
        if (!LRawCount)
            canStart = 1;
        else if (LRawCount > LR_IN_MIN_PTS && !LCorner && canStart) {
            readyMs = 0;
            ROUND_MODE = ROUND_START;
            ALONG = ALONG_L;
            centerLineDistance = LR_READY_DIST;
        }
    } else if (ROUND_MODE == ROUND_START) {
        gpio_set_level(IO_P90, 0);
        ALONG = ALONG_L;
        alongLine(centerLine, centerLineCount, getAimPos());
        if (!RRawCount)
            canIn = 1;
        else if (RRawCount > LR_IN_MIN_PTS && !RCorner && canIn) {
            centerLineDistance = LR_IN_DIST;
            ROUND_MODE = ROUND_IN;
            ALONG = ALONG_R;
        }
    } else if (ROUND_MODE == ROUND_IN) {
        ALONG = ALONG_R;
        alongLine(centerLine, centerLineCount, getAimPos());
        /* 出口缺口先出现在右侧：右边界整条消失（缺口正对车头）记 canOut，
         * 恢复且点数够长说明已经绕过缺口。原来的 `centerLineCount < 11`
         * 在中线改成"双侧融合 + 单侧外推"后基本恒假——中线点数等于
         * 骨架长度，环内一直有 30+ 点，状态机会卡死在 IN 直到 8 s 超时。 */
        if (!RRawCount)
            canOut = 1;
        else if (RRawCount > LR_GAP_MIN_PTS && canOut) {
            centerLineDistance = LR_END_DIST;
            ROUND_MODE = ROUND_END;
            ALONG = ALONG_L;
        }
    } else if (ROUND_MODE == ROUND_END) {
        ALONG = ALONG_L;
        alongLine(centerLine, centerLineCount, 17);
        /* 用原始点数取末点：RRoadCount 被裁短后 RRoad[RRoadCount-1] 指向拐点，
         * 不是边界末梢；末点列号 < 133 表示右边界重新伸向赛道右侧远方
         * （133 ≈ 画面中心 94 + 39，右环岛镜像值是 94 - 39 = 55）。 */
        if (!RCorner && RRawCount > LR_END_MIN_PTS && RRoad[RRawCount - 1].y < 133) {
            centerLineDistance = LR_EXIT_DIST;
            ROUND_MODE = ROUND_EXIT;
            ALONG = ALONG_R;
        }
    } else if (ROUND_MODE == ROUND_EXIT) {
        ALONG = ALONG_R;
        alongLine(centerLine, centerLineCount, LR_EXIT_AIM);
        if (!LRawCount)
            canFinish = 1;
        else if (LRawCount > LR_IN_MIN_PTS && canFinish) {
            ROUND_MODE = ROUND_READY;
            readyMs = 0;
            RUN_MODE = FORWARD;
            canFinish = canStart = canIn = canOut = 0;
        }
    }
}

void lroundReset(void) {
    ROUND_MODE = ROUND_READY;
    canStart = canIn = canOut = canFinish = 0;
    readyMs = 0;
    centerLineDistance = 18;
}

/* 当前阶段（1~5）：主循环按它重置元素超时预算，避免五阶段共享一个 8 s */
uint8 lroundStage(void) {
    return ROUND_MODE;
}
