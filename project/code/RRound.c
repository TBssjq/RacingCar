//
// Created by 31663 on 2026/3/29.
//

#include "RRound.h"

#define ROUND_READY 1
#define ROUND_START 2
#define ROUND_IN 3
#define ROUND_END 4
#define ROUND_EXIT 5

/* ===== 右环岛状态机参数 =====
 * 与左环岛（LRound.c）刻意不同：外推距离 11 / 25（左 13 / 27），
 * START→IN 要求内侧点数 > 20（左 10），EXIT 用 getAimPos 自适应（左固定 40）。
 * 这些差异来自两个方向分别现场调通，**不要按对称原则直接拉平**；
 * 要统一必须先两个方向都实车对比。 */
#define RR_READY_DIST     11    // READY 阶段的外推距离
#define RR_READY_MIN_PTS  10    // READY→START 的内侧点数门槛（与左环岛的 10 相同）
#define RR_IN_DIST        25    // 进入环内后的外推距离
#define RR_END_DIST       11    // 出口阶段的外推距离
#define RR_EXIT_DIST      18    // 出环阶段的外推距离
#define RR_IN_MIN_PTS     20    // 缺口恢复点数门槛（START→IN、IN→END）
#define RR_END_MIN_PTS    25    // 出口边界长度门槛
/* READY 停留上限：入口判据是几何特征，急弯也可能命中；1.2 s 内等不到
 * "内侧边界整条消失"就回 FORWARD，避免带着 50% 限速跑满 8 s。 */
#define ROUND_READY_MS  1200

static uint8 ROUND_MODE = ROUND_READY, canStart = 0, canIn = 0, canOut = 0, canFinish = 0;
static uint16 readyMs = 0;   // 进入 READY 的时刻，用于误判兜底
extern int16 centerLineCount, RUN_MODE;
extern volatile int16 RRoadCount, LRoadCount;
extern int16 LRawCount, RRawCount;      // 拐点裁短前的原始点数，状态机只信它
extern uint8 ALONG;
extern volatile uint8 borderIn;
extern Point centerLine[expRoadCount], LRoad[expRoadCount];

/* 与 LRound.c 对称：判据用 LRawCount / RRawCount（裁短前的原始值），
 * 裁短后的 LRoadCount / RRoadCount（= LCorner / RCorner，3~10）会让
 * ">10 且无拐点" 永远不成立，状态机只能靠 8 s 超时退出。 */
void runRRound(int16 LCorner, int16 RCorner) {
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
        if (!RRawCount)
            canStart = 1;
        else if (RRawCount > RR_READY_MIN_PTS && !RCorner && canStart) {
            readyMs = 0;
            ROUND_MODE = ROUND_START;
            ALONG = ALONG_R;
            centerLineDistance = RR_READY_DIST;
        }
    } else if (ROUND_MODE == ROUND_START) {
        gpio_set_level(IO_P90, 0);
        ALONG = ALONG_R;
        alongLine(centerLine, centerLineCount, getAimPos());
        if (!LRawCount)
            canIn = 1;
        else if (LRawCount > RR_IN_MIN_PTS && !LCorner && canIn) {
            centerLineDistance = RR_IN_DIST;
            ROUND_MODE = ROUND_IN;
            ALONG = ALONG_L;
        }
    } else if (ROUND_MODE == ROUND_IN) {
        ALONG = ALONG_L;
        alongLine(centerLine, centerLineCount, getAimPos());
        /* 出口缺口先出现在左侧：左边界整条消失记 canOut，恢复且够长即绕过缺口。
         * 原来 `centerLineCount < 11` 在中线融合外推后基本恒假，IN 会卡到 8 s 超时。 */
        if (!LRawCount)
            canOut = 1;
        else if (LRawCount > RR_IN_MIN_PTS && canOut) {
            centerLineDistance = RR_END_DIST;
            ROUND_MODE = ROUND_END;
            ALONG = ALONG_R;
        }
    } else if (ROUND_MODE == ROUND_END) {
        ALONG = ALONG_R;
        alongLine(centerLine, centerLineCount, 17);
        /* 用原始点数取末点；末点列号 > 55 表示左边界重新伸向赛道左侧远方
         * （55 ≈ 画面中心 94 - 39，左环岛镜像值是 94 + 39 = 133） */
        if (!LCorner && LRawCount > RR_END_MIN_PTS && LRoad[LRawCount - 1].y > 55) {
            centerLineDistance = RR_EXIT_DIST;
            ROUND_MODE = ROUND_EXIT;
            ALONG = ALONG_L;
        }
    } else if (ROUND_MODE == ROUND_EXIT) {
        ALONG = ALONG_L;
        alongLine(centerLine, centerLineCount, getAimPos());
        if (!RRawCount)
            canFinish = 1;
        else if (RRawCount > 10 && canFinish) {
            ROUND_MODE = ROUND_READY;
            readyMs = 0;
            // handle 由 raceTask 用于锁定刹车段速度，只在 raceStart 复位，不在这里清
            RUN_MODE = FORWARD;
            canFinish = canStart = canIn = canOut = 0;
        }
    }
}

void rroundReset(void) {
    ROUND_MODE = ROUND_READY;
    canStart = canIn = canOut = canFinish = 0;
    readyMs = 0;
    centerLineDistance = 18;
}

/* 当前阶段（1~5）：主循环按它重置元素超时预算 */
uint8 rroundStage(void) {
    return ROUND_MODE;
}
