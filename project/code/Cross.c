//
// Created by 31663 on 2026/3/29.
//

#include "Cross.h"

#define CROSS_READY 1
#define CROSS_IN 2

/* 入口/出口阈值带（按当前视野窗口的点数量级标定）：窗口行 10~99、重采样间距 2，
 * 正常直道中线约 40 点。CROSS_ENTER_COUNT 放在 Cross.h，main.c 的入口判据
 * 与这里必须用同一个门槛，否则会出现"RUN_MODE 已切到 CROSS 但状态机不补线"。 */
extern uint8 switch2;
extern int16 centerLineCount, RUN_MODE;
extern int16 LBorderCount, RBorderCount;   // 出口旁路判断"两侧边界是否已恢复"用
extern Point centerLine[expRoadCount];

static int8 CROSS_MODE = CROSS_READY, canEnd = 0;
static int16 alongR, crossThreshold, minMaxH;

Point getCrossRStart() {
    Point pos = {80, 94};
    int16 min = 187, i;
    uint8 found = 0;
    if (RBorderCount >= 40 || !minMaxH) {
        for (i = 0; i < RBorderCount; i += 2) {
            if (RBorder[i].y <= min) min = RBorder[i].y;
            else {
                // 防止 uint8 下溢成 25x
                pos.x = RBorder[i].x > 4 ? RBorder[i].x - 4 : 0;
                pos.y = minMaxH = RBorder[i].y;
                found = 1;
                break;
            }
        }
        /* 直道上 y 单调不增时循环一次都不会 break，必须给确定起点。
         * 这里只能用本次扫描得到的 min：minMaxH 仅在"列号出现回折"的 else 分支里
         * 赋值，单调边界（直道、弯道外侧）下它始终是 0，起点会落成 (80, 0) ——
         * 那一列已被 cutViewRegion 写成暗区，爬线会在暗区里沿对角线爬出几十个假点。
         * RBorderCount 为 0 时本次没有可用列号，退回缓存值。 */
        if (!found) {
            pos.x = 80;
            pos.y = (RBorderCount > 0) ? (uint8) min : (uint8) minMaxH;
        }
    } else {
        pos.x = 80;
        pos.y = (uint8) minMaxH;
    }
    while (pos.x >= 8 && (getImagePointFilterSelf(pos.x, pos.y) > crossThreshold || getImagePointFilterSelf(pos.x - 8, pos.y) > crossThreshold)) pos.x--;
    if (pos.x <= 7)
        pos.x = pos.y = 0;
    else
        pos.x++;
    return pos;
}

Point getCrossLStart() {
    Point pos = {80, 94};
    int16 max = 0, i;
    uint8 found = 0;
    if (LBorderCount >= 40 || !minMaxH) {
        for (i = 0; i < LBorderCount; i += 2) {
            if (LBorder[i].y >= max) max = LBorder[i].y;
            else {
                pos.x = LBorder[i].x > 4 ? LBorder[i].x - 4 : 0;
                pos.y = minMaxH = LBorder[i].y;
                found = 1;
                break;
            }
        }
        /* 与 getCrossRStart() 同理：单调边界下 minMaxH 一直是 0，
         * 兜底必须用本次扫描得到的 max */
        if (!found) {
            pos.x = 80;
            pos.y = (LBorderCount > 0) ? (uint8) max : (uint8) minMaxH;
        }
    } else {
        pos.x = 80;
        pos.y = (uint8) minMaxH;
    }
    while (pos.x >= 8 && (getImagePointFilterSelf(pos.x, pos.y) > crossThreshold || getImagePointFilterSelf(pos.x - 8, pos.y) > crossThreshold)) pos.x--;
    if (pos.x <= 7)
        pos.x = pos.y = 0;
    else
        pos.x++;
    return pos;
}

void runCross() {
    Point start;
#if (DEBUG_MODE)
    if (switch2) {
        ips200_show_int16(0, MT9V03X_H + 1, LBorderCount);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RBorderCount);
        ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 1, centerLineCount);
    }
#endif
    if (CROSS_MODE == CROSS_READY) {
        //gpio_set_level(IO_P90, 1);
        if (centerLineCount && centerLineCount < CROSS_ENTER_COUNT) {
            crossThreshold = (threshold + LThreshold) >> 1;
            alongR = LBorderCount < RBorderCount;
            CROSS_MODE = CROSS_IN;
        }
        alongLine(centerLine, centerLineCount, getAimPos());
    } else if (CROSS_MODE == CROSS_IN) {
        Point border[expRoadCount], farCenterLine[expRoadCount];
        int16 roadCount = expRoadCount, farCenterLineCount, corner;
        if (!LBorderCount && !RBorderCount)
            canEnd = 1;
        banUpdateThreshold = 1;
        if (alongR) {
            start = getCrossRStart();
            getRBorder(border, start, &roadCount);
        } else {
            start = getCrossLStart();
            getLBorder(border, start, &roadCount);
        }
        getGodRoadLine(border, roadCount);
        resampleRoad(border, &roadCount, 0, 3);
        corner = findCorner(border, roadCount, 1);
        filterRoad(border, roadCount);
        farCenterLineCount = roadCount;
        if (farCenterLineCount > expRoadCount) farCenterLineCount = expRoadCount;
        if (farCenterLineCount < 2) {
            // 远边界太短：本帧保持上一拍控制，不写数组也不做补线
            alongLine(centerLine, centerLineCount, getAimPos());
            return;
        }
        if (alongR)
            getCenterLineByRBorder(border, farCenterLine, farCenterLineCount);
        else getCenterLineByLBorder(border, farCenterLine, farCenterLineCount);
        if (corner < 0 || corner >= farCenterLineCount) corner = 0;
        // 原来 func_min(corner+3, count) 会返回 count=100，写 farCenterLine[100] 越界
        if (corner && farCenterLine[corner].x > 50)
            corner = func_min(corner + 3, farCenterLineCount - 1);
        else
            corner = func_min(5, farCenterLineCount - 1);
        /* 补线终点必须落在"车头行"上：原来写 MT9V03X_H-1（行 119），
         * 视野窗口裁到行 99 以后这段落在车身/窗口之外，沿线的 dx 会变成 0 或负值，
         * 纯跟踪的 atan 直接打满舵。 */
        farCenterLine[corner].x = VIEW_NEAR_ROW;
        farCenterLine[corner].y = MT9V03X_W / 2 - 3;
        resampleRoad(farCenterLine, &farCenterLineCount, corner, 2);
        filterRoad(farCenterLine, farCenterLineCount);
        alongLine(farCenterLine, farCenterLineCount, getAimPos());
        /* 出口条件：中线恢复到正常长度，且（两侧边界都消失过——canEnd，
         * 或此刻两侧边界都已恢复）。原来只认 canEnd：若入口在单侧丢边时
         * 提前触发、两侧从未同时消失，canEnd 永远为 0，十字会补线到
         * 8 s 超时才退出——这就是"十字补线后出不来"。 */
        if (centerLineCount > CROSS_EXIT_COUNT && (canEnd || (LBorderCount && RBorderCount))) {
            canEnd = minMaxH = 0;
            banUpdateThreshold = 0;   // 退出十字才解除阈值冻结
            CROSS_MODE = CROSS_READY;
            RUN_MODE = FORWARD;
            gpio_set_level(IO_P90, 0);
        }
    }
}

void crossReset(void) {
    CROSS_MODE = CROSS_READY;
    canEnd = 0;
    minMaxH = 0;
    alongR = 0;
    crossThreshold = 0;
    banUpdateThreshold = 0;
}

/* 当前阶段（1=READY 2=IN）：主循环按它重置元素超时预算 */
uint8 crossStage(void) {
    return (uint8) CROSS_MODE;
}