//
// Created by 31663 on 2026/3/29.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_CROSS_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_CROSS_H

#include "zf_common_headfile.h"

/* Cross entry/exit thresholds (must match main.c dispatch).
 * Window rows 10..99, normal centerline is about 40 points; enter when the
 * line is cut short by the opening, exit with an 11-point hysteresis. */
#define CROSS_ENTER_COUNT   13
#define CROSS_EXIT_COUNT    24

void runCross();
void crossReset(void);   // 发车/急停时复位十字状态机

#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_CROSS_H
