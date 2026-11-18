//
// 第三部分：十字形环岛（两条赛道带成 X 形相交的路口）
//
// 为什么单独做一块：路口中间是一片很宽的白区，两侧边线同时被缺口打断，
// 中线在那一帧会塌缩甚至消失。此时纯跟踪没有可信的输入——
// 继续跟着算出来的抖动中线走，车就会在路口里画龙，然后从侧面出去。
// 正确做法是：进路口前记住当前朝向，路口内保持该打角直穿，出路口再交回视觉。
//
// 输入：两侧边界的原始点数、中线点数、当前打角
// 识别：两侧边界同时大幅缩短（±中线塌缩）
// 动作：路口中心 250 ms 内保持进路口时的朝向 + 速度压到 60%，之后交回视觉；
//       退出后 500 ms 内不再重新判定（否则残影会让它二次触发、一直出不来）
//
// 与另外两部分的边界：本模块要求"两侧都短"，环岛是"一侧长一侧短"，
// S 弯是"两侧都在"，三者互斥。
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_XCROSS_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_XCROSS_H
#include "zf_common_headfile.h"

#define RUN_X_CROSS     514     // 沿用原有 MODE 值（串口图例不变）

uint8 xcDetect(void);           // 纯谓词：本帧是否满足路口条件
void  xcEnter(void);            // 记住进入时的打角
void  xcRun(void);              // 本帧执行：保持打角直穿
uint8 xcBusy(void);             // 1 = 还要继续占用
void  xcExit(void);             // 收尾
void  xcReset(void);            // 发车/急停强制复位
uint8 xcStage(void);            // 1 = 正在穿路口，0 = 空闲

#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_XCROSS_H
