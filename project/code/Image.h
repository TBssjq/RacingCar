//
// Created by 31663 on 2026/3/16.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
#include "zf_common_headfile.h"

#define block 5               //局域二值化边长
#define cap 4                 //去噪余量（黑白对比用6，蓝白对比低改用4，噪点多再调大）
#define expRoadCount 100      //边线期望长度

/* ---------------- 蓝底白赛道现场标定参数 ---------------- */
/* 摄像头安装：20 cm 高、45° 俯角。
 * reflection 表按旧安装（25 cm / 39°）离线标定，安装一变透视就变，
 * 查表反而把中线拉到错误位置。0 = 旁路查表、直接用原始坐标；
 * 重新标定 reflection 之后再改回 1。 */
#define USE_REFLECTION 0

#define EDGE_DIFF 12          //边界梯度最小差（原15，蓝白对比低改小）
#define EDGE_RATIO 7          //亮度和/差值比上限，越小越严格
/* 车身遮挡区起始行。20 cm / 45° 下画面更陡，车头在画面里比 25 cm / 39°
 * 时更靠上，装车后必须在直道待发车读串口 NOSE，把这两个值改成 NOSE - 2。
 * 未实测前先保持旧值，NOSE 落在 90 以上时不需要动。 */
#define HIDE_ROW 90           //车身遮挡区起始行（该行以下中心区域强制为白）
#define HIDE_COL_L 48         //遮挡区左列
#define HIDE_COL_R 140        //遮挡区右列
/* 车身在画面里出现的最上行。该行及以下的起点搜索只用两侧窄列
 * （左 3~HIDE_COL_L、右 HIDE_COL_R~186），绕开中央的车身暗块。
 * 与 HIDE_ROW 分开：HIDE_ROW 还控制 getImagePointFilterSelf() 的补线遮挡，
 * 改 HIDE_ROW 会把十字补线起点一起推远。 */
#define BODY_ROW 80
#define CORNER_COS100 50      //拐点判定：cos阈值×100，越小越容易出拐点

/* ===== 视野窗口：只保留中间一块，四周各裁掉一圈 =====
 * 摄像头视场明显大于赛道：远端是场地外的底布/反光，近端是车身与车头阴影，
 * 左右边缘还是场地。这些区域对循迹没有信息量，却会让爬线拟合出不存在的"边界"、
 * 让出界采样读到赛道外的暗区。
 * 现场草图标注内框宽 120 列；上下按实车表现分别调：远端多留几行能更早看到弯，
 * 近端裁掉车身。四个裁切量独立可调，边界自动推导：
 *   上 10 行、下 20 行、左 34 列、右 34 列 → 行 10~99（90 行）、列 34~153（120 列）。
 * VIEW_CUT_FAR 从 20 减到 10 的原因：远端裁太多时中线点列短，前瞻点被夹到窗口末端，
 * 远端趋势前馈退化，进弯前看不到弯、打角偏晚，实车表现为"转弯幅度不够"。
 * 调整原则：窗口必须仍包住赛道可能出现的全部位置，正常直道上两侧边线
 * 至少离窗口边缘 10 像素。留窄了会在弯道/环岛里把边线一起裁掉（点数骤降）。
 * 注意别把远端裁得太多：环岛入口判据（main.c 的 ROUND_MIN_PTS / ROUND_FAR_ROW）
 * 要求外侧边界能一路伸到远端，窗口一短这条边界就凑不够点数。 */
#define VIEW_CUT_FAR    10    //远端（上）裁掉的行数；越小看到越远，转弯提前量越大
#define VIEW_CUT_NEAR   20    //近端（下）裁掉的行数：车身与车头阴影
#define VIEW_CUT_LEFT   34    //左侧裁掉的列数
#define VIEW_CUT_RIGHT  34    //右侧裁掉的列数

#define VIEW_FAR_ROW   VIEW_CUT_FAR                        // = 10，保留起始行（含）
#define VIEW_NEAR_ROW  (MT9V03X_H - 1 - VIEW_CUT_NEAR)    // = 99，保留结束行（含）
#define VIEW_COL_L     VIEW_CUT_LEFT                      // = 34，保留起始列（含）
#define VIEW_COL_R     (MT9V03X_W - 1 - VIEW_CUT_RIGHT)   // = 153，保留结束列（含）

void getLBorder(Point LRoad[], Point start, int16 *count);
void getRBorder(Point RRoad[], Point start, int16 *count);
void getCenterLineByLBorder(Point roadLine[], Point centerLine[], int16 count);
void getCenterLineByRBorder(Point roadLine[], Point centerLine[], int16 count);
void filterRoad(Point rodeLine[], int16 count);
void getGodRoadLine(Point roadLine[], int16 count);
void resampleRoad(Point roadLine[], int16 *count, int16 startPos, uint8 dis);
Point getLStart();
Point getRStart();
int16 findCorner(Point roadLine[], int16 count, int8 cross);
uint8 getImagePointFilterSelf(uint8 i, uint8 j);
uint8 imageInitThreshold(void);

/* 把快照的视野窗口以外区域（远端行、近端行、左右列）写成暗区，
 * 屏蔽赛道外的反光、背景与车身。必须在 getLStart() / getLBorder() 之前调用。 */
void cutViewRegion(void);
extern uint8 threshold, LThreshold, RThreshold, centerLineDistance, banUpdateThreshold;
extern int16 LBorderCount, RBorderCount;
extern Point LBorder[expRoadCount], RBorder[expRoadCount];

/* 双缓冲：mt9v03x_image 由 DMA 直接写入（采集缓冲），mt9v03x_frame 是算法统一读取的稳定快照。
 * DMA 完成中断内搬运一帧并置 frameBusy；主循环消费完清 0，消费不过来则丢弃新帧。 */
extern uint8 mt9v03x_frame[MT9V03X_H][MT9V03X_W];
extern volatile uint8 frameBusy;
void mt9v03x_frame_snapshot(void);
#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
