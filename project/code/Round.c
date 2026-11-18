//
// 第二部分：环岛
//
// ============================ 输入 ============================
//   LRawCount / RRawCount       拐点裁短前的边界点数（loadRoad 输出）
//   LRoadCount / RRoadCount     裁短后的点数：有拐点时等于拐点序号（≥3），
//                               没拐点时等于原始点数（0~2）——这一条决定了
//                               下面"内侧 < 4"同时覆盖"被拐点裁短"和"整条消失"
//   LRoad[] / RRoad[]           边界点列，[0] 最近，[n-1] 最远
//   LCorner / RCorner           拐点序号，0 = 没有拐点
//   centerLine / centerLineCount 中线点列（供 alongLine 循迹）
//
// ========================== 识别条件 ==========================
//   右环岛（内侧 = 右边界）：             左环岛（内侧 = 左边界）：
//     左边界原始点数 > RD_OUTER_MIN          右边界原始点数 > RD_OUTER_MIN
//     左边界末点行 ≤ RD_FAR_ROW               右边界末点行 ≤ RD_FAR_ROW
//     右边界点数  < RD_INNER_MAX              左边界点数  < RD_INNER_MAX
//
//   三条同时成立才算"候选"。但候选常常只维持 1~2 帧（内侧边界在画面底部
//   时隐时现），所以用积分确认：候选成立 +1、不成立 -1、到 0 为止，
//   攒到 RD_SCORE_NEED 才真正进入。断续出现能过，偶发一帧不会误判。
//
// ========================== 触发后动作 ========================
//   五阶段，每阶段一个固定的转向偏置（度，正 = 往右；左环岛取负）：
//
//     阶段    进入条件                        偏置            说明
//     READY   积分确认通过                    ±RD_BIAS_READY  入口 0.3 s 就过，先带一点
//     START   内侧缺口出现(<RD_INNER_MAX)     ±RD_BIAS_START  进环，打角最大
//             且内侧重新长回 >RD_INNER_BACK
//     IN      对侧缺口出现后重新长回 >RD_OUT_BACK  ±RD_BIAS_IN 环内视觉在线，只补一点
//     END     出口侧边界恢复且末点伸回外侧      0              出环要顺，交给视觉
//     EXIT    入口侧边界恢复                  0              收尾
//
//   全程 elemSpeedRatio = RD_SPEED_RATIO（50%），与 borderIn 的作用重复但更明确。
//   "缺口锁存"用 <RD_INNER_MAX 而不是 ==0：入口判据本来就是"<4"，
//   用 ==0 会在内侧只剩 1~3 点时永远锁不上（这是"想进进不去"的老毛病）。
//
// ========================== 退出条件 ==========================
//   EXIT 完成 / 单阶段超时 RD_STAGE_MS / 入口误判超时 RD_READY_MS。
//   总时长由仲裁器兜底（ELEM_TOTAL_MS）。
//

#include "Round.h"

/* ---- 入口判据（与 main.c 的视野窗口同尺度） ---- */
#define RD_OUTER_MIN     16   // 外侧边界原始点数下限（直道约 21~28）
/* 外侧末点必须伸到的行号。原值 18 是按"外侧边界一路贴到窗口顶端"设的，
 * 但实车在入口处扫到 L=55 点、末点行仍大于 18，于是 CAND 永远缺这一位
 * （实测 CAND=5：bit0 与 bit2 都成立，只差 bit1）。
 * 30 相当于"伸到画面中上部"即可；特异性改由下面"内侧不能被拐点裁短"保证，
 * 不再靠苛刻的行号门槛。 */
#define RD_FAR_ROW       30
/* 内侧点数上限。原值 4 是按"内侧整条消失（0~2 点）"设的，但实车环岛入口的
 * 内侧线通常不是整条消失，而是弯着退出画面侧边，还剩 5~8 点就一直不满足，
 * 于是"看得见要进环岛、舵机也有打角倾向，但状态机永远不切"。
 * 放宽到 7 的前提是 rdCand() 里加了"内侧不能有拐点"：
 * 普通弯道的内侧线是被拐点裁短的（RRoadCount = RCorner），会被那一条挡掉；
 * 环岛缺口是真的没线（RCorner == 0）。两者用拐点区分，比用点数可靠。 */
#define RD_INNER_MAX      7   // 内侧点数上限：< 7 即视为缺口
#define RD_SCORE_NEED     3   // 积分确认门槛（3 帧里成 3 次）

/* ---- 阶段推进门槛 ---- */
/* START / IN 的时间兜底：入口到环内约 20~30 cm，50% 速度下约 0.6~0.8 s；
 * 环内绕行约 0.8~1.2 s。这两条保证状态机一定往前走，不再依赖
 * "边界先消失再长回"这种必须等车先动起来才成立的几何条件。
 * 这条赛道的环直径固定，时间参数按实测调一次即可（不做通用化）。 */
/* 打角保持的总时长决定"绕多少度"。实拍轨迹显示：大环（水滴岛）进去之后
 * 绕了整整一圈又从来路出来（轨迹自交），说明保持时间偏长。
 * 原来 START 700 + IN 1200 = 1.9 s，按 50% 速度折算约 360°；
 * 正常环岛出口在 270° 左右，所以压到 500 + 800 = 1.3 s（约 -32%）。 */
#define RD_START_MS      500
#define RD_IN_MS         800
#define RD_GAP_CONFIRM   3    // 缺口连续出现几帧就认定"入口已到"，直接进 START
#define RD_INNER_BACK    10   // 内侧边界长回来的点数（确认已绕进环内）
#define RD_OUT_BACK      20   // 对侧边界缺口后长回来的点数
#define RD_END_PTS       25   // 出口侧边界恢复的点数门槛
#define RD_END_COL_R    133   // 右环岛出口：右边界末点列号 < 该值（94+39）
#define RD_END_COL_L     55   // 左环岛出口：左边界末点列号 > 该值（94-39）

/* ---- 各阶段转向偏置（度，右环岛为正；左环岛自动取负） ---- */
/* 打角（度）。实拍：小环（圆岛）入口处舵机只"点一下"就回正，车沿外侧直穿，
 * 说明入口段的转向余量不够 —— READY 从 18 抬到 22，START 从 22 抬到 24。
 * 大环的表现相反（绕多了一圈），但它的主因是保持时间太长，
 * 时间已从 1.9 s 压到 1.3 s，折合少绕约 110°，所以这里加 4° 不会让它更糟。 */
#define RD_BIAS_READY    22
#define RD_BIAS_START    24
#define RD_BIAS_IN       12

/* ---- 时间与速度 ---- */
#define RD_SPEED_RATIO   50   // 环岛内目标速度比例（%）
#define RD_READY_MS    2500   // READY 误判兜底（原 1200 太短：车还没开到环内就超时退回）
#define RD_STAGE_MS    8000   // 单阶段上限

#define RD_ST_READY       1
#define RD_ST_START       2
#define RD_ST_IN          3
#define RD_ST_END         4
#define RD_ST_EXIT        5

extern int16 centerLineCount, LCorner, RCorner;
extern int16 LRawCount, RRawCount;
extern volatile int16 LRoadCount, RRoadCount;
extern Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount];

static uint8 rdStageNo;      // 1~5，0 = 空闲
static uint8 rdSideNo;       // 0 = 右环岛，1 = 左环岛
static uint8 rdScoreR, rdScoreL;
static uint8 rdGap1, rdGap2, rdGap3, rdGap4;   // 四个缺口的锁存
static uint8 rdGapCnt;       // 内侧缺口连续帧数（READY 里用来确认入口）
static uint16 rdMs;          // 本阶段进入时刻

/* ------------------------- 入口候选 ------------------------- */
/* side：0 = 右环岛（内侧是右边界），1 = 左环岛（内侧是左边界） */
static uint8 rdCand(uint8 side) {
    int16 outerRaw, innerPts, innerCorner;
    int16 outerFarRow;

    if (side == RD_SIDE_R) {
        outerRaw = LRawCount;
        innerPts = RRoadCount;
        innerCorner = RCorner;
        if (LRawCount <= 0 || LRawCount > expRoadCount) return 0;
        outerFarRow = (int16) LRoad[LRawCount - 1].x;
    } else {
        outerRaw = RRawCount;
        innerPts = LRoadCount;
        innerCorner = LCorner;
        if (RRawCount <= 0 || RRawCount > expRoadCount) return 0;
        outerFarRow = (int16) RRoad[RRawCount - 1].x;
    }

    if (outerRaw <= RD_OUTER_MIN) return 0;      // 外侧不够长
    if (outerFarRow > RD_FAR_ROW) return 0;      // 外侧没伸到画面中上部（是在弯里拐出去了）
    /* 内侧必须是"真消失"，不能是"被拐点裁短"。
     * RRoadCount 在找到拐点时会等于拐点序号（≥3），这种情况下内侧线其实还在，
     * 只是后面拐弯了——那是普通弯道，不是环岛缺口。
     * 加上这一条之后才敢把 RD_INNER_MAX 放宽到 7、把 RD_FAR_ROW 放宽到 30：
     * 特异性由"有没有拐点"来保证，而不是靠苛刻的行号门槛。 */
    if (innerCorner) return 0;
    if (innerPts >= RD_INNER_MAX) return 0;      // 内侧还太完整，不是缺口
    return 1;
}

static void rdScoreStep(void) {
    if (rdCand(RD_SIDE_R)) {
        if (rdScoreR < RD_SCORE_NEED) rdScoreR++;
    } else if (rdScoreR) rdScoreR--;

    if (rdCand(RD_SIDE_L)) {
        if (rdScoreL < RD_SCORE_NEED) rdScoreL++;
    } else if (rdScoreL) rdScoreL--;
}

/* ------------------------- 对外接口 ------------------------- */
uint8 rdDetect(void) {
    if (rdStageNo) return 1;      // 已经在环岛里，不再重新判定

    rdScoreStep();
    if (rdScoreR >= RD_SCORE_NEED) { rdSideNo = RD_SIDE_R; return 1; }
    if (rdScoreL >= RD_SCORE_NEED) { rdSideNo = RD_SIDE_L; return 1; }
    return 0;
}

void rdEnter(void) {
    rdStageNo = RD_ST_READY;
    rdMs = msTick;
    rdGap1 = rdGap2 = rdGap3 = rdGap4 = 0;
    rdGapCnt = 0;
    rdScoreR = rdScoreL = 0;
}

/* 本帧四个量的取值，按方向取"内侧 / 外侧"两个轴 */
static int16 rdInnerPts(void) {
    return (rdSideNo == RD_SIDE_R) ? RRoadCount : LRoadCount;
}
static int16 rdInnerRaw(void) {
    return (rdSideNo == RD_SIDE_R) ? RRawCount : LRawCount;
}
static int16 rdOuterRaw(void) {
    return (rdSideNo == RD_SIDE_R) ? LRawCount : RRawCount;
}
static int16 rdOuterCorner(void) {
    return (rdSideNo == RD_SIDE_R) ? LCorner : RCorner;
}
static int16 rdInnerCorner(void) {
    return (rdSideNo == RD_SIDE_R) ? RCorner : LCorner;
}

static int8 rdBiasNow(void) {
    int8 b;

    switch (rdStageNo) {
    case RD_ST_READY: b = RD_BIAS_READY; break;
    case RD_ST_START: b = RD_BIAS_START; break;
    case RD_ST_IN:    b = RD_BIAS_IN;    break;
    default:          b = 0;             break;
    }
    /* 右环岛往右（正），左环岛镜像取负 */
    return (int8) (rdSideNo == RD_SIDE_R ? b : -b);
}

void rdRun(void) {
    int16 innerRaw, outerRaw, outerFarCol;

    elemSpeedRatio = RD_SPEED_RATIO;
    elemSteerBiasDeg = rdBiasNow();
    alongLine(centerLine, centerLineCount, getAimPos());

    innerRaw = rdInnerRaw();
    outerRaw = rdOuterRaw();
    outerFarCol = 0;
    if (rdSideNo == RD_SIDE_R) {
        if (LRawCount > 0 && LRawCount <= expRoadCount) outerFarCol = (int16) LRoad[LRawCount - 1].y;
    } else {
        if (RRawCount > 0 && RRawCount <= expRoadCount) outerFarCol = (int16) RRoad[RRawCount - 1].y;
    }

    switch (rdStageNo) {
    case RD_ST_READY:
        /* 缺口连续出现 RD_GAP_CONFIRM 帧就立刻进 START。
         *
         * 原来的条件是"缺口出现过、然后等内侧边界重新长回 >10 点"，
         * 但内侧边界只有车已经拐进环里之后才会长回来，而车能不能拐进去
         * 恰恰取决于 START 之后才实施的那套打角 —— 逻辑上是循环等待。
         * 结果 READY 一直等到 RD_READY_MS 超时被踢回 FORWARD，
         * 实车表现就是"进了 READY 又退出"。
         * 现在缺口本身就是"入口已到"的证据：确认 3 帧就开始转。
         * 保留"缺口已锁存且内侧长回"这条作为并行的快速通道，
         * 覆盖"缺口闪一下、车已经自己拐进去"的情况。 */
        if (rdInnerPts() < RD_INNER_MAX) {
            if (rdGapCnt < 255) rdGapCnt++;
            rdGap1 = 1;
        } else {
            rdGapCnt = 0;
        }
        if (rdGapCnt >= RD_GAP_CONFIRM ||
            (rdGap1 && rdInnerRaw() > RD_INNER_BACK && !rdInnerCorner())) {
            rdStageNo = RD_ST_START;
            rdMs = msTick;
        }
        break;

    case RD_ST_START:
        /* 进入环内的时间是可以用几何算出来的：入口到环内约 20~30 cm，
         * 50% 速度下 0.6~0.8 s。所以除了"对侧缺口 + 长回"这条几何条件，
         * 再加一条时间兜底——否则同样是循环等待（对侧边界要不要消失，
         * 取决于车有没有拐进去，而拐进去正是这一阶段在做的事）。 */
        if (outerRaw < RD_INNER_MAX) rdGap2 = 1;              // 对侧缺口
        if ((rdGap2 && outerRaw > RD_OUT_BACK && !rdOuterCorner()) ||
            (uint16) (msTick - rdMs) > RD_START_MS) {
            rdStageNo = RD_ST_IN;
            rdMs = msTick;
        }
        break;

    case RD_ST_IN:
        if (outerRaw < RD_INNER_MAX) rdGap3 = 1;              // 出口缺口
        if ((rdGap3 && outerRaw > RD_OUT_BACK) ||
            (uint16) (msTick - rdMs) > RD_IN_MS) {
            rdStageNo = RD_ST_END;
            rdMs = msTick;
        }
        break;

    case RD_ST_END:
        /* 出口侧边界恢复且末点重新伸回外侧（说明已经绕过出口的圆弧） */
        if (!rdOuterCorner() && outerRaw > RD_END_PTS &&
            ((rdSideNo == RD_SIDE_R) ? (outerFarCol > RD_END_COL_L)
                                     : (outerFarCol < RD_END_COL_R))) {
            rdStageNo = RD_ST_EXIT;
            rdMs = msTick;
        }
        break;

    case RD_ST_EXIT:
        if (innerRaw < RD_INNER_MAX) rdGap4 = 1;              // 入口侧再出现缺口
        if (rdGap4 && innerRaw > RD_INNER_BACK) rdStageNo = 0; // 完成
        break;

    default:
        rdStageNo = 0;
        break;
    }
}

uint8 rdBusy(void) {
    uint16 limit;

    if (!rdStageNo) return 0;
    limit = (rdStageNo == RD_ST_READY) ? RD_READY_MS : RD_STAGE_MS;
    if ((uint16) (msTick - rdMs) > limit) return 0;
    return 1;
}

void rdExit(void) {
    rdStageNo = 0;
    rdMs = 0;
    rdGap1 = rdGap2 = rdGap3 = rdGap4 = 0;
    rdGapCnt = 0;
    /* 积分一起清零：否则刚出环岛那一帧分数还在门槛上，会立刻被自己重新触发 */
    rdScoreR = rdScoreL = 0;
    elemSteerBiasDeg = 0;
    elemSpeedRatio = 100;
}

void rdReset(void) {
    rdStageNo = 0;
    rdSideNo = RD_SIDE_R;
    rdScoreR = rdScoreL = 0;
    rdGap1 = rdGap2 = rdGap3 = rdGap4 = 0;
    rdGapCnt = 0;
    rdMs = 0;
    elemSteerBiasDeg = 0;
    elemSpeedRatio = 100;
}

uint8 rdStage(void) {
    return rdStageNo;
}

uint8 rdSide(void) {
    return rdSideNo;
}

/* 只读诊断：把入口的三条判据分别暴露出来，只看当前帧，不改任何状态。
 *   右环岛（外侧 = 左边界，内侧 = 右边界）
 *     bit0  外侧原始点数 > RD_OUTER_MIN
 *     bit1  外侧末点行   <= RD_FAR_ROW
 *     bit2  内侧点数     <  RD_INNER_MAX
 *     → 0x07 才是完整候选
 *   左环岛是镜像，用 bit4~bit6（0x70）
 * 推着车沿赛道逐点看这个值，就知道"如果车在跑，这里会不会被判成环岛入口"，
 * 以及缺的是哪一条 —— 不用真的跑起来。 */
uint8 rdCandBits(void) {
    uint8 bits = 0;
    int16 farRow;

    if (LRawCount > 0 && LRawCount <= expRoadCount) {
        farRow = (int16) LRoad[LRawCount - 1].x;
        if (LRawCount > RD_OUTER_MIN) bits |= 0x01;
        if (farRow <= RD_FAR_ROW) bits |= 0x02;
    }
    if (RRoadCount < RD_INNER_MAX && !RCorner) bits |= 0x04;

    if (RRawCount > 0 && RRawCount <= expRoadCount) {
        farRow = (int16) RRoad[RRawCount - 1].x;
        if (RRawCount > RD_OUTER_MIN) bits |= 0x10;
        if (farRow <= RD_FAR_ROW) bits |= 0x20;
    }
    if (LRoadCount < RD_INNER_MAX && !LCorner) bits |= 0x40;

    return bits;
}

/* 只读诊断：积分确认的进度（取两侧较大值），到 RD_SCORE_NEED 就会触发。 */
uint8 rdScore(void) {
    return (uint8) (rdScoreR > rdScoreL ? rdScoreR : rdScoreL);
}
