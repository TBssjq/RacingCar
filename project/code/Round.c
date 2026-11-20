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
#define RD_OUTER_MIN     16   // 外侧边界原始点数下限（66 行窗口直道约 19~24）
#define RD_FAR_ROW       (VIEW_FAR_ROW + 8)   // 外侧末点必须伸到该行以内（34+8=42）
/* 内侧点数上限。原值 4 是按"内侧整条消失（0~2 点）"设的，但实车环岛入口的
 * 内侧线通常不是整条消失，而是**弯着退出画面侧边**，还剩 5~8 点就一直不满足，
 * 于是"看得见要进环岛、舵机也有打角倾向，但状态机永远不切"。
 * 7 是实测口径下的折中：普通弯道被拐点裁短的内侧线一般 3~5 点，环岛入口 5~8 点，
 * 单靠点数分不开，还要靠下面两条（外侧够长 + 外侧末点伸到远端）兜住特异性。
 * 若普通急弯被误判成环岛（表现为入弯突然降到 50% 速度），把它退回 5。 */
#define RD_INNER_MAX      7   // 内侧点数上限：< 7 即视为缺口
#define RD_SCORE_NEED     3   // 积分确认门槛（3 帧里成 3 次）

/* ---- 阶段推进门槛 ---- */
#define RD_INNER_BACK    10   // 内侧边界"长回来"的点数（确认已绕进环内）
#define RD_OUT_BACK      20   // 对侧边界缺口后长回来的点数
#define RD_END_PTS       25   // 出口侧边界恢复的点数门槛
#define RD_END_COL_R    133   // 右环岛出口：右边界末点列号 < 该值（94+39）
#define RD_END_COL_L     55   // 左环岛出口：左边界末点列号 > 该值（94-39）

/* ---- 各阶段转向偏置（度，右环岛为正；左环岛自动取负） ---- */
#define RD_BIAS_READY    12
#define RD_BIAS_START    18
#define RD_BIAS_IN        8

/* ---- 时间与速度 ---- */
#define RD_SPEED_RATIO   50   // 环岛内目标速度比例（%）
#define RD_READY_MS    1800   // READY 误判兜底（原 1200 太短：车还没开到环内就超时退回）
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
static uint16 rdMs;          // 本阶段进入时刻

/* ------------------------- 入口候选 ------------------------- */
/* side：0 = 右环岛（内侧是右边界），1 = 左环岛（内侧是左边界） */
static uint8 rdCand(uint8 side) {
    int16 outerRaw, innerPts;
    int16 outerFarRow;

    if (side == RD_SIDE_R) {
        outerRaw = LRawCount;
        innerPts = RRoadCount;
        if (LRawCount <= 0 || LRawCount > expRoadCount) return 0;
        outerFarRow = (int16) LRoad[LRawCount - 1].x;
    } else {
        outerRaw = RRawCount;
        innerPts = LRoadCount;
        if (RRawCount <= 0 || RRawCount > expRoadCount) return 0;
        outerFarRow = (int16) RRoad[RRawCount - 1].x;
    }

    if (outerRaw <= RD_OUTER_MIN) return 0;      // 外侧不够长
    if (outerFarRow > RD_FAR_ROW) return 0;      // 外侧没伸到远端（是在弯里拐出去了）
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
        if (rdInnerPts() < RD_INNER_MAX) rdGap1 = 1;          // 缺口出现过
        if (rdGap1 && rdInnerRaw() > RD_INNER_BACK && !rdInnerCorner()) {
            rdStageNo = RD_ST_START;
            rdMs = msTick;
        }
        break;

    case RD_ST_START:
        if (outerRaw < RD_INNER_MAX) rdGap2 = 1;              // 对侧缺口
        if (rdGap2 && outerRaw > RD_OUT_BACK && !rdOuterCorner()) {
            rdStageNo = RD_ST_IN;
            rdMs = msTick;
        }
        break;

    case RD_ST_IN:
        if (outerRaw < RD_INNER_MAX) rdGap3 = 1;              // 出口缺口
        if (rdGap3 && outerRaw > RD_OUT_BACK) {
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
    if (RRoadCount < RD_INNER_MAX) bits |= 0x04;

    if (RRawCount > 0 && RRawCount <= expRoadCount) {
        farRow = (int16) RRoad[RRawCount - 1].x;
        if (RRawCount > RD_OUTER_MIN) bits |= 0x10;
        if (farRow <= RD_FAR_ROW) bits |= 0x20;
    }
    if (LRoadCount < RD_INNER_MAX) bits |= 0x40;

    return bits;
}

/* 只读诊断：积分确认的进度（取两侧较大值），到 RD_SCORE_NEED 就会触发。 */
uint8 rdScore(void) {
    return (uint8) (rdScoreR > rdScoreL ? rdScoreR : rdScoreL);
}
