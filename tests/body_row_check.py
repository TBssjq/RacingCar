"""视野窗口 + 车身入镜下起点搜索的核对。

窗口按 24 cm 高 / 27° 俯角：只保留中间一块，四周各裁一圈。
  行 34~99（上裁 34 行、下裁 20 行，保留 66 行；地平线约在行 27）
  列 34~153（左右各裁 34 列，保留 120 列）
窗口外整块写成暗区。脚本核对四件事：
  1. 裁切后窗口外全为 0，窗口内保持原值；
  2. 起点搜索的扫描行区间与二分列范围都落在窗口内；
  3. 车身遮住近处边界时，BODY_ROW=80 的窄列搜索仍能跳到更远处找真起点；
  4. 纵向参考行 STEER_REF_ROW 取窗口下沿后，转向量级相对按画面底边计算提升多少。
"""

H = 120
W = 188
LTHRESHOLD = 130
EDGE_DIFF = 12
EDGE_RATIO = 7
HALF_W = 94
HIDE_COL_L = 48
HIDE_ROW_REF = 90
BODY_ROW = 80

# ---- Image.h ----
VIEW_CUT_FAR = 34
VIEW_CUT_NEAR = 20
VIEW_CUT_LEFT = 34
VIEW_CUT_RIGHT = 34
VIEW_FAR_ROW = VIEW_CUT_FAR
VIEW_NEAR_ROW = H - 1 - VIEW_CUT_NEAR
VIEW_COL_L = VIEW_CUT_LEFT
VIEW_COL_R = W - 1 - VIEW_CUT_RIGHT
HALF_BLOCK = 2

# ---- Motor.h / Motor.c ----
STEER_REF_ROW = VIEW_NEAR_ROW
DEVICE_H = 28
C1 = 260
STEER_DUTY_DIV = 20

# ---- main.c 的出界采样点 ----
OUT_ROW_NEAR = BODY_ROW - 4
OUT_SAMPLE_PT = [
    (OUT_ROW_NEAR, 94),
    (OUT_ROW_NEAR - 2, 74),
    (OUT_ROW_NEAR - 2, 114),
    (OUT_ROW_NEAR - 10, 80),
    (OUT_ROW_NEAR - 10, 108),
]


def cut_view(img):
    """与 cutViewRegion() 等价：窗口外整块写 0。"""
    out = [row[:] for row in img]
    for r in range(H):
        for c in range(W):
            if not (VIEW_FAR_ROW <= r <= VIEW_NEAR_ROW and VIEW_COL_L <= c <= VIEW_COL_R):
                out[r][c] = 0
    return out


def make_image(track_left, track_right, body_left=None, body_right=None,
               body_top=80, body_bottom=99):
    img = [[60] * W for _ in range(H)]
    for row in range(H):
        for col in range(track_left, track_right + 1):
            img[row][col] = 200
    if body_left is not None:
        for row in range(body_top, body_bottom + 1):
            for col in range(body_left, body_right + 1):
                img[row][col] = 40
    return cut_view(img)


def index_start(img, i, row_limit, side):
    """与 getLIndexStart / getRIndexStart 相同的二分，row_limit 决定搜宽列还是窄列。"""
    if side == "L":
        low, high = VIEW_COL_L + HALF_BLOCK, (HALF_W if i < row_limit else HIDE_COL_L)
        while low < high:
            m = (low + high) >> 1
            if (img[i][m] < LTHRESHOLD and
                    img[i][max(0, m - 8)] < LTHRESHOLD and
                    img[i][max(0, m - 25)] < LTHRESHOLD):
                low = m + 1
            else:
                high = m
        return low
    low, high = (HALF_W if i < row_limit else 140), VIEW_COL_R - HALF_BLOCK
    while low < high:
        m = (low + high + 1) >> 1
        if (img[i][m] < LTHRESHOLD and
                img[i][min(187, m + 8)] < LTHRESHOLD and
                img[i][min(187, m + 25)] < LTHRESHOLD):
            high = m - 1
        else:
            low = m
    return high


def check(img, i, y):
    d = abs(img[i][y - 2] - img[i][y + 2])
    a = img[i][y - 2] + img[i][y + 2]
    return d > EDGE_DIFF and a // d <= EDGE_RATIO


def find_left_start(img, row_limit):
    rows = range(VIEW_NEAR_ROW - HALF_BLOCK - 2, VIEW_FAR_ROW + 3 - 1, -3)
    for i in rows:
        y = index_start(img, i, row_limit, "L")
        if img[i][min(y + 2, W - 1)] > LTHRESHOLD and check(img, i, y):
            return i, y + 1
    return 0, 0


print("=== 1. 视野窗口 ===")
keep_rows = VIEW_NEAR_ROW - VIEW_FAR_ROW + 1
keep_cols = VIEW_COL_R - VIEW_COL_L + 1
print("  行 %d~%d（%d 行）、列 %d~%d（%d 列），共 %d 像素"
      % (VIEW_FAR_ROW, VIEW_NEAR_ROW, keep_rows,
         VIEW_COL_L, VIEW_COL_R, keep_cols, keep_rows * keep_cols))
print("  远端裁切 %d 行，地平线约在行 27；窗口从行 %d 开始，保留地面约 2.5 m 前瞻"
      % (VIEW_CUT_FAR, VIEW_FAR_ROW))
img = make_image(40, 145)
outside = [(r, c) for r in range(H) for c in range(W)
           if not (VIEW_FAR_ROW <= r <= VIEW_NEAR_ROW and VIEW_COL_L <= c <= VIEW_COL_R)]
print("  窗口外 %d 像素全为 0: %s" % (len(outside), all(img[r][c] == 0 for r, c in outside)))
print("  扫描行 %d → %d，二分列 %d → %d（左）/ %d → %d（右）"
      % (VIEW_NEAR_ROW - HALF_BLOCK - 2, VIEW_FAR_ROW + 3,
         VIEW_COL_L + HALF_BLOCK, HALF_W, HALF_W, VIEW_COL_R - HALF_BLOCK))
all_in = all(VIEW_FAR_ROW + 1 <= r <= VIEW_NEAR_ROW - 1 and
             VIEW_COL_L + 1 <= c <= VIEW_COL_R - 1 for r, c in OUT_SAMPLE_PT)
print("  5 个出界采样点（行 %d~%d、列 %d~%d）都在窗口内: %s"
      % (min(r for r, _ in OUT_SAMPLE_PT), max(r for r, _ in OUT_SAMPLE_PT),
         min(c for _, c in OUT_SAMPLE_PT), max(c for _, c in OUT_SAMPLE_PT), all_in))

print()
print("=== 2. 起点搜索：HIDE_ROW=90（参考）与 BODY_ROW=80（当前）===")
print("场景                                  参考起点   当前起点   结论")

cases = [
    ("无车身，赛道左边界在列 40", make_image(40, 145), False),
    ("车身只在中央 60~125，不遮边界", make_image(40, 145, 60, 125, 80, 99), False),
    ("车身遮住左边界 34~60，行 80~99", make_image(40, 145, 34, 60, 80, 99), True),
    ("车身延伸到列 34~100，行 80~99", make_image(40, 145, 34, 100, 80, 99), True),
]

for name, im, expect_skip in cases:
    ref = find_left_start(im, HIDE_ROW_REF)
    cur = find_left_start(im, BODY_ROW)
    if expect_skip:
        if cur[1] == 41 and ref[1] != 41:
            note = "当前找到真实边界，参考把车身边缘当成边界"
        elif ref == cur:
            note = "两者一致，车身未影响该行判定"
        else:
            note = "需复查（参考 %s / 当前 %s）" % (ref, cur)
    else:
        note = "两者一致" if ref == cur else "结论不同"
    print("%-38s %-10s %-10s %s" % (name, ref, cur, note))

print()
print("=== 3. 车身行逐行分析（车身 34~60，真实边界 40）===")
im = make_image(40, 145, 34, 60, 80, 99)
for i in (100, 95, 92, 89, 86, 83, 80, 77, 74, 71):
    yr = index_start(im, i, HIDE_ROW_REF, "L")
    yc = index_start(im, i, BODY_ROW, "L")
    okr = im[i][min(yr + 2, 187)] > LTHRESHOLD and check(im, i, yr)
    okc = im[i][min(yc + 2, 187)] > LTHRESHOLD and check(im, i, yc)
    print("  行 %3d  参考: 列%3d 起点有效=%-5s  当前: 列%3d 起点有效=%-5s%s"
          % (i, yr, okr, yc, okc, "（窗口外）" if i > VIEW_NEAR_ROW else ""))

print()
print("=== 4. 窄列搜索在直道上的可用性（车身不遮边界）===")
im = make_image(40, 145)
ref = find_left_start(im, HIDE_ROW_REF)
cur = find_left_start(im, BODY_ROW)
print("  参考起点", ref, " 当前起点", cur, " 一致:", ref == cur)


def steer_angle(dy, aim_row, ref_row):
    """纯跟踪：d = atan(2·DEVICE_H·dy / (dx²+dy²))，返回度。"""
    import math
    dx = ref_row - aim_row + DEVICE_H
    return math.degrees(math.atan2(2 * DEVICE_H * dy, dx * dx + dy * dy))


print()
print("=== 5. 转向量级：STEER_REF_ROW = %d（窗口下沿） vs %d（画面底边）==="
      % (STEER_REF_ROW, H))
print("  前瞻点行  横向偏差  当前转角/duty   按底边转角/duty   放大")
for aim_row, dy in ((50, 10), (47, 20), (31, 20), (31, 30)):
    d_now = steer_angle(dy, aim_row, STEER_REF_ROW)
    d_old = steer_angle(dy, aim_row, H)
    duty_now = int(d_now * C1 / STEER_DUTY_DIV)
    duty_old = int(d_old * C1 / STEER_DUTY_DIV)
    print("  %3d       %2d px    %5.2f° / %3d     %5.2f° / %3d      %.2f×"
          % (aim_row, dy, d_now, duty_now, d_old, duty_old, d_now / d_old))
print("  近端裁掉 %d 行后按画面底边算 dx 会虚增同样的量，转角被压小；"
      % VIEW_CUT_NEAR)
print("  取窗口下沿折算后转角放大约 1.45~1.6 倍，对应实车\"转弯幅度不够\"。")
