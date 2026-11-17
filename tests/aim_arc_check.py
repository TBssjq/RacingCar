"""前瞻点取法对比：Motor.c 的 PP_ARC_MODE = 0（按下标）/ 1（按弧长）。

两种取法用的是同一条中线（resampleRoad(dis=2)，点间距 2 像素），
差别只在"目标距离被中线长度夹逼时怎么办"：

  0  旧：pos = clamp(aimPos, 2, count-1) 之后再 clamp 到 count-1-BEND_FAR_AHEAD，
         前瞻距离 = 2×min(aimPos, count-13) 像素
  1  新：前瞻距离 = min(2×aimPos, 中线总弧长) 像素，不再扣掉 12 个点

本脚本复算同一横向偏差 dy 下，两种取法给出的 dx / 转角 / duty，
用来判断切换后需要把 c1 补回多少。数值实现与固件一致（整数版 atan2_int）。
"""

import math

# ---- 与 zf_common_function.c 一致的整数实现 ----
TAN_TABLE = [
    0, 1143, 2288, 3434, 4582, 5732, 6887, 8046, 9212, 10383, 11562, 12749,
    13945, 15151, 16368, 17597, 18840, 20097, 21369, 22657, 23962, 25286,
    26628, 27989, 29371, 30775, 32203, 33654, 35132, 36637, 37837,
]


def atan2_int(y, x):
    sign = 1 if y >= 0 else -1
    ay = abs(y)
    d = 29
    if ay == 0 or x <= 0:
        return 0
    lhs = ay * 65536
    if lhs >= x * TAN_TABLE[30]:
        return sign * 30
    while d >= 0:
        if lhs >= x * TAN_TABLE[d]:
            break
        d -= 1
    return sign * d


def sqrt_int(x):
    res = 0
    bi = 1 << 30
    if x <= 0:
        return 0
    while bi > x:
        bi >>= 2
    while bi:
        if x >= res + bi:
            x -= res + bi
            res = (res >> 1) + bi
        else:
            res >>= 1
        bi >>= 2
    return res


# ---- 与 Motor.h / Motor.c 一致的参数 ----
BEND_FAR_AHEAD = 12
BEND_FEED_DIV = 3
DEVICE_H = 18            # PP_LOOKAHEAD_PX
STEER_REF_ROW = 99       # VIEW_NEAR_ROW
PP_DX_MIN = 30
PP_ARC_EXT_MAX = 24
CENTER_COL = 94
STEER_DUTY_DIV = 20
STEER_DUTY_LIMIT = 395


def build_line(n, slope=0.0):
    """按 2 像素弧长生成一条中线：返回 [(row, col), ...]，首点在车头行 99。"""
    pts = [(99, CENTER_COL)]
    step = 2.0 / math.sqrt(1.0 + slope * slope)
    dr, dc = -step, slope * step
    r, c = 99.0, float(CENTER_COL)
    for _ in range(n - 1):
        r += dr
        c += dc
        pts.append((int(round(r)), int(round(c))))
    return pts


def line_len(pts):
    total = 0
    for i in range(1, len(pts)):
        total += sqrt_int((pts[i][0] - pts[i - 1][0]) ** 2 + (pts[i][1] - pts[i - 1][1]) ** 2)
    return total


def along_aim_points(line, aim_arc):
    """Motor.c: alongAimPoints() 的逐行移植。"""
    count = len(line)
    aim = line[0]
    far = line[0]
    far_arc = aim_arc + BEND_FAR_AHEAD * 2
    acc = 0
    got_aim = got_far = False
    last_sx = last_sy = last_dn = 0
    for i in range(1, count):
        sx = line[i][0] - line[i - 1][0]
        sy = line[i][1] - line[i - 1][1]
        dn = sqrt_int(sx * sx + sy * sy)
        if dn == 0:
            continue
        if not got_aim and acc + dn >= aim_arc:
            rem = aim_arc - acc
            aim = (line[i - 1][0] + sx * rem // dn, line[i - 1][1] + sy * rem // dn)
            got_aim = True
        if acc + dn >= far_arc:
            rem = far_arc - acc
            far = (line[i - 1][0] + sx * rem // dn, line[i - 1][1] + sy * rem // dn)
            got_far = True
            break
        acc += dn
        last_sx, last_sy, last_dn = sx, sy, dn
    if not got_far:
        if not got_aim:
            aim = line[count - 1]
        rem = BEND_FAR_AHEAD * 2 if got_aim else far_arc - acc
        rem = min(rem, PP_ARC_EXT_MAX)
        if last_dn:
            far = (aim[0] + last_sx * rem // last_dn, aim[1] + last_sy * rem // last_dn)
        else:
            far = aim
    return aim, far


def mode0(line, aim_pos):
    n = len(line)
    pos = min(max(aim_pos, 2), n - 1)
    if n > BEND_FAR_AHEAD + 3:
        pos = min(max(pos, 2), n - 1 - BEND_FAR_AHEAD)
    far_pos = min(max(pos + BEND_FAR_AHEAD, pos), n - 1)
    return line[pos], line[far_pos]


def mode1(line, aim_pos):
    n = len(line)
    pos = min(max(aim_pos, 2), n - 1)
    return along_aim_points(line, pos * 2)


def steer(dy, aim_row):
    dx = STEER_REF_ROW - aim_row + DEVICE_H
    dx = max(dx, PP_DX_MIN)
    return dx, atan2_int(2 * DEVICE_H * dy, dx * dx + dy * dy)


def report(c3, speed, dy_list=(10, 20, 30)):
    aim_pos = c3 + speed // 40
    print("c3=%d  目标速度 %d → aimPos = %d 个点（目标弧长 %d 像素）"
          % (c3, speed, aim_pos, aim_pos * 2))
    head = "  中线点数 | 线长 |   旧:前瞻/行号/dx |   新:前瞻/行号/dx | dy=%s 时 旧d/新d (duty)"
    print(head % "/".join(str(d) for d in dy_list))
    for n in (16, 20, 25, 30, 45):
        line = build_line(n)
        total = line_len(line)
        a0, _ = mode0(line, aim_pos)
        a1, _ = mode1(line, aim_pos)
        len0 = 2 * min(max(aim_pos, 2), max(n - 1 - BEND_FAR_AHEAD, 2))
        len1 = min(aim_pos * 2, total)
        dx0, _ = steer(0, a0[0])
        dx1, _ = steer(0, a1[0])
        cells = []
        for dy in dy_list:
            d0 = steer(dy, a0[0])[1]
            d1 = steer(dy, a1[0])[1]
            cells.append("%2d/%-2d(%3d/%3d)" % (d0, d1, d0 * 13, d1 * 13))
        print("  %8d | %4d | %4d /%4d /%3d | %4d /%4d /%3d | %s"
              % (n, total, len0, a0[0], dx0, len1, a1[0], dx1, "  ".join(cells)))
    print()


def check_invariants():
    """新取法的前瞻弧长必须只由 aimPos 决定，与中线点数无关（线够长时）。"""
    aim_pos = 36
    arcs = []
    for n in (40, 45, 50, 60):
        line = build_line(n)
        a1, _ = mode1(line, aim_pos)
        arcs.append(99 - a1[0])
    print("=== 不变量：中线 ≥40 点时，新取法的前瞻行应恒定 ===")
    print("  前瞻行进深:", arcs, " 一致:", len(set(arcs)) == 1)
    print()


print("=== 前瞻点取法对比（PP_ARC_MODE 0=下标 / 1=弧长）===")
print("说明：dy 为同一横向偏差（像素），d 为 atan2_int 解出的图像角度，duty = d×c1/20（c1=260）")
print()
check_invariants()
report(26, 400)
report(22, 300)
report(26, 0)

print("=== 结论 ===")
print("1. 旧取法的前瞻距离被 count-13 夹逼：中线越短，前瞻越贴车头，dx 越小，")
print("   而 ax = dx²+dy² 是平方项 → 同样的 dy 解出的角度成倍放大。")
print("2. 新取法的前瞻距离只由 aimPos 决定（线短时退到线末），dx 稳定，")
print("   同一弯道的打角不再随手帧点数跳变。")
print("3. 切换后同一 dy 的打角变小，需要用 c1 或 PP_LOOKAHEAD_PX 补回转向量——")
print("   上表的 duty 两列之差就是需要补的比例。")
