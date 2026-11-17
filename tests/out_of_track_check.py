"""出界判据核对：5 点全暗 + 盲区计时 + 连续帧确认，以及发车宽限期。

对应实现（main.c）：
    OUT_SAMPLE_POINTS  = 5    5 个采样点（已在车身顶边以上、赛道内侧）
    OUT_SEEN_POINTS    = 3    任一侧边界点数超过该值即算"看到赛道"
    OUT_BLIND_MS       = 200  连续这么久没看到赛道才允许判出界
    OUT_CONFIRM_FRAMES = 3    连续成立 3 帧
判出界后控制中断再做 3 拍（15 ms）消抖才停车。
"""

POINTS = 5
SEEN_POINTS = 3
BLIND_MS = 200
CONFIRM_FRAMES = 3
GRACE_MS = 800
FRAME_MS = 20          # 一帧的典型处理间隔（约 50 fps）


class Detector:
    """按 main.c 的顺序逐步推进：先更新盲区计时，再判据，再计帧，最后套宽限期。"""

    def __init__(self):
        self.seen_ms = 0
        self.frames = 0
        self.out = False

    def step(self, t_ms, dark, lc, rc, run_ms):
        if lc > SEEN_POINTS or rc > SEEN_POINTS:
            self.seen_ms = t_ms
        blind = (t_ms - self.seen_ms) >= BLIND_MS
        if dark >= POINTS and blind:
            self.frames += 1
            self.out = self.frames >= CONFIRM_FRAMES
        else:
            self.frames = 0
            self.out = False
        if run_ms < GRACE_MS:
            self.frames = 0
            self.seen_ms = t_ms
            self.out = False
        return self.out


def steady(dark, lc, rc, t_ms=2000, run_ms=5000):
    """连续 CONFIRM_FRAMES 帧同样输入后的结论（用于真值表）。"""
    d = Detector()
    v = False
    for _ in range(CONFIRM_FRAMES):
        v = d.step(t_ms, dark, lc, rc, run_ms)
    return v


def old_out_of_track(lc, rc, dark):
    """上一版：5 点全暗，或「两侧边界全丢且至少 3 点暗」，单帧即成立。"""
    return dark >= 5 or (lc == 0 and rc == 0 and dark >= 3)


print("=== 1. 判据真值表（连续 3 帧同样输入，发车 5 s 之后）===")
print("场景                                  L  R  暗点  上一版  当前  期望")
cases = [
    ("直道上两侧都正常", 60, 58, 0, False),
    ("直道，车头前方有阴影", 60, 58, 2, False),
    ("直道，画面整体偏暗", 60, 58, 4, False),
    ("弯道里边界跑出搜索范围", 0, 0, 0, False),
    ("弯道里边界失败，画面多数偏暗", 0, 0, 3, False),
    ("单侧可见，画面全暗（反光）", 40, 0, 5, False),
    ("两侧都无边界，画面全暗", 0, 0, 5, True),
    # 单侧边界还在画面里：说明赛道上仍有可循迹的线，纯跟踪会把车拉回去，
    # 这里刻意不停车——否则直道上"边线短暂只剩一侧 + 采样点落在场地外暗区"
    # 就会变成误停。真正冲出赛道的情形由下一条（两侧全丢）覆盖。
    ("冲出赛道但残留一侧边界", 0, 6, 5, False),
]
ok_all = True
for name, lc, rc, dark, expect in cases:
    old = old_out_of_track(lc, rc, dark)
    cur = steady(dark, lc, rc)
    ok = cur == expect
    ok_all = ok_all and ok
    print("  %-34s %2d %2d  %d    %-5s  %-5s  %s"
          % (name, lc, rc, dark, old, cur, "通过" if ok else "不符预期"))

print()
print("=== 2. 盲区计时：看不到赛道要持续 %d ms 才算数 ===" % BLIND_MS)
print("  距上次看到赛道  边界   结论   说明")
for gap, lc, rc in ((0, 60, 58), (100, 0, 0), (199, 0, 0), (200, 0, 0), (400, 0, 0)):
    d = Detector()
    d.frames = CONFIRM_FRAMES - 1          # 帧数已攒够，单独观察盲区这一条
    v = d.step(gap, 5, lc, rc, 5000)
    if lc > SEEN_POINTS or rc > SEEN_POINTS:
        note = "画面里有边界，不判"
    elif gap < BLIND_MS:
        note = "盲区未到，不判"
    else:
        note = "盲区已到，推进到 3 帧并判出界"
    print("  %4d ms         %2d/%2d  %-5s  %s" % (gap, lc, rc, v, note))

print()
print("=== 3. 直道上偶发暗区（旧判据第 3 帧就误停，当前继续跑）===")
print("  帧  时刻   L   R  暗点  上一版  当前   说明")
seq = [(0, 60, 58, 1), (20, 60, 58, 4), (40, 12, 8, 5), (60, 0, 0, 5), (80, 40, 38, 2), (100, 55, 52, 0)]
d = Detector()
for n, (dt, lc, rc, dark) in enumerate(seq):
    cur = d.step(dt, dark, lc, rc, 1200)
    old = old_out_of_track(lc, rc, dark)
    if old and not cur:
        note = "上一版误停，当前继续"
    elif cur:
        note = "判定出界"
    elif dark >= POINTS:
        note = "帧数还没攒够"
    else:
        note = "未满足条件，计数清零"
    print("  %2d  %4d  %2d  %2d   %d    %-5s  %-5s  %s"
          % (n + 1, dt, lc, rc, dark, old, cur, note))

print()
print("=== 4. 真正冲出赛道：5 点全暗且持续无边界 ===")
print("  帧  时刻   暗点  当前结论")
d = Detector()
stop_at = None
for n, t in enumerate(range(1000, 1000 + 6 * FRAME_MS, FRAME_MS)):
    v = d.step(t, 5, 0, 0, 5000)
    print("  %2d  %4d    5    %s" % (n + 1, t, v))
    if v and stop_at is None:
        stop_at = t
print("  起判到停车: %d ms（3 帧确认）+ 15 ms 消抖 = %d ms"
      % (stop_at - 1000, stop_at - 1000 + 15))

print()
print("=== 5. 发车宽限期 ===")
print("  发车后经过  当前判据  说明")
for t in (0, 200, 799, 800, 1000):
    d = Detector()
    d.frames = CONFIRM_FRAMES - 1          # 时刻取大值，盲区已满足，只看宽限期
    v = d.step(9000, 5, 0, 0, t)
    note = "宽限期内不判出界" if t < GRACE_MS else "已开始判出界"
    print("  %4d ms      %-5s    %s" % (t, v, note))

print()
print("总判定: " + ("全部符合预期" if ok_all else "存在不符预期的用例"))
