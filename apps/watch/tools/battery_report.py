#!/usr/bin/env python3
"""BAT_HIST ログから電池の減り方と電力の内訳を出す。

使い方: battery_report.py <log> [--from EPOCH]
ログの中の最後の BAT_HIST 一式（0 から始まる連番）を使う。
"""
import re
import sys
from datetime import datetime, timedelta, timezone

CAPACITY_MAH = 450
# 起きている間・点灯中の電流の仮定（公式 PowerDemo の測定値から置いた値。実機では未計測）
I_AWAKE_MA = 25.0   # CPU 80MHz で判定中（浅い眠りを含む平均）
I_SHOWN_MA = 45.0   # 画面 50%、CPU 80MHz

# リチウムイオン電池の開放電圧と残量のおおまかな対応（一般的な値）
OCV = [(4200, 100), (4150, 95), (4110, 90), (4080, 85), (4020, 80), (3980, 75), (3950, 70), (3910, 65),
       (3870, 60), (3850, 55), (3840, 50), (3820, 45), (3800, 40), (3790, 35), (3770, 30), (3750, 25),
       (3730, 20), (3710, 15), (3690, 10), (3610, 5), (3270, 0)]

JST = timezone(timedelta(hours=9))
PAT = re.compile(r"BAT_HIST (\d+) epoch=(\d+) mv=(\d+) wakes=(\d+) shows=(\d+) false=(\d+) awake_cs=(\d+) "
                 r"false_cs=(\d+) shown_cs=(\d+)")


def percent(mv):
    if mv >= OCV[0][0]:
        return 100.0
    for (v1, p1), (v2, p2) in zip(OCV, OCV[1:]):
        if v2 <= mv <= v1:
            return p2 + (p1 - p2) * (mv - v2) / (v1 - v2)
    return 0.0


def last_dump(path):
    dumps, cur = [], []
    for line in open(path, errors="replace"):
        m = PAT.search(line)
        if not m:
            continue
        rec = [int(x) for x in m.groups()]
        if rec[0] == 0 and cur:
            dumps.append(cur)
            cur = []
        cur.append(rec)
    if cur:
        dumps.append(cur)
    return max(dumps, key=len) if dumps else []


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    recs = last_dump(sys.argv[1])
    if "--from" in sys.argv:
        start = int(sys.argv[sys.argv.index("--from") + 1])
        recs = [r for r in recs if r[1] >= start]
    if len(recs) < 2:
        print("記録が 2 件未満")
        return 1
    print(f"{'時刻':>11} {'mV':>5} {'残量':>5} {'起床/h':>6} {'空振/h':>6} {'空振ms':>6} {'点灯/h':>6} "
          f"{'点灯s/h':>7} {'起床s/h':>7} {'実測mA':>6}")
    for a, b in zip(recs, recs[1:]):
        h = (b[1] - a[1]) / 3600
        if h <= 0 or b[3] < a[3]:
            continue  # コールドブートをまたいだ
        fw = b[5] - a[5]
        fcs = b[7] - a[7]
        ma = (percent(a[2]) - percent(b[2])) / 100 * CAPACITY_MAH / h
        t = datetime.fromtimestamp(b[1], JST).strftime("%m/%d %H:%M")
        print(f"{t:>11} {b[2]:>5} {percent(b[2]):>4.0f}% {(b[3]-a[3])/h:>6.0f} {fw/h:>6.0f} "
              f"{(fcs*10/fw if fw else 0):>6.0f} {(b[4]-a[4])/h:>6.0f} {(b[8]-a[8])/100/h:>7.1f} "
              f"{(b[6]-a[6])/100/h:>7.1f} {ma:>6.1f}")
    a, b = recs[0], recs[-1]
    h = (b[1] - a[1]) / 3600
    used = (percent(a[2]) - percent(b[2])) / 100 * CAPACITY_MAH
    awake_s = (b[6] - a[6]) / 100
    shown_s = (b[8] - a[8]) / 100
    e_shown = shown_s * I_SHOWN_MA / 3600
    e_awake = max(0.0, awake_s - shown_s) * I_AWAKE_MA / 3600
    print()
    print(f"期間 {h:.1f} 時間  {a[2]} → {b[2]} mV  使った量 約 {used:.0f} mAh（電圧からの推定）")
    if h > 0:
        avg = used / h
        print(f"平均 {avg:.1f} mA → 満充電から約 {CAPACITY_MAH / avg:.0f} 時間" if avg > 0 else "平均 0 mA")
        print(f"内訳の推定: 点灯 {e_shown:.1f} mAh / 判定など {e_awake:.1f} mAh / 残り（寝ている間）"
              f"{used - e_shown - e_awake:.1f} mAh = 平均 {(used - e_shown - e_awake) / h:.2f} mA")
    return 0


if __name__ == "__main__":
    sys.exit(main())
