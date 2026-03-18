#!/usr/bin/env python3
import re
import sys
from datetime import datetime

TIME_RE = re.compile(
    r"\[TIME\]:\s*([0-9.]+)\s+seconds total\s*\(([0-9.]+)\s*hz,\s*([0-9.+-]+)\s*ms behind\)",
    re.IGNORECASE,
)
QPOS_RE = re.compile(
    r"q_GtoI\s*=\s*([0-9eE+\-.,\s]+)\|\s*p_IinG\s*=\s*([0-9eE+\-.,\s]+)\|\s*dist\s*=\s*([0-9eE+\-.]+)",
    re.IGNORECASE,
)
BIAS_RE = re.compile(
    r"bg\s*=\s*([0-9eE+\-.,\s]+)\|\s*ba\s*=\s*([0-9eE+\-.,\s]+)",
    re.IGNORECASE,
)


def fmt_vec(raw: str, n: int = 3) -> str:
    vals = []
    for x in raw.split(','):
        x = x.strip()
        if not x:
            continue
        try:
            vals.append(float(x))
        except ValueError:
            pass
    vals = vals[:n]
    if not vals:
        return "[]"
    return "[" + ", ".join(f"{v:.3f}" for v in vals) + "]"


def main() -> int:
    hz = None
    lag_ms = None
    p = None
    dist = None
    bg = None
    ba = None

    print("# OpenVINS Pretty Log")
    print("# time | hz | lag_ms | pos_m[x,y,z] | dist_m | bg | ba")
    sys.stdout.flush()

    for line in sys.stdin:
        line = line.rstrip("\n")

        m = TIME_RE.search(line)
        if m:
            hz = float(m.group(2))
            lag_ms = float(m.group(3))
            continue

        m = QPOS_RE.search(line)
        if m:
            p = fmt_vec(m.group(2), 3)
            try:
                dist = float(m.group(3))
            except ValueError:
                dist = None
            continue

        m = BIAS_RE.search(line)
        if m:
            bg = fmt_vec(m.group(1), 3)
            ba = fmt_vec(m.group(2), 3)
            ts = datetime.now().strftime("%H:%M:%S")
            hz_s = "-" if hz is None else f"{hz:6.1f}"
            lag_s = "-" if lag_ms is None else f"{lag_ms:6.2f}"
            p_s = p if p is not None else "-"
            d_s = "-" if dist is None else f"{dist:7.2f}"
            bg_s = bg if bg is not None else "-"
            ba_s = ba if ba is not None else "-"
            print(f"{ts} | {hz_s} | {lag_s} | {p_s:>28} | {d_s} | {bg_s} | {ba_s}")
            sys.stdout.flush()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
