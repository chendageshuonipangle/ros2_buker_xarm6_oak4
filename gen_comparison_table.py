#!/usr/bin/env python3
"""
生成 Origin vs Improved AMCL 轨迹点偏差对比表格。
用法:
    python3 gen_comparison_table.py                          # 自动取最新 origin/improved 会话
    python3 gen_comparison_table.py origin_dir improved_dir  # 手动指定目录
"""
import sys
import os
import csv
import math
from pathlib import Path

NAV_DATA = Path.home() / "ros2_ws" / "nav_data"

def latest_session(label: str) -> Path:
    candidates = sorted(
        [d for d in NAV_DATA.iterdir() if d.is_dir() and label in d.name],
        key=lambda d: d.name
    )
    if not candidates:
        raise FileNotFoundError(f"找不到包含 '{label}' 的会话目录，请检查 ~/ros2_ws/nav_data/")
    return candidates[-1]

def load_waypoint_errors(session_dir: Path) -> dict[int, list[float]]:
    """返回 {wp_idx: [dist_error_m, ...]}"""
    fp = session_dir / "waypoint_errors.csv"
    if not fp.exists():
        return {}
    errors: dict[int, list[float]] = {}
    with open(fp) as f:
        reader = csv.DictReader(f)
        for row in reader:
            idx = int(row["wp_idx"])
            dist = float(row["dist_error_m"])
            errors.setdefault(idx, []).append(dist)
    return errors

def mean(vals: list[float]) -> float:
    return sum(vals) / len(vals) if vals else float("nan")

def main():
    if len(sys.argv) == 3:
        origin_dir = Path(sys.argv[1])
        improved_dir = Path(sys.argv[2])
    else:
        origin_dir = latest_session("origin")
        improved_dir = latest_session("improved")

    print(f"\n📂 Origin   会话: {origin_dir.name}")
    print(f"📂 Improved 会话: {improved_dir.name}\n")

    # 读取路点理论坐标（从 dadian.txt）
    dadian = Path.home() / "ros2_ws" / "maps" / "dadian.txt"
    waypoints: list[tuple[float, float]] = []
    if dadian.exists():
        with open(dadian) as f:
            lines = f.read().splitlines()
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            if line.startswith("x:"):
                x = float(line.split(":")[1].strip())
                y_line = lines[i + 1].strip() if i + 1 < len(lines) else ""
                if y_line.startswith("y:"):
                    y = float(y_line.split(":")[1].strip())
                    waypoints.append((x, y))
            i += 1
    n_wp = len(waypoints)

    origin_errs = load_waypoint_errors(origin_dir)
    improved_errs = load_waypoint_errors(improved_dir)

    # 确定共同路点范围
    all_idx = set(origin_errs.keys()) | set(improved_errs.keys())
    if not all_idx:
        print("⚠️  两组数据均无路点触发记录，请确认实验是否成功运行。")
        return

    max_idx = max(all_idx)

    # 表头
    col_widths = [8, 22, 22, 22, 14]
    headers = ["Waypoint\nIndex", "Theoretical\ncoordinate (x,y)/m",
               "Deviation\n(Traditional)/m", "Deviation\n(Improved)/m", "Improvement/%"]

    sep = "+" + "+".join("-" * w for w in col_widths) + "+"
    def fmt_row(cells):
        parts = []
        for cell, w in zip(cells, col_widths):
            parts.append(str(cell).center(w))
        return "|" + "|".join(parts) + "|"

    print("表 2  轨迹点偏差数据 / Trajectory Point Deviation Data")
    print(sep)
    print(fmt_row(["Waypoint", "Theoretical", "Deviation", "Deviation", "Improvement"]))
    print(fmt_row(["Index", "coordinate (x,y)/m", "(Traditional)/m", "(Improved)/m", "/%"]))
    print(sep)

    trad_vals, impr_vals, improv_pcts = [], [], []

    for idx in range(1, max_idx + 1):
        coord_str = f"({waypoints[idx-1][0]:.2f},{waypoints[idx-1][1]:.2f})" if idx <= n_wp else "N/A"
        trad = mean(origin_errs.get(idx, []))
        impr = mean(improved_errs.get(idx, []))

        if math.isnan(trad):
            trad_str = "N/A"
        else:
            trad_str = f"{trad:.3f}"
            trad_vals.append(trad)

        if math.isnan(impr):
            impr_str = "N/A"
        else:
            impr_str = f"{impr:.3f}"
            impr_vals.append(impr)

        if not math.isnan(trad) and not math.isnan(impr) and trad > 0:
            pct = (trad - impr) / trad * 100
            pct_str = f"{pct:.2f}"
            improv_pcts.append(pct)
        else:
            pct_str = "N/A"

        print(fmt_row([str(idx), coord_str, trad_str, impr_str, pct_str]))

    print(sep)

    # 均值行
    avg_trad = f"{mean(trad_vals):.3f}" if trad_vals else "N/A"
    avg_impr = f"{mean(impr_vals):.3f}" if impr_vals else "N/A"
    avg_pct  = f"{mean(improv_pcts):.2f}" if improv_pcts else "N/A"
    print(fmt_row(["Average", "-", avg_trad, avg_impr, avg_pct]))
    print(sep)

    print(f"\n✅ Origin   路点触发: {sorted(origin_errs.keys())}")
    print(f"✅ Improved 路点触发: {sorted(improved_errs.keys())}")

    # 同时保存 Markdown 表格
    out_md = NAV_DATA / "comparison_table_final.md"
    with open(out_md, "w") as f:
        f.write("# 表2 轨迹点偏差数据 / Trajectory Point Deviation Data\n\n")
        f.write(f"- Origin   会话: `{origin_dir.name}`\n")
        f.write(f"- Improved 会话: `{improved_dir.name}`\n\n")
        f.write("| Waypoint Index | Theoretical coordinate (x,y)/m | Deviation (Traditional)/m | Deviation (Improved)/m | Improvement/% |\n")
        f.write("|:-:|:-:|:-:|:-:|:-:|\n")
        for idx in range(1, max_idx + 1):
            coord_str = f"({waypoints[idx-1][0]:.2f},{waypoints[idx-1][1]:.2f})" if idx <= n_wp else "N/A"
            trad = mean(origin_errs.get(idx, []))
            impr = mean(improved_errs.get(idx, []))
            trad_s = f"{trad:.3f}" if not math.isnan(trad) else "N/A"
            impr_s = f"{impr:.3f}" if not math.isnan(impr) else "N/A"
            if not math.isnan(trad) and not math.isnan(impr) and trad > 0:
                pct_s = f"{(trad - impr) / trad * 100:.2f}"
            else:
                pct_s = "N/A"
            f.write(f"| {idx} | {coord_str} | {trad_s} | {impr_s} | {pct_s} |\n")
        f.write(f"| **Average** | - | **{avg_trad}** | **{avg_impr}** | **{avg_pct}** |\n")
    print(f"\n📄 Markdown 表格已保存至: {out_md}")

if __name__ == "__main__":
    main()
