import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import numpy as np
import pandas as pd

from utils.structured_maps import (
    generate_structured_map_30x30,
    generate_structured_map_50x50,
    generate_structured_map_100x100
)

from algorithms.astar import a_star
from algorithms.jps import jps
from algorithms.jps_improved import jps_improved

from visualize.visualize import show_map  # 保持你的可视化模块

# ========== 实验参数配置 ==========
sizes = [30, 50, 100]        # 地图尺寸
num_trials = 10              # 每组实验重复次数

# 算法映射（统一接口）
algorithms = {
    "A*": a_star,
    "JPS": jps,
    "Improved JPS": jps_improved
}

results = []                # 汇总实验数据
fixed_maps = {}             # 存储结构化地图，确保各算法公平使用

# ========== 主实验循环 ==========
for size in sizes:
    print(f"\n=== 实验地图尺寸: {size}×{size} ===")

    # 加载结构化地图
    if size == 30:
        base_grid = generate_structured_map_30x30()
    elif size == 50:
        base_grid = generate_structured_map_50x50()
    elif size == 100:
        base_grid = generate_structured_map_100x100()
    else:
        raise ValueError("Unsupported size")

    base_grid[0][0] = 0
    base_grid[size - 1][size - 1] = 0
    fixed_maps[size] = base_grid.copy()

    for algo_name, algo_func in algorithms.items():
        print(f"  → 运行算法: {algo_name}")
        time_list, length_list, expanded_list, jump_list = [], [], [], []
        img_saved = False

        for trial in range(num_trials):
            grid = fixed_maps[size].copy()
            start = (0, 0)
            goal = (size - 1, size - 1)

            try:
                t0 = time.time()
                if algo_name == "Improved JPS":
                    result = algo_func(grid, start, goal, alpha0=0.3, window=5)
                else:
                    result = algo_func(grid, start, goal)
                t1 = time.time()
                dt = (t1 - t0) * 1000  # 毫秒

                if len(result) == 4:
                    path, length, expanded, jump_count = result
                else:
                    path, length, expanded = result
                    jump_count = None

                if path:
                    time_list.append(dt)
                    length_list.append(length)
                    expanded_list.append(expanded)
                    if jump_count is not None:
                        jump_list.append(jump_count)

                    # 仅保存一次图像
                    if not img_saved and trial == 0:
                        safe_name = algo_name.replace("*", "A_star").replace(" ", "_")
                        img_file = f"images/type_struct/structmap_{safe_name}_{size}x{size}.png"
                        show_map(grid, path, start, goal,
                                 title=f"{algo_name} Path ({size}×{size})",
                                 save_path=img_file)
                        img_saved = True

            except Exception as e:
                print(f"❌ 错误: {algo_name} on {size}×{size}: {e}")

        # 汇总数据
        results.append({
            "地图": f"{size}×{size}",
            "结构": "结构化地图",
            "算法": algo_name,
            "路径长度": round(np.mean(length_list), 2) if length_list else "无路径",
            "规划时间(ms)": round(np.mean(time_list), 2) if time_list else "无",
            "扩展节点": int(np.mean(expanded_list)) if expanded_list else "无",
            "跳点数": int(np.mean(jump_list)) if jump_list else "无"
        })

# ========== 保存结果 ==========
df = pd.DataFrame(results)
df.to_csv("comparison_results2.csv", index=False)
print("\n✅ 实验完成，结果保存为 comparison_results2.csv")
print(df)
