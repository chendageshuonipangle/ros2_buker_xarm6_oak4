import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import numpy as np
import pandas as pd

from utils.grid_map import generate_grid_map

from algorithms.astar import a_star
from algorithms.jps import jps
from algorithms.jps_improved import jps_improved

from visualize.visualize import show_map

# ========== 实验参数 ==========
densities = [0.1, 0.15, 0.2]       # 移除 0.4 密度
size = 30                         # 地图尺寸固定为 30×30
num_trials = 10                   # 每种密度重复次数

algorithms = {
    "A*": a_star,
    "JPS": jps,
    "Improved JPS": jps_improved
}

results = []

# ========== 主实验循环 ==========
for density in densities:
    print(f"\n=== 实验地图尺寸: 30×30，障碍密度: {density} ===")

    # 生成随机障碍地图（密度控制）
    base_grid = generate_grid_map(size=size, obstacle_density=density, seed=None)
    base_grid[0][0] = 0
    base_grid[size - 1][size - 1] = 0

    # 可选地图膨胀，防止穿缝（如启用则取消注释）
    # base_grid = expand_obstacles(base_grid, dilation_size=1)

    for algo_name, algo_func in algorithms.items():
        print(f"  → 运行算法: {algo_name}")
        time_list, length_list, expanded_list, jump_list = [], [], [], []
        img_saved = False

        for trial in range(num_trials):
            grid = base_grid.copy()
            start = (0, 0)
            goal = (size - 1, size - 1)

            try:
                t0 = time.time()
                result = algo_func(grid, start, goal)
                t1 = time.time()
                dt = (t1 - t0) * 1000  # ms

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

                    if not img_saved and trial == 0:
                        safe_name = algo_name.replace("*", "A_star").replace(" ", "_")
                        img_file = f"images/density_scatter/density_{int(density*100)}_{safe_name}_30x30.png"
                        show_map(grid, path, start, goal,
                                 title=f"{algo_name} Path (30×30, 密度={density})",
                                 save_path=img_file)
                        img_saved = True

            except Exception as e:
                print(f"❌ 错误: {algo_name} @ 密度 {density}: {e}")

        # 汇总结果
        results.append({
            "地图": "30×30",
            "密度": density,
            "算法": algo_name,
            "路径长度": round(np.mean(length_list), 2) if length_list else "无路径",
            "规划时间(ms)": round(np.mean(time_list), 2) if time_list else "无",
            "扩展节点": int(np.mean(expanded_list)) if expanded_list else "无",
            "跳点数": int(np.mean(jump_list)) if jump_list else "无"
        })

# ========== 输出汇总表 ==========
df = pd.DataFrame(results)
df.to_csv("data/results/comparison_results4.csv", index=False)
print("\n✅ 实验完成，结果保存为 comparison_results4.csv")
print(df)
