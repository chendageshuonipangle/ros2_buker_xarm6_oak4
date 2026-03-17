import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import numpy as np
import pandas as pd
from utils.map_generator import generate_density_map

from algorithms.astar import a_star
from algorithms.jps import jps
from algorithms.jps_improved import jps_improved

from visualize.visualize import show_map  # 可视化函数

# ========== 实验参数配置 ==========
size = 25                  # 地图尺寸固定为 25x25
density_level = 1          # 固定为低密度（即障碍率为0.1）
num_trials = 10            # 每个算法重复运行次数

# 算法映射（统一接口）
algorithms = {
    "A*": a_star,
    "JPS": jps,
    "Improved JPS": jps_improved
}

results = []  # 汇总实验数据

# ========== 地图生成（低密度） ==========
print(f"\n=== 固定密度地图: {size}×{size}，障碍密度 0.1 ===")
base_grid = generate_density_map(size=size, density_level=density_level, seed=42)
base_grid[0][0] = 0
base_grid[size - 1][size - 1] = 0

# ========== 三算法对比实验 ==========
for algo_name, algo_func in algorithms.items():
    print(f"\n→ 算法: {algo_name}")
    time_list, length_list, expanded_list, jump_list = [], [], [], []
    img_saved = False

    for trial in range(num_trials):
        grid = base_grid.copy()
        start = (0, 0)
        goal = (size - 1, size - 1)

        try:
            t0 = time.time()
            if algo_name == "Improved JPS":
                result = algo_func(grid, start, goal, alpha0=0.3, window=5)
            else:
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
                    img_file = f"images/compare25x25/compare_{safe_name}_D0.1_25x25.png"
                    show_map(grid, path, start, goal,
                             title=f"{algo_name} Path (密度0.1)",
                             save_path=img_file)
                    img_saved = True

        except Exception as e:
            print(f"❌ 错误: {algo_name} 运行中出错: {e}")

    # 汇总平均实验结果
    results.append({
        "地图": f"{size}×{size}",
        "密度等级": "0.1",
        "算法": algo_name,
        "路径长度": round(np.mean(length_list), 2) if length_list else "无路径",
        "规划时间(ms)": round(np.mean(time_list), 2) if time_list else "无",
        "扩展节点": int(np.mean(expanded_list)) if expanded_list else "无",
        "跳点数": int(np.mean(jump_list)) if jump_list else "无"
    })

# ========== 输出与保存 ==========
df = pd.DataFrame(results)
os.makedirs("results", exist_ok=True)
df.to_csv("results/comparison_25x25_low_density.csv", index=False)
print("\n✅ 实验完成，结果保存为 results/comparison_25x25_low_density.csv")
print(df)
