import sys
sys.path.append("C:/Users/Szran/Desktop/path_planning")  # ✅ 绝对路径，直接写死



import time
import numpy as np
import pandas as pd
from utils.map_schematic import schematic_map

from algorithms.astar_pro import a_star
from algorithms.jps_pro import jps
from algorithms.jps_improved import jps_improved

from scripts.visualize.visualize_schematic import show_paths  # 使用路径可视化

# ========== 参数设置 ==========
size = 25
num_trials = 10


algorithms = {
    "A*": a_star,
    "JPS": jps,
    "Improved JPS": lambda grid, start, goal: jps_improved(grid, start, goal, alpha0=0.3, window=5)
}

# 几何路径长度计算函数（欧几里得）
def compute_path_length(path):
    if not path or len(path) < 2:
        return 0.0
    return sum(np.hypot(path[i+1][0] - path[i][0],
                        path[i+1][1] - path[i][1]) for i in range(len(path)-1))

results = []

print(f"\n=== 单密度对比实验 | 地图: {size}×{size} ===")
grid = schematic_map(size=size)
start = (23, 23)
goal = (2, 2)
grid[goal] = 0

paths_for_plot = {}

for algo_name, algo_func in algorithms.items():
    print(f"→ 执行算法：{algo_name}")
    time_list, length_list, expanded_list, jump_list = [], [], [], []

    for trial in range(num_trials):
        try:
            t0 = time.time()
            result = algo_func(grid.copy(), start, goal)
            t1 = time.time()
            dt = (t1 - t0) * 1000

            if len(result) == 4:
                path, _, expanded, jump_count = result
            else:
                path, _, expanded = result
                jump_count = None

            # 使用统一函数计算真实路径长度
            length = compute_path_length(path)

            if path:
                time_list.append(dt)
                length_list.append(length)
                expanded_list.append(expanded)
                if jump_count is not None:
                    jump_list.append(jump_count)

                if trial == 0:
                    paths_for_plot[algo_name] = path

        except Exception as e:
            print(f"❌ 错误：{algo_name} - {e}")

    results.append({
        "地图": f"{size}×{size}",
        "算法": algo_name,
        "路径长度": round(np.mean(length_list), 2) if length_list else "无路径",
        "规划时间(ms)": round(np.mean(time_list), 2) if time_list else "无",
        "扩展节点": int(np.mean(expanded_list)) if expanded_list else "无",
    })

# 可视化所有路径
save_img = "images/Schematic/compare_paths_25x25.tiff"
show_paths(grid, paths_for_plot, start, goal,
           save_path=save_img)

# 保存结果
df = pd.DataFrame(results)
df.to_csv("data/results/compare_results_schematic.csv", index=False)
print("\n✅ 实验完成，结果保存为 compare_results_density01.csv")
print(df)
