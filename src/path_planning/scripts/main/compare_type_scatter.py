import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import numpy as np
import pandas as pd

from utils.grid_map import generate_grid_map, generate_scatter_map
# from grid_expand import expand_obstacles

# 导入各个算法模块
from algorithms.astar import a_star
from algorithms.jps import jps
from algorithms.jps_improved import jps_improved

from visualize.visualize import show_map  # 自定义图像可视化模块（保持不变）

# ========== 实验参数配置 ==========
sizes = [25, 50, 100]      # 地图尺寸
density = 0.15              # 障碍物密度
num_trials = 10            # 每组实验重复次数

# 算法映射（统一接口）
algorithms = {
    "A*": a_star,
    "JPS": jps,
    "Improved JPS": jps_improved
}

results = []              # 用于汇总结果
fixed_maps = {}           # 存储固定障碍地图，确保每个算法使用相同地图

# ========== 主实验循环 ==========
for size in sizes:
    print(f"\n=== 实验地图尺寸: {size}×{size} ===")

    # 生成统一地图，仅执行一次
    # from grid_map import generate_grid_map, expand_obstacles
    # base_grid = generate_grid_map(size=size, obstacle_density=density)
    # base_grid = expand_obstacles(base_grid, kernel_size=3)

    # 使用结构化障碍地图
    # base_grid = generate_grid_map(size=size, num_blocks=6)

    # 使用散点障碍地图（密度模式）
    base_grid = generate_scatter_map(size=size, obstacle_density=density)

    # 确保起点终点通畅
    base_grid[0][0] = 0
    base_grid[size-1][size-1] = 0

     #存储膨胀后的地图副本
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
                result = algo_func(grid, start, goal)
                t1 = time.time()
                dt = (t1 - t0) * 1000  # 转换为毫秒

                # 支持 (path, length, expanded) 或 (path, length, expanded, jump_count)
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

                    # 保存路径图像（首次）
                    if not img_saved and trial == 0:
                        safe_name = algo_name.replace("*", "A_star").replace(" ", "_")
                        img_file = f"images/type_scatter/pathmap_{safe_name}_{size}x{size}.png"
                        show_map(grid, path, start, goal,
                                 title=f"{algo_name} Path ({size}×{size})",
                                 save_path=img_file)
                        img_saved = True

            except Exception as e:
                print(f"❌ 错误: {algo_name} on {size}×{size}: {e}")

        # 添加结果
        
        results.append({
            "地图": f"{size}×{size}",
            "密度": density,
            "算法": algo_name,
            "路径长度": round(np.mean(length_list), 2) if length_list else "无路径",
            "规划时间(ms)": round(np.mean(time_list), 2) if time_list else "无",
            "扩展节点": int(np.mean(expanded_list)) if expanded_list else "无",
            "跳点数": int(np.mean(jump_list)) if jump_list else "无"
        })

# ========== 输出与保存结果 ==========
df = pd.DataFrame(results)
df.to_csv("comparison_results1.csv", index=False)
print("\n✅ 实验完成，结果保存为 comparison_results1.csv")
print(df)
