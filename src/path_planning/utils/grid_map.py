# grid_map.py
import numpy as np
import random

def generate_grid_map(size=50, num_blocks=12, block_size_range=(3, 5), seed=None):
    """
    生成结构化障碍地图（由矩形块组成）。

    参数:
        size: 地图尺寸
        num_blocks: 障碍块数量
        block_size_range: 每个障碍块的边长范围（最小值, 最大值）
        seed: 随机种子（可选）

    返回:
        grid: 2D numpy 地图数组，0 表示可通行，1 表示障碍
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    grid = np.zeros((size, size), dtype=int)

    for _ in range(num_blocks):
        block_w = random.randint(*block_size_range)
        block_h = random.randint(*block_size_range)
        x = random.randint(0, size - block_w - 1)
        y = random.randint(0, size - block_h - 1)
        grid[x:x + block_w, y:y + block_h] = 1

    # 确保起点终点通畅（左下和右上）
    grid[0][0] = 0
    grid[size - 1][size - 1] = 0
    return grid


def generate_scatter_map(size=50, obstacle_density=0.15, seed=None):
    """
    生成散点型障碍地图（随机分布障碍点）。

    参数:
        size: 地图尺寸
        obstacle_density: 障碍密度（0~1）
        seed: 随机种子（可选）

    返回:
        grid: 2D numpy 地图数组
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    grid = np.zeros((size, size), dtype=int)
    num_obstacles = int(size * size * obstacle_density)

    for _ in range(num_obstacles):
        x = random.randint(0, size - 1)
        y = random.randint(0, size - 1)
        grid[x][y] = 1

    # 保证起点终点可通
    grid[0][0] = 0
    grid[size - 1][size - 1] = 0
    return grid



def generate_block_map(size=50, num_rows=3, num_cols=3,
                       block_size=(4, 6), margin=2, seed=32):
    """
    生成结构化地图：规则矩形块状障碍（类似仓库布局）

    参数：
        size       : 地图尺寸（方形地图）
        num_rows   : 行方向障碍块数
        num_cols   : 列方向障碍块数
        block_size : 每个障碍块尺寸 (高度, 宽度)
        margin     : 障碍块与障碍块、边界的最小间隔
        seed       : 随机种子，用于调整对称性或微扰
    返回：
        grid : numpy 二维数组（0=可通行，1=障碍物）
    """
    np.random.seed(seed)
    grid = np.zeros((size, size), dtype=int)
    h, w = block_size

    # 计算步长（确保均匀分布 + 留通道）
    x_spacing = (size - 2 * margin - num_rows * h) // (num_rows + 1)
    y_spacing = (size - 2 * margin - num_cols * w) // (num_cols + 1)

    for i in range(num_rows):
        for j in range(num_cols):
            x0 = margin + (i + 1) * x_spacing + i * h
            y0 = margin + (j + 1) * y_spacing + j * w

            # 添加随机微扰（模拟非规则布局）
            dx = np.random.randint(-1, 2)
            dy = np.random.randint(-1, 2)

            x0 = np.clip(x0 + dx, 0, size - h)
            y0 = np.clip(y0 + dy, 0, size - w)

            grid[x0:x0 + h, y0:y0 + w] = 1

    return grid

import numpy as np

def generate_grid_map(size=30, obstacle_density=0.2, seed=None):
    """
    生成一个带有随机障碍的方格地图，起点与终点确保可通。

    参数：
        size (int): 地图边长
        obstacle_density (float): 障碍物密度（0~1）
        seed (int or None): 随机种子（确保实验可重复）
    
    返回：
        np.ndarray: 生成的地图数组（0为可通，1为障碍）
    """
    if seed is not None:
        np.random.seed(seed)

    # 初始化全空地图
    grid = np.zeros((size, size), dtype=int)

    # 起点终点及其邻域坐标（用于保留通路）
    reserved = set([
        (0, 0), (0, 1), (1, 0), (1, 1),
        (size-1, size-1), (size-2, size-1), (size-1, size-2), (size-2, size-2)
    ])

    # 可放置障碍的位置集合
    candidate_positions = [
        (i, j) for i in range(size) for j in range(size)
        if (i, j) not in reserved
    ]

    # 根据密度计算障碍数量
    num_obstacles = int(obstacle_density * size * size)
    num_obstacles = min(num_obstacles, len(candidate_positions))  # 防止超出范围

    # 随机选取障碍点
    obstacle_coords = np.random.choice(len(candidate_positions), num_obstacles, replace=False)
    for idx in obstacle_coords:
        x, y = candidate_positions[idx]
        grid[x, y] = 1

    return grid



