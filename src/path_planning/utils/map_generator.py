import numpy as np
import random

def generate_density_map(size=50, density_level=1, seed=55, block_size_range=(2, 8), margin=2):
    """
    生成由随机位置 + 随机大小的矩形障碍块构成的地图。
    
    参数：
    - size: 地图大小
    - density_level: 1=低密度，2=中密度，3=高密度
    - seed: 随机种子
    - block_size_range: 障碍块的高度宽度范围，如 (2, 8)
    - margin: 起点/终点保护边界
    """
    random.seed(seed)
    np.random.seed(seed)

    grid = np.zeros((size, size), dtype=int)

    # 不同密度下的障碍块数量
    density_map = {
        1: 10,
        2: 15,
        3: 25
    }
    num_blocks = density_map.get(density_level, 6)

    blocks_added = 0
    max_attempts = 1000
    attempts = 0

    while blocks_added < num_blocks and attempts < max_attempts:
        h = random.randint(*block_size_range)
        w = random.randint(*block_size_range)
        x = random.randint(0, size - h - 1)
        y = random.randint(0, size - w - 1)

        # 检查该区域是否已被使用（避免重叠），并排除起点/终点区域
        region = grid[x:x + h, y:y + w]
        if np.any(region):  # 已被占用
            attempts += 1
            continue
        if ((x <= margin and y <= margin) or
            (x + h >= size - margin and y + w >= size - margin)):
            attempts += 1
            continue

        grid[x:x + h, y:y + w] = 1
        blocks_added += 1
        attempts += 1

    # 确保起点终点通畅
    grid[0, 0] = 0
    grid[size - 1, size - 1] = 0

    return grid
