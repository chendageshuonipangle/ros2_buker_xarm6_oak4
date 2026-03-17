import numpy as np
from scipy.ndimage import binary_dilation

def expand_obstacles(grid, dilation_size=1):
    """
    对障碍物进行膨胀处理，防止路径穿越夹缝

    参数:
        grid (np.ndarray): 原始二值栅格地图,0-可通行,1-障碍
        dilation_size (int): 膨胀半径（单位：格子数）

    返回:
        expanded_grid (np.ndarray): 处理后的地图
    """
    # 构造膨胀卷积核（3x3 或 5x5 等）
    kernel_size = dilation_size * 2 + 1
    kernel = np.ones((kernel_size, kernel_size), dtype=bool)

    # 先转为bool类型（True=障碍）
    obstacle_mask = (grid == 1)

    # 膨胀处理
    expanded_mask = binary_dilation(obstacle_mask, structure=kernel)

    # 返回膨胀后的新地图
    expanded_grid = np.where(expanded_mask, 1, 0).astype(np.uint8)
    return expanded_grid
