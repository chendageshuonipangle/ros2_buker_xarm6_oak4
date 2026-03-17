import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import matplotlib.pyplot as plt
import numpy as np

def visualize_dilation(original_grid, dilated_grid, save_path=None, show=True):
    """
    可视化膨胀障碍层：
    - 原始障碍（黑色）
    - 新增膨胀障碍（灰色）
    - 可通行区域（白色）

    参数:
        original_grid (ndarray): 原始障碍地图 (0=空地, 1=障碍)
        dilated_grid (ndarray): 膨胀后的地图
        save_path (str): 可选，保存图像路径
        show (bool): 是否立即显示图像
    """
    assert original_grid.shape == dilated_grid.shape, "地图尺寸不一致"

    h, w = original_grid.shape
    color_map = np.ones((h, w, 3))  # 初始化为白色

    # 原始障碍为黑色
    color_map[original_grid == 1] = [0, 0, 0]

    # 膨胀新增障碍为灰色
    expanded_only = (dilated_grid == 1) & (original_grid == 0)
    color_map[expanded_only] = [0.6, 0.6, 0.6]

    plt.figure(figsize=(6, 6))
    plt.imshow(color_map, origin='lower')
    plt.title("Obstacle Dilation Visualization")
    plt.axis('off')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300)
    if show:
        plt.show()
    plt.close()
