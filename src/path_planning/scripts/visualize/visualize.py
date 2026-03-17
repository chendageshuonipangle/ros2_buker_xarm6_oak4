import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import matplotlib.pyplot as plt
import numpy as np

def show_map(grid, path=None, start=None, goal=None, title="", save_path=None):
    plt.figure(figsize=(5, 5))
    ax = plt.gca()
    ax.set_aspect('equal')

    # 显示地图，左下为原点
    plt.imshow(grid, cmap="Greys", origin="lower")

    # 路径显示
    if path:
        px, py = zip(*path)
        plt.plot(py, px, color="blue", linewidth=2, label="Path")

    # 起点终点显示
    if start:
        plt.plot(start[1], start[0], "go", markersize=8, label="Start")
    if goal:
        plt.plot(goal[1], goal[0], "ro", markersize=8, label="Goal")

    # 栅格网格线
    ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5, alpha=0.4)

    # 坐标轴隐藏（仅显示网格线）
    ax.set_xticks([])
    ax.set_yticks([])
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    # 设置标题
    plt.title(title, fontsize=13)

    # 图例设置
    # plt.legend(loc="upper right", fontsize=9)

    # 图像保存
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
    else:
        plt.show()

    plt.close()

import matplotlib.pyplot as plt
import numpy as np

def show_multiple_paths(grid, paths_dict, start, goal, title="", save_path=None):
    """
    显示多个算法在同一地图上的路径比较。

    参数:
        grid (ndarray): 二维地图
        paths_dict (dict): 算法名 → path坐标列表 的映射，如 {"A*": path1, "JPS": path2}
        start (tuple): 起点坐标
        goal (tuple): 终点坐标
        title (str): 标题
        save_path (str): 如有，则保存图像到该路径
    """
    plt.figure(figsize=(6, 6))
    ax = plt.gca()
    ax.set_aspect('equal')

    plt.imshow(grid, cmap="Greys", origin="lower")

    # 路径颜色映射
    color_map = {
        "A*": "blue",
        "JPS": "red",
        "Improved JPS": "green"
    }

    for name, path in paths_dict.items():
        if path:
            px, py = zip(*path)
            plt.plot(py, px, color=color_map.get(name, "black"), linewidth=2, label=name)

    # 起点终点
    if start:
        plt.plot(start[1], start[0], "go", markersize=8, label="Start")
    if goal:
        plt.plot(goal[1], goal[0], "ro", markersize=8, label="Goal")

    # 网格线
    ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5, alpha=0.3)

    # 关闭主坐标轴刻度
    ax.set_xticks([])
    ax.set_yticks([])
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    plt.title(title, fontsize=12)
    plt.legend(loc="upper right", fontsize=9)
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300)
        plt.close()
    else:
        plt.show()
