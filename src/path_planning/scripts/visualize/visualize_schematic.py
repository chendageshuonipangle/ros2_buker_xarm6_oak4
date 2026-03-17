import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.family'] = 'Times New Roman'  # 使用论文字体


def show_paths(grid, paths, start, goal, save_path=None, title=None, show_legend=True):
    """
    可视化一个或多个路径，支持图例显示
    
    参数：
        grid (ndarray): 地图矩阵（0 表示可通行，1 表示障碍）
        paths (list 或 dict): 单一路径或多路径字典
        start (tuple): 起点坐标
        goal (tuple): 终点坐标
        save_path (str): 保存图片路径（若为空则显示）
        title (str): 图片标题
        show_legend (bool): 是否显示图例
    """
    plt.figure(figsize=(6, 6))
    ax = plt.gca()
    ax.set_aspect('equal')
    plt.imshow(grid, cmap="Greys", origin="lower", vmin=0, vmax=1.5, interpolation='none')


    # 判断是否是多路径
    is_multi = isinstance(paths, dict)

    color_map = {
        "A*": "blue",
        "JPS": "red",
        "Improved JPS": "green"
    }

    if is_multi:
        for name, path in paths.items():
            if path:
                px, py = zip(*path)
                plt.plot(py, px, color=color_map.get(name, "black"), linewidth=2, label=name)
    else:
        if paths:
            px, py = zip(*paths)
            plt.plot(py, px, color="blue", linewidth=2, label="Path")

    # 起点终点
    if start:
        plt.plot(start[1], start[0], "go", markersize=8, label="Start")
    if goal:
        plt.plot(goal[1], goal[0], "ro", markersize=8, label="Goal")

    # 添加图例
    if show_legend:
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))  # 避免重复图例
        plt.legend(unique.values(), unique.keys(),  bbox_to_anchor=(0.4, 0.95),fontsize=9, framealpha=0)


    # 设置网格线
    ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5, alpha=0.3)

    # 关闭主坐标轴刻度
    ax.set_xticks([])
    ax.set_yticks([])
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    # plt.axis('off')  # 关闭坐标轴所有元素

    # 去除坐标轴边框（spines）
    for spine in ax.spines.values():
        spine.set_visible(False)

    if title:
        plt.title(title, fontsize=12)

    if save_path:
        plt.savefig(save_path, dpi=600, bbox_inches="tight", pad_inches=0.0)
        plt.close()
    else:
        plt.show()
