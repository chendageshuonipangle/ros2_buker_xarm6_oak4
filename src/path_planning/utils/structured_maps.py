import numpy as np

     # 设置障碍物位置（黑色区域）
    # grid[1:29, 0] = 1            # 左边界
    # grid[1:29, 29] = 1           # 右边界
    # grid[0, :] = 1               # 上边界
    # grid[29, :] = 1              # 下边界

import numpy as np

def generate_structured_map_30x30():
    grid = np.zeros((30, 30), dtype=int)
    grid[2:6, 2:5] = 1
    grid[3:6, 10:14] = 1
    grid[8:12, 6:10] = 1
    grid[13:18, 13:17] = 1
    grid[20:24, 5:9] = 1
    grid[20:25, 17:21] = 1
    grid[25:28, 10:14] = 1
    grid[7:11, 22:26] = 1
    grid[15:19, 24:28] = 1
    grid[10:14, 15:19] = 1
    grid[12:16, 20:24] = 1
    grid[5:8, 12:16] = 1
    return grid

def generate_structured_map_50x50():
    grid = np.zeros((50, 50), dtype=int)
    grid[6:12, 6:10] = 1
    grid[12:15, 17:28] = 1
    grid[17:32, 23:30] = 1
    grid[27:32, 5:10] = 1
    grid[37:42, 12:17] = 1
    grid[40:45, 30:38] = 1
    grid[8:12, 35:40] = 1
    grid[35:40, 35:40] = 1
    grid[20:30, 30:38] = 1
    grid[15:20, 20:25] = 1
    grid[10:15, 30:35] = 1
    grid[5:10, 20:25] = 1
    return grid

def generate_structured_map_100x100():
    grid = np.zeros((100, 100), dtype=int)
    grid[10:30, 10] = 1
    grid[10:30, 29] = 1
    grid[10, 10:30] = 1
    grid[29, 10:30] = 1
    grid[19, 10:30:2] = 1
    for i in range(10):
        grid[10 + i, 80 - i:90 - i] = 1
    for i in range(20, 40, 6):
        grid[i:i+2, 40:60] = 1
    grid[45:55, 42:44] = 1
    grid[45:55, 56:58] = 1
    grid[49:51, 42:58] = 1
    grid[52:54, 42:58] = 1
    for x in range(10, 30, 5):
        grid[60:80, x:x+2] = 1
    grid[60:70, 75] = 1
    grid[70, 65:76] = 1
    grid[85:95, 10:30:4] = 1
    grid[75:85, 45] = 1
    grid[75:85, 55] = 1
    grid[75, 45:56] = 1
    grid[85, 45:56] = 1
    grid[80:85, 80:90] = 1
    grid[85:90, 85:95] = 1
    grid[90:95, 90:100] = 1
    return grid


import numpy as np

# def generate_structured_map_100x100(obstacle_density=0.05, seed=None):
#     if seed is not None:
#         np.random.seed(seed)

#     grid = np.zeros((100, 100), dtype=int)

#     # === 固定结构障碍（模拟建筑结构） ===
#     grid[25:70, 12:17] = 1    # 垂直长方形障碍
#     grid[38:43, 3:50] = 1     # 横向障碍
#     grid[50:92, 45:50] = 1    # 垂直块
#     grid[60:68, 65:90] = 1    # 横向长块
#     grid[70:78, 20:35] = 1    # 横向小障碍

#     # === 随机障碍块（扰动） ===
#     num_random = int(100 * 100 * obstacle_density)

#     for _ in range(num_random):
#         x = np.random.randint(0, 98)  # 98 是为了能加 2×2
#         y = np.random.randint(0, 98)
#         h, w = np.random.choice([1, 2]), np.random.choice([1, 2])
#         grid[x:x + h, y:y + w] = 1

#     # 确保起点终点可通
#     grid[0][0] = 0
#     grid[99][99] = 0

#     return grid
