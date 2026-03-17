import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


import numpy as np

def schematic_map(size=25):

    grid = np.zeros((size, size), dtype=int)  # ✅ 初始化地图

    grid[1:size-1, 0] = 1      # 左边界
    grid[1:size-1, -1] = 1     # 右边界
    grid[0, :] = 1             # 上边界
    grid[-1, :] = 1            # 下边界

    # 添加固定结构障碍
    grid[0:3, 10:11] = 1
    grid[8:9, 0:10] = 1
    grid[6:11, 10:11] = 1
    grid[16:17, 0:10] = 1
    grid[14:19, 10:11] = 1
    grid[22:25, 10:11] = 1

    grid[6:8, 15:17] = 1 #块1
    grid[8:10, 19:21] = 1 #块2
    grid[13:15, 16:18] = 1 #块3
    grid[18:20, 19:21] = 1 #块4
    #方形
    grid[0:6, 18:19] = 1
    grid[5:6, 18:25] = 1
    #矩形
    grid[21:22, 14:21] = 1
    grid[24:25, 14:21] = 1
    grid[21:25, 14:15] = 1
    grid[21:25, 20:21] = 1
    #贴墙矩形
    grid[10:15, 22:23] = 1
    grid[10:11, 22:25] = 1  
    grid[14:15, 22:25] = 1
   

  

    return grid
