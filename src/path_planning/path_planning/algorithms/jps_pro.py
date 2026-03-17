import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import heapq
import numpy as np

from .astar import is_walkable

# 方向向量（8邻域）
DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)]

# 欧几里得启发函数
# def heuristic(a, b):
#     return np.hypot(a[0] - b[0], a[1] - b[1])

def heuristic(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (1.4142 - 2) * min(dx, dy)


# 判断合法坐标
def in_bounds(pos, grid):
    x, y = pos
    return 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]

# 是否为可通行点
def passable(pos, grid):
    return in_bounds(pos, grid) and grid[pos[0]][pos[1]] == 0

# 检查强迫邻居
def has_forced_neighbor(grid, pos, dir):
    x, y = pos
    dx, dy = dir

    if dx != 0 and dy != 0:  # 对角线方向
        if (passable((x - dx, y + dy), grid) and not passable((x - dx, y), grid)) or \
           (passable((x + dx, y - dy), grid) and not passable((x, y - dy), grid)):
            return True
    elif dx != 0:
        if (passable((x + dx, y + 1), grid) and not passable((x, y + 1), grid)) or \
           (passable((x + dx, y - 1), grid) and not passable((x, y - 1), grid)):
            return True
    elif dy != 0:
        if (passable((x + 1, y + dy), grid) and not passable((x + 1, y), grid)) or \
           (passable((x - 1, y + dy), grid) and not passable((x - 1, y), grid)):
            return True
    return False

def is_walkable(grid, x, y, nx, ny):
    if not (0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]):
        return False
    if grid[nx][ny] == 1:
        return False
    dx = nx - x
    dy = ny - y
    # 仅禁止真正穿缝（对角线移动时两边都是障碍）
    if abs(dx) + abs(dy) == 2:
        if grid[x][ny] == 1 and grid[nx][y] == 1:
            return False
    return True



# 跳点函数（带缓存）
def jump(grid, current, direction, goal, cache, jump_count):
    key = (current, direction)
    if key in cache:
        return cache[key]

    x, y = current
    dx, dy = direction

    while True:
        prev = (x, y)  # 记录前一个点
        x += dx
        y += dy

        if not is_walkable(grid, prev[0], prev[1], x, y):  # <-- 改成穿缝检测
            cache[key] = None
            return None
        if (x, y) == goal:
            cache[key] = (x, y)
            jump_count[0] += 1
            return (x, y)
        if has_forced_neighbor(grid, (x, y), direction):
            cache[key] = (x, y)
            jump_count[0] += 1
            return (x, y)
        if dx != 0 and dy != 0:
            if jump(grid, (x, y), (dx, 0), goal, cache, jump_count) or \
               jump(grid, (x, y), (0, dy), goal, cache, jump_count):
                cache[key] = (x, y)
                jump_count[0] += 1
                return (x, y)

            


# 剪枝方向（方向推理）
def prune_dirs(parent, current):
    if parent is None:
        return DIRS
    px, py = parent
    cx, cy = current
    dx, dy = cx - px, cy - py
    dx = 0 if dx == 0 else dx // abs(dx)
    dy = 0 if dy == 0 else dy // abs(dy)
    dirs = []

    if dx != 0 and dy != 0:  # 对角线
        dirs = [(dx, dy), (dx, 0), (0, dy)]
    elif dx != 0:
        dirs = [(dx, 0), (dx, 1), (dx, -1)]
    elif dy != 0:
        dirs = [(0, dy), (1, dy), (-1, dy)]
    return dirs

# JPS主函数（统一接口）
def jps(grid, start, goal):
    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start, None))
    came_from = {}
    cost_so_far = {start: 0}
    expanded = 0
    cache = {}
    jump_count = [0]

    while open_list:
        _, g, current, parent = heapq.heappop(open_list)
        if current == goal:
            path = [current]
            while parent:
                path.append(parent)
                parent = came_from[parent]
            return path[::-1], g, expanded, jump_count[0]

        came_from[current] = parent
        expanded += 1

        for dir in prune_dirs(parent, current):
            jp = jump(grid, current, dir, goal, cache, jump_count)
            if jp:
                new_cost = g + heuristic(current, jp)
                if jp not in cost_so_far or new_cost < cost_so_far[jp]:
                    cost_so_far[jp] = new_cost
                    priority = new_cost + heuristic(jp, goal)
                    heapq.heappush(open_list, (priority, new_cost, jp, current))

    return None, float('inf'), expanded, jump_count[0]
