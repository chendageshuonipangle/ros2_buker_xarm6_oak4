import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import heapq
import numpy as np

# 八邻域方向
DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)]

def heuristic(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return min(dx, dy) * 1.4142 + abs(dx - dy) * 1.0  # 对角线估计

def in_bounds(pos, grid):
    x, y = pos
    return 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]

def passable(pos, grid):
    return in_bounds(pos, grid) and grid[pos[0], pos[1]] == 0

def compute_local_density(grid, x, y, window=5):
    half = window // 2
    count, total = 0, 0
    for dx in range(-half, half + 1):
        for dy in range(-half, half + 1):
            nx, ny = x + dx, y + dy
            if in_bounds((nx, ny), grid):
                total += 1
                if grid[nx, ny] == 1:
                    count += 1
    return count / total if total > 0 else 0

def adaptive_heuristic(n, goal, g_cost, grid, alpha0=0.3, window=5):
    x, y = n
    rho = compute_local_density(grid, x, y, window)
    alpha = alpha0 * (1 - rho)
    h = heuristic(n, goal)
    return g_cost + (1 + alpha) * h

def prune_neighbors(current, parent):
    if parent is None:
        return DIRS
    dx = current[0] - parent[0]
    dy = current[1] - parent[1]
    dx = 0 if dx == 0 else dx // abs(dx)
    dy = 0 if dy == 0 else dy // abs(dy)
    dir_primary = (dx, dy)

    neighbors = []
    for d in DIRS:
        if dx != 0 and dy != 0:
            if d in [(dx, 0), (0, dy), (dx, dy)]:
                neighbors.append(d)
        else:
            if d == dir_primary:
                neighbors.append(d)
            if dx == 0 and d in [(-1, dy), (1, dy)]:
                neighbors.append(d)
            elif dy == 0 and d in [(dx, -1), (dx, 1)]:
                neighbors.append(d)
    return neighbors

# 跳点函数（含缓存 + 斜向合法性检测）
def jump(grid, current, direction, goal, jump_cache, jump_count_holder):
    key = (current, direction, goal)
    if key in jump_cache:
        return jump_cache[key]

    x, y = current
    dx, dy = direction

    while True:
        x += dx
        y += dy

        if not in_bounds((x, y), grid) or not passable((x, y), grid):
            jump_cache[key] = None
            return None

        # 防止斜向穿越夹缝障碍
        if dx != 0 and dy != 0:
            if not passable((x - dx, y), grid) or not passable((x, y - dy), grid):
                jump_cache[key] = None
                return None

        jump_count_holder[0] += 1  # 计数合法跳点

        if (x, y) == goal:
            jump_cache[key] = (x, y)
            return (x, y)

        # 强迫邻居检测
        if dx != 0 and dy != 0:
            if (passable((x - dx, y + dy), grid) and not passable((x - dx, y), grid)) or \
               (passable((x + dx, y - dy), grid) and not passable((x, y - dy), grid)):
                jump_cache[key] = (x, y)
                return (x, y)
        else:
            if dx != 0:
                if (passable((x, y + 1), grid) and not passable((x - dx, y + 1), grid)) or \
                   (passable((x, y - 1), grid) and not passable((x - dx, y - 1), grid)):
                    jump_cache[key] = (x, y)
                    return (x, y)
            elif dy != 0:
                if (passable((x + 1, y), grid) and not passable((x + 1, y - dy), grid)) or \
                   (passable((x - 1, y), grid) and not passable((x - 1, y - dy), grid)):
                    jump_cache[key] = (x, y)
                    return (x, y)

        # 对角检测
        if dx != 0 and dy != 0:
            if jump(grid, (x, y), (dx, 0), goal, jump_cache, jump_count_holder) or \
               jump(grid, (x, y), (0, dy), goal, jump_cache, jump_count_holder):
                jump_cache[key] = (x, y)
                return (x, y)

# 主函数：改进 JPS with 密度启发 + 安全跳点检测
def jps_improved(grid, start, goal, alpha0=0.3, window=5):
    open_list = []
    heapq.heappush(open_list, (heuristic(start, goal), 0, start, None))
    came_from = {}
    cost_so_far = {start: 0}
    expanded = 0
    jump_count = [0]
    jump_cache = {}

    while open_list:
        _, g, current, parent = heapq.heappop(open_list)
        expanded += 1

        if current == goal:
            path = [current]
            while parent:
                path.append(parent)
                parent = came_from[parent]
            return path[::-1], g, expanded, jump_count[0]

        came_from[current] = parent
        directions = prune_neighbors(current, parent)

        for d in directions:
            next_jump = jump(grid, current, d, goal, jump_cache, jump_count)
            if next_jump:
                new_cost = g + heuristic(current, next_jump)
                if next_jump not in cost_so_far or new_cost < cost_so_far[next_jump]:
                    cost_so_far[next_jump] = new_cost
                    f_score = adaptive_heuristic(next_jump, goal, new_cost, grid, alpha0, window)
                    heapq.heappush(open_list, (f_score, new_cost, next_jump, current))

    return None, float('inf'), expanded, jump_count[0]
