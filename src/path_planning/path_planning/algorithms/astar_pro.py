import heapq
import numpy as np

DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)]

def heuristic(a, b):
    return np.hypot(a[0] - b[0], a[1] - b[1])
# def heuristic(a, b):
#     dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
#     return min(dx, dy) * 1.4142 + abs(dx - dy) * 1.0  # 对角线估计

def movement_cost(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return 1.4142 if dx and dy else 1.0

# 更严格防斜穿角落障碍
def is_walkable(grid, x, y, nx, ny):
    if not (0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]):
        return False
    if grid[nx][ny] == 1:
        return False
    dx = nx - x
    dy = ny - y
    if abs(dx) + abs(dy) == 2:
        if grid[x][ny] == 1 or grid[nx][y] == 1:
            return False
    return True

def a_star(grid, start, goal):
    open_list = []
    heapq.heappush(open_list, (heuristic(start, goal), 0, start, None))
    came_from = {}
    cost_so_far = {start: 0}
    expanded = 0

    while open_list:
        _, g, current, parent = heapq.heappop(open_list)
        expanded += 1

        if current == goal:
            path = [current]
            while parent:
                path.append(parent)
                parent = came_from[parent]
            path = path[::-1]
            return path, g, expanded  # ✅ 原始路径，主脚本统一重算长度

        came_from[current] = parent

        for dx, dy in DIRS:
            nx, ny = current[0] + dx, current[1] + dy
            if is_walkable(grid, current[0], current[1], nx, ny):
                next_pos = (nx, ny)
                new_cost = g + movement_cost(current, next_pos)
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    f = new_cost + heuristic(next_pos, goal)
                    heapq.heappush(open_list, (f, new_cost, next_pos, current))

    return None, float('inf'), expanded
