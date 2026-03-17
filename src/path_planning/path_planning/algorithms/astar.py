import heapq
import numpy as np

# 八邻域方向
DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)]

# 启发函数：欧几里得距离（可选换成曼哈顿）
def heuristic(a, b):
    return np.hypot(a[0] - b[0], a[1] - b[1])

# 区分直线与斜线的实际代价
def movement_cost(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return 1.4142 if dx and dy else 1.0

# 斜向合法性检查（防止穿缝）
def is_walkable(grid, x, y, nx, ny):
    if not (0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]):
        return False
    if grid[nx][ny] == 1:
        return False
    dx = nx - x
    dy = ny - y
    if abs(dx) + abs(dy) == 2:
        # 合理限制：仅当两侧都是障碍才禁止斜穿
        if grid[x][ny] == 1 and grid[nx][y] == 1:
            return False
    return True

# 可选：路径简化函数（去除震荡）
def simplify_path(path):
    if not path or len(path) <= 2:
        return path
    simplified = [path[0]]
    dx1, dy1 = 0, 0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        if (dx, dy) != (dx1, dy1):
            simplified.append(path[i - 1])
            dx1, dy1 = dx, dy
    simplified.append(path[-1])
    return simplified

# A* 算法：集成 movement_cost 和防穿缝
def a_star(grid, start, goal):
    open_list = []
    heapq.heappush(open_list, (heuristic(start, goal), 0, start, None))  # f, g, pos, parent
    came_from = {}
    cost_so_far = {start: 0}
    expanded = 0

    while open_list:
        _, g, current, parent = heapq.heappop(open_list)
        expanded += 1

        if current == goal:
            # 回溯路径
            path = [current]
            while parent:
                path.append(parent)
                parent = came_from[parent]
            path = path[::-1]
            path = simplify_path(path)  # ✅ 可选：防止震荡
            return path, g, expanded

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
