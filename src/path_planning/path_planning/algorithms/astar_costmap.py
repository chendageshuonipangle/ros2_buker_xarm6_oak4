"""
考虑代价地图的 A* 算法
路径会远离高代价区域（障碍物膨胀区域）
"""

import heapq
import numpy as np

# 八邻域方向
DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)]


def heuristic(a, b):
    """欧几里得距离启发函数"""
    return np.hypot(a[0] - b[0], a[1] - b[1])


def movement_cost(a, b):
    """移动代价：直线1.0，斜线1.414"""
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return 1.4142 if dx and dy else 1.0


def is_walkable(costmap, x, y, nx, ny, obstacle_threshold=253):
    """检查是否可通行"""
    if not (0 <= nx < costmap.shape[0] and 0 <= ny < costmap.shape[1]):
        return False
    if costmap[nx, ny] >= obstacle_threshold:
        return False
    # 斜向移动检查
    dx, dy = nx - x, ny - y
    if abs(dx) + abs(dy) == 2:
        if costmap[x, ny] >= obstacle_threshold and costmap[nx, y] >= obstacle_threshold:
            return False
    return True


def get_cost_penalty(costmap, x, y, cost_weight=0.1):
    """
    根据代价地图值计算额外代价
    代价越高，惩罚越大，路径会远离高代价区域
    """
    cost_value = costmap[x, y]
    if cost_value <= 0:
        return 0.0
    # 代价值 0-252 映射到额外代价
    return cost_weight * cost_value


def astar_costmap(costmap, start, goal, obstacle_threshold=253, cost_weight=0.05):
    """
    考虑代价地图的 A* 算法
    
    参数:
        costmap: 代价地图 (numpy array)，值 0-255
        start: 起点 (row, col)
        goal: 终点 (row, col)
        obstacle_threshold: 障碍物阈值，>= 此值为不可通行
        cost_weight: 代价权重，越大越远离高代价区域
    
    返回:
        path: 路径点列表
        total_cost: 总代价
        expanded: 扩展节点数
    """
    open_list = []
    heapq.heappush(open_list, (heuristic(start, goal), 0, start, None))
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
            return path[::-1], g, expanded

        if current in came_from:
            continue
        came_from[current] = parent

        for dx, dy in DIRS:
            nx, ny = current[0] + dx, current[1] + dy
            
            if is_walkable(costmap, current[0], current[1], nx, ny, obstacle_threshold):
                next_pos = (nx, ny)
                
                # 基础移动代价 + 代价地图惩罚
                base_cost = movement_cost(current, next_pos)
                cost_penalty = get_cost_penalty(costmap, nx, ny, cost_weight)
                new_cost = g + base_cost + cost_penalty
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    f = new_cost + heuristic(next_pos, goal)
                    heapq.heappush(open_list, (f, new_cost, next_pos, current))

    return None, float('inf'), expanded


def simplify_path(path, tolerance=0.5):
    """简化路径，去除共线点"""
    if not path or len(path) <= 2:
        return path
    
    simplified = [path[0]]
    
    for i in range(1, len(path) - 1):
        prev = simplified[-1]
        curr = path[i]
        next_pt = path[i + 1]
        
        # 检查是否共线
        dx1, dy1 = curr[0] - prev[0], curr[1] - prev[1]
        dx2, dy2 = next_pt[0] - curr[0], next_pt[1] - curr[1]
        
        # 如果方向改变，保留这个点
        if (dx1, dy1) != (dx2, dy2):
            simplified.append(curr)
    
    simplified.append(path[-1])
    return simplified
