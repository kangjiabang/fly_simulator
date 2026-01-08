import math
import numpy as np
from heapq import heappush, heappop
from typing import Tuple, List

# 类型别名
Point3D = Tuple[int, int, int]

# 1. 预计算邻居节点的偏移量与代价 (性能优化)
# 包含 26 个方向的移动（面、棱、角）
NEIGHBORS_OFFSETS = []
for dx in [-1, 0, 1]:
    for dy in [-1, 0, 1]:
        for dz in [-1, 0, 1]:
            if dx == 0 and dy == 0 and dz == 0: continue
            # 代价计算：欧几里得距离 + 垂直移动惩罚（abs(dz)*0.5）以鼓励平飞
            dist = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5 + abs(dz) * 0.5
            NEIGHBORS_OFFSETS.append((dx, dy, dz, dist))


class Grid3D:
    """
    使用位掩码（Bitmask）优化的 3D 网格。
    通过在 XY 平面的每个格点存储一个 Python 大整数，利用其二进制位表示 Z 轴的占用情况，
    极大减少了内存占用并提升了障碍物检测速度。
    """

    def __init__(self, min_x, max_x, min_y, max_y, size_z):
        self.min_x, self.max_x = min_x, max_x
        self.min_y, self.max_y = min_y, max_y
        self.size_z = size_z
        # occ_map 存储位掩码，dtype=object 以支持无限长度的 Python 整数
        self.occ_map = np.zeros((max_x - min_x + 1, max_y - min_y + 1), dtype=object)

    def in_bounds(self, x, y, z) -> bool:
        """检查坐标是否在网格边界内"""
        return self.min_x <= x <= self.max_x and \
            self.min_y <= y <= self.max_y and \
            0 <= z < self.size_z

    def is_free(self, x, y, z) -> bool:
        """判断该网格点是否无障碍 (检查位掩码的第 z 位是否为 0)"""
        if not self.in_bounds(x, y, z): return False
        return not (self.occ_map[x - self.min_x, y - self.min_y] & (1 << z))

    def add_obstacle_range(self, x, y, z_min, z_max):
        """高效地在指定 (x, y) 处将 z_min 到 z_max 的范围标记为障碍"""
        mask = ((1 << (z_max - z_min + 1)) - 1) << z_min
        self.occ_map[x - self.min_x, y - self.min_y] |= mask


def astar_3d_grid(grid: Grid3D, start: Point3D, goal: Point3D) -> List[Point3D]:
    """高性能 3D A* 搜索算法"""
    if not grid.is_free(*start) or not grid.is_free(*goal):
        print("警告: 起点或终点被障碍物占据")
        return []

    min_x, min_y = grid.min_x, grid.min_y
    # 动态垂直约束：限制搜索高度范围在起终点上下 20 米内，减少搜索空间
    z_limit_min = max(0, min(start[2], goal[2]) - 20)
    z_limit_max = min(grid.size_z - 1, max(start[2], goal[2]) + 20)

    # 存储每个节点的最小移动代价
    g_score = np.full((grid.max_x - min_x + 1, grid.max_y - min_y + 1, grid.size_z), np.inf, dtype=np.float32)
    g_score[start[0] - min_x, start[1] - min_y, start[2]] = 0.0

    open_set = [(0.0, start)]  # 优先队列 (f_score, node)
    came_from = {}
    closed = set()
    gx, gy, gz = goal

    while open_set:
        _, cur = heappop(open_set)
        if cur in closed: continue
        closed.add(cur)

        # 抵达终点，开始回溯路径
        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            return path[::-1]

        cx, cy, cz = cur
        cg = g_score[cx - min_x, cy - min_y, cz]

        for dx, dy, dz, cost in NEIGHBORS_OFFSETS:
            nx, ny, nz = cx + dx, cy + dy, cz + dz

            # 边界与高度限制检查
            if not (z_limit_min <= nz <= z_limit_max) or \
                    not (grid.min_x <= nx <= grid.max_x and grid.min_y <= ny <= grid.max_y):
                continue

            # 碰撞检测 (位运算)
            if not grid.is_free(nx, ny, nz):
                continue

            ng = cg + cost
            if ng < g_score[nx - min_x, ny - min_y, nz]:
                g_score[nx - min_x, ny - min_y, nz] = ng
                came_from[(nx, ny, nz)] = cur
                # 使用启发式函数 (Euclidean distance)，乘以 2.5 倍权重变为加权 A* 以提升搜索速度
                h = ((nx - gx) ** 2 + (ny - gy) ** 2 + (nz - gz) ** 2) ** 0.5
                heappush(open_set, (ng + h * 2.5, (nx, ny, nz)))
    return []


def smooth_path_3d(path: List[Point3D], grid: Grid3D) -> List[Point3D]:
    """路径平滑：移除不必要的转折点，如果两点间直线无障碍则直接连接"""
    if len(path) <= 2: return path

    def is_line_clear(p1, p2):
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))
        steps = int(dist * 2)  # 两倍采样率
        for i in range(1, steps):
            t = i / steps
            curr = tuple(int(round(p1[j] + (p2[j] - p1[j]) * t)) for j in range(3))
            if not grid.is_free(*curr): return False
        return True

    smoothed = [path[0]]
    curr_idx = 0
    while curr_idx < len(path) - 1:
        # 从最后一点向前搜索，寻找最远的可见点
        for next_idx in range(len(path) - 1, curr_idx, -1):
            if is_line_clear(path[curr_idx], path[next_idx]):
                smoothed.append(path[next_idx])
                curr_idx = next_idx
                break
        else:
            curr_idx += 1
            smoothed.append(path[curr_idx])
    return smoothed