import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, Point as ShapelyPoint
from heapq import heappush, heappop
from typing import Tuple, List, Set, Dict
import pyproj

Point3D = Tuple[int, int, int]
LonLatAlt = Tuple[float, float, float]

# --------------------------
# 之前的 Grid3D + A* 函数保持不变
# --------------------------
class Grid3D:
    def __init__(self, size_x, size_y, size_z):
        self.size_x = size_x
        self.size_y = size_y
        self.size_z = size_z
        self.obstacles: Set[Point3D] = set()

    def in_bounds(self, p: Point3D) -> bool:
        x, y, z = p
        return (
            0 <= x < self.size_x and
            0 <= y < self.size_y and
            0 <= z < self.size_z
        )

    def is_free(self, p: Point3D) -> bool:
        return p not in self.obstacles

def heuristic(a: Point3D, b: Point3D) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def neighbors(p: Point3D) -> List[Point3D]:
    x, y, z = p
    dirs = [-1, 0, 1]
    result = []
    for dx in dirs:
        for dy in dirs:
            for dz in dirs:
                if dx == dy == dz == 0:
                    continue
                result.append((x+dx, y+dy, z+dz))
    return result

def reconstruct_path(came_from: Dict[Point3D, Point3D], current: Point3D) -> List[Point3D]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def astar_3d(grid: Grid3D, start: Point3D, goal: Point3D) -> List[Point3D]:
    open_set = []
    heappush(open_set, (0, start))
    came_from: Dict[Point3D, Point3D] = {}
    g_score: Dict[Point3D, float] = {start: 0}

    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)

        for nxt in neighbors(current):
            if not grid.in_bounds(nxt):
                continue
            if not grid.is_free(nxt):
                continue
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative_g
                f = tentative_g + heuristic(nxt, goal)
                heappush(open_set, (f, nxt))
    return []

# --------------------------
# 经纬度 -> ENU 网格
# --------------------------
def create_proj(lat0: float, lon0: float):
    proj_enu = pyproj.Proj(proj='aeqd', lat_0=lat0, lon_0=lon0)
    return proj_enu

def lonlat_to_grid(lon: float, lat: float, alt: float, proj_enu, scale: float = 1.0) -> Point3D:
    x, y = proj_enu(lon, lat)
    return (int(x*scale), int(y*scale), int(alt))

def grid_to_lonlat(p: Point3D, proj_enu, scale: float = 1.0) -> LonLatAlt:
    x, y, z = p
    lon, lat = proj_enu(x/scale, y/scale, inverse=True)
    return (lon, lat, z)

def add_building_polygon(
    grid: Grid3D,
    building: Dict,
    proj_enu,
    scale: float = 1.0,
    safety_dist: float = 10.0
):
    # 经纬度 → ENU
    poly_coords = [proj_enu(lon, lat) for lon, lat in building["polygon"]]

    # 🔥 建筑水平膨胀 safety_dist 米
    poly = Polygon(poly_coords).buffer(safety_dist)

    minx, miny, maxx, maxy = poly.bounds

    for x in range(int(minx * scale), int(maxx * scale) + 1):
        for y in range(int(miny * scale), int(maxy * scale) + 1):
            if poly.contains(ShapelyPoint(x / scale, y / scale)):
                for z in range(
                    int(building["min_alt"]),
                    int(building["max_alt"])
                ):
                    grid.obstacles.add((x, y, z))


def is_line_clear(p1: Point3D, p2: Point3D, grid: Grid3D) -> bool:
    # 增加采样密度，并检查点周围的邻域
    dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))
    if dist == 0: return True

    # 采样步长改为固定的 0.5 个网格单位
    step_size = 0.5
    steps = int(dist / step_size)

    for i in range(steps + 1):
        t = i / steps
        curr_x = int(round(p1[0] + (p2[0] - p1[0]) * t))
        curr_y = int(round(p1[1] + (p2[1] - p1[1]) * t))
        curr_z = int(round(p1[2] + (p2[2] - p1[2]) * t))

        # 检查当前点及其微小邻域，防止“擦边”
        for ox in [0]:  # 如果想更安全，可以加上 [-1, 0, 1]
            if not grid.is_free((curr_x + ox, curr_y, curr_z)):
                return False
    return True

def smooth_path_3d(path_grid: List[Point3D], grid: Grid3D) -> List[Point3D]:
    """对原始网格路径进行平滑处理"""
    if len(path_grid) <= 2:
        return path_grid

    smoothed = [path_grid[0]]
    current_idx = 0

    while current_idx < len(path_grid) - 1:
        # 从最后一点开始向前尝试，寻找能直线到达的最远点
        for next_idx in range(len(path_grid) - 1, current_idx, -1):
            if is_line_clear(path_grid[current_idx], path_grid[next_idx], grid):
                smoothed.append(path_grid[next_idx])
                current_idx = next_idx
                break
    return smoothed
def plan_3d_path(
        start: LonLatAlt,
        goal: LonLatAlt,
        buildings: List[Dict],
        grid_size: Tuple[int, int, int] = (500, 500, 100),
        scale: float = 1.0
) -> List[LonLatAlt]:
    proj_enu = create_proj(start[1], start[0])
    grid = Grid3D(*grid_size)
    for b in buildings:
        add_building_polygon(grid, b, proj_enu, scale, safety_dist=5.0)

    start_grid = lonlat_to_grid(start[0], start[1], start[2], proj_enu, scale)
    goal_grid = lonlat_to_grid(goal[0], goal[1], goal[2], proj_enu, scale)

    # 1. 获取原始 A* 路径
    raw_path_grid = astar_3d(grid, start_grid, goal_grid)

    # 2. 对路径进行平滑处理 (拉直)
    smoothed_path_grid = smooth_path_3d(raw_path_grid, grid)

    # 3. 转换回经纬度
    path_lonlat = [grid_to_lonlat(p, proj_enu, scale) for p in smoothed_path_grid]
    return path_lonlat

# --------------------------
# 绘图函数
# --------------------------
def plot_path_and_buildings(path: List[LonLatAlt], buildings: List[Dict], start: LonLatAlt, goal: LonLatAlt):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制建筑物
    for b in buildings:
        poly = b["polygon"]
        min_alt = b["min_alt"]
        max_alt = b["max_alt"]

        xs = [p[0] for p in poly] + [poly[0][0]]  # 闭合
        ys = [p[1] for p in poly] + [poly[0][1]]

        # 顶面和底面
        ax.plot(xs, ys, [min_alt]*len(xs), color='brown')
        ax.plot(xs, ys, [max_alt]*len(xs), color='brown')

        # 竖面
        for i in range(len(poly)):
            x0, y0 = poly[i]
            x1, y1 = poly[(i+1)%len(poly)]
            ax.plot([x0,x1], [y0,y1], [min_alt,min_alt], color='brown')
            ax.plot([x0,x1], [y0,y1], [max_alt,max_alt], color='brown')
            ax.plot([x0,x0], [y0,y0], [min_alt,max_alt], color='brown')

    # 绘制路径
    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        zs = [p[2] for p in path]
        ax.plot(xs, ys, zs, color='blue', linewidth=2, label='Path')
        ax.scatter(start[0], start[1], start[2], color='green', s=50, label='Start')
        ax.scatter(goal[0], goal[1], goal[2], color='red', s=50, label='Goal')

    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_zlabel('Altitude')
    ax.legend()
    plt.show()

# --------------------------
# 主程序示例
# --------------------------
if __name__ == "__main__":
    start = (116.397428, 39.90923, 40)  # lon, lat, alt
    goal  = (116.398, 39.91, 40)

    buildings = [
        # 建筑 1（原有）
        {
            "polygon": [
                (116.3976, 39.9095),
                (116.3978, 39.9095),
                (116.3978, 39.9097),
                (116.3976, 39.9097)
            ],
            "min_alt": 0,
            "max_alt": 50
        },

        # 建筑 2（原有）
        {
            "polygon": [
                (116.3982, 39.9094),
                (116.3985, 39.9094),
                (116.3985, 39.9098),
                (116.3982, 39.9098)
            ],
            "min_alt": 0,
            "max_alt": 50
        },

        # # 建筑 3（新增：位于路径中部，较矮）
        # {
        #     "polygon": [
        #         (116.3979, 39.9097),
        #         (116.3981, 39.9097),
        #         (116.3981, 39.9099),
        #         (116.3979, 39.9099)
        #     ],
        #     "min_alt": 0,
        #     "max_alt": 30
        # },
        #
        # # 建筑 4（新增：高楼，测试抬高飞行）
        # {
        #     "polygon": [
        #         (116.3974, 39.9096),
        #         (116.39755, 39.9096),
        #         (116.39755, 39.90985),
        #         (116.3974, 39.90985)
        #     ],
        #     "min_alt": 0,
        #     "max_alt": 80
        # }
    ]

    path = plan_3d_path(start, goal, buildings, grid_size=(200,200,100), scale=2)

    print("Path points:", len(path))
    for p in path:
        print(p)

    # 可视化

    plot_path_and_buildings(path, buildings, start, goal)
