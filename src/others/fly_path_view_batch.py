import math
import random

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, Point as ShapelyPoint
from shapely.strtree import STRtree
from heapq import heappush, heappop
from typing import Tuple, List, Dict
import pyproj
import numpy as np

Point3D = Tuple[int, int, int]
LonLatAlt = Tuple[float, float, float]

# --------------------------
# 优化后的 Grid3D
# --------------------------
class Grid3D:
    def __init__(self, size_x, size_y, size_z):
        self.size_x = size_x
        self.size_y = size_y
        self.size_z = size_z
        # 使用字典存储 Z 区间，key=(x,y)，value=[(min_z,max_z), ...]
        self.obstacles: Dict[Tuple[int,int], List[Tuple[int,int]]] = {}

    def in_bounds(self, p: Point3D) -> bool:
        x, y, z = p
        return 0 <= x < self.size_x and 0 <= y < self.size_y and 0 <= z < self.size_z

    def is_free(self, p: Point3D) -> bool:
        x, y, z = p
        if (x,y) not in self.obstacles:
            return True
        for min_z, max_z in self.obstacles[(x,y)]:
            if min_z <= z <= max_z:
                return False
        return True

    def add_obstacle_voxel(self, x:int, y:int, min_z:int, max_z:int):
        if (x,y) not in self.obstacles:
            self.obstacles[(x,y)] = [(min_z,max_z)]
        else:
            self.obstacles[(x,y)].append((min_z,max_z))

# --------------------------
# 优化后的建筑体添加函数
# --------------------------
def add_building_to_grid(grid: Grid3D, building, proj_enu, scale, safety_dist=3.0):
    # 构造带安全距离的多边形
    poly_coords = [proj_enu(lon, lat) for lon, lat in building["polygon"]]
    safe_poly = Polygon(poly_coords).buffer(safety_dist)
    minx, miny, maxx, maxy = safe_poly.bounds

    # 批量生成 XY 网格
    x_range = np.arange(int(minx*scale), int(maxx*scale)+1)
    y_range = np.arange(int(miny*scale), int(maxy*scale)+1)
    X, Y = np.meshgrid(x_range, y_range)
    points = np.vstack([X.ravel()/scale, Y.ravel()/scale]).T

    # 使用向量化 + STRtree 判断哪些点在建筑内
    mask = np.array([safe_poly.contains(ShapelyPoint(pt)) for pt in points])
    valid_points = points[mask]

    # Z 区间
    min_z = max(0, int(building["min_alt"] - 2))
    max_z = int(building["max_alt"] + 5)

    # 批量添加到 Grid3D
    for x, y in valid_points:
        grid.add_obstacle_voxel(int(round(x*scale)), int(round(y*scale)), min_z, max_z)

# --------------------------
# 其他核心函数保持不变
# --------------------------
def get_neighbors_with_cost(p: Point3D) -> List[Tuple[Point3D, float]]:
    x, y, z = p
    neighbors_list = []
    for dx in [-1,0,1]:
        for dy in [-1,0,1]:
            for dz in [-1,0,1]:
                if dx==0 and dy==0 and dz==0: continue
                cost = math.sqrt(dx**2 + dy**2 + dz**2) + abs(dz)*0.5
                neighbors_list.append(((x+dx, y+dy, z+dz), cost))
    return neighbors_list

def heuristic(a: Point3D, b: Point3D) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def is_line_clear_grid(p1: Point3D, p2: Point3D, grid: Grid3D) -> bool:
    dist = heuristic(p1, p2)
    if dist < 1.0: return True
    steps = int(dist*2)
    for i in range(1, steps):
        t = i/steps
        curr = (int(round(p1[0]+(p2[0]-p1[0])*t)),
                int(round(p1[1]+(p2[1]-p1[1])*t)),
                int(round(p1[2]+(p2[2]-p1[2])*t)))
        if not grid.in_bounds(curr) or not grid.is_free(curr):
            return False
    return True

def smooth_path_3d(path: List[Point3D], grid: Grid3D) -> List[Point3D]:
    if len(path)<=2: return path
    smoothed = [path[0]]
    curr_idx = 0
    while curr_idx < len(path)-1:
        for next_idx in range(len(path)-1, curr_idx, -1):
            if is_line_clear_grid(path[curr_idx], path[next_idx], grid):
                smoothed.append(path[next_idx])
                curr_idx = next_idx
                break
        else:
            curr_idx += 1
            smoothed.append(path[curr_idx])
    return smoothed


# 1. 预计算偏移量
NEIGHBORS_OFFSETS = []
for dx in [-1, 0, 1]:
    for dy in [-1, 0, 1]:
        for dz in [-1, 0, 1]:
            if dx == 0 and dy == 0 and dz == 0: continue
            dist = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5 + abs(dz) * 0.5
            NEIGHBORS_OFFSETS.append((dx, dy, dz, dist))


def astar_3d_grid(grid: Grid3D, start: Point3D, goal: Point3D) -> List[Point3D]:
    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    # 提前取出常用函数引用，减少全局查找
    in_bounds = grid.in_bounds
    is_free = grid.is_free

    while open_set:
        _, current = heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        curr_x, curr_y, curr_z = current
        curr_g = g_score[current]

        for dx, dy, dz, cost in NEIGHBORS_OFFSETS:
            nxt = (curr_x + dx, curr_y + dy, curr_z + dz)

            if not in_bounds(nxt) or not is_free(nxt):
                continue

            tentative_g = curr_g + cost
            if tentative_g < g_score.get(nxt, float('inf')):
                came_from[nxt] = current
                g_score[nxt] = tentative_g
                # 使用更加激进的启发式因子 (w=1.5) 可以极大加快搜索速度，但路径可能不是最短
                h = ((nxt[0] - goal[0]) ** 2 + (nxt[1] - goal[1]) ** 2 + (nxt[2] - goal[2]) ** 2) ** 0.5
                heappush(open_set, (tentative_g + h, nxt))
    return []
# --------------------------
# 路径规划函数
# --------------------------
def plan_3d_grid_path(start_ll, goal_ll, buildings, scale=1.0, safety_dist=10.0):
    proj_enu = pyproj.Proj(proj='aeqd', lat_0=start_ll[1], lon_0=start_ll[0])
    grid = Grid3D(500, 500, 200)
    for b in buildings:
        add_building_to_grid(grid, b, proj_enu, scale, safety_dist)

    def to_grid(ll):
        x, y = proj_enu(ll[0], ll[1])
        return (int(round(x*scale)), int(round(y*scale)), int(round(ll[2])))

    s_grid, g_grid = to_grid(start_ll), to_grid(goal_ll)
    raw_path = astar_3d_grid(grid, s_grid, g_grid)
    smooth_path = smooth_path_3d(raw_path, grid)
    return smooth_path, proj_enu

# --------------------------
# 绘图函数保持不变
# --------------------------
def plot_3d(path_grid, buildings, start_ll, goal_ll, proj_enu, scale):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs, ys, zs = [], [], []
    for p in path_grid:
        xs.append(p[0]/scale)
        ys.append(p[1]/scale)
        zs.append(p[2])
    ax.plot(xs, ys, zs, color='blue', linewidth=3)

    for b in buildings:
        poly = [proj_enu(lon, lat) for lon, lat in b["polygon"]]
        poly.append(poly[0])
        px, py = zip(*poly)
        ax.plot(px, py, [b["max_alt"]]*len(px), color='brown')
        ax.plot(px, py, [b["min_alt"]]*len(px), color='brown')
        for x,y in poly[:-1]:
            ax.plot([x,x],[y,y],[b["min_alt"],b["max_alt"]], color='brown', alpha=0.5)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Altitude (m)")
    ax.view_init(elev=90, azim=-90)
    plt.show()

def grid_path_to_lonlat(grid_path, proj_enu, scale):
    lonlat_path=[]
    for xg, yg, zg in grid_path:
        x = xg/scale
        y = yg/scale
        lon, lat = proj_enu(x, y, inverse=True)
        lonlat_path.append((lon, lat, zg))
    return lonlat_path

# --------------------------
# 测试
# --------------------------
# --------------------------
# 优化后的测试部分
# --------------------------
if __name__ == "__main__":
    # 定义更宽的起终点
    start_point = (116.3950, 39.9070, 20)  # 起点调远
    goal_point = (116.4000, 39.9120, 25)  # 终点调远

    center_lon, center_lat = 116.3975, 39.9095
    building_data = []
    num_buildings = 50  # 生成30个不重叠的建筑

    # 扩大分布范围至约 ±500米 (0.005度)
    search_range = 0.005

    attempts = 0
    while len(building_data) < num_buildings and attempts < 200:
        attempts += 1
        dx = random.uniform(-search_range, search_range)
        dy = random.uniform(-search_range, search_range)

        # 建筑物大小 (约 30-60米)
        width = random.uniform(0.0003, 0.0006)
        height = random.uniform(0.0003, 0.0006)

        new_poly_coords = [
            (center_lon + dx, center_lat + dy),
            (center_lon + dx + width, center_lat + dy),
            (center_lon + dx + width, center_lat + dy + height),
            (center_lon + dx, center_lat + dy + height)
        ]
        new_poly = Polygon(new_poly_coords)

        # --- 核心修改：不重合检测 ---
        is_overlap = False
        for b in building_data:
            exist_poly = Polygon(b["polygon"])
            # 如果新建筑与已有建筑相交（buffer(0.0001)增加一点间距），则舍弃
            if new_poly.buffer(0.0001).intersects(exist_poly):
                is_overlap = True
                break

        # 确保不覆盖起点和终点
        if new_poly.distance(ShapelyPoint(start_point[:2])) < 0.0002 or \
                new_poly.distance(ShapelyPoint(goal_point[:2])) < 0.0002:
            is_overlap = True

        if not is_overlap:
            building_data.append({
                "polygon": new_poly_coords,
                "min_alt": 0,
                "max_alt": random.randint(30, 80)
            })

    print(f"成功生成 {len(building_data)} 个不重叠建筑物")

    # 运行规划 (注意：如果地图扩大了，Grid 尺寸也要相应在函数内调大)
    grid_path, proj_enu = plan_3d_grid_path(start_point, goal_point, building_data, scale=0.5, safety_dist=2)

    if grid_path:
        print("找到路径，点数:", len(grid_path))
        plot_3d(grid_path, building_data, start_point, goal_point, proj_enu, scale=0.5)
    else:
        print("未找到路径，请检查目标点是否在网格范围内或被封死")
