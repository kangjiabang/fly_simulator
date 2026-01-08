import math
import random

import matplotlib.pyplot as plt
from shapely import box
from shapely.geometry import Polygon, Point as ShapelyPoint
from heapq import heappush, heappop
from typing import Tuple, List
import pyproj
import numpy as np

from src.tool.BuildingManager import BuildingManager

Point3D = Tuple[int, int, int]
LonLatAlt = Tuple[float, float, float]

# --------------------------
# 优化后的 Grid3D
# --------------------------

# --------------------------
# 极致优化版 Grid3D
# --------------------------
class Grid3D:
    def __init__(self, min_x, max_x, min_y, max_y, size_z):
        self.min_x, self.max_x = min_x, max_x
        self.min_y, self.max_y = min_y, max_y
        self.size_z = size_z
        # 使用二维数组存储位掩码 (空间换时间)
        # 每个格子用一个 Python 大整数表示 Z 轴的占用情况
        self.occ_map = np.zeros((max_x - min_x + 1, max_y - min_y + 1), dtype=object)
        for i in np.ndindex(self.occ_map.shape):
            self.occ_map[i] = 0

    def in_bounds(self, x, y, z) -> bool:
        return self.min_x <= x <= self.max_x and \
            self.min_y <= y <= self.max_y and \
            0 <= z < self.size_z

    def is_free(self, x, y, z) -> bool:
        # 使用位运算判断第 z 位是否为 1
        return not (self.occ_map[x - self.min_x, y - self.min_y] & (1 << z))

    def add_obstacle_range(self, x, y, z_min, z_max):
        # 将 z_min 到 z_max 的位全部设为 1
        mask = ((1 << (z_max - z_min + 1)) - 1) << z_min
        self.occ_map[x - self.min_x, y - self.min_y] |= mask


# --------------------------
# 性能优化版 A*
# --------------------------
def astar_3d_grid(grid: Grid3D, start: Point3D, goal: Point3D) -> List[Point3D]:
    # 局部变量化常用属性，减少属性访问开销 (非常关键)
    occ_map = grid.occ_map
    m_x, m_y = grid.min_x, grid.min_y
    in_bounds = grid.in_bounds

    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    goal_x, goal_y, goal_z = goal

    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            # 回溯路径... (保持原样)
            path = []
            while current in came_from:
                path.append(current);
                current = came_from[current]
            path.append(start);
            return path[::-1]

        cx, cy, cz = current
        cg = g_score[current]

        for dx, dy, dz, cost in NEIGHBORS_OFFSETS:
            nx, ny, nz = cx + dx, cy + dy, cz + dz

            # 这里的检测逻辑合并，减少函数调用开销
            if not (grid.min_x <= nx <= grid.max_x and grid.min_y <= ny <= grid.max_y and 0 <= nz < grid.size_z):
                continue
            if occ_map[nx - m_x, ny - m_y] & (1 << nz):
                continue

            tg = cg + cost
            if tg < g_score.get((nx, ny, nz), float('inf')):
                came_from[(nx, ny, nz)] = current
                g_score[(nx, ny, nz)] = tg
                # 欧几里得距离启发式
                h = ((nx - goal_x) ** 2 + (ny - goal_y) ** 2 + (nz - goal_z) ** 2) ** 0.5
                heappush(open_set, (tg + h * 1.2, (nx, ny, nz)))  # 略微增加 h 权重可显著提速
    return []
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
    steps = int(dist * 2)
    for i in range(1, steps):
        t = i / steps
        # 计算当前步的坐标
        curr_x = int(round(p1[0] + (p2[0] - p1[0]) * t))
        curr_y = int(round(p1[1] + (p2[1] - p1[1]) * t))
        curr_z = int(round(p1[2] + (p2[2] - p1[2]) * t))

        # 核心修复：解包参数调用
        if not grid.in_bounds(curr_x, curr_y, curr_z) or not grid.is_free(curr_x, curr_y, curr_z):
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



# --------------------------
# 路径规划函数
# --------------------------
def plan_3d_grid_path(start_ll, goal_ll, buildings, scale=1.0, safety_dist=10.0):
    # 1. 坐标投影
    proj_enu = pyproj.Proj(proj='aeqd', lat_0=start_ll[1], lon_0=start_ll[0])

    # 2. 收集所有点的 ENU 坐标以确定地图边界
    all_coords = []
    # 添加起终点
    all_coords.append(proj_enu(start_ll[0], start_ll[1]))
    all_coords.append(proj_enu(goal_ll[0], goal_ll[1]))
    # 添加建筑顶点
    for b in buildings:
        for lon, lat in b["polygon"]:
            all_coords.append(proj_enu(lon, lat))

    # 计算边界（加上缓冲距离避免边缘检测失败）
    buffer_m = 50
    xs, ys = zip(*all_coords)
    min_x_m, max_x_m = min(xs) - buffer_m, max(xs) + buffer_m
    min_y_m, max_y_m = min(ys) - buffer_m, max(ys) + buffer_m

    # 转换为网格坐标范围
    g_min_x, g_max_x = int(min_x_m * scale), int(max_x_m * scale)
    g_min_y, g_max_y = int(min_y_m * scale), int(max_y_m * scale)
    g_size_z = 200  # 高度通常固定即可

    # 3. 初始化优化后的 Grid3D
    # 现在参数匹配了: (min_x, max_x, min_y, max_y, size_z)
    grid = Grid3D(g_min_x, g_max_x, g_min_y, g_max_y, g_size_z)

    # 4. 辅助转换函数：经纬度 -> 网格坐标
    def to_grid(ll):
        ex, ey = proj_enu(ll[0], ll[1])
        return (int(round(ex * scale)), int(round(ey * scale)), int(round(ll[2])))

    # 5. 填充障碍物
    for b in buildings:
        poly_coords = [proj_enu(lon, lat) for lon, lat in b["polygon"]]
        safe_poly = Polygon(poly_coords).buffer(safety_dist)
        minx, miny, maxx, maxy = safe_poly.bounds

        # 批量填充
        x_range = np.arange(int(minx * scale), int(maxx * scale) + 1)
        y_range = np.arange(int(miny * scale), int(maxy * scale) + 1)

        z_min = int(b["min_alt"])
        z_max = min(g_size_z - 1, int(b["max_alt"] + 5))

        for ix in x_range:
            for iy in y_range:
                # 只在网格范围内且在多边形内时添加
                if g_min_x <= ix <= g_max_x and g_min_y <= iy <= g_max_y:
                    if safe_poly.contains(ShapelyPoint(ix / scale, iy / scale)):
                        grid.add_obstacle_range(ix, iy, z_min, z_max)

    # 6. A* 搜索
    s_grid = to_grid(start_ll)
    g_grid = to_grid(goal_ll)

    print(f"Grid Range: X[{g_min_x}:{g_max_x}], Y[{g_min_y}:{g_max_y}]")
    print(f"Searching from {s_grid} to {g_grid}...")

    raw_path = astar_3d_grid(grid, s_grid, g_grid)

    if not raw_path:
        return [], proj_enu

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


def get_buildings_data():
    global building_data, dx, dy
    center_lon, center_lat = 116.3975, 39.9095
    building_data = []
    num_buildings = 100  # 生成30个不重叠的建筑

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
    return building_data


def filter_buildings(start_ll, goal_ll, all_buildings, buffer_degree=0.001):
    """
    过滤出起终点附近的建筑
    """
    # 构造查询矩形
    lon1, lat1 = start_ll[0], start_ll[1]
    lon2, lat2 = goal_ll[0], goal_ll[1]

    search_box = box(
        min(lon1, lon2) - buffer_degree,
        min(lat1, lat2) - buffer_degree,
        max(lon1, lon2) + buffer_degree,
        max(lat1, lat2) + buffer_degree
    )

    # 只有与 search_box 相交的建筑才参与规划
    filtered = []
    for b in all_buildings:
        # 如果建筑多边形与搜索框相交
        if Polygon(b["polygon"]).intersects(search_box):
            filtered.append(b)
    print(f"过滤出 {len(filtered)} 个建筑")
    return filtered

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

    building_data = get_buildings_data()

    # 在测试代码中使用
    #relevant_buildings = filter_buildings(start_point, goal_point, building_data)
    manager = BuildingManager(building_data)
    relevant_buildings = manager.get_corridor_buildings(start_point, goal_point)
    print(f"找到 {len(relevant_buildings)} 个建筑")

    # 运行规划 (注意：如果地图扩大了，Grid 尺寸也要相应在函数内调大)
    grid_path, proj_enu = plan_3d_grid_path(start_point, goal_point, relevant_buildings, scale=0.5, safety_dist=2)

    if grid_path:
        print("找到路径，点数:", len(grid_path))
        plot_3d(grid_path, building_data, start_point, goal_point, proj_enu, scale=0.5)
    else:
        print("未找到路径，请检查目标点是否在网格范围内或被封死")
