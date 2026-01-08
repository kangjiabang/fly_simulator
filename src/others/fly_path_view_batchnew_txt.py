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
def astar_3d_grid(grid, start, goal):
    # 如果起点或终点在障碍中，直接失败
    if not grid.is_free(*start) or not grid.is_free(*goal):
        return []

    occ = grid.occ_map
    min_x, min_y = grid.min_x, grid.min_y

    z_min = min(start[2], goal[2]) - 20
    z_max = max(start[2], goal[2]) + 20

    g_score = np.full(
        (grid.max_x-min_x+1,
         grid.max_y-min_y+1,
         grid.size_z),
        np.inf,
        dtype=np.float32
    )

    open_set = []
    heappush(open_set, (0.0, start))
    g_score[start[0]-min_x, start[1]-min_y, start[2]] = 0.0
    came_from = {}
    closed = set()

    gx, gy, gz = goal

    while open_set:
        _, cur = heappop(open_set)
        if cur in closed:
            continue
        closed.add(cur)

        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            return path[::-1]

        cx, cy, cz = cur
        cg = g_score[cx-min_x, cy-min_y, cz]

        for dx, dy, dz, cost in NEIGHBORS_OFFSETS:
            nx, ny, nz = cx+dx, cy+dy, cz+dz
            if nz < z_min or nz > z_max:
                continue
            if not (grid.min_x <= nx <= grid.max_x and grid.min_y <= ny <= grid.max_y):
                continue
            if occ[nx-min_x, ny-min_y] & (1 << nz):
                continue

            ng = cg + cost
            if ng < g_score[nx-min_x, ny-min_y, nz]:
                g_score[nx-min_x, ny-min_y, nz] = ng
                came_from[(nx, ny, nz)] = cur
                h = ((nx-gx)**2 + (ny-gy)**2 + (nz-gz)**2) ** 0.5
                heappush(open_set, (ng + h*2.5, (nx, ny, nz)))

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
    # 1. 坐标投影转换器
    proj_enu = pyproj.Proj(proj='aeqd', lat_0=start_ll[1], lon_0=start_ll[0])

    # 2. 收集边界坐标
    all_coords = []
    # 添加起终点 ENU
    all_coords.append(proj_enu(start_ll[0], start_ll[1]))
    all_coords.append(proj_enu(goal_ll[0], goal_ll[1]))

    # 遍历建筑获取边界
    for b in buildings:
        # 获取几何对象的经纬度边界 (min_lon, min_lat, max_lon, max_lat)
        bounds = b["geometry"].bounds
        all_coords.append(proj_enu(bounds[0], bounds[1]))  # 左下
        all_coords.append(proj_enu(bounds[2], bounds[3]))  # 右上

    # 计算 ENU 边界
    buffer_m = 50
    xs, ys = zip(*all_coords)
    min_x_m, max_x_m = min(xs) - buffer_m, max(xs) + buffer_m
    min_y_m, max_y_m = min(ys) - buffer_m, max(ys) + buffer_m

    # 转换为网格范围
    g_min_x, g_max_x = int(min_x_m * scale), int(max_x_m * scale)
    g_min_y, g_max_y = int(min_y_m * scale), int(max_y_m * scale)
    g_size_z = 200

    # 3. 初始化 Grid3D
    grid = Grid3D(g_min_x, g_max_x, g_min_y, g_max_y, g_size_z)

    # 4. 辅助函数：经纬度 -> 网格坐标
    def to_grid(ll):
        ex, ey = proj_enu(ll[0], ll[1])
        return (int(round(ex * scale)), int(round(ey * scale)), int(round(ll[2])))

    # 5. 填充障碍物
    for b in buildings:
        # 兼容性处理：获取几何对象
        geom = b["geometry"]

        # 将建筑边界转为 ENU 系
        b_min_lon, b_min_lat, b_max_lon, b_max_lat = geom.bounds
        e_min_x, e_min_y = proj_enu(b_min_lon, b_min_lat)
        e_max_x, e_max_y = proj_enu(b_max_lon, b_max_lat)

        # 在 ENU 系下创建外接矩形并应用安全距离
        # 使用 box 替代手动转换所有点，效率更高且更安全
        safe_poly_enu = box(e_min_x, e_min_y, e_max_x, e_max_y).buffer(safety_dist)
        minx, miny, maxx, maxy = safe_poly_enu.bounds

        # 确定 Z 轴范围（兼容 height 或 max_alt 键）
        z_min = int(b.get("min_alt", 0))
        z_max_val = b.get("max_alt", b.get("height", 50))
        z_max = min(g_size_z - 1, int(z_max_val + 5))  # 顶部加5米安全冗余

        # 遍历受影响的网格区域
        x_range = np.arange(int(minx * scale), int(maxx * scale) + 1)
        y_range = np.arange(int(miny * scale), int(maxy * scale) + 1)

        for ix in x_range:
            for iy in y_range:
                # 边界检查
                if g_min_x <= ix <= g_max_x and g_min_y <= iy <= g_max_y:
                    # 在 ENU 空间进行碰撞判定
                    if safe_poly_enu.contains(ShapelyPoint(ix / scale, iy / scale)):
                        grid.add_obstacle_range(ix, iy, z_min, z_max)

    # 6. A* 搜索
    s_grid = to_grid(start_ll)
    g_grid = to_grid(goal_ll)

    print(f"Grid Bounds: X[{g_min_x}:{g_max_x}], Y[{g_min_y}:{g_max_y}]")
    print(f"Pathfinding: {s_grid} -> {g_grid}")

    raw_path = astar_3d_grid(grid, s_grid, g_grid)

    if not raw_path:
        print("Error: No path found.")
        return [], proj_enu

    # 7. 路径平滑
    smooth_path = smooth_path_3d(raw_path, grid)
    return smooth_path, proj_enu

    # --------------------------
# 绘图函数保持不变
# --------------------------


def plot_3d(path_grid, buildings, start_ll, goal_ll, proj_enu, scale):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 1. 绘制路径
    if path_grid:
        xs = [p[0] / scale for p in path_grid]
        ys = [p[1] / scale for p in path_grid]
        zs = [p[2] for p in path_grid]
        ax.plot(xs, ys, zs, color='blue', linewidth=3, label='Planned Path', zorder=10)

    # 2. 绘制建筑 (处理 MultiPolygon)
    for b in buildings:
        geom = b["geometry"]
        # 获取高度，兼容不同键名
        z_min = b.get("min_alt", 0)
        z_max = b.get("max_alt", b.get("height", 50))

        # 将 MultiPolygon 统一视为几何列表处理
        parts = geom.geoms if hasattr(geom, 'geoms') else [geom]

        for part in parts:
            # 提取外轮廓坐标
            lons, lats = part.exterior.xy
            # 投影到 ENU 坐标系
            enu_coords = [proj_enu(lon, lat) for lon, lat in zip(lons, lats)]
            px, py = zip(*enu_coords)

            # 绘制顶部和底部多边形轮廓
            ax.plot(px, py, [z_max] * len(px), color='brown', alpha=0.7)
            ax.plot(px, py, [z_min] * len(px), color='brown', alpha=0.3)

            # 绘制垂直边柱 (侧墙)
            for i in range(len(px)):
                ax.plot([px[i], px[i]], [py[i], py[i]], [z_min, z_max],
                        color='brown', alpha=0.3, linestyle='--')

    # 3. 绘制起终点
    start_enu = proj_enu(start_ll[0], start_ll[1])
    goal_enu = proj_enu(goal_ll[0], goal_ll[1])
    ax.scatter([start_enu[0]], [start_enu[1]], [start_ll[2]], color='green', s=100, label='Start')
    ax.scatter([goal_enu[0]], [goal_enu[1]], [goal_ll[2]], color='red', s=100, label='Goal')

    # 设置标签和视角
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Altitude (m)")

    # 调整视角：elev=30度仰角，azim=-60度方位角，这样看3D效果更好
    # 如果想从正上方看，请改回 (90, -90)
    #ax.view_init(elev=30, azim=-60)
    ax.view_init(elev=90, azim=-90)
    plt.legend()
    plt.title("3D Path Planning with Buildings")
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
    start_point = (120.0078046 ,30.2994905, 20)  # 起点调远
    goal_point = (120.0111753, 30.2899397, 30)  # 终点调远

    building_data = get_buildings_data()

    # 在测试代码中使用
    #relevant_buildings = filter_buildings(start_point, goal_point, building_data)
    manager = BuildingManager()
    relevant_buildings = manager.get_corridor_buildings(start_point, goal_point)
    print(f"找到 {len(relevant_buildings)} 个建筑")
    for i, b in enumerate(relevant_buildings):
        min_alt = b.get("min_alt", 0)
        max_alt = b.get("max_alt", b.get("height", "N/A"))
        #print(f"  建筑 {i + 1}: min_alt={min_alt}, max_alt={max_alt}")

    # 运行规划 (注意：如果地图扩大了，Grid 尺寸也要相应在函数内调大)
    grid_path, proj_enu = plan_3d_grid_path(start_point, goal_point, relevant_buildings, scale=0.5, safety_dist=5)

    if grid_path:
        print("找到路径，点数:", len(grid_path))

        lonlat_path = grid_path_to_lonlat(grid_path, proj_enu, scale=0.5)

        for i, ((xg, yg, zg), (lon, lat, alt)) in enumerate(zip(grid_path, lonlat_path)):
            print(
                f"{i:03d}: "
                f"Grid=({xg}, {yg}, {zg})  "
                f"LonLatAlt=({lon:.7f}, {lat:.7f}, {alt}m)"
            )

        plot_3d(grid_path, relevant_buildings, start_point, goal_point, proj_enu, scale=0.5)
    else:
        print("未找到路径，请检查目标点是否在网格范围内或被封死")
