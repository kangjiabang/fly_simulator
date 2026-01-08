import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, Point as ShapelyPoint
from heapq import heappush, heappop
from typing import Tuple, List, Set, Dict
import pyproj
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, Point as ShapelyPoint
from heapq import heappush, heappop
from typing import Tuple, List, Set, Dict
import pyproj

Point3D = Tuple[int, int, int]
LonLatAlt = Tuple[float, float, float]


class Grid3D:
    def __init__(self, size_x, size_y, size_z):
        self.size_x = size_x
        self.size_y = size_y
        self.size_z = size_z
        self.obstacles: Set[Point3D] = set()

    def in_bounds(self, p: Point3D) -> bool:
        x, y, z = p
        return 0 <= x < self.size_x and 0 <= y < self.size_y and 0 <= z < self.size_z

    def is_free(self, p: Point3D) -> bool:
        return p not in self.obstacles


def get_neighbors_with_cost(p: Point3D) -> List[Tuple[Point3D, float]]:
    x, y, z = p
    neighbors_list = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                if dx == 0 and dy == 0 and dz == 0: continue
                # 真实物理代价
                cost = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
                # 微调：增加垂直移动的代价，诱导算法优先水平绕行
                final_cost = cost + abs(dz) * 0.5
                neighbors_list.append(((x + dx, y + dy, z + dz), final_cost))
    return neighbors_list


def heuristic(a: Point3D, b: Point3D) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


# --- 核心平滑函数：直线检查 ---
def is_line_clear_grid(p1: Point3D, p2: Point3D, grid: Grid3D) -> bool:
    """检查两点连线在网格中是否无阻碍"""
    dist = heuristic(p1, p2)
    if dist < 1.0: return True

    # 步长设为 0.5 个网格单位，确保不漏掉任何障碍格
    steps = int(dist * 2)
    for i in range(1, steps):
        t = i / steps
        curr = (
            int(round(p1[0] + (p2[0] - p1[0]) * t)),
            int(round(p1[1] + (p2[1] - p1[1]) * t)),
            int(round(p1[2] + (p2[2] - p1[2]) * t))
        )
        if not grid.in_bounds(curr) or not grid.is_free(curr):
            return False
    return True


def smooth_path_3d(path: List[Point3D], grid: Grid3D) -> List[Point3D]:
    """贪心拉直算法"""
    if len(path) <= 2: return path

    smoothed = [path[0]]
    curr_idx = 0
    while curr_idx < len(path) - 1:
        # 尝试从最后一点向前寻找最远的可见点
        found_next = False
        for next_idx in range(len(path) - 1, curr_idx, -1):
            if is_line_clear_grid(path[curr_idx], path[next_idx], grid):
                smoothed.append(path[next_idx])
                curr_idx = next_idx
                found_next = True
                break
        if not found_next:  # 防御性逻辑
            curr_idx += 1
            smoothed.append(path[curr_idx])
    return smoothed


def astar_3d_grid(grid: Grid3D, start: Point3D, goal: Point3D) -> List[Point3D]:
    open_set = []
    heappush(open_set, (0, start))
    came_from: Dict[Point3D, Point3D] = {}
    g_score: Dict[Point3D, float] = {start: 0}

    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for nxt, cost in get_neighbors_with_cost(current):
            if not grid.in_bounds(nxt) or not grid.is_free(nxt): continue
            tentative_g = g_score[current] + cost
            if tentative_g < g_score.get(nxt, float('inf')):
                came_from[nxt] = current
                g_score[nxt] = tentative_g
                f_score = tentative_g + heuristic(nxt, goal)
                heappush(open_set, (f_score, nxt))
    return []


def add_building_to_grid(grid, building, proj_enu, scale, safety_dist=3.0):
    poly_coords = [proj_enu(lon, lat) for lon, lat in building["polygon"]]
    safe_poly = Polygon(poly_coords).buffer(safety_dist)
    minx, miny, maxx, maxy = safe_poly.bounds

    # 优化点：Z 轴也增加安全距离（向上和向下扩展）
    min_z = max(0, int(building["min_alt"] - 2))  # 假设地面为 0
    max_z = int(building["max_alt"] + 5)  # 向上多留 5 米

    for x in range(int(minx * scale), int(maxx * scale) + 1):
        for y in range(int(miny * scale), int(maxy * scale) + 1):
            if safe_poly.contains(ShapelyPoint(x / scale, y / scale)):
                for z in range(min_z, max_z + 1):
                    grid.obstacles.add((x, y, z))


def plan_3d_grid_path(start_ll, goal_ll, buildings, scale=1.0, safety_dist=10.0):
    proj_enu = pyproj.Proj(proj='aeqd', lat_0=start_ll[1], lon_0=start_ll[0])

    grid = Grid3D(300, 300, 150)
    for b in buildings:
        add_building_to_grid(grid, b, proj_enu, scale, safety_dist)

    def to_grid(ll):
        x, y = proj_enu(ll[0], ll[1])
        return (int(round(x * scale)), int(round(y * scale)), int(round(ll[2])))

    s_grid, g_grid = to_grid(start_ll), to_grid(goal_ll)

    raw_path = astar_3d_grid(grid, s_grid, g_grid)

    smooth_path = smooth_path_3d(raw_path, grid)

    # has_collision = check_path_collision_grid(raw_path, grid)
    # print("Collision:", has_collision)

    return smooth_path, proj_enu



# --------------------------
# 绘图逻辑
# --------------------------
def plot_3d(path_grid, buildings, start_ll, goal_ll, proj_enu, scale):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 路径（Grid → ENU）
    xs, ys, zs = [], [], []
    for p in path_grid:
        xs.append(p[0] / scale)
        ys.append(p[1] / scale)
        zs.append(p[2])
    ax.plot(xs, ys, zs, color='blue', linewidth=3)

    # 建筑（ENU）
    for b in buildings:
        poly = [proj_enu(lon, lat) for lon, lat in b["polygon"]]
        poly.append(poly[0])
        px, py = zip(*poly)
        ax.plot(px, py, [b["max_alt"]] * len(px), color='brown')
        ax.plot(px, py, [b["min_alt"]] * len(px), color='brown')
        for x, y in poly[:-1]:
            ax.plot([x, x], [y, y], [b["min_alt"], b["max_alt"]], color='brown', alpha=0.5)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Altitude (m)")

    # ⭐ 核心：设置俯视角
    ax.view_init(elev=90, azim=-90)
    plt.show()

def grid_path_to_lonlat(grid_path, proj_enu, scale):
    lonlat_path = []
    for xg, yg, zg in grid_path:
        x = xg / scale
        y = yg / scale
        lon, lat = proj_enu(x, y, inverse=True)
        lonlat_path.append((lon, lat, zg))
    return lonlat_path

def check_path_collision_grid(path, grid):
    for p in path:
        if p in grid.obstacles:
            return True
    return False

if __name__ == "__main__":
    start_point = (116.397428, 39.90923, 20)
    goal_point = (116.3985, 39.9105, 25)

    # building_data = [
    #     {"polygon": [(116.3976, 39.9095), (116.3980, 39.9095),
    #                  (116.3980, 39.9098), (116.3976, 39.9098)],
    #      "min_alt": 0, "max_alt": 60},
    #
    #     {"polygon": [(116.3982, 39.9094), (116.3985, 39.9094),
    #                  (116.3985, 39.9098), (116.3982, 39.9098)],
    #      "min_alt": 0, "max_alt": 55}
    # ]

    building_data = [
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

        # 建筑 3（新增：位于路径中部，较矮）
        {
            "polygon": [
                (116.3979, 39.9097),
                (116.3981, 39.9097),
                (116.3981, 39.9099),
                (116.3979, 39.9099)
            ],
            "min_alt": 0,
            "max_alt": 50
        },

        # 建筑 4（新增：高楼，测试抬高飞行）
        {
            "polygon": [
                (116.3974, 39.9096),
                (116.39755, 39.9096),
                (116.39755, 39.90985),
                (116.3974, 39.90985)
            ],
            "min_alt": 0,
            "max_alt": 80
        }
    ]

    grid_path, proj_enu = plan_3d_grid_path(
        start_point, goal_point, building_data, scale=0.5, safety_dist=3
    )

    print("Path points:", len(grid_path))
    for p in grid_path:
        print(p)


    lonlat_path = grid_path_to_lonlat(grid_path, proj_enu, scale=0.5)

    for p in lonlat_path:
        print(p)

    plot_3d(grid_path, building_data,
            start_point, goal_point,
            proj_enu, scale=0.5)
