from heapq import heappush, heappop
from typing import Tuple, List, Set, Dict
from shapely.geometry import Polygon, Point as ShapelyPoint
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
        return (
            0 <= x < self.size_x and
            0 <= y < self.size_y and
            0 <= z < self.size_z
        )

    def is_free(self, p: Point3D) -> bool:
        return p not in self.obstacles

def heuristic(a: Point3D, b: Point3D) -> float:
    return abs(a[0]-b[0]) + abs(a[1]-b[1]) + abs(a[2]-b[2])

def neighbors(p: Point3D) -> List[Point3D]:
    x, y, z = p
    return [
        (x+1, y, z), (x-1, y, z),
        (x, y+1, z), (x, y-1, z),
        (x, y, z+1), (x, y, z-1)
    ]

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
    # 以起点为原点的局部坐标系
    proj_enu = pyproj.Proj(proj='aeqd', lat_0=lat0, lon_0=lon0)
    return proj_enu

def lonlat_to_grid(lon: float, lat: float, alt: float, proj_enu, scale: float = 1.0) -> Point3D:
    x, y = proj_enu(lon, lat)
    return (int(x*scale), int(y*scale), int(alt))

def grid_to_lonlat(p: Point3D, proj_enu, scale: float = 1.0) -> LonLatAlt:
    x, y, z = p
    lon, lat = proj_enu(x/scale, y/scale, inverse=True)
    return (lon, lat, z)

# --------------------------
# Polygon + 高度 -> 占据网格
# --------------------------
def add_building_polygon(grid: Grid3D, building: Dict, proj_enu, scale: float = 1.0):
    # building = {"polygon": [(lon, lat), ...], "min_alt": 0, "max_alt": 20}
    poly_coords = [proj_enu(lon, lat) for lon, lat in building["polygon"]]
    poly = Polygon(poly_coords)
    minx, miny, maxx, maxy = poly.bounds

    for x in range(int(minx*scale), int(maxx*scale)+1):
        for y in range(int(miny*scale), int(maxy*scale)+1):
            if poly.contains(ShapelyPoint(x/scale, y/scale)):
                for z in range(int(building["min_alt"]), int(building["max_alt"])):
                    grid.obstacles.add((x, y, z))

# --------------------------
# 主函数
# --------------------------
def plan_3d_path(
    start: LonLatAlt,
    goal: LonLatAlt,
    buildings: List[Dict],
    grid_size: Tuple[int,int,int] = (500, 500, 300),
    scale: float = 1.0
) -> List[LonLatAlt]:
    # 1. 创建局部投影
    proj_enu = create_proj(start[1], start[0])  # lat0, lon0

    # 2. 创建网格
    grid = Grid3D(*grid_size)

    # 3. 添加建筑
    for b in buildings:
        add_building_polygon(grid, b, proj_enu, scale)

    # 4. 起点终点映射
    start_grid = lonlat_to_grid(start[0], start[1], start[2], proj_enu, scale)
    goal_grid  = lonlat_to_grid(goal[0], goal[1], goal[2], proj_enu, scale)

    # 5. A* 搜索
    path_grid = astar_3d(grid, start_grid, goal_grid)

    # 6. 网格路径 -> 经纬度
    path_lonlat = [grid_to_lonlat(p, proj_enu, scale) for p in path_grid]

    return path_lonlat

# --------------------------
# 示例使用
# --------------------------
if __name__ == "__main__":
    start = (116.397428, 39.90923, 10)  # lon, lat, alt
    goal  = (116.398, 39.91, 10)

    buildings = [
        {
            "polygon": [(116.3976, 39.9095), (116.3978, 39.9095), (116.3978, 39.9097), (116.3976, 39.9097)],
            "min_alt": 0,
            "max_alt": 50
        }
    ]

    path = plan_3d_path(start, goal, buildings, grid_size=(200,200,60), scale=1)  # scale放大网格
    print("Path points:", len(path))
    for p in path:
        print(p)
