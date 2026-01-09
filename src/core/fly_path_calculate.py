import pyproj
import numpy as np
from shapely.geometry import box, Point as ShapelyPoint
from src.core.path_planner_core import Grid3D, astar_3d_grid, smooth_path_3d

# 假设存在的外部业务模块
from src.others.fly_path_view_batch import grid_path_to_lonlat
from src.service.load_nofly_zones import get_nofly_zones
from src.tool.BuildingManager import BuildingManager
from src.tool.plot_3d import plot_3d


def plan_3d_grid_path(start_ll, goal_ll, buildings, nofly_zones, scale=1.0, safety_dist=10.0):
    """
    主规划入口
    :param start_ll: 起点 (lon, lat, alt)
    :param goal_ll: 终点 (lon, lat, alt)
    :param scale: 缩放比例 (1.0 代表 1格=1米)
    :param safety_dist: 安全缓冲距离 (米)
    """
    # 1. 初始化坐标投影 (以起点为中心建立等距方位投影 ENU)
    proj_enu = pyproj.Proj(proj='aeqd', lat_0=start_ll[1], lon_0=start_ll[0])

    # 2. 计算网格边界范围
    all_coords = [proj_enu(start_ll[0], start_ll[1]), proj_enu(goal_ll[0], goal_ll[1])]
    for b in buildings:
        bounds = b["geometry"].bounds
        all_coords.extend([proj_enu(bounds[0], bounds[1]), proj_enu(bounds[2], bounds[3])])

    xs, ys = zip(*all_coords)
    buffer_m = 50  # 边界留白
    g_min_x, g_max_x = int((min(xs) - buffer_m) * scale), int((max(xs) + buffer_m) * scale)
    g_min_y, g_max_y = int((min(ys) - buffer_m) * scale), int((max(ys) + buffer_m) * scale)
    g_size_z = 200  # 最大飞行高度限制

    grid = Grid3D(g_min_x, g_max_x, g_min_y, g_max_y, g_size_z)

    # 3. 栅格化函数：将几何体写入网格
    def add_geo_to_grid(geometry, z_min, z_max, buffer_val):
        # 经纬度 bounds 转 ENU
        min_lon, min_lat, max_lon, max_lat = geometry.bounds
        e_min_x, e_min_y = proj_enu(min_lon, min_lat)
        e_max_x, e_max_y = proj_enu(max_lon, max_lat)

        # 在 ENU 系下进行 Buffer 扩展并计算扫描范围
        poly_enu = box(e_min_x, e_min_y, e_max_x, e_max_y).buffer(buffer_val)
        minx, miny, maxx, maxy = poly_enu.bounds

        for ix in range(int(minx * scale), int(maxx * scale) + 1):
            for iy in range(int(miny * scale), int(maxy * scale) + 1):
                if grid.in_bounds(ix, iy, 0):
                    # 判断网格点中心是否在多边形内
                    if poly_enu.contains(ShapelyPoint(ix / scale, iy / scale)):
                        grid.add_obstacle_range(ix, iy, int(z_min), int(z_max))

    # 写入建筑障碍
    for b in buildings:
        z_max_b = b.get("max_alt", b.get("height", 50))
        add_geo_to_grid(b["geometry"], b.get("min_alt", 0), min(g_size_z - 1, z_max_b + 5), safety_dist)

    # 写入禁飞区
    for nfz in nofly_zones:
        add_geo_to_grid(nfz["geometry"], nfz["z_min"], min(g_size_z - 1, nfz["z_max"]), nfz.get("buffer", 0.0))

    # 4. 转换起终点到网格坐标
    s_enu = proj_enu(start_ll[0], start_ll[1])
    g_enu = proj_enu(goal_ll[0], goal_ll[1])
    s_grid = (int(round(s_enu[0] * scale)), int(round(s_enu[1] * scale)), int(start_ll[2]))
    g_grid = (int(round(g_enu[0] * scale)), int(round(g_enu[1] * scale)), int(goal_ll[2]))

    # 5. 执行路径规划与平滑
    try:
        raw_path = astar_3d_grid(grid, s_grid, g_grid)
        if not raw_path: 
            return [], proj_enu

        return smooth_path_3d(raw_path, grid), proj_enu
    except ValueError as e:
        # 重新抛出异常，让上层处理
        raise e


if __name__ == "__main__":
    # 配置参数
    START = (120.002127, 30.284038, 40)
    GOAL = (119.994249, 30.283932, 30)
    SCALE = 0.5  # 1格 = 2米 (1.0/0.5)

    # 加载数据
    manager = BuildingManager()
    buildings = manager.get_corridor_buildings(START, GOAL)  # 获取航道周围建筑

    print(f"找到 {len(buildings)} 个建筑")
    for i, b in enumerate(buildings):
        min_alt = b.get("min_alt", 0)
        max_alt = b.get("max_alt", b.get("height", "N/A"))
        # print(f"  建筑 {i + 1}: min_alt={min_alt}, max_alt={max_alt}")


    nfz = get_nofly_zones()  # 获取禁飞区

    # 执行
    grid_path, projection = plan_3d_grid_path(START, GOAL, buildings, nfz, scale=SCALE, safety_dist=5)

    if grid_path:
        print("找到路径，点数:", len(grid_path))

        lonlat_path = grid_path_to_lonlat(grid_path, projection, scale=0.5)

        for i, ((xg, yg, zg), (lon, lat, alt)) in enumerate(zip(grid_path, lonlat_path)):
            print(
                f"{i:03d}: "
                f"Grid=({xg}, {yg}, {zg})  "
                f"LonLatAlt=({lon:.7f}, {lat:.7f}, {alt}m)"
            )

        plot_3d(grid_path, buildings, START, GOAL, projection, scale=SCALE, nofly_zones=nfz)
    else:
        print("未找到路径，请检查目标点是否在网格范围内或被封死")
