import random

import numpy as np
import os
import re
from shapely import LineString, wkt
from shapely.strtree import STRtree
from shapely.geometry import box, Polygon
from shapely.geometry import Polygon, Point as ShapelyPoint

print("CWD:", os.getcwd())



class BuildingManager:
    def __init__(self, all_buildings=None):
        # 1. 尝试加载数据
        if not all_buildings:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            wkt_path = os.path.join(base_dir, "buildings_output.txt")
            self.all_buildings = self._load_from_wkt_file(wkt_path)
        else:
            self.all_buildings = all_buildings
        print(f"Loaded {len(self.all_buildings)} buildings.")

        # 2. 预先转换多边形
        # 注意：WKT 中是 MULTIPOLYGON，这里我们取其几何对象
        self.building_polys = []
        for b in self.all_buildings:
            # --- 兼容性修改开始 ---
            if "geometry" in b:
                # 如果已经是几何对象（文件读取的情况）
                geom = b["geometry"]
            elif "polygon" in b:
                # 如果是坐标列表（动态生成的情况），将其转换为 Polygon 对象
                geom = Polygon(b["polygon"])
                # 为了后续统一，可以把转换后的对象存回字典
                b["geometry"] = geom
            else:
                continue
            # 如果是 MultiPolygon，为了索引方便，通常取其第一个多边形或保持原样
            # STRtree 支持多种几何类型
            self.building_polys.append(geom)

        # 3. 建立空间索引
        if self.building_polys:
            self.tree = STRtree(self.building_polys)
        else:
            self.tree = None

    def _load_from_wkt_file(self, file_path):
        """解析 WKT 格式文件"""
        buildings = []
        if not os.path.exists(file_path):
            print(f"File not found: {file_path}")
            return []

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue

                    # 使用正则或分割提取："MULTIPOLYGON(...)" 和 "高度"
                    # 匹配双引号内的内容
                    parts = re.findall(r'"([^"]*)"', line)
                    if len(parts) >= 2:
                        wkt_str = parts[0]
                        height = float(parts[1])

                        # 使用 shapely.wkt 加载几何体
                        geom = wkt.loads(wkt_str)

                        buildings.append({
                            "geometry": geom,
                            "height": height
                        })
        except Exception as e:
            print(f"Error parsing WKT file: {e}")

        return buildings

    def get_corridor_buildings(self, start_ll, goal_ll, buffer_degree=0.0005):
        """
        基于走廊的过滤
        """
        if self.tree is None:
            return []

        line = LineString([(start_ll[0], start_ll[1]), (goal_ll[0], goal_ll[1])])
        corridor_area = line.buffer(buffer_degree, cap_style=2)

        indices = self.tree.query(corridor_area)

        # 针对 Shapely 2.0+ 优化处理
        if isinstance(indices, np.ndarray):
            return [self.all_buildings[i] for i in indices]

        # 兼容旧版本
        result = []
        for idx in indices:
            if isinstance(idx, (int, np.integer)):
                result.append(self.all_buildings[idx])
            else:
                # 如果返回的是对象本身
                try:
                    # 注意：这里使用 id 匹配会更快
                    res_idx = self.building_polys.index(idx)
                    result.append(self.all_buildings[res_idx])
                except ValueError:
                    pass
        return result

    def get_relevant_buildings(self, start_ll, goal_ll, buffer_degree=0.001):
        """
        基于矩形区域的过滤
        """
        if self.tree is None:
            return []

        query_area = box(
            min(start_ll[0], goal_ll[0]) - buffer_degree,
            min(start_ll[1], goal_ll[1]) - buffer_degree,
            max(start_ll[0], goal_ll[0]) + buffer_degree,
            max(start_ll[1], goal_ll[1]) + buffer_degree
        )

        indices = self.tree.query(query_area)

        if isinstance(indices, np.ndarray):
            return [self.all_buildings[i] for i in indices]
        return [self.all_buildings[i] for i in indices]

    def get_buildings_data(self,start_point,goal_point):
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
