import numpy as np
from shapely import LineString
from shapely.strtree import STRtree
from shapely.geometry import box, Polygon


class BuildingManager:
    def __init__(self, all_buildings):
        # 1. 预先转换多边形，并保持顺序一致
        self.all_buildings = all_buildings
        self.building_polys = [Polygon(b["polygon"]) for b in all_buildings]

        # 2. 建立空间索引
        # STRtree 会存储 building_polys 中每个对象的索引
        self.tree = STRtree(self.building_polys)

    def get_relevant_buildings(self, start_ll, goal_ll, buffer_degree=0.001):
        # 3. 构造查询矩形
        lon1, lat1 = start_ll[0], start_ll[1]
        lon2, lat2 = goal_ll[0], goal_ll[1]

        query_area = box(
            min(lon1, lon2) - buffer_degree,
            min(lat1, lat2) - buffer_degree,
            max(lon1, lon2) + buffer_degree,
            max(lat1, lat2) + buffer_degree
        )

        # 4. 获取相交的多边形索引
        # query 方法返回的是在 self.building_polys 中的索引列表 (针对较新版本的 Shapely)
        # 或者直接返回几何对象列表。
        indices = self.tree.query(query_area)

        # 5. 根据索引找回原始建筑数据
        filtered_buildings = []
        for idx in indices:
            # 这里的 idx 可能是整数索引，也可能是 Polygon 对象
            # 如果是对象，我们需要找到它在原始列表中的位置
            if isinstance(idx, int):
                filtered_buildings.append(self.all_buildings[idx])
            else:
                # 兼容旧版本：如果是 Polygon 对象，则需要一种方式映射回去
                # 我们可以使用 id(obj) 建立一个映射表，或者直接在初始化时处理
                pass

        # 推荐最通用的处理方式：
        # 在 query 时直接获取索引是最高效的
        return [self.all_buildings[i] for i in indices]

    def get_corridor_buildings(self, start_ll, goal_ll, buffer_degree=0.0005):
        """
        旋转包围盒/走廊过滤：只选取 A-B 路径沿途附近的建筑
        buffer_degree: 走廊的半径（宽度的一半）。
        在中纬度地区，0.0005 约等于 50 米。
        """
        # 1. 创建起点到终点的连线
        line = LineString([(start_ll[0], start_ll[1]), (goal_ll[0], goal_ll[1])])

        # 2. 生成走廊区域（Buffer）
        # cap_style=2 表示末端是平的（flat），不带圆头，更符合包围盒逻辑
        corridor_area = line.buffer(buffer_degree, cap_style=2)

        # 3. 使用 R-Tree 检索
        indices = self.tree.query(corridor_area)

        # 4. 这里的 indices 在不同版本 shapely 中表现不同
        # 为了保险，我们通过检查类型来处理
        result = []
        for idx in indices:
            if isinstance(idx, (int, np.integer)):
                result.append(self.all_buildings[idx])
            else:
                # 如果返回的是 Polygon 对象，则需要根据索引找回
                # 在 Shapely 2.0+ 中，推荐使用 tree.query(corridor_area) 返回的索引数组
                # 如果是旧版本，可以使用以下方式（假设 poly 对象是一致的）
                # 这里我们假设你环境里是较新的版本：
                result.append(self.all_buildings[self.building_polys.index(idx)])

        return result