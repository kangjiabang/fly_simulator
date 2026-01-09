import matplotlib.pyplot as plt
import numpy as np
from pyproj import Transformer
from shapely.ops import nearest_points

# 假设存在的外部业务模块
from src.service.load_fly_paths import get_fly_paths

# 1. 定义转换器 (WGS84 -> Web Mercator)
# Web Mercator 的单位是米，适合这种小范围的距离计算
_transformer = Transformer.from_crs("epsg:4326", "epsg:3857", always_xy=True)

def calculate_distance_3d(point1, point2):
    print("DEBUG: 使用了最新的 pyproj 转换代码")  # 加上这行
    """
    计算两个3D点之间的真实物理距离
    point: (longitude, latitude, altitude)
    """
    lon1, lat1, z1 = point1
    lon2, lat2, z2 = point2

    # 2. 将经纬度(度)转换为平面坐标(米)
    x1, y1 = _transformer.transform(lon1, lat1)
    x2, y2 = _transformer.transform(lon2, lat2)

    # 3. 此时 x, y, z 的单位全部统一成了“米”
    dx = x1 - x2
    dy = y1 - y2
    dz = z1 - z2

    # 4. 计算 3D 欧几里得距离
    distance = np.sqrt(dx**2 + dy**2 + dz**2)
    return float(distance)


def find_nearest_points_between_lines(line1, line2):
    """
    寻找两条 3D 线段之间真正的最近点，并考虑 Z 轴高度。
    """
    # 1. 寻找 2D 投影上的最近点 (p1, p2)
    # p1 在 line1 上，p2 在 line2 上
    p1_2d, p2_2d = nearest_points(line1, line2)

    # 2. 计算 p1_2d 在 line1 中的高度，p2_2d 在 line2 中的高度
    # 注意：shapely 的 nearest_points 有时会丢失 Z 信息，或者不进行 Z 插值
    # 我们通过 project（投影）和 interpolate（插值）来获取精确的 3D 位置

    # 处理第一条线
    dist_on_line1 = line1.project(p1_2d)  # 找到该点在线段上的运行长度
    p1_3d = line1.interpolate(dist_on_line1)  # 在该长度位置插值，获取包含 Z 的完整坐标

    # 处理第二条线
    dist_on_line2 = line2.project(p2_2d)
    p2_3d = line2.interpolate(dist_on_line2)

    # 3. 提取最终的 3D 坐标元组 (lon, lat, alt)
    nearest_point1 = (p1_3d.x, p1_3d.y, p1_3d.z if p1_3d.has_z else 0)
    nearest_point2 = (p2_3d.x, p2_3d.y, p2_3d.z if p2_3d.has_z else 0)

    # 4. 调用你之前修正过的、带 pyproj 转换的距离函数
    # 这样计算出来的距离就是考虑了经纬度转换和高度 Z 差值的真实物理距离（米）
    min_distance = calculate_distance_3d(nearest_point1, nearest_point2)

    return min_distance, nearest_point1, nearest_point2


def visualize_collision_analysis(fly_paths, analysis_result):
    """可视化碰撞分析结果"""
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # 设置颜色映射
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
    
    # 绘制所有飞行路径
    for i, path in enumerate(fly_paths):
        line = path["geometry"]
        coords = list(line.coords)
        lons, lats, alts = zip(*coords)
        
        color = colors[i % len(colors)]
        ax.plot(lons, lats, alts, label=f'Path {i}', color=color, linewidth=2, marker='o', markersize=4)
        
        # 标记起点和终点
        ax.scatter([lons[0]], [lats[0]], [alts[0]], color=color, s=100, alpha=0.7)  # 起点
        ax.text(lons[0], lats[0], alts[0], f'Start {i}', fontsize=8)
        
        ax.scatter([lons[-1]], [lats[-1]], [alts[-1]], color=color, s=100, alpha=0.7)  # 终点
        ax.text(lons[-1], lats[-1], alts[-1], f'End {i}', fontsize=8)
    
    # 高亮显示有风险的路径对
    for risk_path in analysis_result['risk_paths']:
        path1_idx = risk_path['path1_index']
        path2_idx = risk_path['path2_index']
        risk_level = risk_path['risk_level']
        min_distance = risk_path['min_distance']
        
        # 获取最近点
        point1 = risk_path['nearest_points']['point1']
        point2 = risk_path['nearest_points']['point2']
        
        # 根据风险等级设置颜色
        if risk_level == 'fatal':
            risk_color = 'red'
        elif risk_level == 'warn':
            risk_color = 'orange'
        else:
            risk_color = 'yellow'
        
        # 高亮显示最近点
        ax.scatter([point1[0]], [point1[1]], [point1[2]], color=risk_color, s=200, alpha=0.8, edgecolors='black')
        ax.scatter([point2[0]], [point2[1]], [point2[2]], color=risk_color, s=200, alpha=0.8, edgecolors='black')
        
        # 连接最近点的线
        ax.plot([point1[0], point2[0]], [point1[1], point2[1]], [point1[2], point2[2]], 
                color=risk_color, linestyle='--', linewidth=2, label=f'Risk ({risk_level}): {min_distance:.2f}m')
    
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_zlabel('Altitude')
    ax.set_title('Flight Path Collision Analysis\nRed=Fatal Risk, Orange=Warning Risk')
    ax.legend()
    
    plt.tight_layout()
    plt.show()


def collision_analyse(fly_paths : list[dict]):
    """
    主规划入口
    分析飞行路径有没有碰撞风险
    """
    results = {
        "has_risk": False,
        "risk_paths": []
    }
    
    # 检查每对路径之间的距离
    for i in range(len(fly_paths)):
        for j in range(i + 1, len(fly_paths)):
            path1 = fly_paths[i]
            path2 = fly_paths[j]
            
            # 获取路径的几何形状
            line1 = path1["geometry"]
            line2 = path2["geometry"]
            
            # 计算最近距离和最近点
            min_distance, nearest_pt1, nearest_pt2 = find_nearest_points_between_lines(line1, line2)

            print(f"Path {i} and Path {j} min distance: {min_distance:.2f} m")
            
            # 根据距离确定风险等级
            risk_level = ""
            if min_distance <= 1000:
                risk_level = "fatal"
            elif min_distance <= 500:
                risk_level = "warn"
            else:
                risk_level = "safe"
                
            # 如果不是安全等级，则记录风险信息
            if risk_level != "safe":
                results["has_risk"] = True
                risk_path_info = {
                    "path1_index": i,
                    "path2_index": j,
                    "min_distance": min_distance,
                    "risk_level": risk_level,
                    "nearest_points": {
                        "point1": nearest_pt1,
                        "point2": nearest_pt2
                    }
                }

                results["risk_paths"].append(risk_path_info)
    
    return results


if __name__ == "__main__":
    fly_paths = get_fly_paths()
    print(f"加载 {len(fly_paths)} 条飞行路径")

    result = collision_analyse(fly_paths)
    print(result)
    
    # 可视化结果
    visualize_collision_analysis(fly_paths, result)