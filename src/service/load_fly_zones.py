from shapely import box
def get_nofly_zones():
    """
    定义禁飞区（示例）
    """
    zones = []

    # 示例 1：矩形硬禁飞区（无限高）
    zones.append({
        "geometry": box(
            120.0085, 30.2965,
            120.0105, 30.2985
        ),
        "z_min": 0,
        "z_max": 200,   # 覆盖整个 grid
        "buffer": 50.0,  # ✅ 新增：水平缓冲（米）
        "type": "nofly"
    })

    # 示例 2：限高禁飞区（低于 80m 禁飞）
    zones.append({
        "geometry": box(
            120.0090, 30.2925,
            120.0120, 30.2945
        ),
        "z_min": 0,
        "z_max": 80,
        "buffer": 30.0,
        "type": "nofly"
    })

    print(f"加载 {len(zones)} 个禁飞区")
    return zones
