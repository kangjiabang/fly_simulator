from shapely import box
def get_nofly_zones():
    """
    定义禁飞区（示例）
    """
    zones = []

    # 示例 1：矩形硬禁飞区（无限高）
    zones.append({
        "geometry": box(
            119.996877, 30.281884,
            119.999877, 30.291884
        ),
        "z_min": 0,
        "z_max": 60,   # 覆盖整个 grid
        "buffer": 30.0,  # ✅ 新增：水平缓冲（米）
        "type": "nofly"
    })

    # 示例 2：限高禁飞区（低于 80m 禁飞）
    zones.append({
        "geometry": box(
            120.000736, 30.287593,
            120.020736, 30.288593
        ),
        "z_min": 0,
        "z_max": 80,
        "buffer": 30.0,
        "type": "nofly"
    })

    print(f"加载 {len(zones)} 个禁飞区")
    return zones
