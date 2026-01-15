from shapely import box, LineString


def get_fly_paths() -> list[dict]:
    """
    定义无人机飞行路线（示例）
    """
    paths = []

    # 示例 1：一条飞行路线
    paths.append({
        "geometry": LineString([
            (120.0078, 30.29949, 20),  # 起点
            (120.0090, 30.29500, 25),  # 中间点1
            (120.0100, 30.29200, 20),  # 中间点2
            (120.01117, 30.28994, 30), # 终点
        ]),
        "altitude": 30,  # 飞行高度
        "speed": 10.0,  # 飞行速度（米/秒）
        "type": "flight_path",
        "name": "Flight_Path_001",
        "path_id": "path_001"
    })

    # 示例 2：另一条飞行路线
    paths.append({
        "geometry": LineString([
            (120.01200, 30.29000, 25),  # 起点
            (120.01100, 30.29200, 30),  # 中间点1
            (120.01000, 30.29500, 35),  # 中间点2
            (120.00900, 30.29800, 40),  # 中间点3
            (120.00750, 30.30000, 35),  # 终点
        ]),
        "altitude": 35,  # 飞行高度
        "speed": 12.0,  # 飞行速度（米/秒）
        "type": "flight_path",
        "name": "Flight_Path_002",
        "path_id": "path_002"
    })

    paths.append({
        "geometry": LineString([
            (120.0080, 30.3000, 30),
            (120.0080, 30.2950, 30),
            (120.0080, 30.2900, 30),
        ]),
        "altitude": 30,
        "speed": 10.0,
        "type": "flight_path",
        "name": "Flight_Path_003",
        "path_id": "path_003"
    })

    # -------------------------
    # 航线 2：由西向东交叉（高度 32m，横切航线1）
    # 在 (120.0080, 30.2950) 附近与航线1在平面投影上相交
    paths.append({
        "geometry": LineString([
            (120.0060, 30.2950, 35),
            (120.0080, 30.2950, 36),
            (120.0100, 30.2950, 37),
        ]),
        "altitude": 32,
        "speed": 12.0,
        "type": "flight_path",
        "name": "Flight_Path_004",
        "path_id": "path_004"
    })

    # -------------------------
    # 航线 3：对角线飞行（高度 28m -> 35m，斜向爬升）
    # 从东南向西北穿过前两条航线构成的区域
    paths.append({
        "geometry": LineString([
            (120.0100, 30.2900, 28),
            (120.0085, 30.2940, 31.5),
            (120.0070, 30.2980, 35),
        ]),
        "altitude": 31,  # 标称高度
        "speed": 8.0,
        "type": "flight_path",
        "name": "Flight_Path_005",
        "path_id": "path_005"
    })
    print(f"加载 {len(paths)} 条飞行路线")
    return paths