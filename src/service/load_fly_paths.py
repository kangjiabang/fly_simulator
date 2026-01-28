from shapely import box




def get_fly_path_including_surrounding(drone_id: str) -> list[dict]:
    """
      获取无人机飞行路线，包括周围的飞行路线
    该函数调用 get_surrounding_fly_paths 和 get_current_fly_path 函数，
    并将当前无人机的飞行路线添加到周围飞行路线列表中返回。
    参数:
        drone_id (str): 无人机的唯一标识符。
    返回:
        list[dict]: 包含周围飞行路线和当前无人机飞行路线
    """

    surrounding_fly_paths = get_surrounding_fly_paths(drone_id)
    current_fly_path = get_current_fly_path(drone_id)
    surrounding_fly_paths.append(current_fly_path)
    return surrounding_fly_paths

def get_surrounding_fly_paths(drone_id: str) -> list[dict]:
    """
    周围无人机飞行路线（示例）
    这里定义了多个无人机的飞行路径数据，每个路径包含经度、纬度、高度和时间等信息。
    参数:
        drone_id (str): 无人机的唯一标识符。
    返回:
        list[dict]: 包含多个无人机飞行路线的列表。
    """
    paths = [
        {
            "id": 'A1',
            "color": 'yellow',
            "path": [
                {"lon": 119.99, "lat": 30.27, "height": 50, "time": '2025-02-01 10:00:00'},
                {"lon": 119.9924, "lat": 30.272, "height": 60, "time": '2025-02-01 10:00:05'},
                {"lon": 119.994, "lat": 30.274, "height": 70, "time": '2025-02-01 10:00:10'},
                {"lon": 119.997, "lat": 30.276, "height": 80, "time": '2025-02-01 10:00:15'},
                {"lon": 119.998, "lat": 30.278, "height": 90, "time": '2025-02-01 10:00:20'},
                {"lon": 120.0, "lat": 30.28, "height": 100, "time": '2025-02-01 10:00:25'},
            ],
            "name": "A1",
            "isMaster": False,
        },
        {
            "id": 'B2',
            "color": 'cyan',
            "path": [
                {"lon": 119.994, "lat": 30.273, "height": 50, "time": '2025-02-01 10:00:02'},
                {"lon": 119.9925, "lat": 30.272, "height": 60, "time": '2025-02-01 10:00:07'},
                {"lon": 119.995, "lat": 30.274, "height": 70, "time": '2025-02-01 10:00:09'},
                {"lon": 119.9975, "lat": 30.276, "height": 80, "time": '2025-02-01 10:00:12'},
                {"lon": 119.9988, "lat": 30.278, "height": 90, "time": '2025-02-01 10:00:20'},
                {"lon": 120.02, "lat": 30.283, "height": 100, "time": '2025-02-01 10:00:27'},
            ],
            "name": "B2",
            "isMaster": False,
        }
    ]

    # paths = [
    #     {
    #         "id": "path_001",
    #         "path": [
    #             { "lon": 120.0078, "lat": 30.29949, "height": 20, "time": "2025-02-01 10:05:00" },
    #             { "lon": 120.0090, "lat": 30.29500, "height": 25, "time": "2025-02-01 10:05:10" },
    #             { "lon": 120.0100, "lat": 30.29200, "height": 20, "time": "2025-02-01 10:05:18" },
    #             { "lon": 120.01117, "lat": 30.28994, "height": 30, "time": "2025-02-01 10:05:26" }
    #         ],
    #         "name": "path_001",
    #         "isMaster": False
    #     },
    #     {
    #         "id": "path_002",
    #         "path": [
    #             { "lon": 120.0120, "lat": 30.2900, "height": 25, "time": "2025-02-01 10:06:00" },
    #             { "lon": 120.0110, "lat": 30.2920, "height": 30, "time": "2025-02-01 10:06:08" },
    #             { "lon": 120.0100, "lat": 30.2950, "height": 35, "time": "2025-02-01 10:06:16" },
    #             { "lon": 120.0090, "lat": 30.2980, "height": 40, "time": "2025-02-01 10:06:24" },
    #             { "lon": 120.0075, "lat": 30.3000, "height": 35, "time": "2025-02-01 10:06:32" }
    #         ],
    #         "name": "path_002",
    #         "isMaster": False
    #     },
    #     {
    #         "id": "path_003",
    #         "path": [
    #             { "lon": 120.0080, "lat": 30.3000, "height": 30, "time": "2025-02-01 10:07:00" },
    #             { "lon": 120.0080, "lat": 30.2950, "height": 30, "time": "2025-02-01 10:07:12" },
    #             { "lon": 120.0080, "lat": 30.2900, "height": 30, "time": "2025-02-01 10:07:24" }
    #         ],
    #         "name": "path_003",
    #         "isMaster": False
    #     },
    #     {
    #         "id": "path_004",
    #         "path": [
    #             { "lon": 120.0060, "lat": 30.2950, "height": 35, "time": "2025-02-01 10:08:00" },
    #             { "lon": 120.0080, "lat": 30.2950, "height": 36, "time": "2025-02-01 10:08:06" },
    #             { "lon": 120.0100, "lat": 30.2950, "height": 37, "time": "2025-02-01 10:08:12" }
    #         ],
    #         "name": "path_004",
    #         "isMaster": False
    #     },
    #     {
    #         "id": "C3",
    #         "path": [
    #             { "lon": 119.98, "lat": 30.268, "height": 50, "time": "2025-02-01 09:59:58" },
    #             { "lon": 119.9924, "lat": 30.272, "height": 60, "time": "2025-02-01 10:00:05" },
    #             { "lon": 119.9944, "lat": 30.2745, "height": 70, "time": "2025-02-01 10:00:08" },
    #             { "lon": 119.997, "lat": 30.2764, "height": 80, "time": "2025-02-01 10:00:15" },
    #             { "lon": 119.9983, "lat": 30.2782, "height": 90, "time": "2025-02-01 10:00:20" },
    #             { "lon": 120.001, "lat": 30.282, "height": 100, "time": "2025-02-01 10:00:30" }
    #         ],
    #         "name": "C3",
    #         "isMaster": True
    #     }
    # ]

    # Let's ensure the matching drone_id is at index 0.
    matched = [p for p in paths if p["id"] == drone_id]
    others = [p for p in paths if p["id"] != drone_id]
    
    result = matched + others
    print(f"加载 {len(result)} 条飞行路线")
    return result


def get_fly_paths(drone_id: str = None) -> list[dict]:
    """
    无人机飞行路线
    """
    # Simply reuse surrounding paths with a dummy ID or just return the whole list
    return get_fly_path_including_surrounding(drone_id)


def get_current_fly_path(drone_id: str) -> dict:
    return {
        "id": 'C3',
        "path": [
            {"lon": 119.98, "lat": 30.268, "height": 50, "time": '2025-02-01 09:59:58'},
            {"lon": 119.9924, "lat": 30.272, "height": 60, "time": '2025-02-01 10:00:05'},
            {"lon": 119.9944, "lat": 30.2745, "height": 70, "time": '2025-02-01 10:00:08'},
            {"lon": 119.997, "lat": 30.2764, "height": 80, "time": '2025-02-01 10:00:15'},
            {"lon": 119.9983, "lat": 30.2782, "height": 90, "time": '2025-02-01 10:00:20'},
            {"lon": 120.001, "lat": 30.282, "height": 100, "time": '2025-02-01 10:00:30'},
        ],
        "name": "C3",
        "isMaster": True,
    }