from shapely import box


def get_surrounding_fly_paths(drone_id: str) -> list[dict]:
    """
    定义无人机飞行路线（示例）
    """
    paths = [
        {
            "id": "path_001",
            "path": [
                { "lon": 120.0078, "lat": 30.29949, "height": 20, "time": "2025-02-01 10:05:00" },
                { "lon": 120.0090, "lat": 30.29500, "height": 25, "time": "2025-02-01 10:05:10" },
                { "lon": 120.0100, "lat": 30.29200, "height": 20, "time": "2025-02-01 10:05:18" },
                { "lon": 120.01117, "lat": 30.28994, "height": 30, "time": "2025-02-01 10:05:26" }
            ],
            "name": "path_001",
            "isMaster": False
        },
        {
            "id": "path_002",
            "path": [
                { "lon": 120.0120, "lat": 30.2900, "height": 25, "time": "2025-02-01 10:06:00" },
                { "lon": 120.0110, "lat": 30.2920, "height": 30, "time": "2025-02-01 10:06:08" },
                { "lon": 120.0100, "lat": 30.2950, "height": 35, "time": "2025-02-01 10:06:16" },
                { "lon": 120.0090, "lat": 30.2980, "height": 40, "time": "2025-02-01 10:06:24" },
                { "lon": 120.0075, "lat": 30.3000, "height": 35, "time": "2025-02-01 10:06:32" }
            ],
            "name": "path_002",
            "isMaster": False
        },
        {
            "id": "path_003",
            "path": [
                { "lon": 120.0080, "lat": 30.3000, "height": 30, "time": "2025-02-01 10:07:00" },
                { "lon": 120.0080, "lat": 30.2950, "height": 30, "time": "2025-02-01 10:07:12" },
                { "lon": 120.0080, "lat": 30.2900, "height": 30, "time": "2025-02-01 10:07:24" }
            ],
            "name": "path_003",
            "isMaster": False
        },
        {
            "id": "path_004",
            "path": [
                { "lon": 120.0060, "lat": 30.2950, "height": 35, "time": "2025-02-01 10:08:00" },
                { "lon": 120.0080, "lat": 30.2950, "height": 36, "time": "2025-02-01 10:08:06" },
                { "lon": 120.0100, "lat": 30.2950, "height": 37, "time": "2025-02-01 10:08:12" }
            ],
            "name": "path_004",
            "isMaster": False
        },
        {
            "id": "C3",
            "path": [
                { "lon": 119.98, "lat": 30.268, "height": 50, "time": "2025-02-01 09:59:58" },
                { "lon": 119.9924, "lat": 30.272, "height": 60, "time": "2025-02-01 10:00:05" },
                { "lon": 119.9944, "lat": 30.2745, "height": 70, "time": "2025-02-01 10:00:08" },
                { "lon": 119.997, "lat": 30.2764, "height": 80, "time": "2025-02-01 10:00:15" },
                { "lon": 119.9983, "lat": 30.2782, "height": 90, "time": "2025-02-01 10:00:20" },
                { "lon": 120.001, "lat": 30.282, "height": 100, "time": "2025-02-01 10:00:30" }
            ],
            "name": "C3",
            "isMaster": True
        }
    ]
    
    # Sort so that if a drone_id matches, it's at the front or handled specifically if needed
    # But for now, just return them. 
    # The original implementation had some prints and returned all paths.
    
    # If the user wants specific drone_id handling, we can add it.
    # In api.py: fly_paths = get_surrounding_fly_paths(drone_id)
    # analysis_result = collision_analyse_target(fly_paths[0], fly_paths[1:])
    # This implies the first path should be the target path.
    
    # Let's ensure the matching drone_id is at index 0.
    matched = [p for p in paths if p["id"] == drone_id]
    others = [p for p in paths if p["id"] != drone_id]
    
    result = matched + others
    print(f"加载 {len(result)} 条飞行路线")
    return result


def get_fly_paths() -> list[dict]:
    """
    定义无人机飞行路线（示例）
    """
    # Simply reuse surrounding paths with a dummy ID or just return the whole list
    return get_surrounding_fly_paths("C3")