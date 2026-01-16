import json
from fastapi import FastAPI, HTTPException, Query
from pydantic import BaseModel
from typing import Tuple, List, Optional
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

# Ensure project root is on sys.path so `import src.*` works whether
# running as `python src/api.py` or via `uvicorn src.api:app`.
import os
import sys
proj_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if proj_root not in sys.path:
    sys.path.insert(0, proj_root)

from src.service.load_fly_paths import get_fly_paths
from src.service.load_fly_paths import get_surrounding_fly_paths
from src.core.fly_path_collision_analyse import collision_analyse,collision_analyse_target, analyze_nofly_zone_risk, visualize_nofly_zone_risk
# Reuse existing planning code
from src.core.planner_service import compute_flight_path
from src.tool.BuildingManager import BuildingManager
from src.service.load_nofly_zones import get_nofly_zones
from src.core.fly_risk_analyse import start_analyze_drone_risk




@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan handler to preload BuildingManager on startup and attach to app.state."""
    try:
        try:
            manager = BuildingManager()
            count = len(manager.all_buildings) if hasattr(manager, 'all_buildings') else None
            print(f"Startup: Loaded building manager, buildings={count}")
            app.state.building_manager = manager
        except Exception as e:
            print(f"Startup: failed to load BuildingManager: {e}")
            app.state.building_manager = None
        yield
    finally:
        # Optionally cleanup resources here
        bm = getattr(app.state, 'building_manager', None)
        if bm is not None:
            # If BuildingManager had close() or similar, call it here.
            pass

app = FastAPI(title="FlySimulator Planner API", lifespan=lifespan)

# Allow CORS from all origins. When using a wildcard origin, credentials must be disabled.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # allow all origins
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


class Point3(BaseModel):
    lon: float
    lat: float
    alt: float


class PlanRequest(BaseModel):
    start: Point3
    goal: Point3
    scale: Optional[float] = 0.5
    safety_dist: Optional[float] = 5.0


class PathPoint(BaseModel):
    lon: float
    lat: float
    alt: float
    grid: Tuple[int, int, int]


class PlanResponse(BaseModel):
    path: List[PathPoint]
    length: int


@app.get("/", tags=["health"])
def root():
    return {"status": "ok", "message": "FlySimulator Planner API"}


@app.post("/plan", response_model=PlanResponse, tags=["planner"])
def plan(request: PlanRequest):
    """Plan a 3D grid path and return lon/lat/alt points.

    Request JSON:
      {
        "start": {"lon": 120.0078, "lat": 30.29949, "alt": 20},
        "goal": {"lon": 120.01117, "lat": 30.28994, "alt": 30},
        "scale": 0.5
      }
    """
    start = (request.start.lon, request.start.lat, request.start.alt)
    goal = (request.goal.lon, request.goal.lat, request.goal.alt)
    scale = request.scale
    safety_dist = request.safety_dist

    # Delegate core planning logic to planner_service.compute_flight_path
    try:
        manager = getattr(app.state, "building_manager", None)
        out_dicts = compute_flight_path(start, goal, scale=scale, safety_dist=safety_dist, manager=manager)
    except RuntimeError as e:
        # 检查是否是客户端错误（起点/终点被障碍物占据）
        error_msg = str(e)
        if "起点或终点被障碍物占据" in error_msg:
            raise HTTPException(status_code=400, detail=f"{error_msg}")
        else:
            # 其他运行时错误仍视为服务器错误
            raise HTTPException(status_code=500, detail=f"planner error: {e}")
    except Exception as e:
        # 未预期的异常视为服务器错误
        raise HTTPException(status_code=500, detail=f"planner error: {e}")

    if not out_dicts:
        return PlanResponse(path=[], length=0)

    # Convert dicts into PathPoint models
    out = [PathPoint(**p) for p in out_dicts]

    return PlanResponse(path=out, length=len(out))


# Custom encoder to handle Shapely geometries
from shapely.geometry.base import BaseGeometry

class ShapelyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, BaseGeometry):
            return {
                "type": obj.geom_type,
                "coordinates": obj.__geo_interface__["coordinates"]
            }
        return super().default(obj)


@app.get("/nofly-zones", tags=["zones"])
def get_nofly_zones_endpoint():
    """获取禁飞区信息
    
    Returns:
        禁飞区列表，每个禁飞区包含几何信息、最小高度、最大高度、缓冲距离和类型
    """
    try:
        zones = get_nofly_zones()
        # Convert geometries to serializable format
        serializable_zones = []
        for zone in zones:
            serializable_zone = zone.copy()
            if "geometry" in serializable_zone:
                geom = serializable_zone["geometry"]
                serializable_zone["geometry"] = {
                    "type": geom.geom_type,
                    "coordinates": geom.__geo_interface__["coordinates"]
                }
            serializable_zones.append(serializable_zone)
        return {"zones": serializable_zones}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取禁飞区失败: {e}")


@app.get("/fly-paths", tags=["paths"])
def get_fly_paths_endpoint():
    """获取飞行路线信息
    
    Returns:
        飞行路线列表，每条路线包含几何信息、飞行高度、速度和类型
    """
    try:
        paths = get_fly_paths()
        # Convert geometries to serializable format
        serializable_paths = []
        for path in paths:
            serializable_path = path.copy()
            if "geometry" in serializable_path:
                geom = serializable_path["geometry"]
                serializable_path["geometry"] = {
                    "type": geom.geom_type,
                    "coordinates": geom.__geo_interface__["coordinates"]
                }
            serializable_paths.append(serializable_path)
        return {"paths": serializable_paths}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取飞行路线失败: {e}")


@app.post("/collision-analysis", tags=["analysis"])
def post_collision_analysis():
    """分析飞行路线碰撞风险
    
    Returns:
        碰撞分析结果，包含风险等级、风险路线和最近点位信息
    """
    try:
        fly_paths = get_fly_paths()
        analysis_result = collision_analyse(fly_paths)
        
        return analysis_result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"碰撞分析失败: {e}")


from src.model.fly_path_info import FlyPathInfo


class FlyPathResponse(BaseModel):
    flyPathInfos: List[FlyPathInfo]

class FlyPlanPathResponse(BaseModel):
    flyPathInfos: List[FlyPathInfo]


@app.get("/fly-paths-by-drone", response_model=FlyPathResponse, tags=["paths"])
def get_fly_path_by_drone(drone_id: str = Query(..., description="无人机ID"),include_surrounding: bool = Query( False,description="是否包含周围航线（true 时返回所有航线）")):
    """根据无人机ID获取飞行航线信息
    
    Args:
        drone_id: 无人机ID
        
    Returns:
        包含航线信息的响应，格式为：
        {
            "flyPathInfos": [
                {
                    "id": 'A1',
                    "color": 'yellow',
                    "path": [
                        { "lon": 119.99, "lat": 30.27, "height": 50, "time": '2025-02-01 10:00:00' },
                        { "lon": 119.9924, "lat": 30.272, "height": 60, "time": '2025-02-01 10:00:05' },
                        // 更多路径点...
                    ],
                    "isMaster": false,
                }
            ]
        }
        :param drone_id:
        :param include_surrounding:
    """
    try:
        # 根据drone_id返回固定的航线数据
        fly_path_infos = []
        
        # 定义固定的数据集
        fixed_paths = [
            {
                "id": 'A1',
                "color": 'yellow',
                "path": [
                    { "lon": 119.99, "lat": 30.27, "height": 50, "time": '2025-02-01 10:00:00' },
                    { "lon": 119.9924, "lat": 30.272, "height": 60, "time": '2025-02-01 10:00:05' },
                    { "lon": 119.994, "lat": 30.274, "height": 70, "time": '2025-02-01 10:00:10' },
                    { "lon": 119.997, "lat": 30.276, "height": 80, "time": '2025-02-01 10:00:15' },
                    { "lon": 119.998, "lat": 30.278, "height": 90, "time": '2025-02-01 10:00:20' },
                    { "lon": 120.0, "lat": 30.28, "height": 100, "time": '2025-02-01 10:00:25' },
                ],
                "name": "A1",
                "isMaster": False,
            },
            {
                "id": 'B2',
                "color": 'cyan',
                "path": [
                    { "lon": 119.994, "lat": 30.273, "height": 50, "time": '2025-02-01 10:00:02' },
                    { "lon": 119.9925, "lat": 30.272, "height": 60, "time": '2025-02-01 10:00:07' },
                    { "lon": 119.995, "lat": 30.274, "height": 70, "time": '2025-02-01 10:00:09' },
                    { "lon": 119.9975, "lat": 30.276, "height": 80, "time": '2025-02-01 10:00:12' },
                    { "lon": 119.9988, "lat": 30.278, "height": 90, "time": '2025-02-01 10:00:20' },
                    { "lon": 120.02, "lat": 30.283, "height": 100, "time": '2025-02-01 10:00:27' },
                ],
                "name": "B2",
                "isMaster": False,
            },
            {
                "id": 'C3',
                "path": [
                    { "lon": 119.98, "lat": 30.268, "height": 50, "time": '2025-02-01 09:59:58' },
                    { "lon": 119.9924, "lat": 30.272, "height": 60, "time": '2025-02-01 10:00:05' },
                    { "lon": 119.9944, "lat": 30.2745, "height": 70, "time": '2025-02-01 10:00:08' },
                    { "lon": 119.997, "lat": 30.2764, "height": 80, "time": '2025-02-01 10:00:15' },
                    { "lon": 119.9983, "lat": 30.2782, "height": 90, "time": '2025-02-01 10:00:20' },
                    { "lon": 120.001, "lat": 30.282, "height": 100, "time": '2025-02-01 10:00:30' },
                ],
                "name": "C3",
                "isMaster": True,
            }
        ]

        # 是否返回全部航线
        if include_surrounding:
            fly_path_infos = [FlyPathInfo(**path_data) for path_data in fixed_paths]
        else:
            for path_data in fixed_paths:
                if path_data["id"] == drone_id:
                    fly_path_infos = [FlyPathInfo(**path_data)]
                    break

        return FlyPathResponse(flyPathInfos=fly_path_infos)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取飞行航线失败: {e}")


@app.get("/fly-plan-paths-by-drone", response_model=FlyPlanPathResponse, tags=["path"])
def get_fly_plan_path_by_drone(drone_id: str = Query(..., description="无人机ID")):
    """根据无人机ID获取计划飞行的航线信息

    Args:
        drone_id: 无人机ID

    Returns:
        包含航线信息的响应，格式为：
        {
            "flyPathInfos": [
                {
                    "id": 'A1',
                    "color": 'yellow',
                    "path": [
                        { "lon": 119.99, "lat": 30.27, "height": 50, "time": '2025-02-01 10:00:00' },
                        { "lon": 119.9924, "lat": 30.272, "height": 60, "time": '2025-02-01 10:00:05' },
                        // 更多路径点...
                    ],
                    "isMaster": false,
                }
            ]
        }
    """
    try:
        # 根据drone_id返回固定的航线数据
        # 定义固定的数据集
        fixed_path = [
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
            "color": 'yellow',
            "name": "C3",
            "isMaster": True
            },

            {
            "id": "path_001",
            "path": [
                { "lon": 120.0078, "lat": 30.29949, "height": 20, "time": "2025-02-01 10:05:00" },
                { "lon": 120.0090, "lat": 30.29500, "height": 25, "time": "2025-02-01 10:05:10" },
                { "lon": 120.0100, "lat": 30.29200, "height": 20, "time": "2025-02-01 10:05:18" },
                { "lon": 120.01117, "lat": 30.28994, "height": 30, "time": "2025-02-01 10:05:26" }
            ],
            "color": "#00ff00",
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
            "color": "#00ccff",
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
            "color": "#ffaa00",
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
            "color": "#ff4444",
            "name": "path_004",
            "isMaster": False
            },

            {
            "id": "path_005",
            "path": [
                { "lon": 120.0100, "lat": 30.2900, "height": 28, "time": "2025-02-01 10:09:00" },
                { "lon": 120.0085, "lat": 30.2940, "height": 31.5, "time": "2025-02-01 10:09:10" },
                { "lon": 120.0070, "lat": 30.2980, "height": 35, "time": "2025-02-01 10:09:20" }
            ],
            "color": "#bb66ff",
            "name": "path_005",
            "isMaster": False
            }
    ]
        return FlyPlanPathResponse(flyPathInfos=fixed_path)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取计划飞行航线失败: {e}")


@app.post("/drone-nofly-risk-analysis", tags=["analysis"])
def analyze_drone_risk(
    drone_id: str = Query(..., description="无人机ID"),
    show_pic: bool = Query(False, description="是否生成/显示风险分析图片")
):
    """
    分析无人机飞行风险：调用飞行轨迹和禁飞区接口，计算风险并画图。
    """
    try:
        final_result = {}
        path_zone_risk_result = start_analyze_drone_risk(drone_id, show_pic=show_pic)
        
        fly_paths = get_surrounding_fly_paths(drone_id)
        final_result['path_zone_risk'] = path_zone_risk_result

        analysis_result = collision_analyse_target(fly_paths[0],fly_paths[1:])

        final_result['fly_path_risk'] = analysis_result

        return final_result
        
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"风险分析失败: {e}")


if __name__ == "__main__":
    # Run standalone for quick testing: `python src/api.py`
    import uvicorn

    # Allow controlling reload via environment variable UVICORN_RELOAD (default: true)
    reload_env = os.getenv("UVICORN_RELOAD", "1")
    reload_flag = reload_env not in ("0", "false", "False")

    # Allow configuring port via UVICORN_PORT to avoid address conflicts
    port = int(os.getenv("UVICORN_PORT", "8000"))
    # NOTE: programmatic reload will spawn a subprocess similar to `uvicorn --reload`.
    uvicorn.run("src.api:app", host="0.0.0.0", port=port, log_level="info", reload=reload_flag)
