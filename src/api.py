import json
from fastapi import FastAPI, HTTPException
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
from src.core.fly_path_collision_analyse import collision_analyse
# Reuse existing planning code
from src.core.planner_service import compute_flight_path
from src.tool.BuildingManager import BuildingManager
from src.service.load_nofly_zones import get_nofly_zones




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
