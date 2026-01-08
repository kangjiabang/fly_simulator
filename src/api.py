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

# Reuse existing planning code
from src.core.planner_service import compute_flight_path
from src.tool.BuildingManager import BuildingManager


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
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"planner error: {e}")

    if not out_dicts:
        return PlanResponse(path=[], length=0)

    # Convert dicts into PathPoint models
    out = [PathPoint(**p) for p in out_dicts]

    return PlanResponse(path=out, length=len(out))


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
