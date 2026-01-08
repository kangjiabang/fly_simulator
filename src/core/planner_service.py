"""Planner service: encapsulate core planning logic used by the FastAPI endpoint.

Provides compute_flight_path(start, goal, scale, safety_dist) -> List[dict]
Each dict contains: {"lon": float, "lat": float, "alt": float, "grid": (x,y,z)}
"""
from typing import List, Tuple, Dict

from src.tool.BuildingManager import BuildingManager
from src.service.load_fly_zones import get_nofly_zones
from src.core.fly_path_final import plan_3d_grid_path
from src.others.fly_path_view_batch import grid_path_to_lonlat


def compute_flight_path(start: Tuple[float, float, float],
                        goal: Tuple[float, float, float],
                        scale: float = 0.5,
                        safety_dist: float = 5.0,
                        manager: BuildingManager = None) -> List[Dict]:
    """Compute flight path from start to goal.

    Args:
        start: (lon, lat, alt)
        goal: (lon, lat, alt)
        scale: grid scale (e.g. 0.5)
        safety_dist: safety distance in meters

    Returns:
        List of dicts with keys: lon, lat, alt, grid

    Raises:
        RuntimeError on failures with a descriptive message.
    """
    # Build environment (use provided manager if available to avoid re-loading)
    try:
        if manager is None:
            manager = BuildingManager()
        buildings = manager.get_corridor_buildings(start, goal)
    except Exception as e:
        raise RuntimeError(f"BuildingManager error: {e}")

    try:
        nfz = get_nofly_zones()
    except Exception as e:
        raise RuntimeError(f"get_nofly_zones error: {e}")

    # Run planner
    try:
        grid_path, projection = plan_3d_grid_path(start, goal, buildings, nfz, scale=scale, safety_dist=safety_dist)
    except Exception as e:
        raise RuntimeError(f"planner runtime error: {e}")

    if not grid_path:
        return []

    # Convert to lon/lat/alt
    try:
        lonlat_path = grid_path_to_lonlat(grid_path, projection, scale=scale)
    except Exception as e:
        raise RuntimeError(f"grid->lonlat conversion error: {e}")

    # Keep best effort by zipping shortest
    n = min(len(lonlat_path), len(grid_path))

    out: List[Dict] = []
    for (gx, gy, gz), (lon, lat, alt) in zip(grid_path[:n], lonlat_path[:n]):
        out.append({
            "lon": float(lon),
            "lat": float(lat),
            "alt": float(alt),
            "grid": (int(gx), int(gy), int(gz)),
        })

    return out
