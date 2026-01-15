# ======================
# 路径能耗积分
# ======================
import math
from typing import List, Tuple

from src.energy.drone_model import DroneModel, Battery, Environment
from src.energy.environment import EnergyModel


def compute_energy_for_path(
    path: List[Tuple[float, float, float]],
    drone: DroneModel,
    battery: Battery,
    env: Environment,
    cruise_speed: float,
    dt: float = 1.0  # 秒
) -> dict:

    total_energy_wh = 0.0
    total_time = 0.0

    for i in range(1, len(path)):
        x1, y1, z1 = path[i - 1]
        x2, y2, z2 = path[i]

        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1

        horizontal_dist = math.hypot(dx, dy)
        climb_rate = dz / dt

        power_w = EnergyModel.calc_total_power(
            drone=drone,
            env=env,
            speed=cruise_speed,
            climb_rate=climb_rate
        )

        energy_wh = power_w * dt / 3600.0
        total_energy_wh += energy_wh
        total_time += dt

    return {
        "total_time_s": total_time,
        "energy_used_wh": total_energy_wh,
        "battery_used_ratio": total_energy_wh / battery.capacity_wh,
        "safe": total_energy_wh <= battery.usable_energy
    }
