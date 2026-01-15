from dataclasses import dataclass
from typing import List, Tuple
import math

# ======================
# 基础数据结构
# ======================

@dataclass
class Battery:
    capacity_wh: float          # 标称容量
    usable_ratio: float = 0.8   # 安全放电比例

    @property
    def usable_energy(self) -> float:
        return self.capacity_wh * self.usable_ratio


@dataclass
class DroneModel:
    name: str
    weight_kg: float
    payload_kg: float

    # 功率参数（经验值 / 标定值）
    hover_power_w: float
    speed_power_coeff: float    # k_v
    payload_power_coeff: float  # k_p
    wind_power_coeff: float     # k_w

    max_speed: float            # m/s
    max_climb_rate: float       # m/s


@dataclass
class Wind:
    speed: float        # m/s
    direction_deg: float  # 风向（用于扩展）


@dataclass
class Environment:
    wind: Wind
    temperature_c: float = 20.0
