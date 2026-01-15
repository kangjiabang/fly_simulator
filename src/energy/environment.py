# ======================
# 能耗计算核心
# ======================
from src.energy.drone_model import DroneModel, Environment, Wind

GRAVITY = 9.81


class EnergyModel:

    @staticmethod
    def calc_hover_power(drone: DroneModel) -> float:
        return drone.hover_power_w

    @staticmethod
    def calc_move_power(drone: DroneModel, speed: float) -> float:
        speed = min(speed, drone.max_speed)
        return drone.speed_power_coeff * speed

    @staticmethod
    def calc_payload_power(drone: DroneModel) -> float:
        return drone.payload_power_coeff * drone.payload_kg

    @staticmethod
    def calc_wind_power(drone: DroneModel, wind: Wind, speed: float) -> float:
        # 简化：逆风功耗增加，顺风减小
        return drone.wind_power_coeff * wind.speed * speed

    @staticmethod
    def calc_climb_power(drone: DroneModel, climb_rate: float) -> float:
        climb_rate = max(0.0, climb_rate)
        return drone.weight_kg * GRAVITY * climb_rate

    @staticmethod
    def calc_total_power(
        drone: DroneModel,
        env: Environment,
        speed: float,
        climb_rate: float = 0.0
    ) -> float:
        p = 0.0
        p += EnergyModel.calc_hover_power(drone)
        p += EnergyModel.calc_move_power(drone, speed)
        p += EnergyModel.calc_payload_power(drone)
        p += EnergyModel.calc_wind_power(drone, env.wind, speed)
        p += EnergyModel.calc_climb_power(drone, climb_rate)
        return p
