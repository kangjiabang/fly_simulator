from src.energy.drone_model import DroneModel, Battery, Environment, Wind
from src.energy.mission_profile import evaluate_mission

if __name__ == "__main__":
    drone = DroneModel(
        name="Quad-X",
        weight_kg=3.5,
        payload_kg=0.8,
        hover_power_w=350,
        speed_power_coeff=15,
        payload_power_coeff=40,
        wind_power_coeff=2.0,
        max_speed=15,
        max_climb_rate=3
    )

    battery = Battery(capacity_wh=300)

    env = Environment(
        wind=Wind(speed=3.0, direction_deg=180)
    )

    # 模拟一条 3D 路径
    path = [(0, 0, 0)]
    for i in range(1, 200):
        path.append((i * 2, 0, min(i * 0.2, 20)))

    result = evaluate_mission(
        path=path,
        drone=drone,
        battery=battery,
        env=env,
        cruise_speed=8.0
    )

    print(result)
