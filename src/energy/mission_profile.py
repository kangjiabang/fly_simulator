from src.energy.energy_model import compute_energy_for_path


def evaluate_mission(
    path,
    drone,
    battery,
    env,
    cruise_speed
):
    result = compute_energy_for_path(
        path=path,
        drone=drone,
        battery=battery,
        env=env,
        cruise_speed=cruise_speed
    )

    risk = "LOW"
    if result["battery_used_ratio"] > 0.8:
        risk = "HIGH"
    elif result["battery_used_ratio"] > 0.6:
        risk = "MEDIUM"

    return {
        **result,
        "risk_level": risk,
        "approval_suggestion": "PASS" if result["safe"] else "REJECT"
    }
