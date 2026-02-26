from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import dataclass

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from flightsim.config import AircraftParameters
from flightsim.controller import FlightController, PilotTargets
from flightsim.dynamics import AircraftState, FixedWingDynamics


@dataclass
class FlightMetrics:
    takeoff_time_s: float | None
    max_agl_m: float
    final_agl_m: float
    max_speed_m_s: float
    max_abs_q_deg_s: float
    elevator_std_deg: float
    elevator_zero_cross_rate_hz: float


def evaluate_profile(
    duration_s: float = 20.0,
    dt: float = 1.0 / 60.0,
    throttle: float = 1.0,
    pre_rotate_pitch_deg: float = 2.0,
    post_rotate_pitch_deg: float = 18.0,
    rotate_after_s: float = 2.0,
    performance_mode: bool = True,
) -> FlightMetrics:
    params = AircraftParameters()
    dynamics = FixedWingDynamics(params)
    controller = FlightController(rotation_speed_m_s=params.rotation_speed_m_s)
    controller.set_performance_mode(performance_mode)

    state = AircraftState(z_d=0.0, u=0.0, theta=0.0)

    max_agl = 0.0
    max_speed = 0.0
    max_abs_q = 0.0
    takeoff_time = None

    elevator_samples: list[float] = []
    elevator_sign_changes = 0
    prev_sign = 0

    total_steps = int(duration_s / dt)
    for i in range(total_steps):
        t = i * dt

        pitch_deg = pre_rotate_pitch_deg if t < rotate_after_s else post_rotate_pitch_deg
        targets = PilotTargets(bank_rad=0.0, pitch_rad=math.radians(pitch_deg), throttle=throttle)

        controls = controller.update(targets, state, dt)
        state = dynamics.step(state, controls, dt)

        agl = max(0.0, -state.z_d)
        max_agl = max(max_agl, agl)
        max_speed = max(max_speed, state.u)
        max_abs_q = max(max_abs_q, abs(math.degrees(state.q)))

        if takeoff_time is None and agl > 1.0:
            takeoff_time = t

        # Evaluate oscillation after leaving runway phase and once speed is meaningful.
        if state.u > 20.0:
            elev_deg = math.degrees(controls.elevator)
            elevator_samples.append(elev_deg)
            sign = 1 if elev_deg > 0.2 else (-1 if elev_deg < -0.2 else 0)
            if prev_sign != 0 and sign != 0 and sign != prev_sign:
                elevator_sign_changes += 1
            if sign != 0:
                prev_sign = sign

    if elevator_samples:
        mean = sum(elevator_samples) / len(elevator_samples)
        var = sum((x - mean) ** 2 for x in elevator_samples) / len(elevator_samples)
        std = math.sqrt(var)
        zero_cross_rate = elevator_sign_changes / max(1e-6, duration_s)
    else:
        std = 0.0
        zero_cross_rate = 0.0

    return FlightMetrics(
        takeoff_time_s=takeoff_time,
        max_agl_m=max_agl,
        final_agl_m=max(0.0, -state.z_d),
        max_speed_m_s=max_speed,
        max_abs_q_deg_s=max_abs_q,
        elevator_std_deg=std,
        elevator_zero_cross_rate_hz=zero_cross_rate,
    )


if __name__ == "__main__":
    metrics = evaluate_profile()
    print(metrics)
