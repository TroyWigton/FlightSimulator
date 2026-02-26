import math

from flightsim.config import AircraftParameters
from flightsim.controller import FlightController, PilotTargets
from flightsim.dynamics import AircraftState, FixedWingDynamics


def test_idle_power_sustained_pitch_up_enters_stall_and_descends():
    params = AircraftParameters()
    dynamics = FixedWingDynamics(params)
    controller = FlightController(rotation_speed_m_s=params.rotation_speed_m_s)
    controller.set_performance_mode(True)

    dt = 1.0 / 60.0
    state = AircraftState(z_d=-200.0, u=45.0, theta=math.radians(5.0), throttle_actual=0.0)

    min_speed = state.u
    min_climb_rate = 1e9
    max_theta = state.theta
    final_theta = state.theta

    for _ in range(60 * 18):
        targets = PilotTargets(bank_rad=0.0, pitch_rad=math.radians(25.0), throttle=0.0)
        controls = controller.update(targets, state, dt)
        state = dynamics.step(state, controls, dt)

        min_speed = min(min_speed, state.u)
        vertical_speed_up = -(state.u * math.sin(state.theta) - state.w * math.cos(state.theta))
        min_climb_rate = min(min_climb_rate, vertical_speed_up)
        max_theta = max(max_theta, state.theta)
        final_theta = state.theta

    assert min_speed < 30.0
    assert min_climb_rate < -2.0
    assert final_theta < max_theta - math.radians(4.0)
