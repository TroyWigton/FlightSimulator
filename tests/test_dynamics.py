import math

from flightsim.config import AircraftParameters
from flightsim.controller import FlightController, PilotTargets
from flightsim.dynamics import AircraftState, ControlInputs, FixedWingDynamics


def test_ground_friction_slows_aircraft_without_thrust():
    params = AircraftParameters()
    dyn = FixedWingDynamics(params)
    state = AircraftState(z_d=0.0, u=20.0, throttle_actual=0.0)

    for _ in range(200):
        state = dyn.step(state, ControlInputs(throttle_command=0.0), 0.01)

    assert state.u < 20.2


def test_throttle_servo_tracks_target():
    params = AircraftParameters()
    dyn = FixedWingDynamics(params)
    state = AircraftState(z_d=0.0, u=0.0, throttle_actual=0.0)

    for _ in range(200):
        state = dyn.step(state, ControlInputs(throttle_command=0.8), 0.01)

    assert abs(state.throttle_actual - 0.8) < 0.05


def test_takeoff_roll_reaches_rotation_speed():
    params = AircraftParameters()
    dyn = FixedWingDynamics(params)
    ctl = FlightController()
    targets = PilotTargets(bank_rad=0.0, pitch_rad=math.radians(8.0), throttle=1.0)

    state = AircraftState(z_d=0.0, u=0.0, theta=0.0)

    dt = 1.0 / 60.0
    max_speed = state.u
    for _ in range(60 * 20):
        controls = ctl.update(targets, state, dt)
        state = dyn.step(state, controls, dt)
        max_speed = max(max_speed, state.u)

    assert max_speed > params.rotation_speed_m_s


def test_accelerate_then_pitch_up_liftoff_and_climb():
    params = AircraftParameters()
    dyn = FixedWingDynamics(params)
    ctl = FlightController(rotation_speed_m_s=params.rotation_speed_m_s)

    state = AircraftState(z_d=0.0, u=0.0, theta=0.0)
    dt = 1.0 / 60.0

    max_speed = state.u
    max_agl = max(0.0, -state.z_d)
    for i in range(60 * 28):
        if i < 60 * 12:
            targets = PilotTargets(bank_rad=0.0, pitch_rad=math.radians(2.0), throttle=0.9)
        else:
            targets = PilotTargets(bank_rad=0.0, pitch_rad=math.radians(10.0), throttle=0.9)

        controls = ctl.update(targets, state, dt)
        state = dyn.step(state, controls, dt)
        max_speed = max(max_speed, state.u)
        max_agl = max(max_agl, max(0.0, -state.z_d))

    assert max_speed > params.rotation_speed_m_s
    assert max_agl > 0.35
    assert state.z_d < -0.05
