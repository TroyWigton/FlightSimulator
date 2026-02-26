import math

from flightsim.config import AircraftParameters
from flightsim.dynamics import AircraftState, ControlInputs, FixedWingDynamics


def test_deep_stall_develops_wing_drop_and_yaw_divergence():
    params = AircraftParameters()
    dynamics = FixedWingDynamics(params)

    dt = 1.0 / 60.0
    state = AircraftState(
        z_d=-250.0,
        u=42.0,
        v=0.8,
        theta=math.radians(8.0),
        psi=math.radians(2.0),
        throttle_actual=0.0,
    )

    max_abs_phi = abs(state.phi)
    max_abs_r = abs(state.r)

    for _ in range(60 * 10):
        state = dynamics.step(
            state,
            ControlInputs(aileron=0.0, elevator=-0.30, rudder=0.0, throttle_command=0.0),
            dt,
        )
        max_abs_phi = max(max_abs_phi, abs(state.phi))
        max_abs_r = max(max_abs_r, abs(state.r))

    assert max_abs_phi > math.radians(2.0)
    assert max_abs_r > math.radians(2.0)
