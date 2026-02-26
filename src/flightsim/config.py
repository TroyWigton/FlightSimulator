from dataclasses import dataclass


@dataclass(frozen=True)
class AircraftParameters:
    mass_kg: float = 1100.0
    wing_area_m2: float = 16.2
    wing_span_m: float = 10.9
    mean_chord_m: float = 1.5

    i_xx: float = 1285.0
    i_yy: float = 1825.0
    i_zz: float = 2665.0
    i_xz: float = 0.0

    rho_kg_m3: float = 1.225
    gravity_m_s2: float = 9.80665

    max_thrust_n: float = 6500.0
    throttle_time_constant_s: float = 0.35

    # Aerodynamic coefficient set tuned for a stable light aircraft.
    cl0: float = 0.22
    cl_alpha: float = 5.8
    cl_q: float = 7.0
    cl_de: float = 0.85

    # Stall model (effective angle-of-attack, radians)
    alpha_stall_rad: float = 0.28
    cl_max: float = 1.35
    cl_post_stall_slope: float = -2.4
    cd_stall_quadratic: float = 4.2
    cm_post_stall_nose_down: float = -3.0
    cl_stall_autoroll_beta: float = -0.85
    cn_stall_autoyaw_beta: float = 0.55
    cl_stall_autoroll_r: float = 0.30
    cn_stall_autoyaw_p: float = -0.20
    stall_asymmetry_bias: float = 0.02

    cd0: float = 0.030
    k_induced: float = 0.075

    cy_beta: float = -0.82
    cy_da: float = 0.05
    cy_dr: float = 0.16

    cm0: float = 0.00
    cm_alpha: float = -0.85
    cm_q: float = -10.5
    cm_de: float = -1.80

    cl_beta: float = -0.12
    cl_p: float = -0.55
    cl_r: float = 0.1
    cl_da: float = 0.16
    cl_dr: float = 0.02

    cn_beta: float = 0.25
    cn_p: float = -0.02
    cn_r: float = -0.32
    cn_da: float = 0.01
    cn_dr: float = -0.12

    max_aileron_rad: float = 0.35
    max_elevator_rad: float = 0.35
    max_rudder_rad: float = 0.35

    runway_friction_coeff: float = 0.025
    rolling_resistance_n: float = 45.0
    rotation_speed_m_s: float = 25.0
    wing_incidence_rad: float = 0.04
    liftoff_lift_fraction: float = 0.65
    rotation_pitch_assist_nm: float = 350.0


@dataclass(frozen=True)
class SimConfig:
    dt_s: float = 1.0 / 60.0
    screen_w: int = 1200
    screen_h: int = 720
    fps: int = 60
