from __future__ import annotations

from dataclasses import dataclass
import math

from .config import AircraftParameters


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class ControlInputs:
    aileron: float = 0.0
    elevator: float = 0.0
    rudder: float = 0.0
    throttle_command: float = 0.0


@dataclass
class AircraftState:
    # Inertial North-East-Down (NED) position (m). z <= 0 is above ground.
    x_n: float = 0.0
    y_e: float = 0.0
    z_d: float = 0.0

    # Body linear velocity (m/s)
    u: float = 0.0
    v: float = 0.0
    w: float = 0.0

    # Euler attitude (rad)
    phi: float = 0.0
    theta: float = 0.0
    psi: float = 0.0

    # Body rates (rad/s)
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0

    throttle_actual: float = 0.0

    def as_vector(self) -> list[float]:
        return [
            self.x_n,
            self.y_e,
            self.z_d,
            self.u,
            self.v,
            self.w,
            self.phi,
            self.theta,
            self.psi,
            self.p,
            self.q,
            self.r,
            self.throttle_actual,
        ]


class FixedWingDynamics:
    def __init__(self, params: AircraftParameters) -> None:
        self.p = params

    @staticmethod
    def _rotation_body_to_ned(phi: float, theta: float, psi: float) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
        cphi, sphi = math.cos(phi), math.sin(phi)
        cth, sth = math.cos(theta), math.sin(theta)
        cps, sps = math.cos(psi), math.sin(psi)

        # ZYX rotation (yaw-pitch-roll), body to NED.
        return (
            (cth * cps, sphi * sth * cps - cphi * sps, cphi * sth * cps + sphi * sps),
            (cth * sps, sphi * sth * sps + cphi * cps, cphi * sth * sps - sphi * cps),
            (-sth, sphi * cth, cphi * cth),
        )

    def step(self, state: AircraftState, ctrl: ControlInputs, dt: float) -> AircraftState:
        p = self.p

        aileron = clamp(ctrl.aileron, -p.max_aileron_rad, p.max_aileron_rad)
        elevator = clamp(ctrl.elevator, -p.max_elevator_rad, p.max_elevator_rad)
        rudder = clamp(ctrl.rudder, -p.max_rudder_rad, p.max_rudder_rad)
        throttle_cmd = clamp(ctrl.throttle_command, 0.0, 1.0)

        throttle_dot = (throttle_cmd - state.throttle_actual) / p.throttle_time_constant_s
        throttle_actual = clamp(state.throttle_actual + throttle_dot * dt, 0.0, 1.0)

        v_total = max(math.sqrt(state.u * state.u + state.v * state.v + state.w * state.w), 0.1)
        alpha_body = math.atan2(state.w, max(state.u, 0.1))
        beta = math.asin(clamp(state.v / v_total, -0.99, 0.99))

        qbar = 0.5 * p.rho_kg_m3 * v_total * v_total
        b2v = p.wing_span_m / (2.0 * v_total)
        c2v = p.mean_chord_m / (2.0 * v_total)

        on_ground = state.z_d >= 0.0

        # Effective AoA includes a small wing incidence and pitch contribution during ground roll,
        # improving rotation behavior for takeoff while keeping airborne behavior stable.
        alpha_eff = alpha_body + p.wing_incidence_rad
        if on_ground:
            alpha_eff += 0.75 * clamp(state.theta, math.radians(-5.0), math.radians(15.0))

        cl_linear = p.cl0 + p.cl_alpha * alpha_eff + p.cl_q * (state.q * c2v) + p.cl_de * elevator

        # Nonlinear stall model: once AoA exceeds critical value, lift no longer increases linearly,
        # drag rises sharply, and a nose-down pitch moment appears.
        abs_alpha = abs(alpha_eff)
        stall_excess = max(0.0, abs_alpha - p.alpha_stall_rad)
        stall_progress = stall_excess / max(1e-6, (math.pi / 2.0) - p.alpha_stall_rad)
        stall_progress = clamp(stall_progress, 0.0, 1.0)

        if stall_excess > 0.0:
            alpha_sign = 1.0 if alpha_eff >= 0.0 else -1.0
            cl_stall = alpha_sign * (p.cl_max + p.cl_post_stall_slope * stall_excess)
            cl = cl_stall
        else:
            cl = cl_linear

        cd = p.cd0 + p.k_induced * cl * cl + p.cd_stall_quadratic * stall_progress * stall_progress
        control_stall_scale = 1.0 - 0.80 * stall_progress
        aileron_eff = aileron * control_stall_scale
        rudder_eff = rudder * control_stall_scale

        cy = p.cy_beta * beta + p.cy_da * aileron_eff + p.cy_dr * rudder_eff

        lift = qbar * p.wing_area_m2 * cl
        drag = qbar * p.wing_area_m2 * cd
        side = qbar * p.wing_area_m2 * cy

        # Body-axis aerodynamic forces, body z positive down.
        x_aero = -drag * math.cos(alpha_eff) + lift * math.sin(alpha_eff)
        z_aero = -drag * math.sin(alpha_eff) - lift * math.cos(alpha_eff)
        y_aero = side

        thrust = throttle_actual * p.max_thrust_n
        x_prop = thrust

        r_bn = self._rotation_body_to_ned(state.phi, state.theta, state.psi)

        # Gravity force transformed to body frame.
        fg_ned = (0.0, 0.0, p.mass_kg * p.gravity_m_s2)
        fg_body_x = r_bn[0][0] * fg_ned[0] + r_bn[1][0] * fg_ned[1] + r_bn[2][0] * fg_ned[2]
        fg_body_y = r_bn[0][1] * fg_ned[0] + r_bn[1][1] * fg_ned[1] + r_bn[2][1] * fg_ned[2]
        fg_body_z = r_bn[0][2] * fg_ned[0] + r_bn[1][2] * fg_ned[1] + r_bn[2][2] * fg_ned[2]

        fx = x_aero + x_prop + fg_body_x
        fy = y_aero + fg_body_y
        fz = z_aero + fg_body_z

        rotation_pitch_assist = 0.0
        if on_ground:
            normal_force = max(0.0, p.mass_kg * p.gravity_m_s2 - lift)
            friction = p.runway_friction_coeff * normal_force + p.rolling_resistance_n
            if abs(state.u) > 0.2:
                fx -= math.copysign(friction, state.u)
            elif abs(fx) < friction:
                fx = 0.0
            else:
                fx -= math.copysign(friction, fx)

            rotation_progress = clamp((state.u - 0.82 * p.rotation_speed_m_s) / (0.35 * p.rotation_speed_m_s), 0.0, 1.0)
            if rotation_progress > 0.0 and elevator < 0.0:
                rotation_pitch_assist = p.rotation_pitch_assist_nm * rotation_progress * (-elevator / p.max_elevator_rad)

            liftoff_ready = (
                state.u >= p.rotation_speed_m_s
                and (
                    lift >= p.liftoff_lift_fraction * p.mass_kg * p.gravity_m_s2
                    or state.theta >= math.radians(2.5)
                )
            )

            # Keep wheels on runway until enough lift/rotation allows climb.
            if not liftoff_ready:
                state.w = max(0.0, state.w)

        # Moments
        damping_stall_scale = 1.0 - 0.65 * stall_progress
        roll_m = qbar * p.wing_area_m2 * p.wing_span_m * (
            p.cl_beta * beta
            + p.cl_p * damping_stall_scale * (state.p * b2v)
            + p.cl_r * damping_stall_scale * (state.r * b2v)
            + p.cl_da * aileron_eff
            + p.cl_dr * rudder_eff
        )
        pitch_m = qbar * p.wing_area_m2 * p.mean_chord_m * (
            p.cm0 + p.cm_alpha * alpha_eff + p.cm_q * (state.q * c2v) + p.cm_de * elevator
        )
        if stall_progress > 0.0:
            pitch_m += qbar * p.wing_area_m2 * p.mean_chord_m * p.cm_post_stall_nose_down * stall_progress
        pitch_m += rotation_pitch_assist
        yaw_m = qbar * p.wing_area_m2 * p.wing_span_m * (
            p.cn_beta * beta
            + p.cn_p * damping_stall_scale * (state.p * b2v)
            + p.cn_r * damping_stall_scale * (state.r * b2v)
            + p.cn_da * aileron_eff
            + p.cn_dr * rudder_eff
        )

        # Secondary stall dynamics: wing drop / incipient spin tendencies when deeply stalled.
        if stall_progress > 0.0 and not on_ground:
            beta_eff = beta + p.stall_asymmetry_bias
            roll_m += qbar * p.wing_area_m2 * p.wing_span_m * stall_progress * (
                p.cl_stall_autoroll_beta * beta_eff + p.cl_stall_autoroll_r * (state.r * b2v)
            )
            yaw_m += qbar * p.wing_area_m2 * p.wing_span_m * stall_progress * (
                p.cn_stall_autoyaw_beta * beta_eff + p.cn_stall_autoyaw_p * (state.p * b2v)
            )

        u_dot = state.r * state.v - state.q * state.w + fx / p.mass_kg
        v_dot = state.p * state.w - state.r * state.u + fy / p.mass_kg
        w_dot = state.q * state.u - state.p * state.v + fz / p.mass_kg

        p_dot = (roll_m - (p.i_zz - p.i_yy) * state.q * state.r) / p.i_xx
        q_dot = (pitch_m - (p.i_xx - p.i_zz) * state.p * state.r) / p.i_yy
        r_dot = (yaw_m - (p.i_yy - p.i_xx) * state.p * state.q) / p.i_zz

        cth = max(0.01, math.cos(state.theta))
        phi_dot = state.p + math.tan(state.theta) * (state.q * math.sin(state.phi) + state.r * math.cos(state.phi))
        theta_dot = state.q * math.cos(state.phi) - state.r * math.sin(state.phi)
        psi_dot = (state.q * math.sin(state.phi) + state.r * math.cos(state.phi)) / cth

        # Inertial position rates from body velocity.
        x_dot = r_bn[0][0] * state.u + r_bn[0][1] * state.v + r_bn[0][2] * state.w
        y_dot = r_bn[1][0] * state.u + r_bn[1][1] * state.v + r_bn[1][2] * state.w
        z_dot = r_bn[2][0] * state.u + r_bn[2][1] * state.v + r_bn[2][2] * state.w

        next_state = AircraftState(
            x_n=state.x_n + x_dot * dt,
            y_e=state.y_e + y_dot * dt,
            z_d=state.z_d + z_dot * dt,
            u=state.u + u_dot * dt,
            v=state.v + v_dot * dt,
            w=state.w + w_dot * dt,
            phi=state.phi + phi_dot * dt,
            theta=state.theta + theta_dot * dt,
            psi=state.psi + psi_dot * dt,
            p=state.p + p_dot * dt,
            q=state.q + q_dot * dt,
            r=state.r + r_dot * dt,
            throttle_actual=throttle_actual,
        )

        # Ground constraint: aircraft cannot go below runway plane (z_d > 0 means underground).
        if next_state.z_d > 0.0:
            next_state.z_d = 0.0
            if z_dot > 0.0:
                next_state.w = min(next_state.w, 0.0)

        # Prevent unrealistic divergence in this lightweight model.
        next_state.theta = clamp(next_state.theta, math.radians(-85.0), math.radians(85.0))
        next_state.u = max(-5.0, next_state.u)

        return next_state
