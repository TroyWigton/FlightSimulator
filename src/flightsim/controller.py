from dataclasses import dataclass
import math

from .dynamics import AircraftState, ControlInputs
from .pid import PID


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class PilotTargets:
    bank_rad: float = 0.0
    pitch_rad: float = math.radians(4.0)
    throttle: float = 0.0


class FlightController:
    def __init__(self, rotation_speed_m_s: float = 30.0) -> None:
        self.rotation_speed_m_s = rotation_speed_m_s
        self.performance_mode = False
        self.roll_pid = PID(1.8, 0.10, 0.08, -0.35, 0.35)
        self.pitch_pid = PID(1.3, 0.03, 0.03, -0.35, 0.35)
        self.yaw_damper = PID(0.16, 0.0, 0.015, -0.2, 0.2)
        self.throttle_pid = PID(2.0, 1.5, 0.0, 0.0, 1.0)
        self.roll_rate_damping = 0.18
        self.pitch_rate_damping = 0.38
        self.max_aileron_rate = 1.4  # rad/s
        self.max_elevator_rate = 0.50  # rad/s
        self.prev_aileron_cmd = 0.0
        self.prev_elevator_cmd = 0.0
        self.has_liftoff = False
        self.pitch_target_filtered = 0.0
        self.pitch_target_filter_tau = 0.35
        self.climb_rate_gain = 0.04

    def set_performance_mode(self, enabled: bool) -> None:
        self.performance_mode = enabled

    def mode_name(self) -> str:
        return "PERFORMANCE" if self.performance_mode else "STABLE"

    def reset(self) -> None:
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_damper.reset()
        self.throttle_pid.reset()
        self.prev_aileron_cmd = 0.0
        self.prev_elevator_cmd = 0.0
        self.has_liftoff = False
        self.pitch_target_filtered = 0.0

    def update(self, targets: PilotTargets, state: AircraftState, dt: float) -> ControlInputs:
        bank_target = clamp(targets.bank_rad, math.radians(-45.0), math.radians(45.0))
        pitch_target = clamp(targets.pitch_rad, math.radians(-45.0), math.radians(45.0))
        throttle_target = clamp(targets.throttle, 0.0, 1.0)

        # Rotation assist: once near rotation speed on runway, bias toward a safe pitch-up target.
        rotate_floor_deg = 16.0 if self.performance_mode else 14.0
        if state.z_d >= -0.05 and state.u > 0.9 * self.rotation_speed_m_s:
            pitch_target = max(pitch_target, math.radians(rotate_floor_deg))

        agl = max(0.0, -state.z_d)

        # Liftoff latch helps maintain positive climb trend once airborne.
        if not self.has_liftoff and agl > 0.25 and state.u > 0.9 * self.rotation_speed_m_s and throttle_target > 0.7:
            self.has_liftoff = True
        if self.has_liftoff and (agl < 0.05 and state.u < 0.7 * self.rotation_speed_m_s):
            self.has_liftoff = False

        if self.has_liftoff and throttle_target > 0.7 and agl < (40.0 if self.performance_mode else 20.0):
            pitch_target = max(pitch_target, math.radians(16.0 if self.performance_mode else 18.0))

        # Performance mode climb-rate augmentation to reduce AGL bobble after liftoff.
        if self.performance_mode and self.has_liftoff and throttle_target > 0.7:
            climb_rate = state.u * math.sin(state.theta) - state.w * math.cos(state.theta)
            climb_rate_target = 2.5
            pitch_target += self.climb_rate_gain * (climb_rate_target - climb_rate)

        if self.performance_mode and self.has_liftoff and throttle_target > 0.7 and agl < 15.0:
            pitch_target = min(pitch_target, math.radians(16.0))

        pitch_target = clamp(pitch_target, math.radians(-45.0), math.radians(45.0))

        # Filter commanded pitch target to avoid abrupt elevator reversals.
        alpha = dt / max(dt, self.pitch_target_filter_tau)
        self.pitch_target_filtered += alpha * (pitch_target - self.pitch_target_filtered)

        # Slight gain scheduling: reduce excessive control activity at very high speed.
        speed_scale = 1.0
        if self.performance_mode:
            if state.u > 65.0:
                speed_scale = 0.90
        else:
            if state.u > 45.0:
                speed_scale = 0.82

        aileron_cmd = speed_scale * self.roll_pid.update(bank_target, state.phi, dt) - self.roll_rate_damping * state.p
        elevator_cmd = speed_scale * (-self.pitch_pid.update(self.pitch_target_filtered, state.theta, dt) + self.pitch_rate_damping * state.q)

        # Surface command rate limits improve high-speed stability and reduce chatter.
        max_da = self.max_aileron_rate * dt
        max_de = self.max_elevator_rate * dt
        aileron = self.prev_aileron_cmd + clamp(aileron_cmd - self.prev_aileron_cmd, -max_da, max_da)
        elevator = self.prev_elevator_cmd + clamp(elevator_cmd - self.prev_elevator_cmd, -max_de, max_de)

        aileron = clamp(aileron, -0.35, 0.35)
        elevator = clamp(elevator, -0.35, 0.35)
        self.prev_aileron_cmd = aileron
        self.prev_elevator_cmd = elevator

        # Yaw damping tries to drive yaw rate toward 0 for coordinated-feel handling.
        rudder = self.yaw_damper.update(0.0, state.r, dt)

        throttle_cmd = self.throttle_pid.update(throttle_target, state.throttle_actual, dt)

        return ControlInputs(
            aileron=aileron,
            elevator=elevator,
            rudder=rudder,
            throttle_command=throttle_cmd,
        )
