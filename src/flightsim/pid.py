from dataclasses import dataclass


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    min_output: float
    max_output: float
    integrator: float = 0.0
    prev_error: float = 0.0
    prev_measured: float = 0.0
    deriv_state: float = 0.0
    deriv_filter_alpha: float = 0.2
    first_run: bool = True

    def reset(self) -> None:
        self.integrator = 0.0
        self.prev_error = 0.0
        self.prev_measured = 0.0
        self.deriv_state = 0.0
        self.first_run = True

    def update(self, target: float, measured: float, dt: float) -> float:
        error = target - measured

        p_term = self.kp * error
        self.integrator += error * dt
        i_term = self.ki * self.integrator

        derivative = 0.0
        if not self.first_run and dt > 0.0:
            meas_rate = (measured - self.prev_measured) / dt
            derivative = -meas_rate
            self.deriv_state = self.deriv_filter_alpha * derivative + (1.0 - self.deriv_filter_alpha) * self.deriv_state
        else:
            self.deriv_state = 0.0
        d_term = self.kd * self.deriv_state

        output = p_term + i_term + d_term

        # Anti-windup by clamping output and back-calculating integrator contribution.
        if output > self.max_output:
            output = self.max_output
            if self.ki != 0.0:
                self.integrator = (output - p_term - d_term) / self.ki
        elif output < self.min_output:
            output = self.min_output
            if self.ki != 0.0:
                self.integrator = (output - p_term - d_term) / self.ki

        self.prev_error = error
        self.prev_measured = measured
        self.first_run = False
        return output
