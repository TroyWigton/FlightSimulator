from __future__ import annotations

import math
import pygame

from .config import AircraftParameters, SimConfig
from .controller import FlightController, PilotTargets
from .dynamics import AircraftState, FixedWingDynamics
from .ui import FlightDisplay


def initial_state() -> AircraftState:
    return AircraftState(
        x_n=0.0,
        y_e=0.0,
        z_d=0.0,
        u=0.0,
        v=0.0,
        w=0.0,
        phi=0.0,
        theta=0.0,
        psi=0.0,
        p=0.0,
        q=0.0,
        r=0.0,
        throttle_actual=0.0,
    )


class FlightSimulatorApp:
    def __init__(self) -> None:
        self.cfg = SimConfig()
        self.params = AircraftParameters()
        self.dynamics = FixedWingDynamics(self.params)
        self.controller = FlightController(rotation_speed_m_s=self.params.rotation_speed_m_s)
        self.display = FlightDisplay(self.cfg.screen_w, self.cfg.screen_h)
        self.state = initial_state()
        self.targets = PilotTargets(bank_rad=0.0, pitch_rad=math.radians(3.0), throttle=0.0)
        self.notice_text = ""
        self.notice_time_s = 0.0

    @staticmethod
    def _digit_from_key(key: int) -> int | None:
        if pygame.K_0 <= key <= pygame.K_9:
            return key - pygame.K_0
        keypad_map = {
            pygame.K_KP0: 0,
            pygame.K_KP1: 1,
            pygame.K_KP2: 2,
            pygame.K_KP3: 3,
            pygame.K_KP4: 4,
            pygame.K_KP5: 5,
            pygame.K_KP6: 6,
            pygame.K_KP7: 7,
            pygame.K_KP8: 8,
            pygame.K_KP9: 9,
        }
        if key in keypad_map:
            return keypad_map[key]
        return None

    def _process_input(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                if event.key == pygame.K_r:
                    self.state = initial_state()
                    self.targets = PilotTargets(bank_rad=0.0, pitch_rad=math.radians(3.0), throttle=0.0)
                    self.controller.reset()

                if event.key == pygame.K_m:
                    self.controller.set_performance_mode(not self.controller.performance_mode)
                    self.notice_text = f"Mode: {self.controller.mode_name()}"
                    self.notice_time_s = 1.2

                if event.key == pygame.K_UP:
                    self.targets.pitch_rad += math.radians(2.0)
                if event.key == pygame.K_DOWN:
                    self.targets.pitch_rad -= math.radians(2.0)
                if event.key == pygame.K_LEFT:
                    self.targets.bank_rad -= math.radians(2.5)
                if event.key == pygame.K_RIGHT:
                    self.targets.bank_rad += math.radians(2.5)

                digit = self._digit_from_key(event.key)
                if digit is not None:
                    # Latching command: one press sets/holds throttle until next numeric input.
                    self.targets.throttle = 0.0 if digit == 0 else digit / 9.0
                    self.notice_text = f"Throttle latched: {self.targets.throttle * 100:4.0f}%"
                    self.notice_time_s = 1.2

        keys = pygame.key.get_pressed()
        bank_step = math.radians(35.0) * self.cfg.dt_s
        pitch_step = math.radians(35.0) * self.cfg.dt_s

        if keys[pygame.K_LEFT]:
            self.targets.bank_rad -= bank_step
        if keys[pygame.K_RIGHT]:
            self.targets.bank_rad += bank_step
        if keys[pygame.K_UP]:
            self.targets.pitch_rad += pitch_step
        if keys[pygame.K_DOWN]:
            self.targets.pitch_rad -= pitch_step

        self.targets.bank_rad = max(math.radians(-45.0), min(math.radians(45.0), self.targets.bank_rad))
        self.targets.pitch_rad = max(math.radians(-45.0), min(math.radians(45.0), self.targets.pitch_rad))
        return True

    def run(self) -> None:
        running = True
        clock = pygame.time.Clock()

        while running:
            running = self._process_input()

            controls = self.controller.update(self.targets, self.state, self.cfg.dt_s)
            self.state = self.dynamics.step(self.state, controls, self.cfg.dt_s)

            if self.notice_time_s > 0.0:
                self.notice_time_s = max(0.0, self.notice_time_s - self.cfg.dt_s)
            notice = self.notice_text if self.notice_time_s > 0.0 else ""

            self.display.render(self.state, self.targets, controls, notice, self.controller.mode_name())
            clock.tick(self.cfg.fps)

        self.display.close()
