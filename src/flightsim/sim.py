from __future__ import annotations

from datetime import datetime
import math
from pathlib import Path
import pygame
import shutil
import subprocess

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
        self.recording = False
        self.recording_root = Path("recordings")
        self.recording_session_dir: Path | None = None
        self.recording_frame_idx = 0
        self.recording_frame_ext = "png"

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

                if event.key == pygame.K_v:
                    if not self.recording:
                        self._start_recording()
                    else:
                        self._stop_recording()

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

    def _start_recording(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = self.recording_root / f"flight_{stamp}"
        frames_dir = session_dir / "frames"
        frames_dir.mkdir(parents=True, exist_ok=True)

        self.recording = True
        self.recording_session_dir = session_dir
        self.recording_frame_idx = 0
        self.recording_frame_ext = "png"
        self.notice_text = f"Recording ON: {session_dir}"
        self.notice_time_s = 1.5

    def _stop_recording(self) -> None:
        self.recording = False
        if self.recording_session_dir is None:
            self.notice_text = "Recording stopped"
            self.notice_time_s = 1.0
            return

        session_dir = self.recording_session_dir
        frames_dir = session_dir / "frames"
        output_file = session_dir / "flight.mp4"

        if self.recording_frame_idx < 2:
            self.notice_text = "Recording stopped (not enough frames)"
            self.notice_time_s = 1.6
            self.recording_session_dir = None
            return

        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            self.notice_text = "Recording saved as PNG frames (install ffmpeg for MP4)"
            self.notice_time_s = 2.4
            self.recording_session_dir = None
            return

        cmd = [
            ffmpeg,
            "-y",
            "-framerate",
            str(self.cfg.fps),
            "-i",
            str(frames_dir / f"frame_%06d.{self.recording_frame_ext}"),
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            str(output_file),
        ]
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            shutil.rmtree(frames_dir, ignore_errors=True)
            self.notice_text = f"Recording saved: {output_file}"
            self.notice_time_s = 2.4
        except Exception:
            self.notice_text = "ffmpeg encode failed (raw frames kept)"
            self.notice_time_s = 2.4

        self.recording_session_dir = None

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

            if self.recording and self.recording_session_dir is not None:
                frame_path = self.recording_session_dir / "frames" / f"frame_{self.recording_frame_idx:06d}.{self.recording_frame_ext}"
                try:
                    pygame.image.save(self.display.screen, str(frame_path))
                except NotImplementedError:
                    if self.recording_frame_ext == "png":
                        self.recording_frame_ext = "bmp"
                        frame_path = self.recording_session_dir / "frames" / f"frame_{self.recording_frame_idx:06d}.bmp"
                        pygame.image.save(self.display.screen, str(frame_path))
                        self.notice_text = "PNG unsupported, recording as BMP frames"
                        self.notice_time_s = 2.0
                    else:
                        raise
                self.recording_frame_idx += 1

            clock.tick(self.cfg.fps)

        if self.recording:
            self._stop_recording()

        self.display.close()
