from __future__ import annotations

from collections import deque
import importlib
import math
import pygame

from .controller import PilotTargets
from .dynamics import AircraftState, ControlInputs


SKY = (90, 155, 230)
EARTH = (150, 110, 70)
WHITE = (240, 240, 240)
GREEN = (80, 200, 80)
BLACK = (10, 10, 10)
YELLOW = (230, 220, 70)


class _FontAdapter:
    def __init__(self, backend: str, font_obj) -> None:
        self.backend = backend
        self.font_obj = font_obj

    def render(self, text: str, antialias: bool, color: tuple[int, int, int]):
        if self.backend == "freetype":
            surface, _ = self.font_obj.render(text, fgcolor=color)
            return surface
        if self.backend == "pillow":
            return self.font_obj.render(text, color)
        return self.font_obj.render(text, antialias, color)


class _PillowTextRenderer:
    def __init__(self, size: int) -> None:
        from PIL import Image, ImageDraw, ImageFont

        self.Image = Image
        self.ImageDraw = ImageDraw
        self.ImageFont = ImageFont
        self.size = size
        self.cache: dict[tuple[str, tuple[int, int, int]], pygame.Surface] = {}
        try:
            self.font = ImageFont.truetype("DejaVuSans.ttf", size)
        except Exception:
            self.font = ImageFont.load_default()

    def render(self, text: str, color: tuple[int, int, int]) -> pygame.Surface:
        key = (text, color)
        cached = self.cache.get(key)
        if cached is not None:
            return cached

        width = max(8, int(len(text) * self.size * 0.64) + 6)
        height = max(10, int(self.size * 1.5))

        img = self.Image.new("RGBA", (width, height), (0, 0, 0, 0))
        draw = self.ImageDraw.Draw(img)
        draw.text((2, 2), text, fill=(color[0], color[1], color[2], 255), font=self.font)
        surface = pygame.image.fromstring(img.tobytes(), img.size, "RGBA").convert_alpha()

        self.cache[key] = surface
        return surface


class FlightDisplay:
    def __init__(self, width: int, height: int) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Fixed-Wing Flight Simulator")
        self.font = None
        self.small_font = None
        self.font_available = False
        try:
            freetype_mod = importlib.import_module("pygame.freetype")
            if hasattr(freetype_mod, "init"):
                freetype_mod.init()
            self.font = _FontAdapter("freetype", freetype_mod.SysFont("Consolas", 20))
            self.small_font = _FontAdapter("freetype", freetype_mod.SysFont("Consolas", 16))
            self.font_available = True
        except Exception:
            try:
                font_mod = importlib.import_module("pygame.font")
                if hasattr(font_mod, "init"):
                    font_mod.init()
                self.font = _FontAdapter("font", font_mod.SysFont("Consolas", 20))
                self.small_font = _FontAdapter("font", font_mod.SysFont("Consolas", 16))
                self.font_available = True
            except Exception:
                try:
                    self.font = _FontAdapter("pillow", _PillowTextRenderer(20))
                    self.small_font = _FontAdapter("pillow", _PillowTextRenderer(16))
                    self.font_available = True
                except Exception:
                    self.font_available = False
        self.width = width
        self.height = height
        self.max_trace_samples = 360
        self.trace_aileron = deque(maxlen=self.max_trace_samples)
        self.trace_elevator = deque(maxlen=self.max_trace_samples)
        self.trace_p = deque(maxlen=self.max_trace_samples)
        self.trace_q = deque(maxlen=self.max_trace_samples)
        self.trace_r = deque(maxlen=self.max_trace_samples)

    def _draw_ground_pattern(self, state: AircraftState) -> None:
        pitch_scale = 300.0
        horizon_y = self.height * 0.5 + state.theta * pitch_scale
        horizon_y = max(self.height * 0.10, min(self.height * 0.92, horizon_y))
        vanishing_x = self.width * 0.5 - state.phi * 240.0
        forward_phase = (state.x_n * 0.03) % 1.0
        lateral_phase = (state.y_e * 0.05) % 1.0

        altitude_agl = max(0.0, -state.z_d)
        for depth in range(1, 24):
            depth_param = depth + forward_phase
            depth_m = depth_param * 35.0
            y = horizon_y + (self.height - horizon_y) * (1.0 - math.exp(-depth_param * 0.18))
            if y < horizon_y + 8:
                continue
            # Sparse terrain bands for depth without a distracting full grid look.
            if depth % 6 == 0:
                pygame.draw.line(self.screen, (154, 118, 82), (0, y), (self.width, y), 1)

            # Perspective objects that shrink as altitude increases.
            size_scale = 1.0 / (1.0 + altitude_agl / 120.0 + depth_m / 240.0)
            marker_h = max(2, int(22 * size_scale))
            marker_w = max(1, int(8 * size_scale))
            x_offset = int((depth % 6 - 3) * 160 * size_scale)
            mx1 = int(self.width * 0.5 + x_offset - marker_w)
            mx2 = int(self.width * 0.5 + x_offset + marker_w)
            my1 = int(y - marker_h)
            my2 = int(y)
            pygame.draw.rect(self.screen, (95, 78, 55), (mx1, my1, mx2 - mx1, my2 - my1), border_radius=2)

            # Ground texture speckles to increase motion visibility.
            tex_shift = state.x_n * 0.22 + depth * 17.0
            for tex_idx in range(4):
                tx = int((tex_shift + tex_idx * 280 + state.y_e * 0.5) % (self.width + 120)) - 60
                tw = max(2, int(18 * size_scale))
                th = max(1, int(5 * size_scale))
                pygame.draw.rect(self.screen, (150, 116, 78), (tx, int(y), tw, th))

        # Simple trees with perspective scaling and forward-motion drift.
        for band in range(4, 20):
            depth_param = band + forward_phase
            depth_m = depth_param * 42.0
            y = horizon_y + (self.height - horizon_y) * (1.0 - math.exp(-depth_param * 0.20))
            if y < horizon_y + 6 or y > self.height - 6:
                continue

            size_scale = 1.0 / (1.0 + altitude_agl / 110.0 + depth_m / 210.0)
            tree_h = max(4, int(80 * size_scale))
            canopy_r = max(3, int(24 * size_scale))
            trunk_w = max(2, int(8 * size_scale))

            base_phase = state.x_n * 0.015 + band * 0.55
            x1 = int(self.width * (0.18 + 0.16 * math.sin(base_phase + state.y_e * 0.01)))
            x2 = int(self.width * (0.82 + 0.14 * math.sin(base_phase + 2.4 + state.y_e * 0.01)))

            for tx in (x1, x2):
                trunk_top = int(y - tree_h)
                pygame.draw.rect(self.screen, (78, 52, 32), (tx - trunk_w // 2, trunk_top, trunk_w, tree_h))
                pygame.draw.circle(self.screen, (48, 118, 52), (tx, trunk_top - canopy_r // 2), canopy_r)
                pygame.draw.circle(self.screen, (62, 142, 64), (tx - canopy_r // 2, trunk_top), max(2, canopy_r - 2))

    def _draw_scalar_gauges(self, state: AircraftState) -> None:
        airspeed = math.sqrt(state.u * state.u + state.v * state.v + state.w * state.w)
        altitude = max(0.0, -state.z_d)

        gauge_h = 220
        gauge_w = 24
        top = 24

        # Airspeed tape (left side)
        airspeed_max = 80.0
        v_frac = max(0.0, min(1.0, airspeed / airspeed_max))
        x_v = 18
        pygame.draw.rect(self.screen, (35, 35, 35), (x_v, top, gauge_w, gauge_h), border_radius=4)
        pygame.draw.rect(self.screen, (60, 200, 220), (x_v + 3, top + int((1.0 - v_frac) * (gauge_h - 6)), gauge_w - 6, int(v_frac * (gauge_h - 6))), border_radius=3)
        pygame.draw.rect(self.screen, WHITE, (x_v, top, gauge_w, gauge_h), 2, border_radius=4)

        for tick in range(0, 81, 20):
            y_tick = top + int((1.0 - tick / airspeed_max) * gauge_h)
            pygame.draw.line(self.screen, WHITE, (x_v + gauge_w + 2, y_tick), (x_v + gauge_w + 8, y_tick), 1)

        # Altitude AGL tape (right side)
        alt_max = 1000.0
        h_frac = max(0.0, min(1.0, altitude / alt_max))
        x_h = self.width - 42
        pygame.draw.rect(self.screen, (35, 35, 35), (x_h, top, gauge_w, gauge_h), border_radius=4)
        pygame.draw.rect(self.screen, (120, 220, 120), (x_h + 3, top + int((1.0 - h_frac) * (gauge_h - 6)), gauge_w - 6, int(h_frac * (gauge_h - 6))), border_radius=3)
        pygame.draw.rect(self.screen, WHITE, (x_h, top, gauge_w, gauge_h), 2, border_radius=4)

        for tick in range(0, 1001, 250):
            y_tick = top + int((1.0 - tick / alt_max) * gauge_h)
            pygame.draw.line(self.screen, WHITE, (x_h - 8, y_tick), (x_h - 2, y_tick), 1)

        if self.font_available and self.small_font is not None:
            spd = self.small_font.render(f"Airspeed [m/s]: {airspeed:4.1f}", True, BLACK)
            alt = self.small_font.render(f"Altitude AGL [m]: {altitude:5.1f}", True, BLACK)
            self.screen.blit(spd, (10, top + gauge_h + 8))
            self.screen.blit(alt, (self.width - 210, top + gauge_h + 8))

            v_label = self.small_font.render("SPD", True, WHITE)
            h_label = self.small_font.render("AGL", True, WHITE)
            self.screen.blit(v_label, (x_v - 4, top - 16))
            self.screen.blit(h_label, (x_h - 4, top - 16))

            v_live = self.small_font.render("LIVE: true airspeed magnitude", True, (235, 235, 235))
            h_live = self.small_font.render("LIVE: altitude above ground", True, (235, 235, 235))
            self.screen.blit(v_live, (52, top + 2))
            self.screen.blit(h_live, (self.width - 286, top + 2))

    def _draw_throttle_indicator(self, state: AircraftState, targets: PilotTargets, mode_name: str = "STABLE") -> None:
        bar_w = 320
        bar_h = 20
        x = self.width // 2 - bar_w // 2
        y = 18

        actual = max(0.0, min(1.0, state.throttle_actual))
        commanded = max(0.0, min(1.0, targets.throttle))

        pygame.draw.rect(self.screen, (20, 22, 26), (x, y, bar_w, bar_h), border_radius=6)
        pygame.draw.rect(self.screen, WHITE, (x, y, bar_w, bar_h), 1, border_radius=6)

        fill_w = int((bar_w - 4) * actual)
        pygame.draw.rect(self.screen, (235, 168, 58), (x + 2, y + 2, fill_w, bar_h - 4), border_radius=4)

        cmd_x = x + int(commanded * (bar_w - 1))
        pygame.draw.line(self.screen, (60, 230, 90), (cmd_x, y - 4), (cmd_x, y + bar_h + 4), 2)

        if self.font_available and self.small_font is not None:
            title = self.small_font.render(f"Throttle / Power (latched)  |  Mode: {mode_name}", True, WHITE)
            val = self.small_font.render(
                f"ACT {actual * 100:5.1f}%   CMD {commanded * 100:5.1f}%",
                True,
                WHITE,
            )
            note = self.small_font.render("Press 0-9 once to set and hold throttle", True, (225, 225, 225))
            self.screen.blit(title, (x + 4, y - 18))
            self.screen.blit(val, (x + 6, y + bar_h + 4))
            self.screen.blit(note, (x + 6, y + bar_h + 22))

    def _draw_trace_plot(
        self,
        rect: tuple[int, int, int, int],
        series: list[tuple[deque[float], tuple[int, int, int], float]],
        title: str,
        legend: list[str],
    ) -> None:
        x, y, w, h = rect
        pygame.draw.rect(self.screen, (15, 18, 20), rect, border_radius=6)
        pygame.draw.rect(self.screen, WHITE, rect, 1, border_radius=6)
        mid_y = y + h // 2
        pygame.draw.line(self.screen, (90, 90, 90), (x + 6, mid_y), (x + w - 6, mid_y), 1)

        for data, color, y_limit in series:
            if len(data) < 2:
                continue
            points: list[tuple[float, float]] = []
            step_x = (w - 12) / max(1, len(data) - 1)
            for i, value in enumerate(data):
                frac = max(-1.0, min(1.0, value / y_limit))
                py = mid_y - frac * (h * 0.42)
                px = x + 6 + i * step_x
                points.append((px, py))
            pygame.draw.lines(self.screen, color, False, points, 2)

        if self.font_available and self.small_font is not None:
            title_surface = self.small_font.render(title, True, WHITE)
            self.screen.blit(title_surface, (x + 8, y + 6))

            legend_x = x + 8
            legend_y = y + 24
            for idx, ((_, color, _), label) in enumerate(zip(series, legend)):
                marker_y = legend_y + idx * 14
                pygame.draw.line(self.screen, color, (legend_x, marker_y + 6), (legend_x + 16, marker_y + 6), 3)
                legend_surface = self.small_font.render(label, True, (220, 220, 220))
                self.screen.blit(legend_surface, (legend_x + 22, marker_y))

            info_surface = self.small_font.render("Left->Right: oldest to newest samples", True, (180, 180, 180))
            self.screen.blit(info_surface, (x + 8, y + h - 18))

    def _draw_horizon(self, state: AircraftState) -> None:
        cx = self.width // 2
        cy = self.height // 2

        pitch_scale = 300.0
        pitch_offset = state.theta * pitch_scale

        roll = -state.phi
        c = math.cos(roll)
        s = math.sin(roll)

        line_len = 2500

        p1 = (-line_len, pitch_offset)
        p2 = (line_len, pitch_offset)

        rp1 = (cx + p1[0] * c - p1[1] * s, cy + p1[0] * s + p1[1] * c)
        rp2 = (cx + p2[0] * c - p2[1] * s, cy + p2[0] * s + p2[1] * c)

        # Fill sky.
        self.screen.fill(SKY)

        # Fill earth using a large polygon under the rotated horizon line.
        ground_poly = [
            rp1,
            rp2,
            (self.width + 500, self.height + 500),
            (-500, self.height + 500),
        ]
        pygame.draw.polygon(self.screen, EARTH, ground_poly)
        self._draw_ground_pattern(state)

        pygame.draw.line(self.screen, WHITE, rp1, rp2, 3)

        ladder_color = (210, 235, 210)
        for deg in range(-30, 35, 5):
            if deg == 0:
                continue

            y = pitch_offset + math.radians(deg) * pitch_scale
            half_len = 95 if deg % 10 == 0 else 60
            gap = 18

            l1 = (cx + (-half_len) * c - y * s, cy + (-half_len) * s + y * c)
            l2 = (cx + (-gap) * c - y * s, cy + (-gap) * s + y * c)
            r1 = (cx + (gap) * c - y * s, cy + (gap) * s + y * c)
            r2 = (cx + (half_len) * c - y * s, cy + (half_len) * s + y * c)

            pygame.draw.line(self.screen, ladder_color, l1, l2, 2 if deg % 10 == 0 else 1)
            pygame.draw.line(self.screen, ladder_color, r1, r2, 2 if deg % 10 == 0 else 1)

            if deg % 10 == 0 and self.font_available and self.small_font is not None:
                label = self.small_font.render(f"{abs(deg)}", True, ladder_color)
                lx = cx + (-half_len - 22) * c - y * s
                ly = cy + (-half_len - 22) * s + y * c
                rx = cx + (half_len + 8) * c - y * s
                ry = cy + (half_len + 8) * s + y * c
                self.screen.blit(label, (lx, ly - 8))
                self.screen.blit(label, (rx, ry - 8))

    def _draw_aircraft_symbol(self) -> None:
        cx = self.width // 2
        cy = self.height // 2
        pygame.draw.line(self.screen, YELLOW, (cx - 55, cy), (cx + 55, cy), 3)
        pygame.draw.line(self.screen, YELLOW, (cx, cy - 20), (cx, cy + 20), 3)
        pygame.draw.circle(self.screen, YELLOW, (cx, cy), 4)

    def _draw_hud(self, state: AircraftState, targets: PilotTargets, controls: ControlInputs) -> None:
        airspeed = math.sqrt(state.u * state.u + state.v * state.v + state.w * state.w)
        altitude = max(0.0, -state.z_d)

        self.trace_aileron.append(controls.aileron)
        self.trace_elevator.append(controls.elevator)
        self.trace_p.append(state.p)
        self.trace_q.append(state.q)
        self.trace_r.append(state.r)

        lines = [
            f"Airspeed: {airspeed:5.1f} m/s",
            f"Altitude: {altitude:5.1f} m",
            f"Bank  : {math.degrees(state.phi):6.1f} deg (cmd {math.degrees(targets.bank_rad):5.1f})",
            f"Pitch : {math.degrees(state.theta):6.1f} deg (cmd {math.degrees(targets.pitch_rad):5.1f})",
            f"Throttle: {state.throttle_actual * 100:5.1f}% (cmd {targets.throttle * 100:5.1f}%)",
            f"Aileron: {math.degrees(controls.aileron):6.1f} deg | Elevator: {math.degrees(controls.elevator):6.1f} deg",
            f"Rates p/q/r: {math.degrees(state.p):5.1f}/{math.degrees(state.q):5.1f}/{math.degrees(state.r):5.1f} deg/s",
        ]

        if self.font_available and self.font is not None and self.small_font is not None:
            y = 12
            for i, txt in enumerate(lines):
                font = self.font if i < 5 else self.small_font
                surf = font.render(txt, True, BLACK)
                self.screen.blit(surf, (56, y))
                y += 28 if i < 5 else 22

            self._draw_pilot_input_panel(targets)

        self._draw_scalar_gauges(state)

        self._draw_trace_plot(
            (self.width - 390, self.height - 230, 180, 200),
            [
                (self.trace_aileron, (250, 220, 80), 0.35),
                (self.trace_elevator, (120, 220, 220), 0.35),
            ],
            "Control Surface Deflections [rad]",
            ["Aileron (roll control)", "Elevator (pitch control)"],
        )

        self._draw_trace_plot(
            (self.width - 200, self.height - 230, 180, 200),
            [
                (self.trace_p, (240, 120, 120), math.radians(80.0)),
                (self.trace_q, (120, 240, 120), math.radians(80.0)),
                (self.trace_r, (120, 160, 240), math.radians(80.0)),
            ],
            "Body Rates [rad/s]",
            ["p: Roll rate", "q: Pitch rate", "r: Yaw rate"],
        )

        if self.font_available and self.small_font is not None:
            live_text = self.small_font.render(
                "Live attitude: Bank(phi), Pitch(theta) and rates p/q/r update every frame",
                True,
                (25, 25, 25),
            )
            self.screen.blit(live_text, (16, self.height - 194))

    def _draw_pilot_input_panel(self, targets: PilotTargets) -> None:
        panel_x = 16
        panel_y = self.height - 160
        panel_w = 420
        panel_h = 142

        pygame.draw.rect(self.screen, (22, 24, 30), (panel_x, panel_y, panel_w, panel_h), border_radius=8)
        pygame.draw.rect(self.screen, WHITE, (panel_x, panel_y, panel_w, panel_h), 1, border_radius=8)

        if self.small_font is not None and self.font_available:
            title = self.small_font.render("Pilot Inputs", True, WHITE)
            self.screen.blit(title, (panel_x + 10, panel_y + 8))

            annotations = [
                "Arrow Left/Right: command bank angle (\u00b145\u00b0 max)",
                "Arrow Up/Down: command pitch angle (\u00b145\u00b0 max)",
                "Number keys 0..9: latch throttle (0%..100%, 9=max)",
                "R: reset to runway start",
                f"Cmd Bank: {math.degrees(targets.bank_rad):6.1f} deg",
                f"Cmd Pitch: {math.degrees(targets.pitch_rad):6.1f} deg",
                f"Cmd Throttle: {targets.throttle * 100:6.1f}%",
            ]

            text_y = panel_y + 30
            for row in annotations:
                surface = self.small_font.render(row, True, (220, 220, 220))
                self.screen.blit(surface, (panel_x + 10, text_y))
                text_y += 16

    def _draw_display_guide(self) -> None:
        panel_w = 350
        panel_h = 158
        panel_x = self.width - panel_w - 56
        panel_y = 14

        pygame.draw.rect(self.screen, (22, 24, 30), (panel_x, panel_y, panel_w, panel_h), border_radius=8)
        pygame.draw.rect(self.screen, WHITE, (panel_x, panel_y, panel_w, panel_h), 1, border_radius=8)

        if self.small_font is not None and self.font_available:
            title = self.small_font.render("Display Guide", True, WHITE)
            self.screen.blit(title, (panel_x + 10, panel_y + 8))

            rows = [
                "Yellow center symbol: aircraft boresight reference",
                "Sky/Earth split line: true horizon attitude cue",
                "Ground grid/markers: perspective and climb-away depth",
                "Left tape (SPD): current airspeed magnitude",
                "Right tape (AGL): altitude above ground level",
                "Lower-left traces: aileron/elevator command history",
                "Lower-right traces: roll/pitch/yaw rate history (p/q/r)",
                "Pilot Inputs panel: key mapping + commanded setpoints",
            ]

            y = panel_y + 30
            for row in rows:
                surface = self.small_font.render(row, True, (220, 220, 220))
                self.screen.blit(surface, (panel_x + 10, y))
                y += 16

    def _draw_runway_reference(self, state: AircraftState) -> None:
        if state.z_d >= -2.0:
            runway_y = self.height - 60
            pygame.draw.rect(self.screen, (90, 90, 90), (self.width // 2 - 220, runway_y, 440, 40))
            pygame.draw.line(self.screen, WHITE, (self.width // 2 - 200, runway_y + 20), (self.width // 2 + 200, runway_y + 20), 3)
            for offset in range(-160, 200, 80):
                pygame.draw.line(
                    self.screen,
                    GREEN,
                    (self.width // 2 + offset, runway_y + 5),
                    (self.width // 2 + offset + 30, runway_y + 5),
                    3,
                )

    def _draw_notice(self, notice: str) -> None:
        if not notice or not self.font_available or self.small_font is None:
            return

        text_surface = self.small_font.render(notice, True, WHITE)
        x = self.width // 2 - text_surface.get_width() // 2
        y = 54
        pygame.draw.rect(self.screen, (25, 25, 30), (x - 10, y - 4, text_surface.get_width() + 20, text_surface.get_height() + 8), border_radius=6)
        pygame.draw.rect(self.screen, WHITE, (x - 10, y - 4, text_surface.get_width() + 20, text_surface.get_height() + 8), 1, border_radius=6)
        self.screen.blit(text_surface, (x, y))

    def render(self, state: AircraftState, targets: PilotTargets, controls: ControlInputs, notice: str = "", mode_name: str = "STABLE") -> None:
        self._draw_horizon(state)
        self._draw_runway_reference(state)
        self._draw_aircraft_symbol()
        self._draw_hud(state, targets, controls)
        self._draw_throttle_indicator(state, targets, mode_name)
        self._draw_display_guide()
        self._draw_notice(notice)
        pygame.display.flip()

    def close(self) -> None:
        pygame.quit()
