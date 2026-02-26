import math

import pygame

from flightsim.sim import FlightSimulatorApp


class _DummyDisplay:
    def __init__(self, *_args, **_kwargs):
        pass

    def render(self, *_args, **_kwargs):
        pass

    def close(self):
        pass


class _KeyState:
    def __init__(self, pressed: set[int] | None = None):
        self.pressed = pressed or set()

    def __getitem__(self, key: int) -> bool:
        return key in self.pressed


def _make_app(monkeypatch) -> FlightSimulatorApp:
    monkeypatch.setattr("flightsim.sim.FlightDisplay", _DummyDisplay)
    return FlightSimulatorApp()


def test_digit_mapping_supports_top_row_and_keypad(monkeypatch):
    app = _make_app(monkeypatch)
    assert app._digit_from_key(pygame.K_9) == 9
    assert app._digit_from_key(pygame.K_0) == 0
    assert app._digit_from_key(pygame.K_KP9) == 9
    assert app._digit_from_key(pygame.K_KP0) == 0


def test_numeric_key_latches_throttle_without_holding(monkeypatch):
    app = _make_app(monkeypatch)

    monkeypatch.setattr(
        "pygame.event.get",
        lambda: [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_9)],
    )
    monkeypatch.setattr("pygame.key.get_pressed", lambda: _KeyState())

    assert app._process_input() is True
    assert app.targets.throttle == 1.0

    monkeypatch.setattr("pygame.event.get", lambda: [])
    monkeypatch.setattr("pygame.key.get_pressed", lambda: _KeyState())

    assert app._process_input() is True
    assert app.targets.throttle == 1.0


def test_arrow_keydown_updates_pitch_and_bank_targets(monkeypatch):
    app = _make_app(monkeypatch)
    start_pitch = app.targets.pitch_rad
    start_bank = app.targets.bank_rad

    monkeypatch.setattr(
        "pygame.event.get",
        lambda: [
            pygame.event.Event(pygame.KEYDOWN, key=pygame.K_UP),
            pygame.event.Event(pygame.KEYDOWN, key=pygame.K_RIGHT),
        ],
    )
    monkeypatch.setattr("pygame.key.get_pressed", lambda: _KeyState())

    app._process_input()

    assert app.targets.pitch_rad > start_pitch
    assert app.targets.bank_rad > start_bank


def test_held_arrow_keys_continue_adjusting_targets(monkeypatch):
    app = _make_app(monkeypatch)
    start_pitch = app.targets.pitch_rad

    monkeypatch.setattr("pygame.event.get", lambda: [])
    monkeypatch.setattr("pygame.key.get_pressed", lambda: _KeyState({pygame.K_UP}))

    app._process_input()

    assert app.targets.pitch_rad > start_pitch


def test_arrow_targets_are_clamped_to_limits(monkeypatch):
    app = _make_app(monkeypatch)

    app.targets.pitch_rad = math.radians(44.9)
    app.targets.bank_rad = math.radians(44.9)
    monkeypatch.setattr("pygame.event.get", lambda: [])
    monkeypatch.setattr("pygame.key.get_pressed", lambda: _KeyState({pygame.K_UP, pygame.K_RIGHT}))
    app._process_input()

    assert app.targets.pitch_rad <= math.radians(45.0)
    assert app.targets.bank_rad <= math.radians(45.0)


def test_mode_toggle_key_switches_performance_mode(monkeypatch):
    app = _make_app(monkeypatch)
    assert app.controller.performance_mode is False

    monkeypatch.setattr(
        "pygame.event.get",
        lambda: [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_m)],
    )
    monkeypatch.setattr("pygame.key.get_pressed", lambda: _KeyState())

    app._process_input()

    assert app.controller.performance_mode is True
    assert "Mode:" in app.notice_text
