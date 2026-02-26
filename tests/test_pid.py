from flightsim.pid import PID


def test_pid_reaches_target_without_bias():
    pid = PID(kp=1.2, ki=0.8, kd=0.0, min_output=-10.0, max_output=10.0)

    y = 0.0
    dt = 0.02
    for _ in range(250):
        u = pid.update(target=1.0, measured=y, dt=dt)
        y += (u - y) * 0.2  # simple first-order plant

    assert abs(y - 1.0) < 0.1


def test_pid_clamps_output():
    pid = PID(kp=100.0, ki=0.0, kd=0.0, min_output=-0.5, max_output=0.5)
    out = pid.update(target=1.0, measured=0.0, dt=0.1)
    assert out == 0.5
