from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import dataclass

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from flightsim.config import AircraftParameters
from flightsim.dynamics import AircraftState, ControlInputs, FixedWingDynamics


STATE_NAMES = [
    "x_n",
    "y_e",
    "z_d",
    "u",
    "v",
    "w",
    "phi",
    "theta",
    "psi",
    "p",
    "q",
    "r",
    "throttle_actual",
]
INPUT_NAMES = ["aileron", "elevator", "rudder", "throttle_command"]


@dataclass
class TrimPoint:
    state: AircraftState
    control: ControlInputs


def to_state(vec: list[float]) -> AircraftState:
    return AircraftState(
        x_n=vec[0],
        y_e=vec[1],
        z_d=vec[2],
        u=vec[3],
        v=vec[4],
        w=vec[5],
        phi=vec[6],
        theta=vec[7],
        psi=vec[8],
        p=vec[9],
        q=vec[10],
        r=vec[11],
        throttle_actual=vec[12],
    )


def to_control(vec: list[float]) -> ControlInputs:
    return ControlInputs(
        aileron=vec[0],
        elevator=vec[1],
        rudder=vec[2],
        throttle_command=vec[3],
    )


def state_to_list(s: AircraftState) -> list[float]:
    return [
        s.x_n,
        s.y_e,
        s.z_d,
        s.u,
        s.v,
        s.w,
        s.phi,
        s.theta,
        s.psi,
        s.p,
        s.q,
        s.r,
        s.throttle_actual,
    ]


def control_to_list(u: ControlInputs) -> list[float]:
    return [u.aileron, u.elevator, u.rudder, u.throttle_command]


def f_cont(dyn: FixedWingDynamics, s: AircraftState, u: ControlInputs, dt: float = 1e-4) -> list[float]:
    sn = dyn.step(s, u, dt)
    sv = state_to_list(s)
    snv = state_to_list(sn)
    return [(snv[i] - sv[i]) / dt for i in range(len(sv))]


def solve_linear_system(a: list[list[float]], b: list[float]) -> list[float]:
    n = len(a)
    aug = [row[:] + [b[i]] for i, row in enumerate(a)]

    for col in range(n):
        pivot = col
        max_abs = abs(aug[col][col])
        for r in range(col + 1, n):
            v = abs(aug[r][col])
            if v > max_abs:
                max_abs = v
                pivot = r
        if max_abs < 1e-12:
            raise RuntimeError("Singular matrix in linear solve")

        if pivot != col:
            aug[col], aug[pivot] = aug[pivot], aug[col]

        p = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= p

        for r in range(n):
            if r == col:
                continue
            factor = aug[r][col]
            if factor == 0.0:
                continue
            for j in range(col, n + 1):
                aug[r][j] -= factor * aug[col][j]

    return [aug[i][n] for i in range(n)]


def find_trim(dyn: FixedWingDynamics, u0: float = 35.0, z0: float = -100.0) -> TrimPoint:
    # Unknowns: [w, theta, elevator, throttle]
    x = [0.0, math.radians(5.0), math.radians(-2.0), 0.45]

    def residual(xx: list[float]) -> list[float]:
        w, theta, elev, thr = xx
        s = AircraftState(
            x_n=0.0,
            y_e=0.0,
            z_d=z0,
            u=u0,
            v=0.0,
            w=w,
            phi=0.0,
            theta=theta,
            psi=0.0,
            p=0.0,
            q=0.0,
            r=0.0,
            throttle_actual=thr,
        )
        u = ControlInputs(aileron=0.0, elevator=elev, rudder=0.0, throttle_command=thr)
        fd = f_cont(dyn, s, u)
        return [fd[3], fd[5], fd[10], fd[2]]  # u_dot, w_dot, q_dot, z_dot

    for _ in range(20):
        r = residual(x)
        norm = math.sqrt(sum(v * v for v in r))
        if norm < 1e-8:
            break

        eps = [1e-4, 1e-5, 1e-5, 1e-5]
        j = [[0.0] * 4 for _ in range(4)]
        for c in range(4):
            xp = x[:]
            xm = x[:]
            xp[c] += eps[c]
            xm[c] -= eps[c]
            rp = residual(xp)
            rm = residual(xm)
            for rr in range(4):
                j[rr][c] = (rp[rr] - rm[rr]) / (2.0 * eps[c])

        dx = solve_linear_system(j, [-v for v in r])
        step = 1.0
        improved = False
        for _ls in range(8):
            xn = [x[i] + step * dx[i] for i in range(4)]
            xn[3] = max(0.05, min(0.95, xn[3]))
            rn = residual(xn)
            if math.sqrt(sum(v * v for v in rn)) < norm:
                x = xn
                improved = True
                break
            step *= 0.5
        if not improved:
            x = [x[i] + 0.25 * dx[i] for i in range(4)]
            x[3] = max(0.05, min(0.95, x[3]))

    w, theta, elev, thr = x
    state = AircraftState(
        x_n=0.0,
        y_e=0.0,
        z_d=z0,
        u=u0,
        v=0.0,
        w=w,
        phi=0.0,
        theta=theta,
        psi=0.0,
        p=0.0,
        q=0.0,
        r=0.0,
        throttle_actual=thr,
    )
    ctrl = ControlInputs(aileron=0.0, elevator=elev, rudder=0.0, throttle_command=thr)
    return TrimPoint(state=state, control=ctrl)


def linearize(dyn: FixedWingDynamics, trim: TrimPoint) -> tuple[list[list[float]], list[list[float]]]:
    x0 = state_to_list(trim.state)
    u0 = control_to_list(trim.control)
    n = len(x0)
    m = len(u0)

    A = [[0.0 for _ in range(n)] for _ in range(n)]
    B = [[0.0 for _ in range(m)] for _ in range(n)]

    for j in range(n):
        eps = 1e-5 if j not in (3, 4, 5) else 1e-3
        xp = x0[:]
        xm = x0[:]
        xp[j] += eps
        xm[j] -= eps
        fp = f_cont(dyn, to_state(xp), trim.control)
        fm = f_cont(dyn, to_state(xm), trim.control)
        for i in range(n):
            A[i][j] = (fp[i] - fm[i]) / (2.0 * eps)

    for j in range(m):
        eps = 1e-5
        up = u0[:]
        um = u0[:]
        up[j] += eps
        um[j] -= eps
        fp = f_cont(dyn, trim.state, to_control(up))
        fm = f_cont(dyn, trim.state, to_control(um))
        for i in range(n):
            B[i][j] = (fp[i] - fm[i]) / (2.0 * eps)

    return A, B


def fmt_matrix(mat: list[list[float]], digits: int = 4) -> str:
    rows = []
    for row in mat:
        rows.append("[" + ", ".join(f"{v: .{digits}f}" for v in row) + "]")
    return "\n".join(rows)


def main() -> None:
    params = AircraftParameters()
    dyn = FixedWingDynamics(params)
    trim = find_trim(dyn)
    A, B = linearize(dyn, trim)

    fd = f_cont(dyn, trim.state, trim.control)

    print("Trim state:")
    print({
        "u_mps": trim.state.u,
        "w_mps": trim.state.w,
        "theta_deg": math.degrees(trim.state.theta),
        "elevator_deg": math.degrees(trim.control.elevator),
        "throttle": trim.control.throttle_command,
        "alpha_eff_deg_approx": math.degrees(math.atan2(trim.state.w, trim.state.u) + params.wing_incidence_rad),
    })
    print("Residual derivatives at trim [u_dot, w_dot, q_dot, z_dot]:")
    print([fd[3], fd[5], fd[10], fd[2]])

    print("\nA matrix (13x13):")
    print(fmt_matrix(A))
    print("\nB matrix (13x4):")
    print(fmt_matrix(B))

    # Common output choice for full-state output y=x.
    n = len(A)
    m = len(B[0])
    C = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    D = [[0.0 for _ in range(m)] for _ in range(n)]
    print("\nC matrix (identity, full-state output):")
    print(fmt_matrix(C, digits=1))
    print("\nD matrix (zeros for full-state output):")
    print(fmt_matrix(D, digits=1))


if __name__ == "__main__":
    main()
