import numpy as np

from robot import Robot
from scipy.optimize import minimize
from utils import compute_angle, compute_distance


class PID(object):
    def __init__(
        self,
        Kp_v: float = 0.1,
        Ki_v: float = 0.0,
        Kd_v: float = 0.1,
        Kp_w: float = 0.1,
        Ki_w: float = 0.0,
        Kd_w: float = 0.1,
    ) -> None:
        # linear velocity (v) and angular velocity (w)
        self.Kp_v, self.Ki_v, self.Kd_v = Kp_v, Ki_v, Kd_v
        self.Kp_w, self.Ki_w, self.Kd_w = Kp_w, Ki_w, Kd_w

        self.prev_err_v: float = 0.0
        self.prev_err_w: float = 0.0
        self.accu_err_v: float = 0.0
        self.accu_err_w: float = 0.0

    def update(self, state: np.ndarray, target: tuple[int, int]) -> tuple[float, float]:
        pose: tuple = tuple(state[:2].flatten())
        head: np.ndarray = state[2, 0]

        err_v = compute_distance(pose, target)
        err_w = float(
            np.arctan2(
                np.sin(-compute_angle(pose, target) - head),
                np.cos(-compute_angle(pose, target) - head),
            )
        )
        self.accu_err_v += err_v
        self.accu_err_w += err_w

        ctrl_v: float = (
            self.Kp_v * err_v
            + self.Ki_v * self.accu_err_v
            + self.Kd_v * (err_v - self.prev_err_v)
        )
        ctrl_w: float = (
            self.Kp_w * err_w
            + self.Ki_w * self.accu_err_w
            + self.Kd_w * (err_w - self.prev_err_w)
        )

        self.prev_err_v, self.prev_err_w = err_v, err_w

        ctrl_v = np.clip(ctrl_v, -10, 10)
        ctrl_w = np.clip(ctrl_w, -np.pi, np.pi)
        return ctrl_v, ctrl_w

    def info(self) -> tuple[str, str]:
        v: str = f"v: Kp {self.Kp_v:.1f} Ki {self.Ki_v:.1f} Kd {self.Kd_v:.1f}"
        w: str = f"w: Kp {self.Kp_w:.1f} Ki {self.Ki_w:.1f} Kd {self.Kd_w:.1f}"
        return v, w


class MPC(object):
    def __init__(self, window: int = 5) -> None:
        self.window: int = window
        self.Q: np.ndarray = np.diag([1.0, 1.0])    # state cost matrix
        self.R: np.ndarray = np.diag([0.01, 0.01])  # input cost matrix
        self.Rd: np.ndarray = np.diag([0.01, 1.0])  # input difference cost matrix

    def update(self, target: tuple[int, int], robot: Robot) -> tuple[float, float]:
        bounds: list = [(-10, 10), (-np.pi, np.pi)] * self.window
        ctrl = minimize(
            self._cost,
            x0=np.zeros((self.window * 2)),
            args=(np.array(target), robot),
            method="SLSQP",
            bounds=bounds,
        )
        return ctrl.x[0], ctrl.x[1]

    def _cost(self, u_k: np.ndarray, target: np.ndarray, robot: Robot) -> float:
        state: tuple[np.ndarray, np.ndarray] = robot.state

        u_k = u_k.reshape(self.window, 2).T
        x_k = np.zeros((2, self.window + 1))

        # input cost
        cost = np.sum(self.R @ (u_k**2))

        # state cost
        for i in range(self.window):
            robot.set_robot_speeds(u_k[0, i], u_k[1, i])
            robot.update()
            p, _ = robot.state
            x_k[:, i] = p[:2].flatten()

        state_error = target[:, np.newaxis] - x_k[:, :-1]
        cost += np.sum(self.Q @ (state_error**2))

        # input difference cost
        input_diff: np.ndarray = np.diff(u_k, axis=1)
        cost += np.sum(self.Rd @ (input_diff**2))

        robot.set_state(state[0], state[1])

        return float(cost)
