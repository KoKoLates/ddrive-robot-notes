import numpy as np

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
            self.Kp_v * err_v + 
            self.Ki_v * self.accu_err_v + 
            self.Kd_v * (err_v - self.prev_err_v)
        )
        ctrl_w: float = (
            self.Kp_w * err_w +
            self.Ki_w * self.accu_err_w + 
            self.Kd_w * (err_w - self.prev_err_w)
        )

        self.prev_err_v, self.prev_err_w = err_v, err_w
        
        ctrl_v = np.clip(ctrl_v, -5, 5)
        ctrl_w = np.clip(ctrl_w, -np.pi, np.pi)
        return ctrl_v, ctrl_w
