import numpy as np

from dev.robot import Robot
from scipy.optimize import minimize


class MPC(object):
    def __init__(self, robot: Robot, horizon: int = 5) -> None:
        self.horizon: int = horizon
        self.R: np.ndarray = np.diag([0.01, 0.01])  # input cost matrix
        self.Rd: np.ndarray = np.diag([0.01, 1.0])  # input difference cost matrix
        self.Q: np.ndarray = np.diag([1.0, 1.0])  # state cost matrix
        self.Qf: np.ndarray = self.Q  # state final matrix

        self.robot: Robot = robot

    def _cost(self, u_k: np.ndarray, target: tuple[int, int]):
        t: np.ndarray = np.array(target)
        u_k = u_k.reshape(2, self.horizon)
        z_k: np.ndarray = np.zeros((2, self.horizon + 1))

        cost = 0.0

        for i in range(self.horizon):
            self.robot.set_robot_speeds(u_k[0, i], u_k[1, i])
            self.robot.update(0.5)
            x, _ = self.robot.state()
            z_k[:, i] = [x[0, 0], x[1, 0]]
            cost += np.sum(self.R @ (u_k[:, i] ** 2))
            cost += np.sum(self.Q @ ((t - z_k[:, i]) ** 2))
            if i < (self.horizon - 1):
                cost += np.sum(self.Rd @ ((u_k[:, i + 1] - u_k[:, i]) ** 2))

        return cost

    def update(self, target: tuple) -> tuple[float, float]:
        bound: list = [(0, 5), (-3, 3)] * self.horizon
        results = minimize(
            self._cost,
            x0=np.zeros((self.horizon * 2)),
            args=(target,),
            method="SLSQP",
            bounds=bound,
        )
        return results.x[0], results.x[1]


class PID(object):
    def __init__(
        self,
        lkp: float = 0.1,
        lkd: float = 0.1,
        lki: float = 0,
        akp: float = 0.1,
        akd: float = 0.1,
        aki: float = 0,
    ) -> None:
        ## parameters of linear controller
        self.lkp: float = lkp
        self.lkd: float = lkd
        self.lki: float = lki

        ## parameters of angular (attitude) controller
        self.akp: float = akp
        self.akd: float = akd
        self.aki: float = aki

        self.prev_p_err: float = 0
        self.prev_a_err: float = 0
        self.prev_diff: float = 0
        self.prev_index: int = -1

    def update(
        self,
        current_pose: np.ndarray,
        target_pose: tuple,
        nose: np.ndarray,
        waypoint_index: int,
    ) -> tuple[float, float]:
        pose_err: float = self.distance(
            current_pose[0, 0], current_pose[1, 0], target_pose[0], target_pose[1]
        )
        body_to_goal: float = self.angle(
            current_pose[0, 0], current_pose[1, 0], target_pose[0], target_pose[1]
        )
        body_to_nose: float = self.angle(
            current_pose[0, 0], current_pose[1, 0], nose[0], nose[1]
        )

        error_angle: float = (-body_to_goal) - current_pose[2, 0]

        linear_ctrl = self.lkp * pose_err + self.lkd * (pose_err - self.prev_p_err)
        angular_ctrl = self.akp * error_angle + self.akd * (
            error_angle - self.prev_a_err
        )

        self.prev_a_err = error_angle
        self.prev_p_err = pose_err

        self.prev_index = waypoint_index
        self.prev_diff = body_to_goal

        if linear_ctrl > 5:
            linear_ctrl = 5

        return linear_ctrl, angular_ctrl

    @staticmethod
    def distance(x1: int, y1: int, x2: int, y2: int) -> float:
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    @staticmethod
    def angle(x1: int, y1: int, x2: int, y2: int) -> float:
        return np.arctan2(y2 - y1, x2 - x1)
