import numpy as np


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
        body_to_goal: float = self.angle(current_pose[0, 0], current_pose[1, 0], target_pose[0], target_pose[1])
        body_to_nose: float = self.angle(current_pose[0, 0], current_pose[1, 0], nose[0], nose[1])

        error_angle: float = (-body_to_goal) - current_pose[2, 0]

        linear_ctrl = self.lkp * pose_err + self.lkd * (pose_err - self.prev_p_err)
        angular_ctrl = self.akp * error_angle + self.akd * (error_angle - self.prev_a_err)

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
