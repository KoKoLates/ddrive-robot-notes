import numpy as np

class Robot(object):
    def __init__(self, x: float, y: float) -> None:
        self.p: np.ndarray = np.reshape([x, y, 0], (3, 1))
        self.v: np.ndarray = np.reshape([0, 0, 0], (3, 1))
        self.wheel_speeds: np.ndarray = np.reshape(
            [0, 0], (2, 1)
        )

        self.r: int = 5
        self.b: int = 25

        self.dimensions: np.ndarray = np.array([
            [-self.b, -self.b, 1],
            [0, -self.b, 1],
            [self.b, 0, 1],
            [0, self.b, 1],
            [-self.b, self.b, 1]
        ])

    def state(self) -> tuple[np.ndarray, np.ndarray]:
        return self.p, self.v

    def set_wheel_speeds(self, l: float, r: float) -> None:
        self.wheel_speeds = np.reshape([l, r], (2, 1))
        self._forward()

    def set_robot_speeds(self, linear: float, angular: float) -> None:
        self.v = np.reshape([linear, 0, angular], (3, 1))
        self._inverse()
    
    def _forward(self) -> None:
        mat: np.ndarray = np.array([
            [self.r/2, self.r/2],
            [0, 0]
        ])
        self.v = mat @ self.wheel_speeds

    def _inverse(self) -> None:
        mat: np.ndarray = np.array([
            [1/self.r, 0, self.b/self.r],
            [1/self.r, 0, -self.b/self.r]
        ])
        self.wheel_speeds = mat @ self.v
