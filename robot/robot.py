import numpy as np


class Robot(object):
    def __init__(
        self, x: int, y: int, b: int = 15, wheel_radius: int = 5
    ) -> None:
        self.p: np.ndarray = np.reshape([x, y, 0], (3, 1)).astype(np.float64)
        self.v: np.ndarray = np.reshape([0, 0, 0], (3, 1))

        self.wheel_radius: int = wheel_radius
        self.wheel_speeds: np.ndarray = np.zeros((2, 1), dtype=float)

        self.b: int = b
        self.framework: np.ndarray = np.array([
            [-self.b, -self.b, 1], [self.b, -self.b, 1], [self.b, 0, 1], 
            [0, 0, 1], [self.b, 0, 1], [self.b, self.b, 1], [-self.b, self.b, 1]
        ])
        self._update_polygon()

    def update(self, dt: float = 0.5) -> None:
        self.wheel_speeds = np.clip(self.wheel_speeds, -3, 3)
        self._forward()

        mat: np.ndarray = np.array([
            [np.sin(self.p[2, 0] + np.pi / 2) * dt, 0],
            [np.cos(self.p[2, 0] + np.pi / 2) * dt, 0],
            [0, dt]
        ])
        vel: np.ndarray = self.v[[0, 2], :]
        self.p += mat @ vel
        self._inverse()
    
    @property
    def points(self) -> np.ndarray:
        self._update_polygon()
        return self.polygon
    
    @property
    def state(self) -> tuple[np.ndarray, np.ndarray]:
        return self.p, self.v
    
    def set_robot_speeds(self, v: float, w: float) -> None:
        self.v = np.reshape([v, 0, w], (3, 1))
        self._inverse()
    
    def set_wheel_speeds(self, left: float, right: float) -> None:
        self.wheel_speeds = np.reshape([left, right], (2, 1))
        self._forward()

    def _forward(self) -> None:
        mat: np.ndarray = np.array([
            [self.wheel_radius / 2, self.wheel_radius / 2],
            [0, 0], 
            [self.wheel_radius / (self.b * 2), -self.wheel_radius / (self.b * 2)]
        ])
        self.v = mat @ self.wheel_speeds
    
    def _inverse(self) -> None:
        mat: np.ndarray = np.array([
            [1 / self.wheel_radius, 0, self.b / self.wheel_radius],
            [1 / self.wheel_radius, 0,-self.b / self.wheel_radius]
        ])
        self.wheel_speeds = mat @ self.v

    def _update_polygon(self) -> None:
        mat: np.ndarray = np.array([
            [ np.cos(self.p[2, 0]), np.sin(self.p[2, 0]), self.p[0, 0]],
            [-np.sin(self.p[2, 0]), np.cos(self.p[2, 0]), self.p[1, 0]],
            [0, 0, 1]
        ])
        self.polygon = (self.framework @ mat.T).astype(int)

