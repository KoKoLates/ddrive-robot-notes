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

        self._update_polygon()

    def update(self, dt) -> None:
        self.wheel_speeds[self.wheel_speeds>3] = 3
        self.wheel_speeds[self.wheel_speeds<-3] = -3
        self._forward()

        a: np.ndarray = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1] 
        ])
        b: np.ndarray = np.array([
            [np.sin(self.p[2, 0] + np.pi/2) * dt, 0],
            [np.cos(self.p[2, 0] + np.pi/2) * dt, 0],
            [0, dt]
        ])
        vel: np.ndarray = np.array([
            [self.v[0, 0]],
            [self.v[2, 0]]
        ])
        self.p = a @ self.p + b @ vel
        self._inverse()

    def points(self) -> np.ndarray:
        self._update_polygon()
        return self.polygon

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

    def _update_polygon(self) -> None:
        mat: np.ndarray = np.array([
            [ np.cos(self.p[2, 0]), np.sin(self.p[2, 0]), self.p[0, 0]],
            [-np.sin(self.p[2, 0]), np.cos(self.p[2, 0]), self.p[1, 0]],
            [0, 0, 1]
        ])
        self.polygon: np.ndarray = (self.dimensions @ mat.T).astype('int')
