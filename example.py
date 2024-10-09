import os
import argparse
import numpy as np

from typing import Callable

from utils import Application
from robot import Robot, PID, MPC

from map import load_map
from utils import compute_distance


class Pipeline(object):
    def __init__(
        self,
        robot: Robot, 
        waypoints: list[tuple[int, int]],
        update_func: Callable[[np.ndarray, tuple[int, int]], tuple[float, float]]
    ) -> None:
        self.robot: Robot = robot
        self.ctrl: Callable = update_func

        self.waypoints: list = waypoints
        self.index: int = 0
        self.history: list = []
        self.error: list = []

        self.times: list = []
        self.current_time: float = 0

    def update(self, dt: float) -> None:
        if self.index < len(self.waypoints):
            p, _ = self.robot.state
            self.history.append((int(p[0, 0]), int(p[1, 0])))

            target = self.waypoints[self.index]
            v, w = self.ctrl(p, target)
            d: float = compute_distance(tuple(p[:2].flatten()), target)
            self.error.append(d)
            self.times.append(self.current_time)

            if d < 30:
                self.index += 1
        else:
            v, w = 0, 0

        self.robot.set_robot_speeds(v, w)
        self.robot.update(dt)
        self.current_time += dt

    def extract_history(self) -> tuple:
        x: list = [p[0] for p in self.history]
        y: list = [p[1] for p in self.history]
        return x, y, self.error, self.times


def main() -> None:
    waypoints = load_map(args.map)
    app: Application = Application((500, 500), "Example")

    # initialize robot and controller
    pid_ctrl = PID(0.5, 0, 0.1, 0.4, 0, 0.1)
    mpc_ctrl = MPC(5)

    pid_robot = Robot(50, 50)
    mpc_robot = Robot(50, 50)

    # initialize the pipeline
    pid = Pipeline(pid_robot, waypoints, pid_ctrl.update)
    mpc = Pipeline(
        mpc_robot, waypoints, 
        lambda _, target: mpc_ctrl.update(target, mpc_robot)
    )

    while True:
        app.clean()
        app.plot(pid_robot.points, (0, 165, 255))
        app.plot(mpc_robot.points, (255, 255, 0))

        app.plot_path(waypoints)
        app.plot_path(pid.history, (0, 165, 255), True)
        app.plot_path(mpc.history, (255, 255, 0), True)

        if app.show() & 0xFF == ord("q"):
            break

        pid.update(0.5)
        mpc.update(0.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-m", "--map", 
        type=str, required=True, 
        help="the tracking map for the example"
    )
    args = parser.parse_args()

    if not os.path.exists(args.map):
        raise FileNotFoundError

    main()
