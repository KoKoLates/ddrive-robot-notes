import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

from typing import Callable

from utils import Application
from robot import Robot, PID, MPC

from map import load_map
from utils import compute_distance


class Pipeline(object):
    def __init__(
        self, robot: Robot, waypoints: list[tuple[int, int]], ctrl_func: Callable
    ) -> None:
        self.robot = robot
        self.waypoints = waypoints
        self.ctrl_func = ctrl_func
        self.history = []
        self.error = []
        self.index = 0

        self.times = []
        self.current_time = 0

    def update(self, dt: float) -> None:
        if self.index < len(self.waypoints):
            p, _ = self.robot.state
            target = self.waypoints[self.index]
            self.history.append((int(p[0, 0]), int(p[1, 0])))
            self.times.append(self.current_time)

            v, w = self.ctrl_func(p, target)
            d: float = compute_distance(tuple(p[:2].flatten()), target)
            self.error.append(d)
            if d < 30:
                self.index += 1
        else:
            v, w = 0, 0

        self.robot.set_robot_speeds(v, w)
        self.robot.update(dt)
        self.current_time += dt

    def extract_history(self) -> tuple:
        x = [p[0] for p in self.history]
        y = [p[1] for p in self.history]
        return x, y, self.error, self.times


def main() -> None:
    waypoints = load_map(args.f)
    app: Application = Application((500, 500), "Comparison")

    # PID
    pid_robot = Robot(50, 50)
    mpc_robot = Robot(50, 50)

    pid_ctrl = PID(0.5, 0, 0.1, 0.5, 0, 0.1)
    mpc_ctrl = MPC(5)

    pid = Pipeline(pid_robot, waypoints, pid_ctrl.update)
    mpc = Pipeline(
        mpc_robot, waypoints, lambda _, target: mpc_ctrl.update(target, mpc_robot)
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

    # plot history
    # _, axs = plt.subplots(3, figsize=(6, 6))

    # pid_x, pid_y, pid_err, pid_time = pid.extract_history()
    # mpc_x, mpc_y, mpc_err, mpc_time = mpc.extract_history()

    # pid_color = np.array((255, 165, 0)) / 255
    # mpc_color = np.array((0, 255, 255)) / 255


    # axs[0].plot(pid_time, pid_x, label="PID", color=pid_color)
    # axs[0].plot(mpc_time, mpc_x, label="MPC", color=mpc_color)
    # axs[0].legend()

    # axs[1].plot(pid_time, pid_y, label="PID", color=pid_color)
    # axs[1].plot(mpc_time, mpc_y, label="MPC", color=mpc_color)
    # axs[1].legend()

    # axs[2].plot(pid_time, pid_err, label="PID", color=pid_color)
    # axs[2].plot(mpc_time, mpc_err, label="MPC", color=mpc_color)
    # axs[2].legend()

    # plt.tight_layout()
    # plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", type=str, required=True)
    args = parser.parse_args()

    if not os.path.exists(args.f):
        raise FileNotFoundError

    main()
