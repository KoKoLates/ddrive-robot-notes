import random
import numpy as np
import matplotlib.pyplot as plt

from typing import Callable
from scipy.interpolate import CubicSpline

from utils import Application
from robot import Robot, PID, MPC

from utils import compute_distance


def generate_random_curved_path(
    num_points: int, width: int, height: int
) -> list[tuple[int, int]]:
    # Generate random control points within the specified width and height
    control_points_x = [random.randint(50, width - 50) for _ in range(num_points)]
    control_points_y = [random.randint(50, height - 50) for _ in range(num_points)]

    # Parameterize the curve
    t = np.linspace(0, 1, num_points)

    # Use cubic spline to interpolate a smooth curve through the random control points
    spline_x = CubicSpline(t, control_points_x, bc_type="natural")
    spline_y = CubicSpline(t, control_points_y, bc_type="natural")

    # Sample more points from the splines for a smoother curve
    t_fine = np.linspace(0, 1, num_points * 10)
    waypoints = [(int(spline_x(ti)), int(spline_y(ti))) for ti in t_fine]

    return waypoints


def square_path_generator(
    x: int, y: int, border: int, step: int = 20
) -> list[tuple[int, int]]:
    waypoints: list[tuple[int, int]] = []

    waypoints += [(i, y) for i in range(x, x + border, step)]
    waypoints += [(x + border, j) for j in range(y, y + border, step)]
    waypoints += [(i, y + border) for i in range(x + border, x, -step)]
    waypoints += [(x, j) for j in range(y + border, y, -step)]

    return waypoints


def circle_path_generator(
    x: int, y: int, radius: int, num_wps: int = 100
) -> list[tuple[int, int]]:
    waypoints: list = []
    for i in range(num_wps):
        angle: float = np.pi * 2 * (i / num_wps)
        _x: int = x + int(radius * np.sin(angle))
        _y: int = y - int(radius * np.cos(angle))
        waypoints.append((_x, _y))
    return waypoints


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
            if d < 10:
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

def generate_waypoints_time(waypoints: list[tuple[int, int]], speed: float = 50) -> list[float]:
    times = [0.0]  # Start time at 0
    for i in range(1, len(waypoints)):
        d = compute_distance(waypoints[i - 1], waypoints[i])
        dt = d / speed
        times.append(times[-1] + dt)
    return times

def main() -> None:
    waypoints: list[tuple[int, int]] = square_path_generator(100, 100, 100)
    # waypoints = circle_path_generator(250, 250, 50)
    app: Application = Application((500, 500), "Comparison")

    # PID
    pid_robot = Robot(100, 100)
    mpc_robot = Robot(100, 100)

    pid_ctrl = PID(0.5, 0.1, 0, 3, 0.1, 0)
    mpc_ctrl = MPC(5)

    pid = Pipeline(pid_robot, waypoints, pid_ctrl.update)
    mpc = Pipeline(
        mpc_robot, waypoints, lambda _, target: mpc_ctrl.update(target, mpc_robot)
    )

    while True:
        app.clean()
        app.plot_path(waypoints)
        app.plot_path(pid.history, (0, 165, 255), True)
        app.plot_path(mpc.history, (255, 255, 0), True)

        app.plot(pid_robot.points, (0, 165, 255))
        app.plot(mpc_robot.points, (255, 255, 0))

        if app.show() == ord("q"):
            break

        pid.update(0.5)
        mpc.update(0.5)

    # plot history
    fig, axs = plt.subplots(3, figsize=(6, 6))

    pid_x, pid_y, pid_err, pid_time = pid.extract_history()
    mpc_x, mpc_y, mpc_err, mpc_time = mpc.extract_history()
    wp_x = [p[0] for p in waypoints]
    wp_y = [p[1] for p in waypoints]
    wp_time = generate_waypoints_time(waypoints)


    axs[0].plot(pid_time, pid_x, label="PID")
    axs[0].plot(mpc_time, mpc_x, label="MPC")
    # axs[0].plot(pid_time, wp_x, label="Ground Truth")
    axs[0].legend()

    axs[1].plot(pid_time, pid_y, label="PID")
    axs[1].plot(mpc_time, mpc_y, label="MPC")
    # axs[1].plot(pid_time, wp_y, label="Ground Truth")
    axs[1].legend()

    axs[2].plot(pid_time, pid_err, label="PID")
    axs[2].plot(mpc_time, mpc_err, label="MPC")
    axs[2].legend()

    plt.tight_layout()
    plt.show()
    


if __name__ == "__main__":
    main()
