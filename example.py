import random
import numpy as np

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


def square_path_generator(x: int, y: int, border: int, step: int = 20) -> list[tuple[int, int]]:
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


def main() -> None:
    # waypoints: list[tuple[int, int]] = square_path_generator(125, 125, 250)
    waypoints = circle_path_generator(250, 250, 125)
    app: Application = Application((500, 500), "Comparison")

    # PID
    pid_robot = Robot(250, 125)
    pid_ctrl = PID(0.5, 0.1, 0, 3, 0.1, 0)
    pid_traj = []
    pid_idx = 0

    # MPC
    mpc_robot = Robot(250, 125)
    mpc_ctrl = MPC(5)
    mpc_traj = []
    mpc_idx = 0

    while True:
        app.clean()

        if len(waypoints):
            app.plot_path(waypoints)

        if len(pid_traj):
            app.plot_path(pid_traj, (0, 165, 255), True)
        
        if len(mpc_traj):
            app.plot_path(mpc_traj, (255, 255, 0), True)

        app.plot(pid_robot.points, (0, 165, 255))
        app.plot(mpc_robot.points, (255, 255, 0))

        k: int = app.show()

        
        p, _ = pid_robot.state
        if   len(waypoints)>0 and pid_idx < len(waypoints):
            pid_traj.append((int(p[0, 0]), int(p[1, 0])))
            target = waypoints[pid_idx]

            v, w = pid_ctrl.update(p, target)
            d: float = compute_distance(tuple(p[:2].flatten()), target)

            if d < 10:
                pid_idx += 1
        else:
            v, w = 0, 0

        pid_robot.set_robot_speeds(v, w)
        pid_robot.update(0.5)

        p, _ = mpc_robot.state
        if  len(waypoints) and mpc_idx < len(waypoints):
            mpc_traj.append((int(p[0, 0]), int(p[1, 0])))
            target = waypoints[mpc_idx]

            # v, w = ctrl.update(p, target)
            v, w = mpc_ctrl.update(target, mpc_robot)
            d: float = compute_distance(tuple(p[:2].flatten()), target)

            if d < 10:
                mpc_idx += 1
        else:
            v, w = 0, 0

        mpc_robot.set_robot_speeds(v, w)
        mpc_robot.update(0.5)

        if k == ord("q"):
            break


if __name__ == "__main__":
    main()
