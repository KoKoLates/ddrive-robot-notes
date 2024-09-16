import cv2
import numpy as np

from dev import PID, MPC, Robot, Application


def main() -> None:
    waypoints: list[tuple[int, int]] = []

    app: Application = Application(700, 700, "simulation", None)

    ## PID controller
    pid_r: Robot = Robot(50, 50)
    pid_ctrl: PID = PID(0.5, 0.1, 0, 3, 0.1, 0)

    ## MPC controller
    mpc_r: Robot = Robot(50, 50)
    mpc_ctrl: MPC = MPC()

    ## define waypoints
    for x in range(200, 600, 2):
        y: np.float32 = 350 + np.sin(np.pi * 2 * 0.25 * (x + 200) / 100) * 200
        waypoints.append((x, int(y)))

    pid_idx: int = 0
    pid_traj: list = []
    
    mpc_idx: int = 0
    mpc_traj: list = []

    while True:
        app.clear()

        if len(waypoints):
            app.plot_path(waypoints)

        if len(pid_traj):
            app.plot_path(pid_traj, (0, 0, 255))

        if len(mpc_traj):
            app.plot_path(mpc_traj, (0, 255, 0))

        app.plot(mpc_r.points(), (0, 255, 0))
        app.plot(pid_r.points(), (0, 0, 255))

        app.text("PID", color=(0, 0, 255), org=(100, 50))
        app.text("MPC", color=(0, 255, 0), org=(100, 75))
        app.text("Trajectory", org=(100, 100))

        k: int = app.show()

        x, _ = pid_r.state()
        if len(waypoints) and pid_idx != len(waypoints):
            pid_traj.append((int(x[0, 0]), int(x[1, 0])))
            target = waypoints[pid_idx]
            l_v, a_v = pid_ctrl.update(x, target, pid_r.points()[2], pid_idx)
            distance: float = PID.distance(x[0, 0], x[1, 0], target[0], target[1])

            if distance < 5:
                pid_idx += 1

        else:
            l_v, a_v = 0, 0
        
        pid_r.set_robot_speeds(l_v, a_v)
        pid_r.update(.5)

        x, _ = mpc_r.state()
        if len(waypoints) and mpc_idx != len(waypoints):
            mpc_traj.append((int(x[0, 0]), int(x[1, 0])))
            target = waypoints[mpc_idx]
            l_v, a_v = mpc_ctrl.update(mpc_r, target)
            distance: float = PID.distance(x[0, 0], x[1, 0], target[0], target[1])

            if distance < 5:
                mpc_idx += 1

        else:
            l_v, a_v = 0, 0
        
        mpc_r.set_robot_speeds(l_v, a_v)
        mpc_r.update(.5)

        if k == ord('q'):
            break


if __name__ == '__main__':
    main()
