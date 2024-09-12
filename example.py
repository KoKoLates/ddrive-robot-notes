import cv2
import numpy as np

from dev import PID, Robot, Application


def main() -> None:
    waypoints: list[tuple[int, int]] = []

    app: Application = Application(700, 700, "simulation", None)
    robot: Robot = Robot(50, 50)
    controller: PID = PID(0.5, 0.1, 0, 3, 0.1, 0)

    ## define waypoints
    for x in range(200, 600, 2):
        y: np.float32 = 350 + np.sin(np.pi * 2 * 0.25 * (x + 200) / 100) * 200
        waypoints.append((x, int(y)))

    index: int = 0
    trajectory: list = []
    
    while True:
        app.clear()

        if len(waypoints):
            app.plot_path(waypoints)

        if len(trajectory):
            app.plot_path(trajectory, (0, 255, 0))

        app.plot(robot.points(), (0, 255, 0))

        k: int = app.show()

        x, _ = robot.state()
        if len(waypoints) and index != len(waypoints):
            trajectory.append((int(x[0, 0]), int(x[1, 0])))
            target = waypoints[index]
            l_v, a_v = controller.update(x, target, robot.points()[2], index)
            distance: float = PID.distance(x[0, 0], x[1, 0], target[0], target[1])

            if distance < 5:
                index += 1

        else:
            l_v, a_v = 0, 0
        
        robot.set_robot_speeds(l_v, a_v)
        robot.update(.5)

        if k == ord('q'):
            break


if __name__ == '__main__':
    main()
