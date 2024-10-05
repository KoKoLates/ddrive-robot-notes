
from robot import Robot, PID, MPC
from utils import WaypointHandler, Application

from utils import compute_distance


def main() -> None:
    handler = WaypointHandler()
    app: Application = Application(
        (800, 800),
        "Tracking Simulation",
        handler.add_waypoint
    )

    robot = Robot(50, 50)
    # ctrl = PID(0.5, 0, 0.1, 0.3, 0, 0.1)
    ctrl = MPC()

    index: int = 0
    trajectory: list[tuple[int, int]] = []

    while True:
        app.clean()
        if handler.path:
            app.plot_path(handler.path)

        if trajectory:
            app.plot_path(trajectory, (255, 255, 0), dot=True)

        app.plot(robot.points, (255, 255, 0))

        k: int = app.show()

        p, _ = robot.state
        if  len(handler.path) and index < len(handler.path):
            trajectory.append((int(p[0, 0]), int(p[1, 0])))
            target = handler.path[index]

            # v, w = ctrl.update(p, target)
            v, w = ctrl.update(target, robot)
            d: float = compute_distance(tuple(p[:2].flatten()), target)

            if d < 10:
                print(f"[INFO] reach {target}")
                index += 1
        else:
            v, w = 0, 0

        robot.set_robot_speeds(v, w)
        robot.update(0.5)

        if k & 0xff == ord('q'):
            break


if __name__ == "__main__":
    main()
