from robot import Robot, PID
from utils import WaypointHandler, Application

from utils import compute_distance


def main() -> None:
    handler = WaypointHandler()
    app: Application = Application((500, 500), "Tracking", handler.add_waypoint)

    robot = Robot(50, 50)
    ctrl = PID(0.5, 0, 0.1, 0.3, 0, 0.1)

    index: int = 0
    history: list[tuple[int, int]] = []

    while True:
        app.clean()
        app.plot(robot.points, (0, 165, 255))
        app.plot_path(history, (0, 165, 255), dot=True)
        app.plot_path(handler.path)

        if app.show() & 0xFF == ord("q"):
            break

        p, _ = robot.state
        if len(handler.path) and index < len(handler.path):
            history.append((int(p[0, 0]), int(p[1, 0])))
            target = handler.path[index]

            v, w = ctrl.update(p, target)
            d: float = compute_distance(tuple(p[:2].flatten()), target)

            if d < 10:
                index += 1
        else:
            v, w = 0, 0

        robot.set_robot_speeds(v, w)
        robot.update(0.5)


if __name__ == "__main__":
    main()
