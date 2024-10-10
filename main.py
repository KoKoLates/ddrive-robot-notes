import argparse

from typing import Callable

from robot import Robot, PID
from utils import WaypointHandler, Application

from utils import compute_distance
from map import square_path_generator, circle_path_generator


def tracking(
    app: Application,
    robot: Robot,
    color: tuple[int, int, int],
    handler: Callable[[], list[tuple[int, int]]],
) -> None:
    waypoints: list = handler()
    ctrl = PID(0.5, 0, 0.1, 0.9, 0, 0.1)

    index: int = 0
    history: list[tuple[int, int]] = []

    while True:
        app.clean()
        app.plot(robot.points, color)
        app.plot_path(history, color, True)
        app.plot_path(waypoints)

        p, _ = robot.state
        if len(waypoints) and index < len(waypoints):
            history.append((int(p[0, 0]), int(p[1, 0])))
            target = waypoints[index]

            v, w = ctrl.update(p, target)
            d: float = compute_distance(tuple(p[:2].flatten()), target)

            if d < 15:
                index += 1
        else:
            v, w = 0, 0

        robot.set_robot_speeds(v, w)
        robot.update(0.5)

        if app.show() & 0xFF == ord("q"):
            break


def cursor() -> None:
    handler = WaypointHandler()
    tracking(
        Application((500, 500), "Waypoint Handler", handler.add_waypoint),
        Robot(250, 250),
        (20, 255, 57),
        lambda: handler.path,
    )


def square() -> None:
    tracking(
        Application((500, 500), "Square Tracking"),
        Robot(125, 125),
        (20, 255, 57),
        lambda: square_path_generator(125, 125, 250, 30),
    )


def circle() -> None:
    tracking(
        Application((500, 500), "Circle Tracking"),
        Robot(250, 130),
        (20, 255, 57),
        lambda: circle_path_generator(250, 250, 120),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run path tracking simulation.")
    parser.add_argument(
        "-t",
        "--type",
        type=str,
        default="cursor",
        choices=["cursor", "square", "circle"],
        help="type of simulation to be executed (default: cursor)",
    )
    args = parser.parse_args()

    function_table: dict[str, Callable] = {
        "square": square,
        "circle": circle,
        "cursor": cursor,
    }
    function_table[args.type]()
