import cv2
import argparse

from dev import Application, Robot, PID


class Waypoints(object):
    def __init__(self) -> None:
        self.wps: list[tuple[int, int]] = []

    def add_waypoint(self, event: int, x: int, y: int, flags: int, param: int) -> None:
        """the callback function for mouse event handler"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.wps.append((x, y))

        if event == cv2.EVENT_RBUTTONDOWN:
            self.wps.pop()

def main() -> None:
    wp_handler: Waypoints = Waypoints()
    app: Application = Application(
        700,
        700,
        "simulation",
        wp_handler.add_waypoint,
    )

    robot: Robot = Robot(50, 50)

    controller: PID = PID(0.5, 0.1, 0, 3, 0.1, 0)
    trajectory: list = []

    index: int = 0

    while True:
        app.clear()
        app.text("Please update or clean waypoints", (0, 0, 0), 1, 0.5, (5, 20))
        if len(wp_handler.wps):
            app.plot_path(wp_handler.wps)

        if len(trajectory):
            app.plot_path(trajectory, (200, 200, 200), dotted=True)

        app.plot(robot.points(), (255, 0, 0))

        k: int = app.show()

        p, _ = robot.state()
        if len(wp_handler.wps) and index != len(wp_handler.wps):
            trajectory.append((int(p[0, 0]), int(p[1, 0])))
            target = wp_handler.wps[index]

            l_v, a_v = controller.update(p, target, robot.points()[2], index)
            distance: float = PID.distance(p[0, 0], p[1, 0], target[0], target[1])
            if distance < 10:
                index += 1
        else:
            l_v, a_v = 0, 0

        robot.set_robot_speeds(l_v, a_v)
        robot.update(.5)

        if k == ord('q'):
            break

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--ctrl", type=str, default="pid")
    args = parser.parse_args()

    if args.ctrl not in ("pid", "mpc"):
        raise ValueError("The select type in not in ctrl list")

    print(f"[INFO] using controller: {args.ctrl}")
    main()
