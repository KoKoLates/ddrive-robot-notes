import cv2
import argparse

from typing import Any


class Waypoints(object):
    def __init__(self) -> None:
        self.points: list[tuple[int, int]] = []

    def add_points(self, event: int, x: int, y: int, flags: int, param: Any) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x, y))

    def get_point(self) -> tuple[int, int]:
        return self.points[-1]


def main() -> None:
    """"""


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--ctrl", type=str, default="pid")
    args = parser.parse_args()

    if args.ctrl not in ("pid", "mpc"):
        raise ValueError("The select type in not in ctrl list")

    print(f"[INFO] using controller: {args.ctrl}")
    main()
