import cv2
import argparse
import numpy as np


def plot_path(filename: str, window_size: tuple = (500, 500)) -> None:
    w, h = window_size
    waypoints: list = []

    with open(filename, "r") as f:
        for line in f:
            x, y = map(int, line.strip().split(", "))
            waypoints.append((x, y))

    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    for i in range(1, len(waypoints)):
        p1 = waypoints[i - 1]
        p2 = waypoints[i]
        cv2.line(canvas, p1, p2, (128, 128, 128), 1)

    cv2.imshow("Waypoints", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def load_map(filename: str) -> list[tuple[int, int]]:
    waypoints: list = []

    with open(filename, "r") as f:
        for line in f:
            x, y = map(int, line.strip().split(", "))
            waypoints.append((x, y))

    return waypoints


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-m", "--map",
        type=str, required=True,
        help="the map to be loaded"
    )
    args = parser.parse_args()

    plot_path(args.map, (500, 500))
