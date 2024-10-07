import numpy as np


def square_path_generator(x: int, y: int, b: int, step: int) -> list[tuple[int, int]]:
    """ Generator of square-shape waypoints
    @param x: initial x
    @param y: initial y
    @param b: border of the square path
    @param step: distance between each waypoint
    """
    waypoints = [
        *((i, y) for i in range(x, x + b, step)),
        *((x + b, j) for j in range(y, y + b, step)),
        *((i, y + b) for i in range(x + b, x, -step)),
        *((x, j) for j in range(y + b, y, -step)),
    ]
    return waypoints


def circle_path_generator(
    x: int, y: int, r: int, n: int = 100
) -> list[tuple[int, int]]:
    """ Generator of circle-shape waypoints
    @param x: x of the center of circle
    @param y: y of the center of circle
    @param r: the radius of the circle
    @param n: number of waypoints in the circle path
    """
    angles = np.linspace(0, np.pi * 2, n, endpoint=False)
    waypoints = [
        (x + int(r * np.sin(angle)), y - int(r * np.cos(angle))) for angle in angles
    ]
    return waypoints
