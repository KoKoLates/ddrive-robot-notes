import math


def compute_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def compute_angle(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])
