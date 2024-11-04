import math

def compute_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """ Compute the Euclidean distance between two given points 
    @param p1: the first point `(x, y)` 
    @param p2: the second point `(x, y)`
    @return: the distance between two points
    """
    dx, dy = p1[0] - p2[0], p1[1] - p2[1]
    return math.sqrt(dx ** 2 + dy ** 2)


def compute_angle(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """ Compute the angle in radians from p1 ro p2, relative to the x-axis 
    @param p1: the start point `(x, y)`
    @param p2: the ending point `(x, y)`
    @return: the angle in radians between two points
    """
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])
