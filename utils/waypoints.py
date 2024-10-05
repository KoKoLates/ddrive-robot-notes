import cv2


class WaypointHandler(object):
    def __init__(self) -> None:
        self.path: list[tuple[int, int]] = []
        self.index: int = 0

    def add_waypoint(self, event: int, x: int, y: int, flags: int, param: int) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self.path.append((x, y))

        if event == cv2.EVENT_RBUTTONDOWN:
            self.path.pop()
