import cv2
from typing import Optional

class WaypointHandler(object):
    def __init__(self) -> None:
        self._path: list[tuple[int, int]] = []
        self.index: int = 0

    def add_waypoint(
        self, 
        event: int, 
        x: int, y: int, 
        flags: Optional[int] = None, 
        param: Optional[int] = None
    ) -> None:
        """ Adds or removes waypoints based on mouse events """
        if event == cv2.EVENT_LBUTTONDOWN:
            self._path.append((x, y))
        if event == cv2.EVENT_RBUTTONDOWN and self._path:
            self._path.pop()

    @property
    def path(self) -> list[tuple[int, int]]:
        return self._path
