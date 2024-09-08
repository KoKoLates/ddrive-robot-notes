import cv2
import numpy as np

from typing import Optional, Callable

class Application(object):
    def __init__(
        self,
        w: int,
        h: int,
        window_name: str,
        mouse_callback: Optional[Callable],
        color: tuple[int, int, int] = (255, 255, 255)
    ) -> None:
        self.w, self.h = w, h
        self.name: str = window_name
        self.color: tuple[int, int, int] = color
        
        cv2.namedWindow(self.name)
        if mouse_callback is not None:
            cv2.setMouseCallback(self.name, mouse_callback)

    def plot(
        self,
        points: np.ndarray,
        color: tuple[int, int, int] = (255, 0, 0),
        thickness: int = 2
    ) -> None:
        for i in range(len(points) - 1):
            cv2.line(
                self.canvas,
                (points[i, 0], points[i, 1]),
                (points[i+1, 0], points[i+1, 1]),
                color,
                thickness
            )

        cv2.line(
            self.cavas,
            (points[0, 0], points[0, 1]),
            (points[-1, 0], points[-1, 1]),
            color,
            thickness
        )
