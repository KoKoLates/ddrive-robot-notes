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
        color: tuple[int, int, int] = (255, 255, 255),
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
        thickness: int = 2,
    ) -> None:
        for i in range(len(points) - 1):
            cv2.line(
                self.canvas,
                (points[i, 0], points[i, 1]),
                (points[i + 1, 0], points[i + 1, 1]),
                color,
                thickness,
            )

        cv2.line(
            self.canvas,
            (points[0, 0], points[0, 1]),
            (points[-1, 0], points[-1, 1]),
            color,
            thickness,
        )

    def clear(self) -> None:
        self.canvas: np.ndarray = np.ones((self.h, self.w, 3)).astype("uint8")
        for i in range(len(self.color)):
            self.canvas[:, :, i] = self.canvas[:, :, i] * self.color[i]

    def plot_path(
        self,
        points: list,
        color: tuple[int, int, int] = (255, 0, 0),
        thickness: int = 2,
        dotted: bool = False,
    ) -> None:
        for i in range(len(points) - 1):
            if dotted:
                if not (i % 2):
                    cv2.circle(
                        self.canvas, (points[i][0], points[i][1]), 2, color, thickness
                    )
                continue

            cv2.line(
                self.canvas,
                (points[i][0], points[i][1]),
                (points[i + 1][0], points[i + 1][1]),
                color,
                thickness,
            )
            cv2.circle(self.canvas, (points[i][0], points[i][1]), 3, (0, 0, 0), 1)

        if not dotted:
            cv2.circle(self.canvas, (points[-1][0], points[-1][1]), 3, (0, 0, 0), 1)

    def text(
        self,
        text: str,
        color: tuple[int, int, int] = (255, 0, 0),
        thickness: int = 2,
        font_scale: int = 1,
        org: tuple[int, int] = (100, 50),
    ) -> None:
        """adding text to the canvas"""
        font: int = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(
            self.canvas, text, org, font, font_scale, color, thickness, cv2.LINE_AA
        )

    def show(self) -> int:
        cv2.imshow(self.name, self.canvas)
        return cv2.waitKey(30)
