import cv2
import numpy as np
import time


def draw_apriltag_boxes(image, detections):
    """
    Draw bounding boxes, centers, and IDs for AprilTag detections.

    Parameters
    ----------
    image : np.ndarray
        Input image (grayscale or BGR).
    detections : list of dict
        AprilTag detections returned by detector.detect()
    """

    # If grayscale, convert to BGR so we can draw colored lines
    if len(image.shape) == 2:
        vis = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        vis = image.copy()

    for det in detections:
        # Corners: lb, rb, rt, lt
        corners = det["lb-rb-rt-lt"].astype(int)

        # Draw tag outline
        for i in range(4):
            p1 = tuple(corners[i])
            p2 = tuple(corners[(i + 1) % 4])
            cv2.line(vis, p1, p2, (0, 255, 0), 2)

        # Draw center
        center = tuple(det["center"].astype(int))
        cv2.circle(vis, center, 4, (0, 0, 255), -1)

        # Draw tag ID
        tag_id = det["id"]
        cv2.putText(
            vis,
            f"ID {tag_id}",
            (center[0] + 5, center[1] - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2,
            cv2.LINE_AA
        )

    return vis


class Timer:
    def __init__(self):
        self.set_time = time.time()
    def reset(self):
        self.set_time = time.time()
    def lap(self):
        return time.time() - self.set_time
    def lapReset(self):
        timelapsed = self.lap()
        self.reset()
        return timelapsed
