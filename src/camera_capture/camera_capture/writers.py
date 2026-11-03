"""In-process OpenCV video writers of the legacy recorders (no ROS imports).

- ``mjpg``: OpenCV's own MJPEG encoder into an AVI, JPEG quality set after open (``ElpCam._start_segment``; the
  ELP's MJPG AVI recorder and the fallback of the HEVC recorder).
- ``mp4v``: ``cv2.VideoWriter`` with fourcc ``mp4v`` (``BoreScope._start_segment``).
"""


class CvWriter:
    """``cv2.VideoWriter`` for one segment file."""

    def __init__(self, path, codec: str, fps: float, size, jpeg_quality: int = 95):
        import cv2
        self.path, self.codec = str(path), codec
        fps = max(1.0, float(fps))          # the nominal rate tag only (the CSV has the frame times)
        if codec == "mjpg":
            self.w = cv2.VideoWriter(self.path, cv2.CAP_OPENCV_MJPEG, cv2.VideoWriter_fourcc(*"MJPG"), fps,
                                     tuple(size))
            self.w.set(cv2.VIDEOWRITER_PROP_QUALITY, int(jpeg_quality))
        elif codec == "mp4v":
            self.w = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), fps, tuple(size))
        else:
            raise ValueError(f"no OpenCV writer for codec {codec!r} (mjpg | mp4v)")

    def isOpened(self):  # noqa: N802 - cv2.VideoWriter name
        return self.w.isOpened()

    def write(self, frame):
        self.w.write(frame)

    def release(self):
        self.w.release()
