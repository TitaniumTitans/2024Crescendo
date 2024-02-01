import dataclasses
from typing import Tuple

import cv2 as cv

from src.config.Config import ConfigStore


class Capture:
    """Base interface for reading from a camera"""

    def __init__(self) -> None:
        raise NotImplementedError

    def get_frame(self) -> Tuple[bool, cv.Mat]:
        """Returns the current camera frame"""
        raise NotImplementedError

    @classmethod
    def _config_changed(cls, config_a: ConfigStore, config_b: ConfigStore) -> bool:
        if config_a is None and config_b is None:
            return False
        if config_a is None or config_b is None:
            return True

        remote_a = config_a.remote_config
        remote_b = config_b.remote_config

        return (remote_a.camera_id != remote_b.camera_id
                or remote_a.camera_resolution_width != remote_b.camera_resolution_width
                or remote_a.camera_resolution_height != remote_b.camera_resolution_height
                or remote_a.camera_auto_exposure != remote_b.camera_auto_exposure
                or remote_a.camera_exposure != remote_b.camera_exposure)


class CVCapture(Capture):
    """Uses the default opencv capture stream"""

    def __init__(self) -> None:
        pass

    m_video = None

    def get_frame(self, config: ConfigStore) -> Tuple[bool, cv.Mat]:
        if self.m_video is None:
            print("Creating camera with device id {}.".format(config.remote_config.camera_id))
            self.m_video = cv.VideoCapture(0)
            self.m_video.set(cv.CAP_PROP_FRAME_WIDTH, config.remote_config.camera_resolution_width)
            self.m_video.set(cv.CAP_PROP_FRAME_HEIGHT, config.remote_config.camera_resolution_height)
            self.m_video.set(cv.CAP_PROP_AUTO_EXPOSURE, config.remote_config.camera_auto_exposure)
            self.m_video.set(cv.CAP_PROP_EXPOSURE, config.remote_config.camera_exposure)
            self.m_video.set(cv.CAP_PROP_GAIN, config.remote_config.camera_gain)
            print("Created camera with device id {}.".format(config.remote_config.camera_id))

        retval, image = self.m_video.read()

        image_rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        return retval, image_rgb
