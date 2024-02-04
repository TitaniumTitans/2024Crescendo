import sys

from src.config.Config import ConfigStore
from typing import Dict

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects, Object
from pycoral.utils.edgetpu import run_inference
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.dataset import read_label_file
import cv2 as cv


class Detector:
    """Runs a tflite model on the coral to detect objects"""
    _labels: Dict[int, str]
    _config: ConfigStore

    def __init__(self, config: ConfigStore):
        self._config = config

        print("Loading {} with {} labels.".format(config.local_config.model_path, config.local_config.label_path))

        self._interpreter = make_interpreter(config.local_config.model_path)
        self._interpreter.allocate_tensors()
        self._labels = read_label_file(config.local_config.label_path)

        self._inference_size = input_size(self._interpreter)

        if self._interpreter is None:
            print("Failed to create interpreter. Exiting.")
            sys.exit(1)

        print("Created interpreter successfully.")

    def run_inference(self, image: cv.Mat) -> list[Object]:
        run_inference(self._interpreter, image)
        objs = get_objects(self._interpreter, self._config.remote_config.detection_threshold)[:self._config.remote_config.max_targets]
        return objs
