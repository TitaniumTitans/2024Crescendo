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

    def draw_bbox(self, image: cv.Mat, objs: list[Object]) -> cv.Mat:
        height, width, channels = image.shape
        scale_x, scale_y = width / self._inference_size[0], height / self._inference_size[1]
        for obj in objs:
            bbox = obj.bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)

            percent = int(100 * obj.score)
            label = '{}% {}'.format(percent, self._labels.get(obj.id, obj.id))

            image = cv.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 2)
            image = cv.putText(image, label, (x0, y0+30),
                               cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)

        return image
