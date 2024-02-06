import sys

import numpy as np

from src.config.Config import ConfigStore
from src.pipeline.Sort import Sort

from typing import Dict
from dataclasses import dataclass

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects, Object
from pycoral.utils.edgetpu import run_inference
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.dataset import read_label_file
import cv2 as cv


@dataclass
class TrackedObject:
    object: Object
    tracker_id: int


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
        self._tracker = Sort()

        self._inference_size = input_size(self._interpreter)

        if self._interpreter is None:
            print("Failed to create interpreter. Exiting.")
            sys.exit(1)

        print("Created interpreter successfully.")

    def run_inference(self, image: cv.Mat) -> list[Object]:
        run_inference(self._interpreter, image)
        objs = get_objects(self._interpreter, self._config.remote_config.detection_threshold)[
               :self._config.remote_config.max_targets]
        return objs

    def update_tracker(self, objs: list[Object]) -> list[TrackedObject]:
        detections = []
        tracked_targets = []
        for obj in objs:
            element = [obj.bbox.xmin,
                       obj.bbox.ymin,
                       obj.bbox.xmax,
                       obj.bbox.ymax,
                       obj.score]
            detections.append(element)

        detections = np.array(detections)
        
        if detections.any():
            trdata = self._tracker.update(detections)

            # null check for empty tracking data
            if not (np.array(trdata)).size:
                for obj in objs:
                    tracked_targets.append(TrackedObject(obj, -1))

            for td in trdata:
                x0, y0, x1, y1, track_id = (td[0].item(), td[1].item(),
                                            td[2].item, td[3].item, td[4].item())
                overlap = 0
                obj = None
                # Find the overlap between each object and the tracked object
                for ob in objs:
                    dx0, dy0, dx1, dy1 = (int(ob.bbox.xmin), int(ob.bbox.ymin),
                                          int(ob.bbox.xmax), int(ob.bbox.ymax))
                    area = (min(dx1, x1) - max(dx0, x0)) * (min(dy1, y1) - max(dy0, y0))
                    if area > overlap:
                        overlap = area
                        obj = ob

                if obj is not None:
                    tracked_targets.append(TrackedObject(obj, track_id))

        return tracked_targets

    def draw_bbox(self, image: cv.Mat, objs: list[TrackedObject]) -> cv.Mat:
        height, width, channels = image.shape
        scale_x, scale_y = width / self._inference_size[0], height / self._inference_size[1]
        for obj in objs:
            bbox = obj.object.bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)

            percent = int(100 * obj.object.score)
            label = '{}% {}'.format(percent, self._labels.get(obj.object.id, obj.object.id))

            image = cv.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 2)
            image = cv.putText(image, label, (x0, y0 + 30),
                               cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)

            if obj.tracker_id != -1:
                image = cv.putText(image, str(obj.tracker_id), (x0, y0-30),
                                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)

        return image
