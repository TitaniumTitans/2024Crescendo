import math

import ntcore

from src.config.Config import ConfigStore
from src.pipeline.Detector import TrackedObject

from typing import List, Union


class OutputPublisher:
    def publish(self, config: ConfigStore, timestamp: float, observations: Union[List[TrackedObject], None], fps: Union[int, None]):
        raise NotImplementedError


class NTOutputPublisher(OutputPublisher):
    _init_complete: bool = False
    _observation_pub: ntcore.DoubleArrayPublisher
    _fps_pub: ntcore.IntegerPublisher
    _connection_pub: ntcore.BooleanPublisher

    def publish(self, config: ConfigStore, timestamp: float,
                observations: Union[List[TrackedObject], None], fps: Union[int, None]):
        if not self._init_complete:
            nt_table = ntcore.NetworkTableInstance.getDefault().getTable(
                "/" + config.local_config.device_id + "/output")
            _observation_pub = nt_table.getDoubleArrayTopic("observations").publish(
                ntcore.PubSubOptions(periodic=0, sendAll=True, keepDuplicates=True))
            _fps_pub = nt_table.getIntegerTopic("fps").publish()
            _connection_pub = nt_table.getBooleanTopic("connected").publish()

        if fps is not None:
            self._fps_pub.set(fps)

        observation_data: List[float] = [len(observations)]
        if observations is not None:
            for observation in observations:
                observation_data.append(observation.tracker_id)
                # calculate tx and ty for the objects
                center_x, center_y = (
                    ((int(observation.object.bbox.xmax) - int(observation.object.bbox.xmin)) / 2) * config.local_config.scale_x,
                    ((int(observation.object.bbox.ymax) - int(observation.object.bbox.ymin)) / 2) * config.local_config.scale_y)
                observation_data.append((config.remote_config.camera_resolution_width / 2) - center_x)
                observation_data.append((config.remote_config.camera_resolution_height / 2) - center_y)
                observation_data.append(observation.object.score)
                # find the percent area of the target
                length, width = center_x * 2, center_y * 2
                bbox_area = length * width
                image_area = (
                        config.remote_config.camera_resolution_width * config.remote_config.camera_resolution_height)
                observation_data.append(image_area - bbox_area)

        self._observation_pub.set(observation_data, math.floor(timestamp * 1000000))
        self._fps_pub.set(fps)
        self._connection_pub.set(True)
