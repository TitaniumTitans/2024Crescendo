from dataclasses import dataclass
from typing import Tuple


@dataclass
class LocalConfig:
    device_id: str = ""
    server_ip: str = ""
    stream_port: int = 8000
    model_path: str = ""
    label_path: str = ""
    inference_size: Tuple[int, int] = 0, 0


@dataclass
class RemoteConfig:
    camera_id: int = 0
    camera_resolution_width: int = 0
    camera_resolution_height: int = 0
    camera_auto_exposure: int = 0
    camera_exposure: int = 0
    detection_threshold: float = 0.1
    max_targets: int = 5


@dataclass
class ConfigStore:
    local_config: LocalConfig
    remote_config: RemoteConfig
