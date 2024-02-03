import ntcore
import json

from src.config.Config import RemoteConfig, ConfigStore

class ConfigSource:
    def update(self, config: ConfigStore) -> None:
        raise NotImplementedError


class FileConfigSource(ConfigSource):
    CONFIG_FILENAME = "config.json"

    def __init__(self) -> None:
        pass

    def update(self, config: ConfigStore) -> None:
        # Get config
        with open(self.CONFIG_FILENAME, "r") as config_file:
            config_data = json.loads(config_file.read())
            config.local_config.device_id = config_data["device_id"]
            config.local_config.server_ip = config_data["server_ip"]
            config.local_config.model_path = config_data["model_path"]
            config.local_config.label_path = config_data["label_path"]


class NTConfigSource(ConfigSource):
    _init_complete: bool = False
    _camera_id_sub: ntcore.IntegerSubscriber
    _camera_res_width_sub: ntcore.IntegerSubscriber
    _camera_res_height_sub: ntcore.IntegerSubscriber
    _camera_auto_exposure_sub: ntcore.IntegerSubscriber
    _camera_exposure_sub: ntcore.IntegerSubscriber
    _camera_threshold_sub: ntcore.DoubleSubscriber
    _camera_targets_sub: ntcore.IntegerSubscriber
