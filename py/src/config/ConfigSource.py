import ntcore
import json

from Config import ConfigStore, RemoteConfig


class ConfigSource:
    def update(self, config: ConfigStore) -> None:
        raise NotImplementedError


class FileConfigSource(ConfigSource):
    CONFIG_FILENAME = "config.json"

    def __init__(self) -> None:
        # dataclass updater doesn't need a constructor
        super().__init__()

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

    def update(self, config: ConfigStore) -> None:
        # Initialize subscribers on first call
        if not self._init_complete:
            nt_table = ntcore.NetworkTableInstance.getDefault().getTable(
                "/" + config.local_config.device_id + "/config"
            )
            self._camera_id_sub = nt_table.getIntegerTopic(
                "camera_id").subscribe(RemoteConfig.camera_id)
            self._camera_res_width_sub = nt_table.getIntegerTopic(
                "camera_resolution_width").subscribe(RemoteConfig.camera_resolution_width)
            self._camera_res_height_sub = nt_table.getIntegerTopic(
                "camera_resolution_height").subscribe(RemoteConfig.camera_resolution_height)
            self._camera_exposure_sub = nt_table.getIntegerTopic(
                "camera_auto_exposure").subscribe(RemoteConfig.camera_auto_exposure)
            self._camera_exposure_sub = nt_table.getIntegerTopic(
                "camera_exposure").subscribe(RemoteConfig.camera_exposure)
            self._camera_threshold_sub = nt_table.getDoubleTopic(
                "camera_threshold_sub").subscribe(RemoteConfig.detection_threshold)
            self._camera_targets_sub = nt_table.getIntegerTopic(
                "max_targets").subscribe(RemoteConfig.max_targets)

            # Read config data
            config.remote_config.camera_id = self._camera_id_sub.get()
            config.remote_config.camera_resolution_width = self._camera_res_width_sub.get()
            config.remote_config.camera_resolution_height = self._camera_res_height_sub.get()
            config.remote_config.camera_auto_exposure = self._camera_auto_exposure_sub.get()
            config.remote_config.camera_exposure = self._camera_exposure_sub.get()
            config.remote_config.detection_threshold = self._camera_threshold_sub.get()
            config.remote_config.max_targets = self._camera_targets_sub.get()
