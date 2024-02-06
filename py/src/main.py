from config.Config import ConfigStore, LocalConfig, RemoteConfig
from config.ConfigSource import FileConfigSource, NTConfigSource
from pipeline.Capture import CVCapture
from pipeline.Detector import Detector
from output.StreamServer import MjpegServer

import time

if __name__ == '__main__':
    config = ConfigStore(LocalConfig(), RemoteConfig())

    capture = CVCapture()
    stream_server = MjpegServer()
    file_config = FileConfigSource()
    remote_config = NTConfigSource()

    file_config.update(config)

    detector = Detector(config)
    stream_server.start(config)

    frame_count = 0
    last_print = 0
    while True:
        remote_config.update(config)

        timestamp = time.time()
        success, image = capture.get_frame(config)
        if not success:
            time.sleep(0.5)
            continue

        fps = None
        frame_count += 1
        if time.time() - last_print > 1:
            last_print = time.time()
            fps = frame_count
            print("Running at", frame_count, "fps")
            frame_count = 0

        objs = detector.run_inference(image)
        tracked_objs = detector.update_tracker(objs)

        image = detector.draw_bbox(image, tracked_objs)
        stream_server.set_frame(image)
