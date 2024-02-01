from src.config.Config import ConfigStore, LocalConfig, RemoteConfig
from src.pipeline.Capture import CVCapture
from src.output.StreamServer import StreamServer

import time

if __name__ == '__main__':
    config = ConfigStore(LocalConfig(), RemoteConfig())

    capture = CVCapture()
    stream_server = StreamServer()

    stream_server.start(config)

    frame_count = 0
    last_print = 0
    while True:
        timestamp = time.time()
        success, image = capture.get_frame(config)
        if not success:
            time.sleep(0.5)
            continue

        fps = None
        frame_count += 1
        if time.time() - last_print > 1:
            fps = frame_count
            print("Running at", frame_count, "fps")
            frame_count = 0

        stream_server.set_frame(image)
