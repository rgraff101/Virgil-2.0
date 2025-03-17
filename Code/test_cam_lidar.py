import hailo
from adafruit_rplidar import RPLidar
from gi.repository import Gst, GLib
from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

import threading


class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.lidar_distance = None  # Store latest LiDAR reading
        self.lidar = RPLidar(None, "/dev/ttyUSB0", timeout=3)

        # Start LiDAR update thread
        self.lidar_thread = threading.Thread(target=self.refresh_scan_data, daemon=True)
        self.lidar_thread.start()

    def refresh_scan_data(self):
        self.scan_data = [0] * 360
        for scan in self.lidar.iter_scans():
            # print(scan)
            for _, angle, distance in scan:
                self.scan_data[min([359, int(angle)])] = distance


def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK
    user_data.increment()  # next frame

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    if detections:
        latest_detection = detections[-1]  # Use the latest detected object
        bbox = latest_detection.get_bbox()

    # Debugging log
    print(
        f"LiDAR scans 345 to 14 deg: {user_data.scan_data[345:] + user_data.scan_data[:15]}"
    )
    print(f"bounding box: x_min: {bbox.xmin()}, x_max: {bbox.xmax()}")

    return Gst.PadProbeReturn.OK


if __name__ == "__main__":
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()
