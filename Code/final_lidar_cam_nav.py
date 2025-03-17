import gi
import time
import os
import numpy as np
import cv2
import hailo
from adafruit_rplidar import RPLidar
import serial
import threading
from gi.repository import Gst, GLib
from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

def decide_movement(user_data, width, bbox):
    """Determine movement based on AI detection."""
    x_min = bbox.xmin()
    x_max = bbox.xmax()
    x_center = (x_min + x_max) / 2
    
    print(f"Detection: xmin={x_min}, xmax={x_max}, center={x_center}")
    
    if x_center < .4:
        return "LEFT"
    elif x_center > .6:
        return "RIGHT"
    else:
        return "FORWARD"

class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.lidar_distance = None  # Store latest LiDAR reading
        self.lidar = RPLidar(None, "/dev/ttyUSB0", timeout=3)
        self.stage = 1  # Tracks the current stage
        self.frame_skip = 5  # Process every 2nd frame for more frequent updates
        self.frame_count = 0

        # Start LiDAR update thread
        self.lidar_thread = threading.Thread(target=self.refresh_scan_data, daemon=True)
        self.lidar_thread.start()

    def refresh_scan_data(self):
        self.scan_data = [0] * 360
        for scan in self.lidar.iter_scans():
            for _, angle, distance in scan:
                self.scan_data[min(359, int(angle))] = distance
                if 345 >= angle or angle <= 15:  # Forward-facing region
                    self.lidar_distance = distance
              
    def send_command(self, command):
        self.serial_conn.write((command + "\n").encode())
        print(f"Sent: {command}")

    def set_stage(self, new_stage):
        """Move to the next stage if not already in it."""
        if self.stage != new_stage:
            self.stage = new_stage
            print(f"Transitioning to Stage {self.stage}!")

def app_callback(pad, info, user_data):
    user_data.frame_count += 1
    if user_data.frame_count % user_data.frame_skip != 0:
        return Gst.PadProbeReturn.OK  # Skip frame processing

    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK
    user_data.increment()

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    move_command = None
    format, width, height = get_caps_from_pad(pad)
  
    if detections:
        latest_detection = detections[-1]  # Use the latest detected object
        if latest_detection.get_label() == "person":
            move_command = decide_movement(user_data, width, latest_detection.get_bbox())
        else:
            user_data.send_command("NO DETECTION")

    distance = user_data.lidar_distance  # Use background LiDAR distance
    print(f"LiDAR Distance: {distance}m")

    if user_data.stage == 1:
        if distance is not None and distance <= 1.0:
            user_data.send_command("STOP")
            user_data.set_stage(2)
        elif move_command:
            user_data.send_command(move_command)

    elif user_data.stage == 2:
        if distance is not None and 0.4 <= distance <= 0.6:
            user_data.send_command("FORWARD")
        else:
            if distance < 0.4:
                user_data.send_command("LEFT")
            elif distance > 0.6:
                user_data.send_command("RIGHT")
    
    elif user_data.stage == 3:
        user_data.send_command("ADJUST")
        user_data.set_stage(4)

    return Gst.PadProbeReturn.OK

if __name__ == "__main__":
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()
