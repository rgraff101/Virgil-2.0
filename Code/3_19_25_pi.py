import gi
import time
import os
import numpy as np
import cv2
import hailo
from adafruit_rplidar import RPLidar
import serial
import threading
from time import sleep
from gi.repository import Gst, GLib
from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp
    
def decide_movement(user_data, width, bbox):
    x_min = bbox.xmin()
    x_max = bbox.xmax()
    x_center = (x_min + x_max) / 2

    print(f"Detection: xmin={x_min}, xmax={x_max}, center={x_center}")
    
    if x_center < 0.4:
        print("Decision: RIGHT")
        return "RIGHT"
    elif x_center > 0.6:
        print("Decision: LEFT")
        return "LEFT"
    else:
        print("Decision: FORWARD")
        return "FORWARD"


class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.lidar_distance = None  # Store latest LiDAR reading
        self.lidar = RPLidar(None, "/dev/ttyUSB0", baudrate=115200, timeout=3)
        self.stage = 1  # Tracks the current stage
        self.frame_skip = 8  
        self.frame_count = 0
        self.counter = 0  # Counter for stage 2
        
        # Start LiDAR update thread
        self.lidar_thread = threading.Thread(target=self.refresh_scan_data, daemon=True)
        self.lidar_thread.start()

    def refresh_scan_data(self):
        self.scan_data = [0] * 360
        while True:
            try:
                time.sleep(0.05)  # ✅ Small delay to prevent CPU overload
                for scan in self.lidar.iter_scans():
                    for _, angle, distance in scan:
                        if distance is None or distance == 0:
                            distance = float('inf')  # ✅ Avoid None values
                        self.scan_data[min(359, int(angle))] = distance / 1000.0  # ✅ Convert to meters
                        if 345 >= angle or angle <= 15:
                            self.lidar_distance = distance / 1000.0
            except Exception as e:
                print(f"⚠️ LiDAR Error: {e} ⚠️")
                self.lidar.stop()
                time.sleep(1)  # ✅ Give LiDAR time to recover


    def send_command(self, command):
        self.serial_conn.reset_output_buffer()
        self.serial_conn.write((command + "\n").encode())
        print(f"Sent: {command}")

    def set_stage(self, new_stage):
        if self.stage != new_stage:
            self.stage = new_stage
            print(f"Transitioning to Stage {self.stage}!")    

    def detections(self, pad, buffer):
        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
        move_command = None
        format, width, height = get_caps_from_pad(pad)

        if detections:
            latest_detection = detections[-1]  # Use the latest detected object
            if latest_detection.get_label() == "person":
                move_command = decide_movement(self, width, latest_detection.get_bbox())
                print(f"Move Command to Send: {move_command}")
        else:
            self.send_command("NO DETECTION")
        
        if self.lidar_distance is not None:
            if self.lidar_distance <= 1.25:
                self.send_command("STOP")
                self.set_stage(2)  # Move to next stage when obstacle is detected
            elif move_command:
                self.send_command(move_command)


def app_callback(pad, info, user_data):
    user_data.frame_count += 1
    if user_data.frame_count % user_data.frame_skip != 0:
        return Gst.PadProbeReturn.OK  # Skip frame processing

    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK
    user_data.increment()

    distance = user_data.lidar_distance  # Use background LiDAR distance
    print(f"LiDAR Distance: {distance}m")

    # Handling stages
    if user_data.stage == 1:  # Stage 1 - Navigate towards object
        user_data.detections(pad, buffer)
        
    elif user_data.stage == 2:  # Stage 2 - Avoid obstacle
        user_data.send_command("90LEFT")
        sleep(2)
        user_data.set_stage(3)
#         if user_data.counter < 30:
#             user_data.send_command("90LEFT")
#         elif 30 <= user_data.counter < 60:
#             user_data.send_command("FORWARD1")
#         elif 60 <= user_data.counter < 90:
#             user_data.send_command("90RIGHT")
#         elif 90 <= user_data.counter < 120:
#             user_data.send_command("FORWARD2")
#         elif 120 <= user_data.counter < 150:
#             user_data.send_command("90RIGHT")
#         elif 150 <= user_data.counter < 180:
#             user_data.send_command("FORWARD2")
#         elif 180 <= user_data.counter < 210:
#             user_data.send_command("90RIGHT")
#         elif 210 <= user_data.counter < 230:
#             user_data.send_command("FORWARD1")
#         user_data.counter += 1
# 
#         if user_data.counter >= 230:
#             user_data.set_stage(3)  # Transition to Stage 3 after enough iterations
        
    elif user_data.stage == 3:  # Stage 3 - Just stop and do nothing more
        user_data.send_command("STOP")  # Only send STOP in Stage 3
    else:
        user_data.send_command("STOP")

    return Gst.PadProbeReturn.OK


if __name__ == "__main__":
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()

