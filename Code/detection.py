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
from time import time
    
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
        self.lidar_distance = 1000  # Store latest LiDAR reading
        self.lidar = RPLidar(None, "/dev/ttyUSB0", baudrate=115200, timeout=3)
        self.stage = 1  # Tracks the current stage
        self.frame_skip = 8  
        self.frame_count = 0
        self.counter = 0  # Counter for stage 2
        
        # Start LiDAR update thread
        self.lidar_thread = threading.Thread(target=self.refresh_scan_data, daemon=True)
        self.lidar_thread.start()

    def refresh_scan_data(self):
        self.scan_data = [10000] * 360
        while True:
            try:
                for scan in self.lidar.iter_scans():
                    for _, angle, distance in scan:
                        if distance is None or distance == 0:
                            distance = float('inf')  # ✅ Avoid None values
                        self.scan_data[min(359, int(angle))] = distance / 1000.0  # ✅ Convert to meters
                    self.lidar_distance = self.scan_data[0]
            except Exception as e:
                print(f"⚠️ LiDAR Error: {e} ⚠️")
                self.lidar.stop()
                time.sleep(1)  # ✅ Give LiDAR time to recover


    def send_command(self, command):
        #self.serial_conn.reset_output_buffer()
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
            # ✅ Select the largest "bucket" detection based on bbox area
            bucket_detections = [d for d in detections if d.get_label() == "bucket"]
            if bucket_detections:
                largest_bucket = max(bucket_detections, key=lambda d: (d.get_bbox().xmax() - d.get_bbox().xmin()) * (d.get_bbox().ymax() - d.get_bbox().ymin()))
                move_command = decide_movement(self, width, largest_bucket.get_bbox())
                print(f"Move Command to Send: {move_command}")
        else:
            self.send_command("NO DETECTION")

        if self.lidar_distance is not None:
            if self.lidar_distance <= .5:
                self.send_command("STOP")
                #self.set_stage(2)
            elif move_command:
                self.send_command(move_command)


#     def detections(self, pad, buffer):
#         roi = hailo.get_roi_from_buffer(buffer)
#         detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
#         move_command = None
#         format, width, height = get_caps_from_pad(pad)
# 
#         if detections:
#             latest_detection = detections[-1]  # Use the latest detected object
#             if latest_detection.get_label() == "bucket":
#                 move_command = decide_movement(self, width, latest_detection.get_bbox())
#                 print(f"Move Command to Send: {move_command}")
#         else:
#             self.send_command("NO DETECTION")
#         
#         if self.lidar_distance is not None:
#             if self.lidar_distance <= .5:
#                 self.send_command("STOP")
#                 #self.set_stage(2)
#             elif move_command:
#                 self.send_command(move_command)


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
    if user_data.stage == 1:  # Stage 1 - Navigate towards bucket 1
        print("stage 1 starting")
        user_data.detections(pad, buffer)
        print("lidar distance: ", user_data.lidar_distance)
        if user_data.lidar_distance <= .5:
            user_data.set_stage(2)
            
    elif user_data.stage == 2:  # Stage 2 - Avoid bucket 1
        print("stage 2 starting")
        print("Count: ", user_data.counter)
        if user_data.counter == 0:
            user_data.send_command("90LEFT")
            user_data.counter += 1
        elif user_data.counter == 1:
            user_data.send_command("FORWARD1")
            user_data.counter += 1
        elif user_data.counter == 2:
            user_data.send_command("90RIGHT")
            user_data.counter += 1
        elif user_data.counter == 3:
            user_data.send_command("FORWARD2")
            user_data.counter += 1
        elif user_data.counter == 4:
            user_data.send_command("90RIGHT")
            user_data.counter += 1
        elif user_data.counter == 5:
            user_data.send_command("FORWARD2")
            user_data.counter += 1
        elif user_data.counter == 6:
            user_data.send_command("90RIGHT")
            user_data.counter += 1
        elif user_data.counter == 7:
            user_data.send_command("FORWARD1")
            user_data.counter += 1
        elif user_data.counter == 8:
            user_data.send_command("45LEFT")
            user_data.counter += 1
        else:
            print("Stage 2 done going to stage 3 so should stop")
            user_data.counter = 0
            user_data.set_stage(3)
        
    if user_data.stage == 3:  # Stage 3 - Navigate towards bucket 2
        user_data.detections(pad, buffer)
        if user_data.lidar_distance <= .5:
            user_data.set_stage(4)
            
    elif user_data.stage == 4:  # Stage 4 - Avoid bucket 2
        print("stage 4 starting")
        print("Count: ", user_data.counter)
        if user_data.counter == 0:
            user_data.send_command("90RIGHT")
            user_data.counter += 1
        elif user_data.counter == 1:
            user_data.send_command("FORWARD1")
            user_data.counter += 1
        elif user_data.counter == 2:
            user_data.send_command("90LEFT")
            user_data.counter += 1
        elif user_data.counter == 3:
            user_data.send_command("FORWARD2")
            user_data.counter += 1
        elif user_data.counter == 4:
            user_data.send_command("90LEFT")
            user_data.counter += 1
        elif user_data.counter == 5:
            user_data.send_command("FORWARD1")
            user_data.counter += 1
        elif user_data.counter == 6:
            user_data.send_command("45RIGHT")
            user_data.counter += 1
        elif user_data.counter == 7:
            user_data.send_command("FORWARD2")
            user_data.counter += 1
        else:
            print("Stage 4 done going to stage 5 so should stop")
            user_data.counter = 0
            user_data.set_stage(5)
        
    elif user_data.stage == 5: # Stage 5 - Navigate towards bucket 3
        user_data.detections(pad, buffer)
        if user_data.lidar_distance <= .5:
            user_data.set_stage(6)
        
    elif user_data.stage == 6:  # Stage 6 - Avoid bucket 3
        print("stage 6 starting")
        print("Count: ", user_data.counter)
        if user_data.counter == 0:
            user_data.send_command("90LEFT")
            user_data.counter += 1
        elif user_data.counter == 1:
            user_data.send_command("FORWARD1")
            user_data.counter += 1
        elif user_data.counter == 2:
            user_data.send_command("90RIGHT")
            user_data.counter += 1
        elif user_data.counter == 3:
            user_data.send_command("FORWARD2")
            user_data.counter += 1
        elif user_data.counter == 4:
            user_data.send_command("90RIGHT")
            user_data.counter += 1
        elif user_data.counter == 5:
            user_data.send_command("FORWARD2")
            user_data.counter += 1
        else:
            print("Stage 6 done going to stage 7 so should stop")
            user_data.counter = 0
            user_data.set_stage(7)
        
    elif user_data.stage == 7:  # Stage 7 - Navigate toward ramp
        user_data.send_command("FORWARD15")
        user_data.set_stage(8)

    elif user_data.stage == 8: # Stage 8 Navigate around ramp
        if user_data.counter == 0:
            user_data.send_command("90RIGHT")
            user_data.counter += 1
        else:
            print("Stage 8 done going to stage 9 so should stop")
            user_data.counter = 0
            user_data.set_stage(9)

    elif user_data.stage == 9: # Stage 9 - Navigate towards bucket 4
        user_data.detections(pad, buffer)
        if user_data.counter >= 5 and user_data.lidar_distance <= .5:
            user_data.counter = 0
            user_data.set_stage(10)

    elif user_data.stage == 10:  # Stage 10 - Avoid bucket 4
        if user_data.counter == 0:
            user_data.send_command("90LEFT")
            user_data.counter += 1
        elif user_data.counter == 1:
            user_data.send_command("FORWARD1")
            user_data.counter += 1
        elif user_data.counter == 2:
            user_data.send_command("90RIGHT")
            user_data.counter += 1
        elif user_data.counter == 3:
            user_data.send_command("FORWARD2")
            user_data.counter += 1
        elif user_data.counter == 4:
            user_data.send_command("30RIGHT")
            user_data.counter += 1
        elif user_data.counter == 5:
            user_data.send_command("FORWARD2")
            user_data.counter += 1
        else:
            print("Stage 10 done going to stage 11 so should stop")
            user_data.counter = 0
            user_data.set_stage(11)
        
    elif user_data.stage == 11: # Stage 11 - Straight through tunnel
        user_data.send_command("FOWARD30")
        user_data.set_stage(12)
        
    elif user_data.stage == 12: # Stage 12 - Navigate towards bucket 5
        user_data.detections(pad, buffer)
        if user_data.lidar_distance <= .5:
            user_data.set_stage(13)
        
    elif user_data.stage == 13:
        if user_data.counter == 0:
            user_data.send_command("90LEFT")
            user_data.counter += 1
        elif user_data.counter == 1:
            user_data.send_command("FORWARD1")
            user_data.counter += 1
        elif user_data.counter == 2:
            user_data.send_command("90RIGHT")
            user_data.counter += 1
        elif user_data.counter == 3:
            user_data.send_command("FORWARD2")
            user_data.counter += 1
        elif user_data.counter == 4:
            user_data.send_command("90RIGHT")
            user_data.counter += 1
        elif user_data.counter == 5:
            user_data.send_command("FORWARD2")
            user_data.counter += 1
        elif user_data.counter == 6:
            user_data.send_command("90RIGHT")
            user_data.counter += 1
        elif user_data.counter == 7:
            user_data.send_command("FORWARD1")
            user_data.counter += 1
        elif user_data.counter == 8:
            user_data.send_command("90LEFT")
            user_data.counter += 1
        else:
            print("Stage 13 done going to stage 14 so should stop")
            user_data.counter = 0
            user_data.set_stage(14)
        
    elif user_data.stage == 14:
        user_data.send_command("FORWARD20")
        user_data.set_stage(15)

#     else:
#         user_data.send_command("NO DETECTION")

    return Gst.PadProbeReturn.OK


if __name__ == "__main__":
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()

