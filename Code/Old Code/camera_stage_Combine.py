import time
import numpy as np
import serial
import cv2
from adafruit_rplidar import RPLidar
import os

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
import hailo
from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

class LidarNavigator:
    def __init__(self, pico_serial, lidar):
        self.pico_serial = pico_serial
        self.lidar = lidar
        self.is_bucket_seen = False
        self.is_bucket_close = False
        self.is_lidar45_picked = False
        self.stage_cleared = False
        self.arc_threshold = 1.0
        self.wheel_diameter = 0.099
        self.encoder_ticks_per_rev = 20
        self.encoder_count = 0

    def estimate_arc_increment(self):
        wheel_circumference = self.wheel_diameter * 3.14159
        distance_per_tick = wheel_circumference / self.encoder_ticks_per_rev
        arc_increment = self.encoder_count * distance_per_tick
        self.encoder_count = 0  
        return arc_increment

    def approach_bucket(self):
        print("Approaching Bucket...")

        if not self.is_bucket_seen:
            print("Bucket not seen, driving straight...")
            self.pico_serial.write("move_forward\n".encode())
            time.sleep(1)
        else:
            print("Bucket detected, navigating...")
            self.scan_and_navigate()

            if self.is_bucket_close:
                print("Bucket is close, moving to next stage...")
                self.stage_cleared = True

    def scan_and_navigate(self):
        try:
            print("Starting LiDAR navigation...")
            for scan in self.lidar.iter_scans():
                scan_data = np.zeros(360)
                for _, angle, distance in scan:
                    scan_data[int(angle) % 360] = distance  

                left_side = scan_data[:15]
                right_side = scan_data[345:]

                ns_l = np.count_nonzero(right_side)
                ns_r = np.count_nonzero(left_side)

                if ns_l - ns_r > 4:
                    command = "turn_right\n"
                elif ns_l - ns_r < -4:
                    command = "turn_left\n"
                else:
                    command = "move_forward\n"

                self.pico_serial.write(command.encode())
                time.sleep(0.2)
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.stop()

    def post_maneuver(self):
        print("Executing Post Maneuver...")

        if not self.is_lidar45_picked:
            print("Spinning right to pick LiDAR 45-degree reference...")
            self.pico_serial.write("turn_right\n".encode())
            time.sleep(2)
            self.pico_serial.write("stop\n".encode())
            self.is_lidar45_picked = True
        else:
            print("Navigating around the bucket...")
            arc_length = 0  

            while arc_length < self.arc_threshold:
                self.pico_serial.write("move_forward\n".encode())
                time.sleep(0.5)
                arc_length += self.estimate_arc_increment()

            print("Bucket cleared. Spinning 45 degrees...")
            self.pico_serial.write("turn_right_45\n".encode())
            time.sleep(1)
            self.pico_serial.write("stop\n".encode())

            self.stage_cleared = True

    def approach_ramp(self):
        print("Approaching Ramp...")
        self.pico_serial.write("move_forward\n".encode())
        time.sleep(2)  
        self.stage_cleared = True  

    def ramp_maneuver(self):
        print("Climbing Ramp...")
        self.pico_serial.write("ramp\n".encode())   
        self.stage_cleared = True  

    def approach_tunnel(self):
        print("Approaching Tunnel...")
        self.pico_serial.write("move_forward\n".encode())
        time.sleep(2)  
        self.stage_cleared = True  

    def under_tunnel(self):
        print("Driving Under Tunnel...")  
        self.pico_serial.write("move_forward\n".encode())
        time.sleep(2)  
        self.stage_cleared = True  

    def finish(self):
        print("Finishing sequence: Moving forward 6 meters...")
        target_distance = 6.0  # Meters
        current_distance = 0.0
        while current_distance < target_distance:
            self.bot.left_motor.forward(speed=0.5)
            self.bot.right_motor.forward(speed=0.5)
            time.sleep(0.1)  # Adjust based on encoder feedback
            current_distance += 0.1  # Replace with actual encoder-based distance measurement
        print("Finish line reached!")
        self.bot.left_motor.stop()
        self.bot.right_motor.stop()

    def stop(self):
        self.pico_serial.write("stop\n".encode())
        self.lidar.stop()


class UserAppCallback(app_callback_class):
    def __init__(self, navigator):
        super().__init__()
        self.navigator = navigator

    def process_frame(self, frame, detections):
        detection_count = 0
        for detection in detections:
            label = detection.get_label()
            bbox = detection.get_bbox()
            confidence = detection.get_confidence()
            
            if label == "bucket":
                detection_count += 1
                self.navigator.is_bucket_seen = True

                if bbox.xmax() - bbox.xmin() > 150:
                    self.navigator.is_bucket_close = True

                cv2.rectangle(frame, (bbox.xmin(), bbox.ymin()), (bbox.xmax(), bbox.ymax()), (0, 255, 0), 2)

        if detection_count == 0:
            self.navigator.is_bucket_seen = False

    def app_callback(self, pad, info, user_data):
        buffer = info.get_buffer()
        if buffer is None:
            return Gst.PadProbeReturn.OK

        format, width, height = get_caps_from_pad(pad)
        frame = None

        if user_data.use_frame and format and width and height:
            frame = get_numpy_from_buffer(buffer, format, width, height)

        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

        if frame is not None:
            self.process_frame(frame, detections)

        return Gst.PadProbeReturn.OK


if __name__ == "__main__":
    pico_serial = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
    lidar = RPLidar(None, '/dev/ttyUSB0', timeout=3)
    
    navigator = LidarNavigator(pico_serial, lidar)
    user_data = UserAppCallback(navigator)
    app = GStreamerDetectionApp(user_data.app_callback, user_data)

    stage = 1
    while True:
        if stage == 1:
            navigator.approach_bucket()
        elif stage == 2:
            navigator.post_maneuver()
        elif stage == 3:
            navigator.approach_bucket()
        elif stage == 4:
            navigator.post_maneuver()
        elif stage == 5:
            navigator.approach_bucket()
        elif stage == 6:
            navigator.post_maneuver()
        elif stage == 7:
            navigator.approach_ramp()
        elif stage == 8:
            navigator.ramp_maneuver()
        elif stage == 9:
            navigator.approach_bucket()
        elif stage == 10:
            navigator.post_maneuver()    
        elif stage == 11:
            navigator.approach_tunnel()
        elif stage == 12:
            navigator.under_tunnel()
        elif stage == 13:
            navigator.approach_bucket
        elif stage == 14:
            navigator.post_maneuver()
        elif stage == 15:
            navigator.finish()

        else:
            print("error with stages")

        if navigator.stage_cleared:
            print(f"Stage {stage} complete. Moving to next stage...")
            stage += 1
            navigator.stage_cleared = False

        time.sleep(1)
