import gi
import os
import numpy as np
import cv2
import hailo
import rplidar
import serial
from gi.repository import Gst, GLib
from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

# Import movement functions
from movement import move_forward, turn_left, turn_right, stop, spin_45

# Function to get front-facing LiDAR distance
def get_front_distance():
    """Returns the front-facing LiDAR distance in meters."""
    lidar = rplidar.RPLidar('/dev/ttyUSB0')  # Adjust based on setup
    distances = []

    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            if -10 <= angle <= 10:  # Forward-facing readings
                distances.append(distance)
        if distances:
            lidar.stop()
            lidar.disconnect()
            return (sum(distances) / len(distances)) / 1000.0  # Convert mm to meters
    return None

def get_distance_at_45():
    """Returns the LiDAR distance at a 45-degree angle in meters."""
    lidar = rplidar.RPLidar('/dev/ttyUSB0')  # Adjust based on setup
    distances = []

    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            if 35 <= angle <= 55:  # Right-facing readings at 45 degrees
                distances.append(distance)
            elif -55 <= angle <= -35:  # Left-facing readings at -45 degrees
                distances.append(distance)
        if distances:
            lidar.stop()
            lidar.disconnect()
            return (sum(distances) / len(distances)) / 1000.0  # Convert mm to meters
    return None


# User-defined class to handle serial communication and movement logic
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.serial_conn = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.previous_move = None  # Store last move
        self.lidar_distance = None  # Store latest LiDAR reading
        self.stage = 1  # Tracks the current stage

    def send_command(self, command):
        """Send movement command to Raspberry Pi Pico."""
        self.serial_conn.write((command + "\n").encode())
        print(f"Sent: {command}")
      
    def update_lidar(self, distance):
        """Store latest LiDAR distance."""
        self.lidar_distance = distance

    def set_stage(self, new_stage):
        """Move to the next stage if not already in it."""
        if self.stage != new_stage:
            self.stage = new_stage
            print(f"Transitioning to Stage {self.stage}!")

# Function to decide movement direction based on AI detection
def decide_movement(user_data, width, bbox):
    """Determine movement based on AI detection."""
    x_min = bbox.xmin()
    x_max = bbox.xmax()
    x_center = (x_min + x_max) / 2
    frame_center = width / 2

    if x_center < frame_center - 20:
        return "left"
    elif x_center > frame_center + 20:
        return "right"
    else:
        return "forward"

# The callback function that handles the frame processing and movement decisions
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.increment()
    string_to_print = f"Frame count: {user_data.get_count()}\n"

    format, width, height = get_caps_from_pad(pad)
    frame = None
    if user_data.use_frame and format and width and height:
        frame = get_numpy_from_buffer(buffer, format, width, height)

    # Perform object detection
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    move_command = None

    for detection in detections:
        if detection.get_label() == "bucket":
            move_command = decide_movement(user_data, width, detection.get_bbox())

    # Get LiDAR distance
    distance = get_front_distance()
    user_data.update_lidar(distance)
    string_to_print += f"LiDAR Distance: {distance}m\n"

    # ---- Stage 1: Approach Bucket ----
    if user_data.stage == 1:
        if distance is not None and distance <= 1.0:
            stop()  # Stop when 1 meter away
            user_data.set_stage(2)  # Move to Stage 2
        elif move_command:
            if move_command == "left":
                user_data.send_command("LEFT")
            elif move_command == "right":
                user_data.send_command("RIGHT")
            elif move_command == "forward":
                user_data.send_command("FORWARD")

    # ---- Stage 2: Move Around Bucket ----
    elif user_data.stage == 2:
        if distance is not None and 0.4 <= distance <= 0.6:
            if user_data.start_time is None:
                user_data.start_wall_following()

            if user_data.is_wall_following_complete():
                user_data.set_stage(3)  # Move to Stage 3
            else:
                # Keep the robot at the correct distance and move
                move_forward()
                user_data.send_command("FORWARD")

        else:
            if distance < 0.4:
                turn_left()  # Adjust movement if too close to bucket
                user_data.send_command("LEFT")
            elif distance > 0.6:
                turn_right()  # Adjust movement if too far from bucket
                user_data.send_command("RIGHT")
                
    # ---- Stage 3: Align for Next Goal ----
    elif user_data.stage == 3:
        spin_45()  # Spin 45 degrees
        user_data.set_stage(4)  # Move to Stage 4

    # ---- Stage 4: Continue Navigation ----
    elif user_data.stage == 4:
        move_forward()  # Continue navigating

    print(string_to_print)
    return Gst.PadProbeReturn.OK

# Main entry point of the program
if __name__ == "__main__":
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()
