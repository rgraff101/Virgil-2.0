import time
import numpy as np
import serial
from adafruit_rplidar import RPLidar

class LidarNavigator:
    def __init__(self, pico_serial, lidar):
        self.pico_serial = pico_serial
        self.lidar = lidar

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
                time.sleep(0.1)
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.stop()

    def stop(self):
        self.pico_serial.write("stop\n".encode())
        self.lidar.stop()

class DistanceController:
    def __init__(self, pico_serial):
        self.pico_serial = pico_serial
    
    def move_distance(self, distance):
        command = f"move_distance {distance}\n"
        print(f"Sending command: {command.strip()}")
        self.pico_serial.write(command.encode())
        time.sleep(distance * 2)  # Estimate wait time before switching modes

    def start_lidar_mode(self):
        print("Switching to LiDAR mode...")
        self.pico_serial.write("start_lidar\n".encode())
        time.sleep(1)  # Allow time for transition

if __name__ == '__main__':
    # Initialize Pico serial connection and LiDAR
    pico_serial = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
    lidar = RPLidar(None, '/dev/ttyUSB0', timeout=3)
    
    # Create the DistanceController and LidarNavigator objects with the initialized serial ports
    controller = DistanceController(pico_serial)
    lidar_nav = LidarNavigator(pico_serial, lidar)
    
    # Move a set distance
    controller.move_distance(1)  # Move 1 meter
    time.sleep(2)

    # Switch to LiDAR mode
    controller.start_lidar_mode()
    lidar_nav.scan_and_navigate()
    time.sleep(2)

    # Move another set distance
    controller.move_distance(4)  # Move 4 meters after LiDAR navigation
    time.sleep(2)
    
    # Optionally repeat LiDAR navigation
    controller.start_lidar_mode()
    lidar_nav.scan_and_navigate()

    # Clean up
    lidar.stop()
    lidar.disconnect()
    pico_serial.close()
