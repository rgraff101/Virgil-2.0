import time
import numpy as np
import serial
from adafruit_rplidar import RPLidar

class LidarNavigator:
    def __init__(self, pico_serial, lidar):
        self.pico_serial = pico_serial
        self.lidar = lidar
        self.is_bucket_seen = False
        self.is_bucket_close = False
        self.is_lidar45_picked = False
        self.stage_cleared = False
        self.arc_threshold = 1.0  # Adjust as needed
        self.wheel_diameter = 0.099
        self.encoder_ticks_per_rev = 20  # Adjust based on encoders
        self.encoder_count = 0
    
    def estimate_arc_increment(self):
        wheel_circumference = self.wheel_diameter * 3.14159
        distance_per_tick = wheel_circumference / self.encoder_ticks_per_rev
        arc_increment = self.encoder_count * distance_per_tick
        self.encoder_count = 0  # Reset encoder count after estimation
        return arc_increment
    
    def approach_target(self):
        print("Approaching Target...")
        
        # Send the status of is_bucket_seen and is_bucket_close
        self.pico_serial.write(f"is_bucket_seen:{self.is_bucket_seen},is_bucket_close:{self.is_bucket_close}\n".encode())
        
        if not self.is_bucket_seen:
            print("Bucket not seen, driving straight...")
            self.pico_serial.write("move_forward\n".encode())
            time.sleep(1)  # Increased sleep for smoother motion
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
                time.sleep(0.2)  # Increased sleep for smoother response
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.stop()
    
    def post_maneuver(self):
        print("Executing Post Maneuver...")

        if not self.is_lidar45_picked:
            print("Spinning right to pick LiDAR 45-degree reference...")
            self.pico_serial.write("turn_right\n".encode())
            time.sleep(2)  # Adjust timing
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
            time.sleep(1)  # Adjust timing
            self.pico_serial.write("stop\n".encode())
            
            self.stage_cleared = True
    
    def stop(self):
        self.pico_serial.write("stop\n".encode())
        self.lidar.stop()

if __name__ == '__main__':
    pico_serial = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
    lidar = RPLidar(None, '/dev/ttyUSB0', timeout=3)
    
    navigator = LidarNavigator(pico_serial, lidar)
    
    # Example of executing different functions based on stage
    stage = 1
    while True:
        if stage == 1:
            navigator.approach_target()
        elif stage == 2:
            navigator.post_maneuver()
        
        if navigator.stage_cleared:
            print(f"Stage {stage} complete. Moving to next stage...")
            stage += 1
            navigator.stage_cleared = False
        
        time.sleep(1)
