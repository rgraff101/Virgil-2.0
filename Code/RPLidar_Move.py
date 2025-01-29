import serial
from math import floor
from adafruit_rplidar import RPLidar
import time

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

# Setup serial communication with the Pico
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust port name as needed

def process_lidar_data(scan_data):
    """
    Process the lidar data and send appropriate commands to the Pico.
    """
    distance_at_180 = None
    angle_of_bucket = None
    min_distance = float('inf')

    # Loop over the scan data and process the angle and distance
    for (_, angle, distance) in scan_data:
        if 150 <= angle <= 210:  # Bucket is in this range
            if distance < min_distance:
                min_distance = distance / 1000
                angle_of_bucket = angle

    if angle_of_bucket is not None:
        print(f"Bucket detected at {angle_of_bucket}° with distance {min_distance} meters.")

        if min_distance > 0.5:
            print("Move forward toward the bucket.")
            ser.write(b"move_forward\n")  # Send move forward command to Pico
        else:
            print("Stop moving; bucket is too close.")
            ser.write(b"stop\n")  # Send stop command to Pico
    else:
        print("No bucket detected in the range of 150° to 210°.")
        ser.write(b"stop\n")  # Stop if no bucket is detected

def main():
    try:
        for scan in lidar.iter_scans():
            # Process the scan data
            process_lidar_data(scan)
            time.sleep(0.5)  # Pause to control the frequency of sending commands
    except KeyboardInterrupt:
        print('Stopping.')
    finally:
        lidar.stop()
        lidar.disconnect()

if __name__ == "__main__":
    main()
