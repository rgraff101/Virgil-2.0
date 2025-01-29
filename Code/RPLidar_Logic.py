import serial
import numpy as np
from math import floor
from adafruit_rplidar import RPLidar
import time

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

# Setup serial communication with the Pico
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust port name as needed

thres = 5  # Define the threshold for turning logic

def process_lidar_data(scan_data):
    """
    Process the lidar data and send appropriate commands to the Pico based on the number of zero values 
    (indicating obstacles or missing data) in the left and right sides of the scan.
    """
    # Extracting distances from the scan data
    distances = [distance for (_, angle, distance) in scan_data]

    # Counting zeros in the left and right parts of the scan
    num_0_left = len(np.where(distances[330:] == 0)[0])
    num_0_right = len(np.where(distances[:30] == 0)[0])

    # Decision based on the difference in zero counts
    if num_0_left - num_0_right > thres:
        print("Turn right")
        ser.write(b"turn_right\n")  # Send turn right command to Pico
    elif num_0_left - num_0_right < -thres:
        print("Turn left")
        ser.write(b"turn_left\n")  # Send turn left command to Pico
    else:
        print("Go straight")
        ser.write(b"go_straight\n")  # Send go straight command to Pico

    # Continue checking bucket distance in the range of 150° to 210°
    distance_at_150_210 = None
    min_distance = float('inf')
    angle_of_bucket = None

    # Loop over the scan data and find the minimum distance in the range of 150° to 210°
    for (_, angle, distance) in scan_data:
        if 150 <= angle <= 210:  # Bucket is in this range
            # Convert the distance from mm to meters
            distance_in_meters = distance / 1000.0
            
            # Track the minimum distance and its angle
            if distance_in_meters < min_distance:
                min_distance = distance_in_meters
                angle_of_bucket = angle

    if angle_of_bucket is not None:
        print(f"Bucket detected at {angle_of_bucket}° with distance {min_distance} meters.")
        
        if min_distance > 0.5:  # Only move forward if the distance is greater than 0.5 meters
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
    except Exception as e:
        print(f"Error with LIDAR: {e}")
    finally:
        lidar.stop()
        lidar.disconnect()

if __name__ == "__main__":
    main()
