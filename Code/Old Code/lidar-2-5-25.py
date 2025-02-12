import serial
import time
from adafruit_rplidar import RPLidar

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

# Setup serial communication with the Pico
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust port as needed

# Distance threshold in meters
STOP_DISTANCE = 1.0  

def get_distance_at_angle(target_angle=180):
    """Gets the distance at a specific LiDAR angle (default 180°)."""
    try:
        for scan in lidar.iter_scans():
            for _, angle, distance in scan:
                if round(angle) == target_angle:
                    return distance / 1000.0  # Convert mm to meters
    except Exception as e:
        print(f"LiDAR error: {e}")
    return None

def read_encoder_counts():
    """Reads encoder counts from the Pico."""
    ser.flushInput()
    ser.write(b"get_encoders\n")  # Request encoder data
    response = ser.readline().decode('utf-8').strip()
    
    if response:
        try:
            left_count, right_count = map(int, response.split(","))
            return left_count, right_count
        except ValueError:
            print(f"Invalid encoder data: {response}")
    
    return None, None

def main():
    try:
        while True:
            distance = get_distance_at_angle(180)  # Read distance at 180°

            if distance is not None:
                print(f"Object detected at {distance:.2f}m")
                if distance > STOP_DISTANCE:
                    print("Moving forward")
                    ser.write(b"move_forward\n")
                else:
                    print("Stopping")
                    ser.write(b"stop\n")
            
            # Read and display encoder counts
            left_enc, right_enc = read_encoder_counts()
            if left_enc is not None and right_enc is not None:
                print(f"Encoders -> Left: {left_enc}, Right: {right_enc}")

            time.sleep(0.5)  # Delay to avoid excessive command sending

    except KeyboardInterrupt:
        print("Stopping program.")
    finally:
        lidar.stop()
        lidar.disconnect()

if __name__ == "__main__":
    main()

