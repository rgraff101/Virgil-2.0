import time
import numpy as np
import serial  # To communicate with the Pico via USB
from adafruit_rplidar import RPLidar

# Setup serial connection with the Pico (adjust with your port)
pico_serial = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)

# Set the correct USB port for your Raspberry Pi's RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

try:
    print("Starting scan...")
    # Start scanning
    for scan in lidar.iter_scans():
        scan_data = np.zeros(360)
        for _, angle, distance in scan:
            scan_data[int(angle) % 360] = distance  # Store distance at corresponding angle

        # Process left and right side data
        print("Left side (15°):", scan_data[:15])  # First 15 degrees
        print("Right side (345°):", scan_data[345:])  # Last 15 degrees

        # Count non-zero values in left and right regions
        ns_l = np.count_nonzero(scan_data[345:])
        ns_r = np.count_nonzero(scan_data[:15])

        print("Left:", ns_l, "Right:", ns_r)

        # Make decisions based on sensor data
        if ns_l - ns_r > 4:
            command = "turn_right\n"
            print("Sending command: Turn right")
        elif ns_l - ns_r < -4:
            command = "turn_left\n"
            print("Sending command: Turn left")
        else:
            command = "move_forward\n"
            print("Sending command: Move forward")

        # Send the command to the Pico
        pico_serial.write(command.encode())  # Send command to Pico

        # Wait for a short period before the next scan to prevent flooding
        time.sleep(0.1)

except Exception as e:
    print(f"Error: {e}")

finally:
    lidar.stop()  # Stop the Lidar after the scan
    lidar.disconnect()  # Disconnect the Lidar
    pico_serial.close()  # Close the serial connection
