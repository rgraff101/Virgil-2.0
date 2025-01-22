import serial
from math import floor
from adafruit_rplidar import RPLidar

# Setup the RPLidar and serial connection to Pico
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

# Serial port for communication with the Pico
pico_serial = serial.Serial('/dev/ttyAMA0', 9600)  # Adjust port as needed

scan_data = [0] * 360

try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            # Update scan_data with the distance for the corresponding angle
            scan_data[min([359, floor(angle)])] = distance
        
        # Example logic: If something is too close at 180° (i.e., 1 meter or less), stop
        if scan_data[180] < 1000:  # Distance is in mm, so 1000mm = 1m
            pico_serial.write(b"STOP")  # Send "STOP" command to Pico
        elif scan_data[180] > 1000 and scan_data[180] < 2000:  # Distance between 1m and 2m
            pico_serial.write(b"FORWARD")  # Send "FORWARD" command to Pico
        else:
            pico_serial.write(b"TURN")  # Send "TURN" command to Pico
        
        print(f"Distance at 180°: {scan_data[180]} mm")

except KeyboardInterrupt:
    print('Stopping.')
lidar.stop()
lidar.disconnect()

