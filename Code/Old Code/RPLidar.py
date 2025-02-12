from math import floor
from adafruit_rplidar import RPLidar

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            if floor(angle) == 180:  # print distance at angle 180
                print(f"Distance at 180°: {distance}")
                break  
except KeyboardInterrupt:
    print('Stopping.')
finally:
    lidar.stop()
    lidar.disconnect()
