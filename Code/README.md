# Code Docs

## Get Camera and Lidar Working Together

Unit test script: [test_cam_lidar.py](test_cam_lidar.py)

- Turned out threading is a must. See [Tutorial](https://realpython.com/intro-to-python-threading/).
  - Haven't tested yet, but include a while loop in the target function may not be a good idea.
- Issue: LiDAR may spit out all zeros.
  - **Solution**: just `Ctrl-c` and rerun.

## Hard Coded Navigation Usage

1. Upload [pulse_count_motor.py](./Pico/pulse_count_motor.py) to your pico.
2. Modify following lines in [hard_nav.py](./Pico/hard_nav.py)

  ```python
  PPR = 11
  GEAR_RATIO = 50
  WHEEL_RADIUS = 0.055  # m
  WHEEL_SEP = 0.355  # m
  ```

  > line 6 to 9.

3. Fill your measurements in [hard_nav.py](./Pico/hard_nav.py)

  ```python
  stage_distances = [1.4, 1.]  # FILL IN YOUR MEASUREMENTS
  stage_angles = [-3*pi/4, 0.]  # FILL IN YOUR MEASUREMENTS
  ```

  > line 62, 63.

4. Run [hard_nav.py](./Pico/hard_nav.py) on your Pico (**use MicroPython**).

  > Of course, you can set it as `main.py`.
