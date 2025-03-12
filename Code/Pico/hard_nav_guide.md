# Hard Coded Navigation Usage
1. Upload [pulse_count_motor.py](Code/Pico/pulse_count_motor.py) to your pico.
2. Modify following lines in [hard_nav.py](Code/Pico/hard_nav.py)
  ```python
  PPR = 11
  GEAR_RATIO = 50
  WHEEL_RADIUS = 0.055  # m
  WHEEL_SEP = 0.355  # m
  ```
  > line 6 to 9.
3. Fill your measurements in [hard_nav.py](Code/Pico/hard_nav.py)
  ```python
  stage_distances = [1.4, 1.]  # FILL IN YOUR MEASUREMENTS
  stage_angles = [-3*pi/4, 0.]  # FILL IN YOUR MEASUREMENTS
  ```
  > line 62, 63.
4. Run [hard_nav.py](Code/Pico/hard_nav.py). 
  > Of course, you can set it as `main.py`.
