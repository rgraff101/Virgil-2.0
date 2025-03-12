from math import pi
from pulse_count_motor import PulseCountMotor
from time import sleep

# Constants
PPR = 11
GEAR_RATIO = 50
WHEEL_RADIUS = 0.055  # m
WHEEL_SEP = 0.355  # m

# Utilities
def straight_forward(target_distance=0., speed=0.4, diff_thres=10,):
    targ_revs = target_distance /  (2 * pi * WHEEL_RADIUS)
    targ_pcnt = targ_revs * GEAR_RATIO * PPR
    while lm.pulse_count < targ_pcnt and rm.pulse_count < targ_pcnt:
        if lm.pulse_count - rm.pulse_count > diff_thres:
            lm.stop()
            rm.forward(speed)
        elif lm.pulse_count - rm.pulse_count < -diff_thres:
            lm.forward(speed)
            rm.stop()
        else:
            lm.forward(speed)
            rm.forward(speed)
        sleep(0.01)    
    lm.stop()
    rm.stop()
    sleep(1)
    print(f"Traveled {target_distance} meters.")
    print(f"left motor pulses: {lm.pulse_count}; right motor pulses: {rm.pulse_count}.")
    lm.pulse_count = 0
    rm.pulse_count = 0

def spin(target_angle=0., speed=0.25):
    targ_arc = target_angle * WHEEL_SEP / 2
    targ_revs = targ_arc /  (2 * pi * WHEEL_RADIUS)
    targ_pcnt = targ_revs * GEAR_RATIO * PPR
    while lm.pulse_count < abs(targ_pcnt) and rm.pulse_count < abs(targ_pcnt):
        if targ_pcnt > 0:
            lm.reverse(speed)
            rm.forward(speed)
        else:
            lm.forward(speed)
            rm.reverse(speed)
        sleep(0.01)
    lm.stop()
    rm.stop()
    sleep(1)
    print(f"Turned {target_angle} radians.")
    print(f"left motor pulses: {lm.pulse_count}; right motor pulses: {rm.pulse_count}.")
    lm.pulse_count = 0
    rm.pulse_count = 0

# SETUP
# Init motors
lm = PulseCountMotor(6, 7, 8, 14, 15)
rm = PulseCountMotor(2, 3, 4, 12, 13)
lm.enable()
rm.enable()
### START CODING HERE ###
# Define stages
stage_distances = [1.4, 1.]  # FILL IN YOUR MEASUREMENTS
stage_angles = [-3*pi/4, 0.]  # FILL IN YOUR MEASUREMENTS
### END CODING HERE ###
assert len(stage_distances) == len(stage_angles)

# LOOP
from machine import freq
freq(166_000_000)
for i, s in enumerate(stage_distances):
    # Straight forward
    straight_forward(target_distance=s)
    # Spin in place
    spin(stage_angles[i])

# SHUTDOWN
lm.stop()
rm.stop()
sleep(1)
lm.disable()
rm.disable()
freq(125_000_000)
