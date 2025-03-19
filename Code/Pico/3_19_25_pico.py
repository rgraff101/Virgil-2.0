from machine import Pin, PWM
import time
import sys
import math
import select

class Motor:
    def __init__(self, dir_pin, pwm_pin, enc_a, enc_b, slp_pin):
        self._dir_pin = Pin(dir_pin, Pin.OUT)
        self._pwm_pin = PWM(Pin(pwm_pin))
        self._slp_pin = Pin(slp_pin, Pin.OUT)
        self._enc_a = Pin(enc_a, Pin.IN)
        self._enc_b = Pin(enc_b, Pin.IN)
        self._pwm_pin.freq(1000)
        self._pwm_pin.duty_u16(0)  
        self._slp_pin.value(1) 
        self.encoder_count = 0

        self._enc_a.irq(trigger=Pin.IRQ_RISING, handler=self._encoder_callback)
        self._enc_b.irq(trigger=Pin.IRQ_RISING, handler=self._encoder_callback)

    def _encoder_callback(self, pin):
        self.encoder_count += 1

    def reset_encoder(self):
        self.encoder_count = 0

    def get_encoder_count(self):
        return self.encoder_count

    def forward(self, duty=1.0):
        self._dir_pin.value(0)  # Forward direction
        pwm_value = int(duty * 65535)
        self._pwm_pin.duty_u16(pwm_value)

    def reverse(self, duty=1.0):  # ✅ Added missing reverse function
        self._dir_pin.value(1)  # Reverse direction
        pwm_value = int(duty * 65535)
        self._pwm_pin.duty_u16(pwm_value)

    def stop(self):
        self._pwm_pin.duty_u16(0)  

# ✅ Define constants
WHEEL_RADIUS = 0.0495  # Example: 3.5 cm = 0.035 meters
WHEEL_SEP = 0.295  # Example: 14 cm = 0.14 meters
GEAR_RATIO = 70  # Example gear ratio
PPR = 16  # Pulses per revolution

lm = Motor(3, 2, 14, 15, 4)  
rm = Motor(7, 6, 12, 13, 8) 

lm._slp_pin.value(1)
rm._slp_pin.value(1)

poller = select.poll()

poller.register(sys.stdin, select.POLLIN)

def straight_forward(target_distance=0., speed=0.3, diff_thres=10):
    targ_revs = target_distance / (2 * math.pi * WHEEL_RADIUS)
    targ_pcnt = targ_revs * GEAR_RATIO * PPR
    lm.reset_encoder()
    rm.reset_encoder()

    while lm.encoder_count < targ_pcnt and rm.encoder_count < targ_pcnt:
        if lm.encoder_count - rm.encoder_count > diff_thres:
            lm.stop()
            rm.forward(speed)
        elif lm.encoder_count - rm.encoder_count < -diff_thres:
            lm.forward(speed)
            rm.stop()
        else:
            lm.forward(speed)
            rm.forward(speed)
        time.sleep(0.01)    

    lm.stop()
    rm.stop()
    time.sleep(1)
    lm.reset_encoder()
    rm.reset_encoder()

def spin(target_angle=0., speed=0.2):
    targ_arc = target_angle * WHEEL_SEP / 2
    targ_revs = targ_arc / (2 * math.pi * WHEEL_RADIUS)
    targ_pcnt = targ_revs * GEAR_RATIO * PPR
    lm.reset_encoder()
    rm.reset_encoder()

    while lm.encoder_count < abs(targ_pcnt) and rm.encoder_count < abs(targ_pcnt):
        if targ_pcnt > 0:
            lm.reverse(speed)
            rm.forward(speed)
        else:
            lm.forward(speed)
            rm.reverse(speed)
        time.sleep(0.01)

    lm.stop()
    rm.stop()
    time.sleep(1)
    lm.reset_encoder()
    rm.reset_encoder()

def handle_command(command):
    print(f"Received command: {command}")
    
    if command == "FORWARD":
        lm.reset_encoder()
        rm.reset_encoder()
        lm.forward(duty=0.3)
        rm.forward(duty=0.3)
        print("Moving forward...")
    elif command == "STOP":
        lm.stop()
        rm.stop()
        print("Stopping motors...")
    elif command == "LEFT":
        lm.forward(duty=0.2)  
        rm.forward(duty=0.3)  
        print("Turning left...")
    elif command == "RIGHT":
        lm.forward(duty=0.3) 
        rm.forward(duty=0.2)
        print("Turning right...")
    elif command == "NO DETECTION":
        lm.reset_encoder()
        rm.reset_encoder()
        lm.forward(duty=0.2)
        rm.forward(duty=0.2)
        print("Moving forward...")
    elif command == "90LEFT":
        spin(math.pi, 0.4)
    elif command == "90RIGHT":
        spin(-math.pi, 0.4)
    elif command == "FORWARD1":
        straight_forward(1)
    elif command == "FORWARD2":
        straight_forward(2)
    else:
        print(f"Unrecognized command: {command}")
        lm.stop()
        rm.stop()

if __name__ == '__main__':
    print("Motor control system initialized. Awaiting commands...")
    while True:
        events = poller.poll()  # Check if there's any data
        for event in events:
            if event[0] == sys.stdin:
                command = sys.stdin.readline().rstrip()  # Read and clean up the command
                handle_command(command)

        time.sleep(0.1)  # Small delay to prevent overloading the poller

