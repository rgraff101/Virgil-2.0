from machine import Pin, PWM
import time
import sys
import select
import math

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
        self._dir_pin.value(0)
        pwm_value = int(duty * 65535)
        self._pwm_pin.duty_u16(pwm_value)

    def stop(self):
        self._pwm_pin.duty_u16(0)

# Initialize motors
lm = Motor(3, 2, 14, 15, 4)
rm = Motor(7, 6, 12, 13, 8)

lm._slp_pin.value(1)
rm._slp_pin.value(1)

WHEEL_RADIUS = 0.0495  # meters
WHEEL_CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS
CPR = 64
GEAR_RATIO = 70

poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

def move_distance(distance_meters):
    target_counts = (distance_meters / WHEEL_CIRCUMFERENCE) * GEAR_RATIO * CPR
    lm.reset_encoder()
    rm.reset_encoder()
    
    while lm.encoder_count < target_counts and rm.encoder_count < target_counts:
        if lm.encoder_count - rm.encoder_count > 32:
            lm.stop()
            rm.forward(0.5)
        elif lm.encoder_count - rm.encoder_count < -32:
            lm.forward(0.5)
            rm.stop()
        else:
            lm.forward(0.5)
            rm.forward(0.5)
        time.sleep(0.01)
    
    lm.stop()
    rm.stop()
    time.sleep(1)

if __name__ == '__main__':
    print("Autonomous motor control initialized. Awaiting commands...")
    while True:
        events = poller.poll()
        for event in events:
            if event[0] == sys.stdin:
                command = sys.stdin.readline().strip()
                print(f"Received command: {command}")
                
                if command.startswith("move_distance"):
                    _, distance = command.split()
                    move_distance(float(distance))
                elif command == "stop":
                    lm.stop()
                    rm.stop()
                elif command == "turn_left":
                    lm.forward(0.1)
                    rm.forward(0.3)
                elif command == "turn_right":
                    lm.forward(0.3)
                    rm.forward(0.1)
                elif command == "start_lidar":
                    print("Switching to LiDAR mode...")
                    sys.stdout.write("lidar_ready\n")
                    sys.stdout.flush()
        time.sleep(0.1)
