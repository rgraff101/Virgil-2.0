import sys
from math import pi
from machine import Pin, PWM
from time import sleep

class Motor:

    def __init__(self, dir_pin, pwm_pin, enca_pin, encb_pin, slp_pin):
        self._dir_pin = Pin(dir_pin, Pin.OUT)
        self._pwm_pin = PWM(Pin(pwm_pin))
        self._enca_pin = Pin(enca_pin, Pin.IN, Pin.PULL_UP)
        self._encb_pin = Pin(encb_pin, Pin.IN, Pin.PULL_UP)
        self._slp_pin = Pin(slp_pin, Pin.OUT)  
        self._pwm_pin.freq(1000)
        self.disable()
        self._enca_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._inc_counts)
        self._encb_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._inc_counts)
        self.encoder_counts = 0
        self.stop()

    def _inc_counts(self, pin):
        self.encoder_counts += 1

    def stop(self):
        self._pwm_pin.duty_u16(0)

    def enable(self):
        self._slp_pin.value(1)

    def disable(self):
        self._pwm_pin.duty_ns(0)
        self._slp_pin.value(0)

    def forward(self, duty=1.0):
        self._dir_pin.value(0)
        self._pwm_pin.duty_u16(int(duty*65536))

    def backward(self, duty=1.0):
        self._dir_pin.value(1)
        self._pwm_pin.duty_u16(int(duty*65536))

    def reset_encoder(self):
        self.encoder_counts = 0

# Initialize UART to communicate with Raspberry Pi
from machine import UART
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))  # Adjust TX/RX pins as needed

if __name__ == "__main__":
    from machine import freq
    freq(200_000_000)
    
    # Constants for robot setup
    WHEEL_RADIUS = 0.0495  # meters
    WHEEL_SEPARATION = 0.295  # meters
    CPR = 64  # Counts per revolution
    GEAR_RATIO = 70

    # Initialize motors
    lm = Motor(3, 2, 14, 15, 4)
    rm = Motor(7, 6, 12, 13, 8)
    lm.enable()
    rm.enable()

    # Main loop to receive LiDAR commands and control motors
    while True:
        if uart.any():
            # Read the incoming command from Raspberry Pi
            command = uart.read().decode('utf-8').strip()
            print(f"Received command: {command}")
            
            # Control motors based on the received command
            if command == "STOP":
                lm.stop()
                rm.stop()
            elif command == "FORWARD":
                lm.forward(duty=0.5)  # Move forward at 50% speed
                rm.forward(duty=0.5)  # Move forward at 50% speed
            elif command == "BACKWARD":
                lm.backward(duty=0.5)  # Move backward at 50% speed
                rm.backward(duty=0.5)  # Move backward at 50% speed
            elif command == "TURN_LEFT":
                lm.backward(duty=0.5)  # Left motor moves backward
                rm.forward(duty=0.5)  # Right motor moves forward
            elif command == "TURN_RIGHT":
                lm.forward(duty=0.5)  # Left motor moves forward
                rm.backward(duty=0.5)  # Right motor moves backward
            sleep(0.1)

