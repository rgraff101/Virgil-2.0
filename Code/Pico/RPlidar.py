from machine import Pin, PWM
import time
from machine import UART

class Motor:
    def __init__(self, dir_pin, pwm_pin, slp_pin):
        self._dir_pin = Pin(dir_pin, Pin.OUT)
        self._pwm_pin = PWM(Pin(pwm_pin))
        self._slp_pin = Pin(slp_pin, Pin.OUT)  # Sleep pin
        self._pwm_pin.freq(1000)
        self._pwm_pin.duty_u16(0)  # Initially stop motor
        self._slp_pin.value(1)  # Ensure the motor is awake

    def forward(self, duty=1.0):
        self._dir_pin.value(0)  # Forward direction
        self._pwm_pin.duty_u16(int(duty * 65536))

    def backward(self, duty=1.0):
        self._dir_pin.value(1)  # Backward direction
        self._pwm_pin.duty_u16(int(duty * 65536))

    def stop(self):
        self._pwm_pin.duty_u16(0)  # Stop motor

# Initialize motor instances
lm = Motor(3, 2, 4)  # Left motor pins
rm = Motor(7, 6, 8)  # Right motor pins

# Setup serial communication with the Raspberry Pi
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# Main loop to receive commands from Raspberry Pi
while True:
    if uart.any():
        command = uart.read().decode('utf-8').strip()  # Read the command from the Raspberry Pi
        print(f"Received command: {command}")

        # Execute commands based on the received data
        if command == "move_forward":
            lm.forward(duty=0.8)
            rm.forward(duty=0.8)
        elif command == "rotate_left":
            lm.backward(duty=0.5)
            rm.forward(duty=0.5)
        elif command == "rotate_right":
            lm.forward(duty=0.5)
            rm.backward(duty=0.5)
        elif command == "stop":
            lm.stop()
            rm.stop()

        time.sleep(0.1)  # Small delay to prevent overloading the UART

