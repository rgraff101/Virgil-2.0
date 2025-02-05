from machine import Pin, PWM
import time
import sys
import select

class Motor:
    def __init__(self, dir_pin, pwm_pin, slp_pin, enc_a, enc_b):
        self._dir_pin = Pin(dir_pin, Pin.OUT)
        self._pwm_pin = PWM(Pin(pwm_pin))
        self._slp_pin = Pin(slp_pin, Pin.OUT)
        self._enc_a = Pin(enc_a, Pin.IN)
        self._enc_b = Pin(enc_b, Pin.IN)
        self._pwm_pin.freq(1000)
        self._pwm_pin.duty_u16(0)  # Initially stop motor
        self._slp_pin.value(1)  # Wake up the motor driver
        self.encoder_count = 0

        # Interrupt handlers for encoder pulses
        self._enc_a.irq(trigger=Pin.IRQ_RISING, handler=self._encoder_callback)
        self._enc_b.irq(trigger=Pin.IRQ_RISING, handler=self._encoder_callback)

    def _encoder_callback(self, pin):
        """Increment encoder count on pulse detection."""
        self.encoder_count += 1

    def reset_encoder(self):
        """Reset encoder count to zero."""
        self.encoder_count = 0

    def get_encoder_count(self):
        """Return the encoder count."""
        return self.encoder_count

    def forward(self, duty=1.0):
        self._dir_pin.value(0)  # Forward direction
        self._pwm_pin.duty_u16(int(duty * 65535))

    def stop(self):
        self._pwm_pin.duty_u16(0)  # Stop motor

# Initialize motors with encoder support
lm = Motor(3, 2, 14, 15, 4)  # Left motor
rm = Motor(7, 6, 12, 13, 8)  # Right motor

# Create a poll object for non-blocking polling
poller = select.poll()

# Register sys.stdin for polling (Poll for input)
poller.register(sys.stdin, select.POLLIN)

if __name__ == '__main__':
    while True:
        # Poll for available data
        events = poller.poll()  # Check if there's any data
        for event in events:
            if event[0] == sys.stdin:
                command = sys.stdin.readline().rstrip()  # Read and clean up the command
                print(f"Received command: {command}")

                if command == "move_forward":
                    lm.reset_encoder()
                    rm.reset_encoder()
                    lm.forward(duty=0.6)
                    rm.forward(duty=0.6)
                    print("Moving forward...")
                elif command == "stop":
                    lm.stop()
                    rm.stop()
                    print("Stopping motors...")
                elif command == "turn_left":
                    lm.forward(duty=0.1)  # Slow down left motor to turn
                    rm.forward(duty=0.3)  # Keep right motor faster to turn
                    print("Turning left...")
                elif command == "turn_right":
                    lm.forward(duty=0.3)  # Keep left motor faster to turn
                    rm.forward(duty=0.1)  # Slow down right motor to turn
                    print("Turning right...")

        time.sleep(0.1)  # Small delay to prevent overloading the poller

