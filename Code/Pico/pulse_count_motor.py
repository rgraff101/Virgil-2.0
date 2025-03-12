from machine import Pin, PWM

class PulseCountMotor:
    def __init__(self, pwm_pin_id, dir_pin_id, slp_pin_id, enca_pin_id, encb_pin_id):
        # Config GP pins
        self.slp_pin = Pin(slp_pin_id, Pin.OUT)
        self.slp_pin.value(0)
        self.dir_pin = Pin(dir_pin_id, Pin.OUT)
        self.pwm_pin = PWM(Pin(pwm_pin_id))
        self.pwm_pin.freq(1000)
        self.enca_pin = Pin(enca_pin_id, Pin.IN)
        self.encb_pin = Pin(encb_pin_id, Pin.IN)
        self.pwm_pin.duty_u16(0)  

        self.enca_pin.irq(trigger=Pin.IRQ_RISING, handler=self.inc_pulse_count)
        
        # Variables
        self.pulse_count = 0
        
        # Constants
        self.WHEEL_RADIUS = 0.055  # m
        self.GEAR_RATIO = 50
        self.PPR = 11

    def inc_pulse_count(self, pin):
        self.pulse_count += 1

    def forward(self, dc_percent=0.):
        self.dir_pin.value(1) 
        duty_val = int(dc_percent * 65535)
        self.pwm_pin.duty_u16(duty_val)

    def reverse(self, dc_percent=0.):
        self.dir_pin.value(0) 
        duty_val = int(dc_percent * 65535)
        self.pwm_pin.duty_u16(duty_val)

    def stop(self):
        self.pwm_pin.duty_u16(0)  

    def enable(self):
        self.stop()
        self.slp_pin.value(1)

    def disable(self):
        self.stop()
        self.slp_pin.value(0)

if __name__=='__main__':
    from machine import freq
    from time import sleep
    
    # SETUP
    freq(166_000_000)  # overclock
    print(f"CPU frequency set to: {freq()}")
    pin_ids = (2, 3, 4, 12, 13)
    # pin_ids = (6, 7, 8, 14, 15)
    m = PulseCountMotor(*pin_ids)
    m.enable()
    # LOOP
    for dcp in range(100):
        m.forward(dcp/100)
        sleep(0.05)
        print(f"FORWARD, pulse count: {m.pulse_count}")
    for dcp in reversed(range(100)):
        m.forward(dcp/100)
        sleep(0.05)
        print(f"FORWARD, pulse count: {m.pulse_count}")
    for dcp in range(100):
        m.reverse(dcp/100)
        sleep(0.05)
        print(f"REVERSE, pulse count: {m.pulse_count}")
    for dcp in reversed(range(100)):
        m.reverse(dcp/100)
        sleep(0.05)
        print(f"REVERSE, pulse count: {m.pulse_count}")

    # SHUTDOWN
    m.stop()
    sleep(1)
    m.disable()
    freq(125_000_000)
    print(f"CPU frequency set to: {freq()}")
