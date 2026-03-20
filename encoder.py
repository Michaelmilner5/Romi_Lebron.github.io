from pyb import Pin, Timer
from time import ticks_us, ticks_diff
class encoder:
    def __init__(self, tim: Timer, chA_pin: Pin, chB_pin: Pin, period: int = 0xFFFF):
        self.tim = tim
        self.period = period
        self.half_period = (period + 1) // 2
        self.tim.channel(1, Timer.ENC_AB, pin=Pin(chA_pin))
        self.tim.channel(2, Timer.ENC_AB, pin=Pin(chB_pin))
        self.position = 0
        self.prev_count = self.tim.counter()
        self.delta = 0
        self.prev_time = ticks_us()
        self.dt = 1e-6
    def update(self):
        now = ticks_us()
        dt_us = ticks_diff(now, self.prev_time)
        self.prev_time = now
        if dt_us <= 0:
            dt_us = 1
        self.dt = dt_us / 1_000_000.0
        count = self.tim.counter()
        raw_delta = -(count - self.prev_count)
        self.prev_count = count
        if raw_delta > self.half_period:
            raw_delta -= (self.period + 1)
        elif raw_delta < -self.half_period:
            raw_delta += (self.period + 1)
        self.delta = raw_delta
        self.position += raw_delta
    def get_position(self):
        return self.position
    def get_velocity(self):
        return self.delta / self.dt if self.dt > 0 else 0.0
    def zero(self):
        self.position = 0
        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()
