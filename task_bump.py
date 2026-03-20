from pyb import Pin
import micropython
S0_RUN = micropython.const(0)
class task_bump:
    def __init__(self, bumpEventShare):
        self._state = S0_RUN
        self.bump_left  = Pin(Pin.cpu.B2, Pin.IN, Pin.PULL_UP)
        self.bump_right = Pin(Pin.cpu.B7, Pin.IN, Pin.PULL_UP)
        self._bumpEvent = bumpEventShare
        self._prev_left = 1
        self._prev_right = 1
        print("Bump Task object instantiated")
    def run(self):
        while True:
            left_val = self.bump_left.value()
            right_val = self.bump_right.value()
            if (self._prev_left == 1 and left_val == 0):
                print("LEFT BUMP -> EVENT")
                self._bumpEvent.put(True)
            if (self._prev_right == 1 and right_val == 0):
                print("RIGHT BUMP -> EVENT")
                self._bumpEvent.put(True)
            self._prev_left = left_val
            self._prev_right = right_val
            yield self._state
