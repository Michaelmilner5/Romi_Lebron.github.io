from pyb import Pin
import micropython
from utime import ticks_ms, ticks_diff
S0_RUN = micropython.const(0)
class task_start_button:
    def __init__(self, courseEnableShare, bumpEventShare=None,
                 pin=Pin.cpu.C13, debounce_ms=60):
        self._state = S0_RUN
        self._btn = Pin(pin, Pin.IN, Pin.PULL_UP)
        self._courseEnable = courseEnableShare
        self._bumpEvent = bumpEventShare
        self._prev = self._btn.value()
        self._t_last = ticks_ms()
        self._debounce = int(debounce_ms)
        self._hb_t = ticks_ms()
        print("Start Button Task object instantiated")
    def _clear_bump(self):
        if self._bumpEvent is not None:
            try:
                self._bumpEvent.put(False)
            except Exception:
                pass
    def run(self):
        while True:
            now = ticks_ms()
            val = self._btn.value()
            if ticks_diff(now, self._hb_t) >= 1000:
                self._hb_t = now
                print("StartBtn alive, btn=", val,
                      "courseEnable=", int(self._courseEnable.get()))
            if (self._prev == 1) and (val == 0):
                if ticks_diff(now, self._t_last) >= self._debounce:
                    self._t_last = now
                    en = bool(self._courseEnable.get())
                    if not en:
                        print("USER BTN -> START COURSE")
                        self._clear_bump()
                        self._courseEnable.put(True)
                    else:
                        print("USER BTN -> STOP COURSE")
                        self._courseEnable.put(False)
            self._prev = val
            yield self._state
