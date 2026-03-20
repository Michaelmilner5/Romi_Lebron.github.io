from utime import ticks_us, ticks_diff
from pyb import UART
import micropython
S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)
class LineFollowTask:
    def __init__(self, line_sensors,
                 share_v_base, share_vL_cmd, share_vR_cmd,
                 lfEnableShare,
                 Kp=60.0, Ki=0.0, Kd=8.0,
                 u_max=220.0,
                 max_min=0.08, sum_min=0.20,
                 oversample=5,
                 den_min=10,
                 pos_alpha=0.25,
                 pos_deadband=0.03,
                 stream_hz=0):
        self._state = S0_INIT
        self.sens     = line_sensors
        self.v_base   = share_v_base
        self.vL_cmd   = share_vL_cmd
        self.vR_cmd   = share_vR_cmd
        self.lfEnable = lfEnableShare
        self.Kp = float(Kp)
        self.Ki = float(Ki)
        self.Kd = float(Kd)
        self.u_max      = float(u_max)
        self.max_min    = float(max_min)
        self.sum_min    = float(sum_min)
        self.oversample = int(oversample)
        self.den_min    = int(den_min)
        self.e_prev   = 0.0
        self.i_term   = 0.0
        self.t_prev   = ticks_us()
        self.last_pos = 0.0
        self.pos_filt     = 0.0
        self.pos_alpha    = float(pos_alpha)
        self.pos_deadband = float(pos_deadband)
        self._uart = None
        self._stream_enabled = (int(stream_hz) > 0)
        self._t_stream_prev = ticks_us()
        self._stream_period_us = 1_000_000
        if self._stream_enabled:
            hz = int(stream_hz)
            if hz < 1:
                hz = 1
            self._stream_period_us = int(1_000_000 / hz)
            try:
                self._uart = UART(2, baudrate=115200, timeout_char=0)
            except Exception:
                self._uart = None
                self._stream_enabled = False
        print("Line Follow Task object instantiated")
    @staticmethod
    def _sat(x, lim):
        if x > lim:
            return lim
        if x < -lim:
            return -lim
        return x
    def _reset_pid(self):
        self.e_prev = 0.0
        self.i_term = 0.0
        self.t_prev = ticks_us()
        self.pos_filt = self.last_pos
        self._t_stream_prev = ticks_us()
    def run(self):
        while True:
            if self._state == S0_INIT:
                self._reset_pid()
                self.last_pos = 0.0
                self._state = S1_RUN
            elif self._state == S1_RUN:
                if not self.lfEnable.get():
                    self._reset_pid()
                    yield self._state
                    continue
                t_now = ticks_us()
                dt = ticks_diff(t_now, self.t_prev) / 1_000_000.0
                if dt <= 0.0:
                    dt = 1e-3
                self.t_prev = t_now
                norm = self.sens.read_norm(oversample=self.oversample, den_min=self.den_min)
                mx = max(norm)
                sm = sum(norm)
                pos = self.sens.centroid(norm)
                line_found = (mx >= self.max_min) and (sm >= self.sum_min)
                if not line_found:
                    pos = self.last_pos
                else:
                    self.last_pos = pos
                a = self.pos_alpha
                self.pos_filt = (1.0 - a) * self.pos_filt + a * pos
                pos = self.pos_filt
                if -self.pos_deadband < pos < self.pos_deadband:
                    pos = 0.0
                if self._stream_enabled and (self._uart is not None):
                    if ticks_diff(t_now, self._t_stream_prev) >= self._stream_period_us:
                        self._t_stream_prev = t_now
                        try:
                            self._uart.write("{},{}\r\n".format(t_now, pos))
                        except Exception:
                            pass
                e = -pos
                self.i_term += e * dt
                d_term = (e - self.e_prev) / dt
                self.e_prev = e
                u = (self.Kp * e) + (self.Ki * self.i_term) + (self.Kd * d_term)
                u = self._sat(u, self.u_max)
                vb = self.v_base.get()
                self.vL_cmd.put(vb - u)
                self.vR_cmd.put(vb + u)
            yield self._state
