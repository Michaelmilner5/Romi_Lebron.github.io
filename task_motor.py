''' Motor task: encoder update + PI velocity control + Option-B logging
    + publishes observer signals (u in volts, s in meters)
    FIX: always update encoder + publish s_share even when Go flag is false,
    so pushing the robot by hand still updates sL_meas/sR_meas for the observer.
    NEW: automatic encoder direction detection so forward motion is always positive.
'''
from motor_driver import motor_driver
from encoder      import encoder
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
import micropython
import math
S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)
S2_RUN  = micropython.const(2)
class task_motor:
    def __init__(self,
                 mot: motor_driver, enc: encoder,
                 goFlag: Share, vRef: Share, Kp: Share, Ki: Share,
                 dataValues: Queue, timeValues: Queue, effortValues: Queue = None,
                 u_share: Share = None,
                 s_share: Share = None,
                 Vbatt: float = 5.0,
                 wheel_radius_m: float = 0.035,
                 counts_per_rev: int = 1440,
                 zero_encoder_on_start: bool = False
                 ):
        self._state = S0_INIT
        self._mot = mot
        self._enc = enc
        self._goFlag = goFlag
        self._vRef   = vRef
        self._Kp     = Kp
        self._Ki     = Ki
        self._dataValues   = dataValues
        self._timeValues   = timeValues
        self._effortValues = effortValues
        self._startTime = 0
        self._v_filt = 0.0
        self._e_int = 0.0
        self._dt = 0.005
        self._logging = True
        self._u_share = u_share
        self._s_share = s_share
        self._Vbatt   = float(Vbatt)
        self._R = float(wheel_radius_m)
        self._CPR = int(counts_per_rev)
        self._m_per_count = (2.0 * math.pi * self._R) / self._CPR
        self._zero_on_start = bool(zero_encoder_on_start)
        self._sign = 1.0
        self._sign_initialized = False
        print("Motor Task object instantiated")
    def _sat(self, u, lim=100.0):
        if u > lim:  return lim
        if u < -lim: return -lim
        return u
    def _clear_queue(self, q: Queue):
        try:
            while q.any():
                q.get()
        except AttributeError:
            try:
                while q.num_in() > 0:
                    q.get()
            except AttributeError:
                pass
    def _update_encoder_and_publish_s(self):
        """Always call this, even in WAIT, so s_share keeps updating."""
        try:
            self._enc.update()
        except Exception:
            return
        if self._s_share is not None:
            try:
                counts = self._enc.get_position()
                s_m = counts * self._m_per_count
                if s_m < 0:
                    s_m = -s_m
                self._s_share.put(s_m)
            except Exception:
                pass
    def run(self):
        while True:
            if self._state == S0_INIT:
                self._mot.set_effort(0)
                if self._u_share is not None:
                    self._u_share.put(0.0)
                self._state = S1_WAIT
            elif self._state == S1_WAIT:
                self._update_encoder_and_publish_s()
                self._mot.set_effort(0)
                try:
                    self._mot.disable()
                except Exception:
                    pass
                if self._u_share is not None:
                    self._u_share.put(0.0)
                if self._goFlag.get():
                    self._startTime = ticks_us()
                    if self._zero_on_start:
                        try:
                            self._enc.zero()
                        except Exception:
                            pass
                    if self._dataValues is not None:
                        self._clear_queue(self._dataValues)
                    if self._timeValues is not None:
                        self._clear_queue(self._timeValues)
                    if self._effortValues is not None:
                        self._clear_queue(self._effortValues)
                    self._v_filt = 0.0
                    self._e_int  = 0.0
                    self._logging = True
                    self._state = S2_RUN
            elif self._state == S2_RUN:
                if not self._goFlag.get():
                    self._mot.set_effort(0)
                    try:
                        self._mot.disable()
                    except Exception:
                        pass
                    if self._u_share is not None:
                        self._u_share.put(0.0)
                    self._state = S1_WAIT
                    yield self._state
                    continue
                self._mot.enable()
                self._update_encoder_and_publish_s()
                try:
                    v_raw = self._enc.get_velocity()
                except Exception:
                    v_raw = 0.0
                if not self._sign_initialized:
                    if abs(v_raw) > 0.01:
                        if v_raw < 0:
                            self._sign = -1.0
                        self._sign_initialized = True
                v_raw *= self._sign
                self._v_filt = 0.9 * self._v_filt + 0.1 * v_raw
                v_meas = self._v_filt
                t_rel = ticks_diff(ticks_us(), self._startTime)
                e = self._vRef.get() - v_meas
                e_int_new = self._e_int + e * self._dt
                u_unsat = self._Kp.get() * e + self._Ki.get() * e_int_new
                u = self._sat(u_unsat)
                if (u == u_unsat) or ((u == 100.0) and (e < 0)) or ((u == -100.0) and (e > 0)):
                    self._e_int = e_int_new
                self._mot.set_effort(u)
                if self._u_share is not None:
                    self._u_share.put(float((u / 100.0) * self._Vbatt))
                if self._logging and self._dataValues is not None:
                    if (self._dataValues.full() or self._timeValues.full() or
                        (self._effortValues is not None and self._effortValues.full())):
                        self._logging = False
                    else:
                        self._timeValues.put(t_rel)
                        self._dataValues.put(v_meas)
                        if self._effortValues is not None:
                            self._effortValues.put(u)
            yield self._state
