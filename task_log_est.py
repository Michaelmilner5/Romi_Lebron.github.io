# ============================ task_log_est.py ============================
# Logs observer estimates over time (non-blocking, uses Queues).
# Signals:
#   s_hat (m), psi_hat (rad), wL_hat (rad/s), wR_hat (rad/s)
# Also computes x_hat, y_hat via dead-reckoning:
#   ds_hat = s_hat - s_hat_prev
#   x_hat += ds_hat*cos(psi_hat)
#   y_hat += ds_hat*sin(psi_hat)

from utime import ticks_us
import micropython
import math

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_log_est:
    def __init__(self,
                 logEnableShare,
                 s_hat, psi_hat, wL_hat, wR_hat,
                 q_t_us, q_s_hat, q_psi_hat, q_wL_hat, q_wR_hat,
                 q_x_hat=None, q_y_hat=None):

        self._state = S0_INIT

        self._en = logEnableShare
        self._s  = s_hat
        self._psi = psi_hat
        self._wL = wL_hat
        self._wR = wR_hat

        self._qt = q_t_us
        self._qs = q_s_hat
        self._qpsi = q_psi_hat
        self._qwL = q_wL_hat
        self._qwR = q_wR_hat
        self._qx = q_x_hat
        self._qy = q_y_hat

        self._s_prev = 0.0
        self._x = 0.0
        self._y = 0.0

        self._was_enabled = False

        print("Observer Log Task instantiated")

    def _reset(self):
        self._s_prev = float(self._s.get())
        self._x = 0.0
        self._y = 0.0

    def run(self):
        while True:

            if self._state == S0_INIT:
                self._reset()
                self._was_enabled = False
                self._state = S1_RUN

            elif self._state == S1_RUN:
                enabled = bool(self._en.get())

                if enabled and (not self._was_enabled):
                    self._reset()

                self._was_enabled = enabled

                if not enabled:
                    yield self._state
                    continue

                if self._qt.full() or self._qs.full() or self._qpsi.full() or \
                   self._qwL.full() or self._qwR.full() or \
                   ((self._qx is not None) and self._qx.full()) or \
                   ((self._qy is not None) and self._qy.full()):
                    self._en.put(False)
                    yield self._state
                    continue

                t = ticks_us()

                s   = float(self._s.get())
                psi = float(self._psi.get())
                wL  = float(self._wL.get())
                wR  = float(self._wR.get())

                ds = s - self._s_prev
                self._s_prev = s

                self._x += ds * math.cos(psi)
                self._y += ds * math.sin(psi)

                self._qt.put(t)
                self._qs.put(s)
                self._qpsi.put(psi)
                self._qwL.put(wL)
                self._qwR.put(wR)

                if self._qx is not None:
                    self._qx.put(self._x)
                if self._qy is not None:
                    self._qy.put(self._y)

            yield self._state
