import micropython
from utime import ticks_us
from pyb import UART
import math
S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)
try:
    from ulab import numpy as np
    _HAS_ULAB = True
except Exception:
    _HAS_ULAB = False
_HAS_MATS = False
try:
    import observer_matrices as om
    _HAS_MATS = True
except Exception:
    om = None
    _HAS_MATS = False
class task_observer:
    def __init__(self,
                 uL_volts, uR_volts,
                 sL_meas, sR_meas, psi_meas, psiDot_meas,
                 s_hat, psi_hat, wL_hat, wR_hat,
                 Ts_ms=20,
                 debug_print=True,
                 estStreamEnable=None,
                 stream_hz=20,
                 uart2=None):
        self._state = S0_INIT
        self.uL = uL_volts
        self.uR = uR_volts
        self.sL = sL_meas
        self.sR = sR_meas
        self.psi = psi_meas
        self.psiDot = psiDot_meas
        self.s_hat = s_hat
        self.psi_hat = psi_hat
        self.wL_hat = wL_hat
        self.wR_hat = wR_hat
        self.Ts = float(Ts_ms) * 1e-3
        self._xhat = [0.0, 0.0, 0.0, 0.0]
        self._use_mats = False
        self._use_bd_full = False
        self._Ad = None
        self._Bd = None
        self._C  = None
        self._Ld = None
        self._debug_print = bool(debug_print)
        self._printed_startup = False
        self._estStreamEnable = estStreamEnable
        self._x_path = 0.0
        self._y_path = 0.0
        self._s_prev_stream = 0.0
        self._streaming_prev = False
        self._stream_hz = int(stream_hz)
        self._stream_period_us = 0 if self._stream_hz <= 0 else int(1_000_000 / self._stream_hz)
        self._last_stream_us = 0
        self._uart = uart2
        print("Observer Task object instantiated")
    def _load_matrices(self):
        if (not _HAS_MATS) or (not _HAS_ULAB):
            return False
        if hasattr(om, "Ts"):
            try:
                self.Ts = float(om.Ts)
            except Exception:
                pass
        def to_np(x):
            return x if hasattr(x, "shape") else np.array(x)
        Aname = "Ad" if hasattr(om, "Ad") else ("A" if hasattr(om, "A") else None)
        Bname = "Bd" if hasattr(om, "Bd") else ("B" if hasattr(om, "B") else None)
        if (Aname is None) or (Bname is None):
            return False
        try:
            self._Ad = to_np(getattr(om, Aname))
            self._Bd = to_np(getattr(om, Bname))
        except Exception:
            return False
        try:
            nx = self._Ad.shape[0]
            if self._Ad.shape[1] != nx:
                return False
        except Exception:
            return False
        try:
            bd_cols = self._Bd.shape[1]
        except Exception:
            return False
        if bd_cols == 6:
            self._use_bd_full = True
            self._C = None
            self._Ld = None
            return True
        elif bd_cols == 2:
            if (not hasattr(om, "C")) or (not hasattr(om, "Ld")):
                return False
            try:
                self._C  = to_np(om.C)
                self._Ld = to_np(om.Ld)
            except Exception:
                return False
            self._use_bd_full = False
            return True
        else:
            return False
    def _startup_print(self):
        if self._printed_startup or (not self._debug_print):
            return
        self._printed_startup = True
        if not _HAS_ULAB:
            print("Observer: ulab numpy not available -> fallback mode")
            return
        if not _HAS_MATS:
            print("Observer: observer_matrices.py not found -> fallback mode")
            return
        if self._use_mats:
            try:
                print("Observer: matrices loaded. Ad={}x{}, Bd={}x{}"
                      .format(self._Ad.shape[0], self._Ad.shape[1],
                              self._Bd.shape[0], self._Bd.shape[1]))
                print("Observer: Bd form = {}".format("4x6 concatenated" if self._use_bd_full else "innovation"))
                print("Observer: expected u* order = [uL,uR,sL,sR,psi,psiDot]")
            except Exception:
                print("Observer: matrices loaded.")
        else:
            print("Observer: matrices import failed / bad dimensions -> fallback mode")
    def _stream_active(self):
        return False if self._estStreamEnable is None else bool(self._estStreamEnable.get())
    def _reset_stream_path(self):
        self._x_path = 0.0
        self._y_path = 0.0
        self._s_prev_stream = float(self._xhat[0])
        self._last_stream_us = 0
        self._write_uart(b"---EST_BEGIN---\r\n")
        self._write_uart(b"t_us,s_hat_m,psi_hat_rad,wL_hat_radps,wR_hat_radps,x_hat_m,y_hat_m\r\n")
    def _write_uart(self, b):
        if self._uart is None:
            return
        try:
            self._uart.write(b)
        except Exception:
            pass
    def _maybe_stream_sample(self):
        if self._uart is None:
            return
        now_us = ticks_us()
        if self._stream_period_us > 0:
            if self._last_stream_us != 0 and (now_us - self._last_stream_us) < self._stream_period_us:
                return
        s   = float(self._xhat[0])
        psi = float(self._xhat[1])
        wL  = float(self._xhat[2])
        wR  = float(self._xhat[3])
        ds = s - self._s_prev_stream
        self._s_prev_stream = s
        self._x_path += ds * math.cos(psi)
        self._y_path += ds * math.sin(psi)
        line = "{},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}\r\n".format(
            now_us, s, psi, wL, wR, self._x_path, self._y_path
        )
        self._write_uart(line.encode())
        self._last_stream_us = now_us
    def run(self):
        while True:
            if self._state == S0_INIT:
                self._use_mats = self._load_matrices()
                self._startup_print()
                sL0  = float(self.sL.get())
                sR0  = float(self.sR.get())
                psi0 = float(self.psi.get())
                s0 = 0.5 * (sL0 + sR0)
                self._xhat = [s0, psi0, 0.0, 0.0]
                self._sL_prev = sL0
                self._sR_prev = sR0
                self._reset_stream_path()
                self._streaming_prev = False
                self._state = S1_RUN
            elif self._state == S1_RUN:
                uL = float(self.uL.get())
                uR = float(self.uR.get())
                sL_now = float(self.sL.get())
                sR_now = float(self.sR.get())
                sL_m = (sL_now - self._sL_prev) / self.Ts
                sR_m = (sR_now - self._sR_prev) / self.Ts
                self._sL_prev = sL_now
                self._sR_prev = sR_now
                psi_m  = float(self.psi.get())
                psid_m = float(self.psiDot.get())
                if self._use_mats:
                    x = np.array(self._xhat)
                    if self._use_bd_full:
                        u_star = np.array([uL, uR, sL_m, sR_m, psi_m, psid_m])
                        x_next = np.dot(self._Ad, x) + np.dot(self._Bd, u_star)
                    else:
                        u_vec = np.array([uL, uR])
                        y_vec = np.array([sL_m, sR_m, psi_m, psid_m])
                        y_hat = np.dot(self._C, x)
                        innov = y_vec - y_hat
                        x_next = np.dot(self._Ad, x) + np.dot(self._Bd, u_vec) + np.dot(self._Ld, innov)
                    self._xhat = [float(x_next[i]) for i in range(4)]
                else:
                    s_est = 0.5 * (sL_now + sR_now)
                    self._xhat = [s_est, psi_m, sL_m, sR_m]
                self.s_hat.put(float(self._xhat[0]))
                self.psi_hat.put(float(self._xhat[1]))
                self.wL_hat.put(float(self._xhat[2]))
                self.wR_hat.put(float(self._xhat[3]))
                active = self._stream_active()
                if active and (not self._streaming_prev):
                    self._reset_stream_path()
                if active:
                    self._maybe_stream_sample()
                self._streaming_prev = active
            yield self._state
