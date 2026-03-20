import micropython
from utime import ticks_us, ticks_diff
import math
S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)
DEG2RAD = math.pi / 180.0
class task_imu:
    def __init__(self, imu, psi_share, psiDot_share,
                 zero_on_start=True,
                 unwrap=True,
                 calib_file="bno_calib.bin",
                 try_load_calib=True,
                 yawrate_limit_rad_s=20.0,
                 cal_req=None,
                 yawrate_spike_dps=2000.0):
        self._state = S0_INIT
        self.imu = imu
        self.psi = psi_share
        self.psiDot = psiDot_share
        self._zero_on_start = bool(zero_on_start)
        self._unwrap = bool(unwrap)
        self._calib_file = calib_file
        self._try_load_calib = bool(try_load_calib)
        self._cal_req = cal_req
        self._cal_state = 0
        self._cal_printed_start = False
        self._cal_printed_wait  = False
        self._yaw_lim = float(yawrate_limit_rad_s)
        self._yaw_spike_dps = float(yawrate_spike_dps)
        self._psi0 = 0.0
        self._psi_prev = 0.0
        self._psi_cont = 0.0
        self._r_filt = 0.0
        self._r_alpha = 0.25
        self._t_prev = ticks_us()
        self.psi.put(0.0)
        self.psiDot.put(0.0)
    def _print_coeffs(self, label, data_bytes):
        """Print calibration blob as a 22-byte integer list (for debug/demo)."""
        try:
            if data_bytes is None:
                print("{}: <None>".format(label))
                return
            print("{} ({} bytes):".format(label, len(data_bytes)))
            print(list(data_bytes))
        except Exception as e:
            print("WARNING: could not print coeffs:", e)
    def _load_calib_if_available(self):
        try:
            with open(self._calib_file, "rb") as f:
                data = f.read()
            if data and len(data) == 22:
                self.imu.write_calib_coeffs(data)
                self._print_coeffs("CAL COEFFS (from file '{}')".format(self._calib_file), data)
                return True
        except Exception:
            pass
        return False
    def save_calib_to_file(self):
        try:
            data = self.imu.read_calib_coeffs()
            with open(self._calib_file, "wb") as f:
                f.write(data)
            return True
        except Exception:
            return False
    def _unwrap_deg_to_rad(self, heading_deg):
        """
        Robust unwrap:
        - Convert heading to rad (wrapped)
        - Compute delta to previous
        - Wrap delta into [-pi, pi]
        - Integrate delta into continuous angle
        """
        psi = heading_deg * DEG2RAD
        if not self._unwrap:
            return psi
        d = psi - self._psi_prev
        if d > math.pi:
            d -= 2.0 * math.pi
        elif d < -math.pi:
            d += 2.0 * math.pi
        self._psi_cont += d
        self._psi_prev = psi
        return self._psi_cont
    def _cal_request_active(self):
        try:
            return (self._cal_req is not None) and bool(self._cal_req.get())
        except Exception:
            return False
    def _finish_and_rezero(self):
        """Clear request, reset unwrap/zero reference so psi reads 0 immediately."""
        try:
            if self._cal_req is not None:
                self._cal_req.put(False)
        except Exception:
            pass
        try:
            h0 = self.imu.heading_deg()
        except Exception:
            h0 = 0.0
        self._psi_prev = h0 * DEG2RAD
        self._psi_cont = 0.0
        print("REZERO: h0={:.3f} deg, psi_prev={:.4f} rad, psi_cont={:.4f}".format(h0, self._psi_prev, self._psi_cont))
        self._psi0 = 0.0
        self._r_filt = 0.0
    def run(self):
        while True:
            if self._state == S0_INIT:
                try:
                    h0 = self.imu.heading_deg()
                except Exception:
                    h0 = 0.0
                self._psi_prev = h0 * DEG2RAD
                self._psi_cont = 0.0
                self._psi0 = 0.0
                self._t_prev = ticks_us()
                self._r_filt = 0.0
                self._cal_state = 0
                self._cal_printed_start = False
                self._cal_printed_wait  = False
                self._pending_rezero = 0
                self._state = S1_RUN
            elif self._state == S1_RUN:
                if self._cal_request_active():
                    if not self._cal_printed_start:
                        print("IMU CAL: requested -> running flowchart")
                        self._cal_printed_start = True
                        self._cal_printed_wait  = False
                    if self._cal_state == 0:
                        self._cal_state = 1
                    if self._cal_state == 1:
                        print("IMU CAL: checking for file '{}'...".format(self._calib_file))
                        loaded = False
                        if self._try_load_calib:
                            loaded = self._load_calib_if_available()
                        if loaded:
                            print("Calibration loaded into IMU.")
                            self._cal_state = 4
                        else:
                            if not self._cal_printed_wait:
                                print("No calibration file found. Move robot to calibrate (GYR+ACC).")
                                self._cal_printed_wait = True
                            self._cal_state = 2
                    if self._cal_state == 2:
                        try:
                            sys_, gyr, acc, mag = self.imu.calib_status()
                        except Exception:
                            yield self._state
                            continue
                        if not ((gyr == 3) and (acc == 3)):
                            self.psi.put(0.0)
                            self.psiDot.put(0.0)
                            yield self._state
                            continue
                        print("Calibration complete! Saving to '{}'...".format(self._calib_file))
                        self._cal_state = 3
                    if self._cal_state == 3:
                        ok = self.save_calib_to_file()
                        if ok:
                            print("Calibration saved.")
                            try:
                                coeffs = self.imu.read_calib_coeffs()
                                self._print_coeffs("CAL COEFFS (from IMU after save)", coeffs)
                            except Exception as e:
                                print("WARNING: could not read coeffs after save:", e)
                        else:
                            print("WARNING: calibration save failed.")
                        self._cal_state = 4
                    if self._cal_state == 4:
                        self._finish_and_rezero()
                        self._pending_rezero = 5
                        print("IMU CAL: done (request cleared, heading rezeroed).")
                        self._cal_state = 0
                        self._cal_printed_start = False
                        self._cal_printed_wait  = False
                    yield self._state
                    continue
                t_now = ticks_us()
                dt = ticks_diff(t_now, self._t_prev) * 1e-6
                self._t_prev = t_now
                if dt <= 0.0:
                    dt = 0.02
                try:
                    h_deg = self.imu.heading_deg()
                    r_dps = self.imu.yaw_rate_dps()
                except Exception:
                    yield self._state
                    continue
                if self._pending_rezero > 0:
                    self._psi_prev = h_deg * DEG2RAD
                    self._psi_cont = 0.0
                    self._psi0 = 0.0
                    self._pending_rezero -= 1
                    self.psi.put(0.0)
                    self.psiDot.put(0.0)
                    yield self._state
                    continue
                psi_rad = self._unwrap_deg_to_rad(h_deg) - self._psi0
                if abs(r_dps) > self._yaw_spike_dps:
                    r_rad_s = self._r_filt
                else:
                    r_rad_s = r_dps * DEG2RAD
                    if abs(r_rad_s) > self._yaw_lim:
                        r_rad_s = self._r_filt
                a = self._r_alpha
                self._r_filt = (1.0 - a) * self._r_filt + a * r_rad_s
                self.psi.put(float(psi_rad))
                self.psiDot.put(float(self._r_filt))
            yield self._state
