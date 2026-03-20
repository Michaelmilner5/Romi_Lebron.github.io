from pyb import USB_VCP
import gc
UI_prompt = ">: "
def _get_help_menu():
    return (
        "\r\n+----------------------+\r\n"
        "| h:help  x:stop       |\r\n"
        "| k:gains v:speed      |\r\n"
        "| w:white b:black p:sv |\r\n"
        "| s:sens  f:lf  r:crse |\r\n"
        "| g:log   1:L   2:R    |\r\n"
        "| i:imu   d:imuD c:cal |\r\n"
        "| u:saveC o:obs        |\r\n"
        "+----------------------+\r\n"
    )
class UserCmds:
    def _write_ui(self, b):
        try:
            self._ser.write(b)
        except Exception:
            pass
    def _prompt(self):
        self._write_ui(UI_prompt.encode())
    def _print_help(self):
        self._write_ui(_get_help_menu().encode())
    def _getc(self):
        if self._buf is not None:
            ch = self._buf
            self._buf = None
            return ch
        if self._ser.any():
            try:
                return self._ser.read(1)
            except Exception:
                return None
        return None
    def _get_number(self):
        self._num_str = ""
        self._state = 2
    def _parse_and_set_gains(self):
        try:
            vals = self._num_str.strip().split()
            if len(vals) != 2:
                raise ValueError
            self._Kp.put(float(vals[0]))
            self._Ki.put(float(vals[1]))
            self._write_ui(b"Gains set.\r\n")
        except Exception:
            self._write_ui(b"Invalid. Enter: <Kp> <Ki>\r\n")
    def _parse_and_set_vbase(self):
        try:
            self._vBase.put(float(self._num_str.strip()))
            self._write_ui(b"vBase set.\r\n")
        except Exception:
            self._write_ui(b"Invalid.\r\n")
    def _estop(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(False)
        self._rightMotorGo.put(False)
        self._estStreamEnable.put(False)
        if self._courseEnable is not None:
            self._courseEnable.put(False)
        if self._velL is not None:
            self._velL.put(0.0)
        if self._velR is not None:
            self._velR.put(0.0)
        self._write_ui(b"STOPPED.\r\n")
        self._prompt()
    def _print_imu_once(self):
        if self._psi_meas is None:
            self._write_ui(b"IMU shares not connected.\r\n")
            return
        self._write_ui(("psi:{:.4f} psiDot:{:.4f}\r\n".format(
            self._psi_meas.get(), self._psiDot_meas.get())).encode())
        if self._imu is not None:
            try:
                c = self._imu.calib_status()
                self._write_ui(("cal sys,g,a,m={},{},{},{}\r\n".format(
                    int(c[0]),int(c[1]),int(c[2]),int(c[3]))).encode())
            except Exception as e:
                self._write_ui(("cal err:{}\r\n".format(e)).encode())
    def _print_imu_direct_once(self):
        if self._imu is None:
            self._write_ui(b"No IMU.\r\n")
            return
        try:
            self._write_ui(("h:{} r:{}\r\n".format(
                self._imu.heading_deg(), self._imu.yaw_rate_dps())).encode())
        except Exception as e:
            self._write_ui(("err:{}\r\n".format(e)).encode())
    def _print_observer_once(self):
        if self._s_hat is None:
            self._write_ui(b"Observer not connected.\r\n")
            return
        self._write_ui(("s:{:.4f} psi:{:.4f} wL:{:.4f} wR:{:.4f}\r\n".format(
            self._s_hat.get(), self._psi_hat.get(),
            self._wL_hat.get(), self._wR_hat.get())).encode())
    def _print_sensors_once(self):
        try:
            norm = self._sens.read_norm(oversample=2, den_min=10)
            pos  = self._sens.centroid(norm)
            mx, sm = self._sens.confidence(norm)
            self._write_ui(("pos:{:.3f} max:{:.3f} sum:{:.3f}\r\n".format(pos,mx,sm)).encode())
        except Exception as e:
            self._write_ui(("err:{}\r\n".format(e)).encode())
    def _clear_queues(self):
        for q in (self._dataL, self._timeL, self._effL,
                  self._dataR, self._timeR, self._effR):
            try:
                while q.any():
                    q.get()
            except Exception:
                pass
    def _handle_cmd(self, ch):
        if ch in (b'x', b'X'):
            self._estop()
        elif ch in (b'h', b'H'):
            self._print_help()
            self._prompt()
        elif ch in (b'k', b'K'):
            self._write_ui(b"Enter Kp Ki: ")
            self._mode = 'k'
            self._get_number()
        elif ch in (b'v', b'V'):
            self._write_ui(b"Enter vBase: ")
            self._mode = 'v'
            self._get_number()
        elif ch in (b'w', b'W'):
            try:
                self._sens.cal_white(samples=200, oversample=4, emit=True)
                self._write_ui(b"WHITE done.\r\n")
            except Exception as e:
                self._write_ui(("err:{}\r\n".format(e)).encode())
            self._prompt()
        elif ch in (b'b', b'B'):
            try:
                self._sens.cal_black(samples=200, oversample=4, emit=True)
                self._write_ui(b"BLACK done.\r\n")
            except Exception as e:
                self._write_ui(("err:{}\r\n".format(e)).encode())
            self._prompt()
        elif ch in (b'p', b'P'):
            try:
                self._sens.save_calib("line_calib.json")
                self._write_ui(b"Calib saved.\r\n")
            except Exception as e:
                self._write_ui(("err:{}\r\n".format(e)).encode())
            self._prompt()
        elif ch in (b's', b'S'):
            self._print_sensors_once()
            self._prompt()
        elif ch in (b'i', b'I'):
            self._print_imu_once()
            self._prompt()
        elif ch in (b'd', b'D'):
            self._print_imu_direct_once()
            self._prompt()
        elif ch in (b'c', b'C'):
            if self._imuCalReq is not None:
                self._imuCalReq.put(True)
                self._write_ui(b"IMU cal requested.\r\n")
            self._prompt()
        elif ch in (b'o', b'O'):
            self._print_observer_once()
            self._prompt()
        elif ch in (b'r', b'R'):
            if self._courseEnable is not None:
                self._estStreamEnable.put(False)
                self._lfEnable.put(False)
                if self._velL is not None: self._velL.put(0.0)
                if self._velR is not None: self._velR.put(0.0)
                self._leftMotorGo.put(True)
                self._rightMotorGo.put(True)
                self._courseEnable.put(True)
                self._write_ui(b"COURSE ON\r\n")
            self._prompt()
        elif ch in (b'f', b'F'):
            if self._courseEnable is not None:
                self._courseEnable.put(False)
            self._lfEnable.put(True)
            self._leftMotorGo.put(True)
            self._rightMotorGo.put(True)
            self._estStreamEnable.put(True)
            self._write_ui(b"LF+EST ON\r\n")
            self._prompt()
        elif ch in (b'1',):
            if self._courseEnable is not None: self._courseEnable.put(False)
            self._lfEnable.put(False)
            if self._velL is not None: self._velL.put(200.0)
            if self._velR is not None: self._velR.put(0.0)
            self._leftMotorGo.put(True)
            self._rightMotorGo.put(False)
            self._prompt()
        elif ch in (b'2',):
            if self._courseEnable is not None: self._courseEnable.put(False)
            self._lfEnable.put(False)
            if self._velL is not None: self._velL.put(0.0)
            if self._velR is not None: self._velR.put(200.0)
            self._leftMotorGo.put(False)
            self._rightMotorGo.put(True)
            self._prompt()
        elif ch in (b'g', b'G'):
            if self._courseEnable is not None: self._courseEnable.put(False)
            self._clear_queues()
            self._lfEnable.put(False)
            self._leftMotorGo.put(True)
            self._rightMotorGo.put(True)
            self._state = 3
        elif ch in (b'u', b'U'):
            if self._imu is None:
                self._write_ui(b"No IMU.\r\n")
            else:
                try:
                    data = self._imu.read_calib_coeffs()
                    with open("bno_calib.bin", "wb") as f:
                        f.write(data)
                    self._write_ui(b"Calib saved.\r\n")
                except Exception as e:
                    self._write_ui(("err:{}\r\n".format(e)).encode())
            self._prompt()
