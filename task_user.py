from pyb import USB_VCP
from task_share import Share, Queue
import micropython
import gc
from task_user_cmds import UserCmds
S0_INIT = micropython.const(0)
S1_CMD  = micropython.const(1)
S2_GET  = micropython.const(2)
S3_COL  = micropython.const(3)
S4_DIS  = micropython.const(4)
class task_user(UserCmds):
    def __init__(self,
                 leftMotorGo, rightMotorGo,
                 dataL, timeL, effortL,
                 dataR, timeR, effortR,
                 Kp, Ki, vBase, lfEnable, estStreamEnable,
                 line_sensors,
                 imu=None, psi_meas=None, psiDot_meas=None,
                 sL_meas=None, sR_meas=None,
                 s_hat=None, psi_hat=None, wL_hat=None, wR_hat=None,
                 imuCalReq=None, uart2=None,
                 courseEnable=None, bumpEvent=None,
                 velSetpoint_L=None, velSetpoint_R=None):
        self._state = S0_INIT
        self._leftMotorGo  = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._Kp = Kp
        self._Ki = Ki
        self._vBase    = vBase
        self._lfEnable = lfEnable
        self._estStreamEnable = estStreamEnable
        self._sens     = line_sensors
        self._dataL = dataL; self._timeL = timeL; self._effL = effortL
        self._dataR = dataR; self._timeR = timeR; self._effR = effortR
        self._imu = imu
        self._psi_meas = psi_meas; self._psiDot_meas = psiDot_meas
        self._sL_meas = sL_meas; self._sR_meas = sR_meas
        self._s_hat = s_hat; self._psi_hat = psi_hat
        self._wL_hat = wL_hat; self._wR_hat = wR_hat
        self._imuCalReq = imuCalReq
        self._ser = USB_VCP()
        self._uart = uart2
        self._courseEnable = courseEnable
        self._bumpEvent = bumpEvent
        self._velL = velSetpoint_L; self._velR = velSetpoint_R
        self._buf = None; self._mode = None; self._num_str = ""
        print("User Task object instantiated")
    def run(self):
        while True:
            if self._state == S0_INIT:
                self._print_help()
                self._prompt()
                self._state = S1_CMD
            elif self._state == S1_CMD:
                ch = self._getc()
                if ch is None:
                    yield self._state
                    continue
                self._handle_cmd(ch)
            elif self._state == S2_GET:
                ch = self._getc()
                if ch is None:
                    yield self._state
                    continue
                if ch in (b'\r', b'\n'):
                    self._write_ui(b"\r\n")
                    if self._mode == 'k':
                        self._parse_and_set_gains()
                    elif self._mode == 'v':
                        self._parse_and_set_vbase()
                    self._prompt()
                    self._state = S1_CMD
                else:
                    try:
                        self._num_str += ch.decode()
                        self._write_ui(ch)
                    except Exception:
                        pass
            elif self._state == S3_COL:
                if (not self._dataL.full()) and (not self._dataR.full()):
                    yield self._state
                    continue
                self._leftMotorGo.put(False)
                self._rightMotorGo.put(False)
                self._write_ui(b"---DATA_BEGIN---\r\n")
                self._write_ui(("Kp:{} Ki:{} vBase:{}\r\n".format(
                    self._Kp.get(), self._Ki.get(), self._vBase.get())).encode())
                self._write_ui(b"t_us,vL,uL,vR,uR\r\n")
                self._state = S4_DIS
            elif self._state == S4_DIS:
                if (self._timeL.any() and self._dataL.any() and self._effL.any()
                        and self._dataR.any() and self._effR.any()):
                    self._write_ui("{},{},{},{},{}\r\n".format(
                        self._timeL.get(), self._dataL.get(), self._effL.get(),
                        self._dataR.get(), self._effR.get()).encode())
                else:
                    self._write_ui(b"---DATA_END---\r\n")
                    gc.collect()
                    self._prompt()
                    self._state = S1_CMD
            yield self._state
