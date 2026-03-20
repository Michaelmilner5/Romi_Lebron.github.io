from utime import sleep_ms
import math
class BNO055:
    ADDR = 0x28
    REG_CHIP_ID      = 0x00
    REG_OPR_MODE     = 0x3D
    REG_PWR_MODE     = 0x3E
    REG_UNIT_SEL     = 0x3B
    REG_PAGE_ID      = 0x07
    REG_CALIB_STAT   = 0x35
    REG_EUL_H_LSB    = 0x1A
    REG_GYR_X_LSB    = 0x14
    REG_GYR_Z_LSB    = 0x18
    REG_CALIB_START  = 0x55
    CALIB_LEN        = 22
    MODE_CONFIG = 0x00
    MODE_IMU    = 0x08
    def __init__(self, i2c, addr=ADDR):
        self.i2c = i2c
        self.addr = addr
        self._mode = None
        sleep_ms(700)
        self._write_u8(self.REG_PAGE_ID, 0x00)
        sleep_ms(10)
        chip = self._read_u8(self.REG_CHIP_ID)
        if chip != 0xA0:
            raise RuntimeError("BNO055 not found (chip id = 0x{:02X})".format(chip))
        self.set_mode(self.MODE_CONFIG)
        self._write_u8(self.REG_PWR_MODE, 0x00)
        sleep_ms(10)
        self._write_u8(self.REG_UNIT_SEL, 0x00)
        self.set_mode(self.MODE_IMU)
    def _write_u8(self, reg, val):
        self.i2c.mem_write(bytes([val & 0xFF]), self.addr, reg)
    def _read_u8(self, reg):
        return int.from_bytes(self.i2c.mem_read(1, self.addr, reg), "little")
    def _read_bytes(self, reg, n):
        return self.i2c.mem_read(n, self.addr, reg)
    @staticmethod
    def _s16(lsb, msb):
        v = (msb << 8) | lsb
        return v - 65536 if (v & 0x8000) else v
    def _read_i16(self, reg):
        b = self._read_bytes(reg, 2)
        return int.from_bytes(b, "little", True)
    def get_mode(self):
        return self._read_u8(self.REG_OPR_MODE) & 0x0F
    def set_mode(self, mode):
        mode = mode & 0x0F
        if self._mode is None:
            self._write_u8(self.REG_OPR_MODE, self.MODE_CONFIG)
            sleep_ms(25)
            self._write_u8(self.REG_OPR_MODE, mode)
            sleep_ms(25)
            self._mode = mode
            return
        if mode == self._mode:
            return
        self._write_u8(self.REG_OPR_MODE, self.MODE_CONFIG)
        sleep_ms(25)
        self._write_u8(self.REG_OPR_MODE, mode)
        sleep_ms(25)
        self._mode = mode
    def calib_status(self):
        s = self._read_u8(self.REG_CALIB_STAT)
        sys_ = (s >> 6) & 0x03
        gyr  = (s >> 4) & 0x03
        acc  = (s >> 2) & 0x03
        mag  = (s >> 0) & 0x03
        return (sys_, gyr, acc, mag)
    def read_calib_coeffs(self):
        prev = self._mode if self._mode is not None else self.get_mode()
        self.set_mode(self.MODE_CONFIG)
        sleep_ms(10)
        data = bytes(self._read_bytes(self.REG_CALIB_START, self.CALIB_LEN))
        self.set_mode(prev)
        return data
    def write_calib_coeffs(self, data: bytes):
        if (data is None) or (len(data) != self.CALIB_LEN):
            raise ValueError("calib coeffs must be exactly 22 bytes")
        prev = self._mode if self._mode is not None else self.get_mode()
        self.set_mode(self.MODE_CONFIG)
        sleep_ms(10)
        self.i2c.mem_write(data, self.addr, self.REG_CALIB_START)
        sleep_ms(10)
        self.set_mode(prev)
    def read_euler_deg(self):
        b = self._read_bytes(self.REG_EUL_H_LSB, 6)
        h = self._s16(b[0], b[1]) / 16.0
        r = self._s16(b[2], b[3]) / 16.0
        p = self._s16(b[4], b[5]) / 16.0
        return (h, r, p)
    def heading_deg(self):
        raw = self._read_i16(self.REG_EUL_H_LSB)
        return -(raw / 16.0)
    def heading_rad(self):
        return math.radians(self.heading_deg())
    def read_gyro_dps(self):
        b = self._read_bytes(self.REG_GYR_X_LSB, 6)
        gx = self._s16(b[0], b[1]) / 16.0
        gy = self._s16(b[2], b[3]) / 16.0
        gz = self._s16(b[4], b[5]) / 16.0
        return (gx, gy, gz)
    def yaw_rate_dps(self):
        raw = self._read_i16(self.REG_GYR_Z_LSB)
        return raw / 16.0
    def yaw_rate_rads(self):
        return math.radians(self.yaw_rate_dps())
