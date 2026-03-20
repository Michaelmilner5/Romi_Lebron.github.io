from pyb import ADC, Pin
from time import sleep_ms
import ujson
class LineSensorArray:
    def __init__(self, adc_pins, emitter_pin=None, weights=None, invert=False,
                 emit_active_high=True):
        self.adcs = [ADC(Pin(p)) if not isinstance(p, Pin) else ADC(p) for p in adc_pins]
        self.emit = None
        self.emit_active_high = emit_active_high
        if emitter_pin is not None:
            self.emit = Pin(emitter_pin, Pin.OUT_PP)
            self._emit_off()
        n = len(self.adcs)
        if weights is None:
            mid = (n - 1) / 2
            self.weights = [i - mid for i in range(n)]
        else:
            self.weights = weights
        self.invert = invert
        self.white = [4095] * n
        self.black = [0] * n
    def _emit_on(self):
        if self.emit is None:
            return
        if self.emit_active_high:
            self.emit.high()
        else:
            self.emit.low()
    def _emit_off(self):
        if self.emit is None:
            return
        if self.emit_active_high:
            self.emit.low()
        else:
            self.emit.high()
    def read_raw(self, oversample=1, emit=True, ambient_cancel=False):
        """
        Returns ADC readings.
        ambient_cancel subtracts (emitter off) from (emitter on).
        If your emitter isn't working or polarity is wrong, ambient_cancel will produce ~0.
        """
        n = len(self.adcs)
        if emit and (self.emit is not None) and ambient_cancel:
            self._emit_off()
            amb = [0] * n
            for _ in range(oversample):
                for i, adc in enumerate(self.adcs):
                    amb[i] += adc.read()
            amb = [a // oversample for a in amb]
            self._emit_on()
            lit = [0] * n
            for _ in range(oversample):
                for i, adc in enumerate(self.adcs):
                    lit[i] += adc.read()
            self._emit_off()
            lit = [l // oversample for l in lit]
            out = []
            for i in range(n):
                v = lit[i] - amb[i]
                if v < 0:
                    v = 0
                out.append(v)
            return out
        if emit:
            self._emit_on()
        sums = [0] * n
        for _ in range(oversample):
            for i, adc in enumerate(self.adcs):
                sums[i] += adc.read()
        if emit:
            self._emit_off()
        return [s // oversample for s in sums]
    def cal_white(self, samples=50, oversample=1, emit=True, ambient_cancel=False):
        """
        Calibrate WHITE using average.
        Put sensors on white surface and run.
        """
        acc = [0] * len(self.adcs)
        for _ in range(samples):
            vals = self.read_raw(oversample=oversample, emit=emit, ambient_cancel=ambient_cancel)
            for i, v in enumerate(vals):
                acc[i] += v
            sleep_ms(5)
        self.white = [a // samples for a in acc]
    def cal_black(self, samples=50, oversample=1, emit=True, ambient_cancel=False):
        """
        Calibrate BLACK using average.
        Put sensors on black line and run.
        """
        acc = [0] * len(self.adcs)
        for _ in range(samples):
            vals = self.read_raw(oversample=oversample, emit=emit, ambient_cancel=ambient_cancel)
            for i, v in enumerate(vals):
                acc[i] += v
            sleep_ms(5)
        self.black = [a // samples for a in acc]
    def read_norm(self, oversample=1, emit=True, ambient_cancel=False, den_min=50):
        raw = self.read_raw(oversample=oversample, emit=emit, ambient_cancel=ambient_cancel)
        norm = []
        for i, v in enumerate(raw):
            w = self.white[i]
            b = self.black[i]
            if b < w:
                w, b = b, w
            den = (b - w)
            if den < den_min:
                x = 0.0
            else:
                x = (v - w) / den
            if self.invert:
                x = 1.0 - x
            if x < 0.0:
                x = 0.0
            if x > 1.0:
                x = 1.0
            norm.append(x)
        return norm
    def centroid(self, norm_vals, eps=1e-6):
        num = 0.0
        den = 0.0
        for w, x in zip(self.weights, norm_vals):
            num += w * x
            den += x
        if den < eps:
            return 0.0
        return num / den
    def confidence(self, norm_vals):
        return max(norm_vals), sum(norm_vals)
    def save_calib(self, filename="line_calib.json"):
        """
        Save calibration arrays (white/black) to a JSON file.
        Call this after you run cal_white() and cal_black().
        """
        if (self.white is None) or (self.black is None):
            raise ValueError("Calibration missing: run cal_white() and cal_black() first")
        data = {
            "white": [int(x) for x in self.white],
            "black": [int(x) for x in self.black],
        }
        with open(filename, "w") as f:
            ujson.dump(data, f)
        return True
    def load_calib(self, filename="line_calib.json"):
        """
        Load calibration from a JSON file and apply it to this object.
        """
        with open(filename, "r") as f:
            data = ujson.load(f)
        white = data.get("white", None)
        black = data.get("black", None)
        if (white is None) or (black is None):
            raise ValueError("Calibration file missing keys 'white'/'black'")
        self.white = [int(x) for x in white]
        self.black = [int(x) for x in black]
        return True
