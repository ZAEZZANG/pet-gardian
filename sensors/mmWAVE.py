#호흡 검출 기능만 담당하는 최소 코어 모듈#

import serial
import numpy as np
from scipy.signal import detrend
from scipy.fft import rfft, rfftfreq

class mmwave:
    def __init__(self, port="/dev/ttyAMA10", baud=115200, fs=10):
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.fs = fs
        self.breath_signal = []
        self.motion_signal = []

    def read(self):
        header = self.ser.read(2)
        if header != b'\x53\x59':
            return None, None

        frame_type = self.ser.read(1)[0]
        length = self.ser.read(1)[0]
        payload = self.ser.read(length)
        self.ser.read(2)

        motion = None
        breath = None

        if frame_type == 0x85:
            motion = payload[-1]
            self.motion_signal.append(motion)

        elif frame_type in [0x83, 0x89, 0x8A]:
            breath = payload[-1]
            self.breath_signal.append(breath)

        if len(self.breath_signal) > self.fs * 10:
            self.breath_signal.pop(0)
            self.motion_signal.pop(0)

        return breath, motion

    def compute_breathing(self):
        if len(self.breath_signal) < self.fs * 5:
            return None
        
        cleaned = detrend(np.array(self.breath_signal))
        freqs = rfftfreq(len(cleaned), 1/self.fs)
        spectrum = np.abs(rfft(cleaned))

        mask = (freqs >= 0.1) & (freqs <= 1.0)
        if not np.any(mask):
            return None

        peak = freqs[mask][np.argmax(spectrum[mask])]
        return round(peak * 60, 1)

