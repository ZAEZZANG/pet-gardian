#센서에서 들어온 데이터를 
#의미있는 생체 지표로 변환하는 알고리즘 모듈
#mmWave 신호는 FFT 및 로우패스 필터를 이용해 호흡수(BPM)를 추정, RMS 기반 분석으로 활동 강도 계산
#변동성과 움직임 특성을 기준으로 수면 여부를 판별하는 알고리즘
import numpy as np
from scipy.signal import detrend, butter, filtfilt
from scipy.fft import rfft, rfftfreq


# ======================================
# 🔹 1) Moving Average Filter (평활화)
# ======================================
def smooth(data, window_size=5):
    if len(data) < window_size:
        return data
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')


# ======================================
# 🔹 2) Butterworth Low-Pass Filter
#    - Breathing band: < 1.5 Hz recommended
# ======================================
def butter_lowpass_filter(data, cutoff=1.5, fs=10, order=4):
    if len(data) < fs * 2:
        return data
    b, a = butter(order, cutoff / (0.5 * fs), btype='low')
    return filtfilt(b, a, data)


# ======================================
# 🔹 3) FFT 기반 호흡수 계산
# ======================================
def estimate_breathing_rate(signal, fs=10):
    if len(signal) < fs * 5:
        return None  # 최소 5초 필요
    
    cleaned = detrend(signal)
    cleaned = butter_lowpass_filter(cleaned, fs=fs)

    freqs = rfftfreq(len(cleaned), d=1/fs)
    spectrum = np.abs(rfft(cleaned))

    # mmWave breathing range → 0.1–1.2 Hz (6–72 BPM)
    mask = (freqs >= 0.1) & (freqs <= 1.2)

    if not np.any(mask):
        return None

    peak_freq = freqs[mask][np.argmax(spectrum[mask])]
    bpm = peak_freq * 60

    return round(bpm, 1)


# ======================================
# 🔹 4) RMS 기반 Motion Level 측정
# ======================================
def compute_motion_level(raw_motion_values):
    if len(raw_motion_values) == 0:
        return 0
    arr = np.array(raw_motion_values)
    rms = np.sqrt(np.mean(arr ** 2))
    return round(float(rms), 2)


# ======================================
# 🔹 5) Sleep Stability Detection
# ======================================
def detect_sleep(breath_history, motion_history, bpm_variance_threshold=2, motion_threshold=1):
    """Returns True if stable breathing + low motion → sleeping"""
    
    if len(breath_history) < 20:
        return False

    # breathing stability
    bpm_var = np.var(breath_history[-20:])
    
    # motion stability
    motion_recent = max(motion_history[-10:]) if len(motion_history) >= 10 else max(motion_history)

    if bpm_var < bpm_variance_threshold and motion_recent < motion_threshold:
        return True
    
    return False
