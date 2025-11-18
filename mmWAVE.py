import serial
import time
import numpy as np
from scipy.signal import detrend
from scipy.fft import rfft, rfftfreq
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ----------------------------------
# UART 설정
# ----------------------------------
ser = serial.Serial('/dev/ttyAMA10', 115200, timeout=0.5)

fs = 10                  # 샘플링 주파수(Hz)
window_size = 100       # 그래프 window (10초 데이터)

breath_signal = []
motion_signal = []
distance_signal = []
bpm_history = []

# ----------------------------------
# FFT 기반 호흡 계산 함수
# ----------------------------------
def estimate_breathing_rate(signal):
    if len(signal) < fs * 5:  # 최소 5초 데이터 필요
        return None

    cleaned = detrend(np.array(signal))
    n = len(cleaned)
    freqs = rfftfreq(n, 1/fs)
    spectrum = np.abs(rfft(cleaned))

    mask = (freqs >= 0.1) & (freqs <= 1.0)  # 6~60 BPM
    if not np.any(mask):
        return None

    peak = freqs[mask][np.argmax(spectrum[mask])]
    return peak * 60  # Hz → BPM 변환

# ----------------------------------
# 센서 데이터 파싱
# ----------------------------------
def read_packet():
    header = ser.read(2)
    if header != b'\x53\x59': 
        return None, None, None

    frame_type = ser.read(1)[0]
    length = ser.read(1)[0]
    payload = ser.read(length)
    end = ser.read(2)

    if end != b'\x54\x43':
        return None, None, None

    # 데이터 분류
    motion = None
    distance = None
    breath = None

    if frame_type == 0x85:   # motion energy
        motion = payload[-1]

    elif frame_type == 0x81: # presence/distance
        distance = payload[-1]

    elif frame_type in [0x83,0x89,0x8A]:
        breath = payload[-1]  # raw breathing signal

    return breath, distance, motion

# ----------------------------------
# 실시간 그래프
# ----------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8,6))

motion_line, = ax1.plot([], [], 'b-', linewidth=2)
breath_line, = ax2.plot([], [], 'g-', linewidth=1)

ax1.set_title("📈 실시간 활동량(Motion Energy)")
ax2.set_title("🌬 실시간 호흡 신호 (Raw)")
ax1.set_ylim(0,200)
ax2.set_ylim(0,255)

def update(frame):
    breath, dist, motion = read_packet()

    if motion is not None:
        motion_signal.append(motion)
    if breath is not None:
        breath_signal.append(breath)

    # 데이터 길이 window 제한
    if len(motion_signal) > window_size:
        motion_signal.pop(0)
    if len(breath_signal) > window_size:
        breath_signal.pop(0)

    # -------- 그래프 업데이트 --------
    motion_line.set_data(range(len(motion_signal)), motion_signal)
    breath_line.set_data(range(len(breath_signal)), breath_signal)

    ax1.set_xlim(0, len(motion_signal))
    ax2.set_xlim(0, len(breath_signal))

    # -------- BPM 계산 (움직임 낮을 때만) --------
    if len(breath_signal) > fs*5 and np.mean(motion_signal[-5:]) < 30:
        bpm = estimate_breathing_rate(breath_signal)
        if bpm:
            bpm_history.append(bpm)
            ax2.set_title(f"🌬 호흡 신호 — BPM: {bpm:.1f}")
        else:
            ax2.set_title("🌬 호흡 신호 — 분석중...")
    else:
        ax2.set_title("🚨 움직임 감지 — 호흡 분석 잠시 중지")

    return motion_line, breath_line

ani = FuncAnimation(fig, update, interval=100)  # 0.1초 주기
plt.tight_layout()
plt.show()
