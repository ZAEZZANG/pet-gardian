#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial
import board
import busio
import digitalio
import adafruit_mlx90640

# =========================
# 하드웨어 설정
# =========================

# PIR: GPIO17 (보드 핀 11)
PIR_PIN = board.D17

# mmWave UART
MMWAVE_PORT = "/dev/serial0"
MMWAVE_BAUD = 115200

# MLX90640 온도 관련 기준
TEMP_THRESHOLD = 30.0
WARN_TEMP = 28.0
MIN_HOT_PIXELS = 15

OFF_DELAY = 5.0

MLX_ROWS = 24
MLX_COLS = 32
MLX_SIZE = MLX_ROWS * MLX_COLS


# =========================
# PIR 초기화
# =========================

pir = digitalio.DigitalInOut(PIR_PIN)
pir.direction = digitalio.Direction.INPUT
# pir.pull = digitalio.Pull.DOWN


# =========================
# mmWave 초기화
# =========================

ser = serial.Serial(MMWAVE_PORT, MMWAVE_BAUD, timeout=0.05)

mm_body_level = 0
mm_resp = 0
mm_people = 0
mm_existence = "nobody"


def read_mmwave_frame():
    """MR60BHA1 한 프레임 읽어서 (cd, od, data) 반환."""
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == 0x53:
            b2 = ser.read(1)
            if not b2:
                return None
            if b2[0] == 0x59:
                break

    hdr = ser.read(4)
    if len(hdr) < 4:
        return None
    cd, od, lh, ll = hdr
    length = (lh << 8) | ll

    rest = ser.read(length + 3)
    if len(rest) < length + 3:
        return None

    data = rest[:length]
    ft1, ft2 = rest[length + 1:length + 3]
    if ft1 != 0x54 or ft2 != 0x43:
        return None

    return cd, od, data


def update_mmwave_state():
    """mmWave 프레임 하나 읽어 상태 갱신 (resp 필터 제거 버전)."""
    global mm_body_level, mm_resp, mm_people, mm_existence

    frame = read_mmwave_frame()
    if frame is None:
        return

    cd, od, data = frame
    value = data[0]

    if cd == 0x80:
        if od == 0x01:      # existence
            mm_existence = "occupied" if value == 1 else "nobody"
        elif od == 0x02:    # motion
            motion = {0:"none",1:"close",2:"away",3:"disorderly"}.get(value, f"unknown({value})")
            print(f"[mm] motion: {motion}")
        elif od == 0x03:    # body_level
            mm_body_level = value
        elif od == 0x04:    # people
            mm_people = value

    elif cd == 0x81:
        if od == 0x02:      # respiration
            # ★ 여기서 필터 없이 무조건 저장 ★
            mm_resp = value



def mmwave_has_target():
    """펫 기준 mmWave 타겟 여부."""
    if mm_body_level >= 5:
        return True
    if mm_resp >= 10:
        return True
    if mm_people >= 1 or mm_existence == "occupied":
        return True
    return False


# =========================
# MLX90640 초기화
# =========================

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ

frame = [0.0] * MLX_SIZE

thermal_avg_all = 0.0
thermal_hot_count = 0
thermal_avg_hot = None

last_thermal_frame_time = 0.0
THERMAL_INTERVAL = 0.5


def update_thermal_state():
    """열화상 프레임 읽기."""
    global thermal_avg_all, thermal_hot_count, thermal_avg_hot

    try:
        mlx.getFrame(frame)
    except Exception as e:
        print("[MLX] 읽기 에러:", e)
        return

    temps = frame
    thermal_avg_all = sum(temps) / len(temps)

    hot_pixels = [t for t in temps if t >= TEMP_THRESHOLD]
    thermal_hot_count = len(hot_pixels)
    thermal_avg_hot = sum(hot_pixels) / thermal_hot_count if hot_pixels else None


def thermal_has_target():
    """열화상 타겟 여부."""
    if thermal_hot_count >= MIN_HOT_PIXELS and thermal_avg_all >= WARN_TEMP:
        return True
    return False


# =========================
# 상태머신 메인 루프
# =========================

print("=== Pet Sensor 통합 테스트(Resp 필터 제거 버전) 시작 ===")

active = False
last_active_time = 0.0

try:
    while True:
        pir_state = pir.value

        # PIR ON → Active 진입
        if pir_state and not active:
            active = True
            last_active_time = time.monotonic()
            print("\n[PIR] 감지 → 감시 모드 ON")

        if active:
            update_mmwave_state()

            now = time.monotonic()
            if now - last_thermal_frame_time > THERMAL_INTERVAL:
                last_thermal_frame_time = now
                update_thermal_state()

                print("\n=== ACTIVE 상태 ===")
                print(f"[PIR] state={'HIGH' if pir_state else 'LOW'}")
                print(f"[mm] existence={mm_existence}, people={mm_people}, "
                      f"body_level={mm_body_level}, resp={mm_resp}")
                print(f"[MLX] avg_all={thermal_avg_all:5.2f} °C, "
                      f"hot_count={thermal_hot_count}, "
                      f"avg_hot={thermal_avg_hot if thermal_avg_hot else 'None'}")

            mm_on = mmwave_has_target()
            th_on = thermal_has_target()

            if mm_on or th_on:
                last_active_time = time.monotonic()
            else:
                if time.monotonic() - last_active_time > OFF_DELAY:
                    active = False
                    print("\n[STATE] mmWave & Thermal 모두 대상 없음 → 감시 모드 OFF")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n종료합니다.")