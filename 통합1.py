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

PIR_PIN = board.D17
MMWAVE_PORT = "/dev/serial0"
MMWAVE_BAUD = 115200

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
# pir.pull = digitalio.Pull.DOWN  # 모듈에 따라 필요시 사용

# =========================
# mmWave 초기화
# =========================

ser = serial.Serial(MMWAVE_PORT, MMWAVE_BAUD, timeout=0.05)

# mmWave 상태 변수
mm_body_level = 0
mm_resp = 0
mm_people = 0
mm_existence = "nobody"

# resp가 실제로 의미 있는 값인지 표시하는 플래그 ★ 변경
mm_resp_valid = False

def read_mmwave_frame():
    """MR60BHA1 한 프레임 읽어서 (cd, od, data) 반환. 못 읽으면 None."""
    # 헤더 'S'(0x53), 'Y'(0x59) 찾기
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == 0x53:  # 'S'
            b2 = ser.read(1)
            if not b2:
                return None
            if b2[0] == 0x59:  # 'Y'
                break

    hdr = ser.read(4)  # CD, OD, lenH, lenL
    if len(hdr) < 4:
        return None
    cd, od, lh, ll = hdr
    length = (lh << 8) | ll

    rest = ser.read(length + 3)  # data + checksum + 'T','C'
    if len(rest) < length + 3:
        return None

    data = rest[:length]
    ft1, ft2 = rest[length + 1:length + 3]
    if ft1 != 0x54 or ft2 != 0x43:  # 'T','C'
        return None

    return cd, od, data


def update_mmwave_state():
    """mmWave 프레임 하나 읽어 전역 상태(mm_body_level, mm_resp 등) 갱신."""
    global mm_body_level, mm_resp, mm_people, mm_existence, mm_resp_valid

    frame = read_mmwave_frame()
    if frame is None:
        return

    cd, od, data = frame
    if not data:
        return

    value = data[0]

    if cd == 0x80:
        # Human presence 그룹
        if od == 0x01:          # 존재 여부
            mm_existence = "occupied" if value == 1 else "nobody"
            # 존재가 사라지면 resp는 더 이상 유효하지 않다고 봄 ★ 변경
            if mm_existence == "nobody":
                mm_resp_valid = False
        elif od == 0x02:        # motion 방향 (로그만 찍을 수도 있음)
            motion = {0: "none", 1: "close", 2: "away", 3: "disorderly"}.get(value, f"unknown({value})")
            print(f"[mm] motion: {motion}")
        elif od == 0x03:        # body level
            mm_body_level = value
        elif od == 0x04:        # people count
            mm_people = value

    elif cd == 0x81:
        # 호흡/심박 그룹
        if od == 0x02:          # respiration
            # 존재가 있을 때만 resp를 유효한 값으로 인정 ★ 변경
            if mm_existence == "occupied" or mm_people >= 1:
                mm_resp = value
                mm_resp_valid = True
            else:
                # 존재가 없으면 이 값은 노이즈로 보고 무시
                mm_resp_valid = False


def mmwave_has_target():
    """
    펫 기준으로 '대상 있음' 판단:
      - body_level이 충분히 크거나
      - (존재가 있다고 판단된 상태에서) resp가 유의미하거나
      - 또는 people / existence 가 사람 감지된 경우
    """
    # 펫(또는 작은 대상) 기준
    if mm_body_level >= 5:
        return True

    # resp는 존재가 있을 때만 신뢰 ★ 변경
    if mm_resp_valid and mm_resp >= 10:
        return True

    # 사람 기준 신호는 보조
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
THERMAL_INTERVAL = 0.5   # [초], 열화상 업데이트 주기


def update_thermal_state():
    """열화상 한 프레임 읽어서 평균 온도 / hot 픽셀 통계 갱신."""
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
    """
    열화상이 '대상 있음'이라고 보는 조건:
      - 30도 이상 픽셀 개수가 충분히 많고
      - 전체 평균 온도가 너무 낮지 않을 때
    """
    if thermal_hot_count >= MIN_HOT_PIXELS and thermal_avg_all >= WARN_TEMP:
        return True
    return False


# =========================
# 상태머신 메인 루프
# =========================

print("=== Pet Sensor 통합 테스트 시작 ===")
print("PIR → ON 트리거, mmWave + Thermal AND → OFF 트리거\n")

active = False
last_active_time = 0.0

try:
    while True:
        pir_state = pir.value

        # 1) PIR HIGH가 되는 순간 → 감시 모드 ON
        if pir_state and not active:
            active = True
            last_active_time = time.monotonic()
            print("\n[PIR] 감지 → 감시 모드 ON")

        if active:
            # mmWave는 빠르게 계속 갱신
            update_mmwave_state()

            # Thermal은 일정 간격으로만 갱신
            now = time.monotonic()
            if now - last_thermal_frame_time > THERMAL_INTERVAL:
                last_thermal_frame_time = now
                update_thermal_state()

                # resp 출력도 "유효/무효" 구분해서 보여주기 ★ 변경
                resp_print = mm_resp if mm_resp_valid else "None"

                print("\n=== ACTIVE 상태 ===")
                print(f"[PIR] state: {'HIGH' if pir_state else 'LOW'}")
                print(f"[mm] existence={mm_existence}, people={mm_people}, "
                      f"body_level={mm_body_level}, resp={resp_print}")
                print(f"[MLX] avg_all={thermal_avg_all:5.2f} °C, "
                      f"hot_count(>= {TEMP_THRESHOLD:.1f}°C)={thermal_hot_count}, "
                      f"avg_hot={thermal_avg_hot if thermal_avg_hot is not None else 'None'}")

            # 두 센서의 "대상 있음" 여부 판단
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