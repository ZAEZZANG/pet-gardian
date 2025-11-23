#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🔥 Integrated Pet Health Monitor (FINAL MAIN)

연결되는 모듈:
- sensors/mmWAVE.py        → MmWave 클래스 (호흡 + 모션 raw 신호 수집)
- sensors/열화상카메라.py   → init_thermal(), get_thermal_readings(), classify_thermal_entity()
- sensors/integrated_pir.py → setup_pir(), update_sensor_power()  (파일 이름은 네가 맞춰서 수정)
- utils/signal_processing.py→ estimate_breathing_rate(), compute_motion_level(), detect_sleep()
- utils/fusion.py          → fuse_sensors() (mmWave + Thermal 융합)
- utils/telegram.py        → evaluate_sensor(bpm, temp) (텔레그램 알림 + 상태 리턴)

※ 위성 PIR + 사람입퇴장(완화모드) 로직은 `사람입퇴장.py`에 따로 돌아가도 되고,
  나중에 필요하면 이 main에서 import 해서 엮을 수 있음.
"""

import time
import numpy as np

# --------- 센서 모듈 import ---------
from sensors.mmwave import mmwave
from sensors.열화상카메라 import (
    init_thermal,
    get_thermal_readings,
    classify_thermal_entity,
)

# 파일 이름을 integrated_pir.py 로 바꿨다고 가정
from sensors.integrated_pir import setup_pir, update_sensor_power

# --------- 유틸 모듈 import ---------
from utils.signal_processing import (
    estimate_breathing_rate,
    compute_motion_level,
    detect_sleep,
)

from utils.fusion import fuse_sensors
from utils.telegram import evaluate_sensor  # bpm, temperature → 상태 판단 + 텔레그램 전송



# ==============================
# 🔧 USER CONFIG
# ==============================
FS_MMWAVE = 10                 # mmWave 샘플링(팀원 코드에서 fs=10 사용 중)
MAIN_LOOP_INTERVAL = 0.5       # 루프 주기 (초)
NO_ACTIVITY_TIMEOUT = 120      # 활동 없을 때 슬립 고려 시간 (초)

# Thermal 기준 (열화상카메라.py 안의 HOT_* 값과 함께 튜닝 대상)
TEMP_FEVER_THRESHOLD = 39.5    # 개 기준 고열 의심 온도



# ==============================
# 🔧 STATE BUFFERS
# ==============================
bpm_history = []        # 호흡수 히스토리
motion_raw_history = [] # mmWave 모션 raw
temp_history = []       # 체온 히스토리

last_activity_time = time.time()
system_awake = True     # 일단 켜진 상태로 시작 (PIR 연동 시 조정 가능)



# ==============================
# 🔋 POWER CONTROL 래퍼
# ==============================
def mark_activity():
    """센서에서 의미 있는 활동이 감지된 경우 마지막 활동 시각 업데이트."""
    global last_activity_time
    last_activity_time = time.time()


def maybe_enter_sleep():
    """일정 시간 이상 활동 없으면, PIR 모듈을 통해 센서 전원 OFF."""
    global system_awake
    now = time.time()
    if system_awake and (now - last_activity_time) > NO_ACTIVITY_TIMEOUT:
        # integrated_pir 내부에서 mmWave/열화상 전원을 실제로 제어하도록 구현했다면
        # 여기서는 단순히 상태만 false 로 두고, update_sensor_power() 가 알아서 OFF 해주게 둘 수 있음.
        print("\n[POWER] 활동 없음 → 저전력 모드 고려 중 (integrated_pir 가 관리)")
        system_awake = False
    elif not system_awake and (now - last_activity_time) <= NO_ACTIVITY_TIMEOUT:
        system_awake = True



# ==============================
# 🧠 MAIN LOOP
# ==============================
def main():
    global system_awake

    print("\n========== PET HEALTH MONITOR START ==========\n")

    # 1) 일체형 PIR 초기화 (움직임 감지 → 센서 깨우기/전원제어는 integrated_pir 내부)
    setup_pir()

    # 2) 열화상 카메라 초기화
    thermal_ok = init_thermal()
    if not thermal_ok:
        print("[THERMAL] 초기화 실패. 열화상 없이 mmWave만 동작합니다.")

    # 3) mmWave 초기화
    mm = mmwave(port="/dev/ttyAMA10", baud=115200, fs=FS_MMWAVE)
    print("[MMWAVE] 초기화 완료.")

    try:
        while True:
            now = time.time()

            # --- PIR 기반 전원 관리 업데이트 ---
            update_sensor_power()
            maybe_enter_sleep()

            if not system_awake:
                # 저전력 모드일 땐 센서 읽지 않고 대기
                print("[STATUS] 📴 Idle (PIR가 깨워줄 때까지 대기)")
                time.sleep(1.0)
                continue

            # ============================
            # 1) mmWave 읽기
            # ============================
            breath_sample, motion_sample = mm.read()

            if motion_sample is not None:
                motion_raw_history.append(motion_sample)
                # 너무 길어지지 않게 제한
                if len(motion_raw_history) > FS_MMWAVE * 60:
                    motion_raw_history.pop(0)

            # mmWave 전체 breath_signal 을 기반으로 FFT 호흡 추정
            bpm = None
            if len(mm.breath_signal) >= FS_MMWAVE * 5:
                bpm = estimate_breathing_rate(
                    np.array(mm.breath_signal, dtype=float),
                    fs=FS_MMWAVE
                )
                if bpm is not None:
                    bpm_history.append(bpm)
                    if len(bpm_history) > 120:
                        bpm_history.pop(0)

            # ============================
            # 2) Thermal 읽기
            # ============================
            temp = None
            hot_pixels = 0
            thermal_entity = "none"

            if thermal_ok:
                temp, hot_pixels = get_thermal_readings()
                thermal_entity = classify_thermal_entity(hot_pixel_count=hot_pixels)
                if temp:
                    temp_history.append(temp)
                    if len(temp_history) > 120:
                        temp_history.pop(0)

            # ============================
            # 3) Motion Level / Sleep Detection
            # ============================
            motion_level = compute_motion_level(motion_raw_history[-50:]) if motion_raw_history else 0
            sleeping = detect_sleep(
                breath_history=bpm_history,
                motion_history=motion_raw_history,
            )

            # 활동이 있으면 last_activity_time 갱신
            if (bpm is not None) or (motion_level > 0.5) or (hot_pixels > 0):
                mark_activity()

            # ============================
            # 4) Fusion (mmWave + Thermal)
            #     - 거리/size 정보가 없으므로 None/0으로 채움
            # ============================
            mm_data = {
                "bpm": bpm if bpm is not None else 0,
                "motion": motion_level,
                "distance": None,  # 현재 mmWave 모듈에서는 제공 X
                "size": 0,         # 현재 mmWave 모듈에서는 제공 X
            }

            fusion_result = fuse_sensors(
                mm_data=mm_data,
                temp=temp if temp is not None else 0.0,
                hot_pixels=hot_pixels,
                thermal_entity=thermal_entity,
            )

            # ============================
            # 5) 상태 출력 (디버깅용)
            # ============================
            print("\n------ FRAME ------")
            if bpm is not None:
                print(f"🌬 Breathing: {bpm} BPM")
            else:
                print("🌬 Breathing: 측정 불가/데이터 부족")

            if temp is not None:
                print(f"🌡 Temp: {temp:.1f}°C (hot_pixels={hot_pixels}, thermal_entity={thermal_entity})")
            else:
                print("🌡 Temp: N/A")

            print(f"🏃 Motion level: {motion_level}")
            print(f"😴 Sleeping? {'YES' if sleeping else 'NO'}")
            print(f"🤖 Fusion entity: {fusion_result.entity} (conf={fusion_result.confidence:.2f})")
            print(f"   reason: {fusion_result.reason}")

            # ============================
            # 6) 텔레그램 평가 로직 호출
            #    (utils/telegram.py의 evaluate_sensor 사용)
            # ============================
            if bpm is not None and temp is not None:
                status = evaluate_sensor(bpm, temp)
                print(f"[ALERT LOGIC] 상태 = {status}")

            # ============================
            # 7) Loop interval
            # ============================
            time.sleep(MAIN_LOOP_INTERVAL)

    except KeyboardInterrupt:
        print("\n[MAIN] Ctrl+C 입력 → 프로그램 안전 종료 시도 중...")
    finally:
        print("[MAIN] 종료 완료.")
