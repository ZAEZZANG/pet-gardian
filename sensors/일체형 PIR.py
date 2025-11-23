#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
integrated_pir.py
- 일체형 PIR로 움직임 감지
- 움직임 감지 시 mmWave + Thermal 센서 깨우기
- 일정 시간 동안 추가 움직임 없으면 센서 OFF (전력 절약)
"""

import RPi.GPIO as GPIO
import time

# ============================================
# 🔵 1. 핀 번호 & 상태 변수 설정
# ============================================

PIR_PIN = 17      # PIR 센서 입력 (BCM 번호 기준)

# 필요하면 아래처럼 전원 제어 핀도 쓸 수 있음 (지금은 예시로만 둠)
# MMWAVE_EN_PIN = 22
# THERMAL_EN_PIN = 27

mmwave_on = False
thermal_on = False

sensor_active_until = 0                # 이 시각까지는 센서를 켜두자
SENSOR_ACTIVE_TIME = 60                # 움직임 감지 후 60초 유지 (초)


# ============================================
# 🔵 2. 센서 전원 ON/OFF 함수
# ============================================

def mmwave_power_on():
    """mmWave 센서를 켜는 함수 (실제 전원 제어 코드는 TODO)."""
    global mmwave_on
    if not mmwave_on:
        print("[일체형 PIR] mmWave ON")
        # TODO: 실제 전원 제어 코드
        # GPIO.output(MMWAVE_EN_PIN, GPIO.HIGH)
        mmwave_on = True

def mmwave_power_off():
    """mmWave 센서를 끄는 함수."""
    global mmwave_on
    if mmwave_on:
        print("[일체형 PIR] mmWave OFF")
        # TODO: 실제 전원 OFF 코드
        # GPIO.output(MMWAVE_EN_PIN, GPIO.LOW)
        mmwave_on = False

def thermal_power_on():
    """열화상 카메라를 켜는 함수."""
    global thermal_on
    if not thermal_on:
        print("[일체형 PIR] Thermal ON")
        # TODO: 실제 열화상 카메라 ON / 초기화 코드
        thermal_on = True

def thermal_power_off():
    """열화상 카메라를 끄는 함수."""
    global thermal_on
    if thermal_on:
        print("[일체형 PIR] Thermal OFF")
        # TODO: 실제 열화상 카메라 OFF 코드
        thermal_on = False

def wake_sensors():
    """
    ✅ 일체형 PIR이 '무언가 움직임' 감지했을 때 호출:
       - mmWave ON
       - Thermal ON
       - SENSOR_ACTIVE_TIME 동안은 최소한 ON 유지
    """
    global sensor_active_until
    now = time.time()
    sensor_active_until = now + SENSOR_ACTIVE_TIME

    mmwave_power_on()
    thermal_power_on()


# ============================================
# 🔵 3. PIR 콜백 & GPIO 초기 설정
# ============================================

def pir_callback(channel):
    """PIR이 움직임 감지할 때마다 호출되는 콜백 함수."""
    print("[일체형 PIR] 움직임 감지 → mmWave + Thermal 깨우기")
    wake_sensors()

def setup_pir():
    """PIR 및 GPIO 초기 설정."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(PIR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # GPIO.setup(MMWAVE_EN_PIN, GPIO.OUT, initial=GPIO.LOW)
    # GPIO.setup(THERMAL_EN_PIN, GPIO.OUT, initial=GPIO.LOW)

    GPIO.add_event_detect(
        PIR_PIN,
        GPIO.RISING,
        callback=pir_callback,
        bouncetime=200
    )
    print("[일체형 PIR] 설정 완료 (움직임 감지 대기 중)")


# ============================================
# 🔵 4. 센서 전원 유지 / OFF 관리
# ============================================

def update_sensor_power():
    """
    메인 루프에서 주기적으로 호출:
      - sensor_active_until 시간이 지나면
        mmWave + Thermal OFF 해서 전력 절약.
    """
    now = time.time()
    if now > sensor_active_until:
        mmwave_power_off()
        thermal_power_off()


# ============================================
# 🔵 5. 메인 실행
# ============================================

def main():
    setup_pir()
    try:
        while True:
            update_sensor_power()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[일체형 PIR] 프로그램 종료")
    finally:
        mmwave_power_off()
        thermal_power_off()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
