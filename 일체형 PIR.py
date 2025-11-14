#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
일체형 PIR: 반려동물 감지시에만 mmWave + 열화상 센서를 '깨우는' 전력 절약 컨트롤러
- PIR HIGH(감지) → 센서 ON(전원/EN) → 웜업 → 데이터 수집 시작
- PIR 무감지 일정 시간 지속 → 센서 OFF
- 라즈베리파이 4/5 + HC-SR501 + MLX90640 + MR60BHA1 가정
"""

import time
import threading
import signal
from datetime import datetime
from gpiozero import MotionSensor, OutputDevice

# ===== 사용자 설정 =====
PIR_PIN_BCM        = 17     # PIR OUT (HC-SR501)
MMW_PWR_PIN_BCM    = 22     # mmWave 전원/EN GPIO (없으면 None)
THERM_PWR_PIN_BCM  = 27     # 열화상 전원/EN GPIO (없으면 None)
ACTIVE_HIGH_POWER  = True   # True: HIGH=ON, False: LOW=ON

INACTIVITY_SEC     = 30     # 이 시간 동안 PIR이 조용하면 센서 OFF
RETRIGGER_GAP_SEC  = 1.0    # 연속 트리거 과민 방지
WARMUP_MMW_SEC     = 0.5    # mmWave 웜업
WARMUP_THERM_SEC   = 1.5    # MLX90640 웜업

# (선택) 센서 사용: 라이브러리가 없으면 자동 더미 모드로 동작
USE_THERMAL        = True
USE_MMWAVE         = True
# ======================

# MLX90640 준비(옵션)
MLX_OK = False
if USE_THERMAL:
    try:
        import board, busio, adafruit_mlx90640
        MLX_OK = True
    except Exception:
        MLX_OK = False

# UART 준비(옵션)
UART_OK = False
if USE_MMWAVE:
    try:
        import serial
        UART_OK = True
    except Exception:
        UART_OK = False


class PowerSwitch:
    """GPIO로 전원/EN 신호를 제어하는 래퍼"""
    def __init__(self, pin_bcm, active_high=True):
        self.dev = None
        if pin_bcm is not None:
            self.dev = OutputDevice(pin_bcm, active_high=active_high, initial_value=False)

    def on(self):
        if self.dev: self.dev.on()

    def off(self):
        if self.dev: self.dev.off()


class ThermalTask:
    """MLX90640 열화상 간단 수집 루프(더미 지원)"""
    def __init__(self):
        self._stop = threading.Event()
        self._th = None
        self._ready = False
        self.mlx = None

    def start(self):
        if MLX_OK:
            try:
                i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
                self.mlx = adafruit_mlx90640.MLX90640(i2c)
                self.mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ
                self._ready = True
                print("[THERM] MLX90640 초기화 완료")
            except Exception as e:
                self._ready = False
                print(f"[THERM] 초기화 실패: {e}")
        else:
            print("[THERM] 라이브러리 없음 → 더미 모드")

        self._stop.clear()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        frame = [0]*768  # 32x24
        last_print = 0
        while not self._stop.is_set():
            if self._ready and self.mlx:
                try:
                    self.mlx.getFrame(frame)
                    now = time.time()
                    if now - last_print >= 1.0:
                        tmin, tmax = min(frame), max(frame)
                        tcenter = frame[12*32 + 16]
                        print(f"[THERM] Tmin={tmin:.1f}°C, Tcenter={tcenter:.1f}°C, Tmax={tmax:.1f}°C")
                        last_print = now
                except Exception as e:
                    print(f"[THERM] 읽기 실패: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.25)

    def stop(self):
        self._stop.set()
        if self._th: self._th.join(timeout=2)
        self._ready = False
        self.mlx = None
        print("[THERM] 정지")


class MmWaveTask:
    """mmWave(MR60BHA1 등) UART 수신 루프(더미 지원)"""
    def __init__(self, port="/dev/serial0", baud=115200):
        self.port = port
        self.baud = baud
        self._stop = threading.Event()
        self._th = None
        self.ser = None
        self._ready = False

    def start(self):
        if UART_OK:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
                self._ready = True
                print("[MMW] UART 연결 완료")
            except Exception as e:
                self._ready = False
                print(f"[MMW] UART 실패: {e}")
        else:
            print("[MMW] pyserial 없음 → 더미 모드")

        self._stop.clear()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        while not self._stop.is_set():
            if self._ready and self.ser:
                try:
                    data = self.ser.read(128)
                    if data:
                        # TODO: 프로토콜 파싱/호흡수 추정 연결
                        print(f"[MMW] {len(data)} bytes 수신")
                except Exception as e:
                    print(f"[MMW] 수신오류: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.25)

    def stop(self):
        self._stop.set()
        if self._th: self._th.join(timeout=2)
        if self.ser:
            try: self.ser.close()
            except: pass
        self.ser = None
        self._ready = False
        print("[MMW] 정지")


class PetTriggerController:
    """PIR로 반려동물 감지 시 센서 ON/OFF를 관리"""
    def __init__(self):
        # PIR (HC-SR501: active-high)
        self.pir = MotionSensor(PIR_PIN_BCM, queue_len=1, sample_rate=50, threshold=0.5)
        self.pir.when_motion = self._on_motion

        # 전원/EN
        self.mmw_pw   = PowerSwitch(MMW_PWR_PIN_BCM,   ACTIVE_HIGH_POWER)
        self.therm_pw = PowerSwitch(THERM_PWR_PIN_BCM, ACTIVE_HIGH_POWER)

        # 센서 작업
        self.mmw   = MmWaveTask()
        self.therm = ThermalTask()

        # 상태
        self.sensors_on = False
        self.last_motion = datetime.min
        self._lock = threading.Lock()

    # PIR이 움직임(반려동물) 감지
    def _on_motion(self):
        now = datetime.now()
        with self._lock:
            if (now - self.last_motion).total_seconds() < RETRIGGER_GAP_SEC:
                return  # 과민 방지
            self.last_motion = now

            if not self.sensors_on:
                print("[SYS] 반려동물 감지 → 센서 ON")
                self._power_on_and_start()

    def _power_on_and_start(self):
        # 전원/EN ON
        self.mmw_pw.on()
        self.therm_pw.on()

        # 웜업 후 시작 순서
        time.sleep(WARMUP_MMW_SEC)
        if USE_MMWAVE: self.mmw.start()
        remain = max(WARMUP_THERM_SEC - WARMUP_MMW_SEC, 0)
        time.sleep(remain)
        if USE_THERMAL: self.therm.start()

        self.sensors_on = True

    def _power_off_and_stop(self):
        # 작업 중지 → 전원 OFF
        if USE_THERMAL: self.therm.stop()
        if USE_MMWAVE:  self.mmw.stop()

        self.therm_pw.off()
        self.mmw_pw.off()

        self.sensors_on = False
        print("[SYS] 무감지 타임아웃 → 센서 OFF")

    def run(self):
        print("=== Pet Trigger Controller (PIR→mmWave+Thermal) ===")
        print(f"PIR={PIR_PIN_BCM}, MMW_PWR={MMW_PWR_PIN_BCM}, THERM_PWR={THERM_PWR_PIN_BCM}, ACTIVE_HIGH={ACTIVE_HIGH_POWER}")
        print("대기 중… (반려동물 감지 시 자동으로 켜짐)")

        try:
            while True:
                time.sleep(0.2)
                if self.sensors_on:
                    idle = (datetime.now() - self.last_motion).total_seconds()
                    if idle >= INACTIVITY_SEC:
                        with self._lock:
                            idle2 = (datetime.now() - self.last_motion).total_seconds()
                            if self.sensors_on and idle2 >= INACTIVITY_SEC:
                                self._power_off_and_stop()
        except KeyboardInterrupt:
            pass
        finally:
            if self.sensors_on:
                self._power_off_and_stop()


def main():
    ctrl = PetTriggerController()

    def _sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm)

    ctrl.run()


if __name__ == "__main__":
    main()
