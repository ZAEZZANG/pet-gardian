#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
일체형 PIR + 열화상(MLX90640) + mmWave(MR60BHA1 등) 통합 컨트롤러

- PIR:
    · 반려동물/사람 움직임 감지
    · 유효 트리거(너무 자주 들어오는 트리거는 무시) 시 센서 ON 요청

- 열화상(MLX90640):
    · PIR이 감지하면 전원/EN ON 후 웜업 → 측정 시작
    · 30℃ 이상 픽셀 개수 카운트
    · 최근 window_sec 동안의 Tmax가 danger_delta 이상 급격히 상승하면
        → [THERM][위기감지!] 메시지 출력

- mmWave 레이더(MR60BHA1 등):
    · PIR이 감지하면 전원/EN ON 후 웜업 → UART 수신 시작
    · 수신된 데이터에서 "호흡/주파수" 값을 추출한다고 가정 (TODO: parse_resp_value 구현)
    · 최근 window_sec 동안의 평균값과 비교해 큰 오차가 나오면
        → [MMW][위기감지!] 메시지 출력

라즈베리파이 4/5 + Raspberry Pi OS + Python 3.xx 가정
"""

import time
import threading
import signal
from datetime import datetime

# ===== gpiozero + lgpio(Pi 5) 설정 =====
from gpiozero import MotionSensor, OutputDevice, Device
from gpiozero.pins.lgpio import LGPIOFactory
Device.pin_factory = LGPIOFactory()  # Pi 5에서 /dev/gpiochip* 기반으로 GPIO 제어

# ===== 사용자 설정 =====
# --- 핀/전원 관련 ---
PIR_PIN_BCM        = 17     # PIR OUT (HC-SR501) → BCM 17
MMW_PWR_PIN_BCM    = 22     # mmWave 전원/EN GPIO (없으면 None)
THERM_PWR_PIN_BCM  = 27     # 열화상 전원/EN GPIO (없으면 None)
ACTIVE_HIGH_POWER  = True   # True: HIGH=ON, False: LOW=ON

# --- 동작 타이밍 ---
INACTIVITY_SEC     = 30     # 이 시간 동안 PIR이 조용하면 센서 OFF
RETRIGGER_GAP_SEC  = 1.0    # 연속 트리거 과민 방지 최소 간격 [s]
WARMUP_MMW_SEC     = 0.5    # mmWave 웜업 [s]
WARMUP_THERM_SEC   = 1.5    # MLX90640 웜업 [s]

# --- 센서 사용 여부 ---
USE_THERMAL        = True   # MLX90640 실제 연결되어 있으면 True
USE_MMWAVE         = True   # MR60BHA1 등 실제 연결되어 있으면 True

# --- 열화상 위기 감지 설정 ---
THERM_THRESHOLD        = 30.0  # 30℃ 이상 픽셀만 "생명체"로 본다
THERM_DANGER_DELTA     = 5.0   # 최근 window 내에서 Tmax가 이 값 이상 뛰면 "위기감지"
THERM_DANGER_WINDOW    = 5.0   # 최근 몇 초 동안 Tmax 변화를 볼지

# --- mmWave 위기 감지 설정 (호흡/주파수 값 기준) ---
MMW_WINDOW_SEC         = 10.0  # 최근 몇 초 동안의 값을 가지고 평균/이상값 검출
MMW_DANGER_DELTA       = 6.0   # 평균 대비 이 값 이상 차이나면 "위기감지" (예: bpm)
MMW_MIN_VALID          = 4.0   # 유효한 최소 호흡/주파수 값 (이하 값은 무시)
MMW_MAX_VALID          = 40.0  # 유효한 최대 호흡/주파수 값 (이상 값은 무시)
# ======================

# MLX90640 준비(옵션)
MLX_OK = False
if USE_THERMAL:
    try:
        import board, busio, adafruit_mlx90640
        MLX_OK = True
    except Exception as e:
        print(f"[THERM] MLX90640 라이브러리 로드 실패: {e}")
        MLX_OK = False

# UART 준비(옵션)
UART_OK = False
if USE_MMWAVE:
    try:
        import serial
        UART_OK = True
    except Exception as e:
        print(f"[MMW] pyserial 로드 실패: {e}")
        UART_OK = False


# ===== 공통 GPIO 전원 제어 클래스 =====
class PowerSwitch:
    """GPIO로 전원/EN 신호를 제어하는 래퍼"""
    def __init__(self, pin_bcm, active_high=True):
        self.dev = None
        if pin_bcm is not None:
            self.dev = OutputDevice(pin_bcm, active_high=active_high, initial_value=False)

    def on(self):
        if self.dev:
            self.dev.on()

    def off(self):
        if self.dev:
            self.dev.off()


# ===== PIR 센서 클래스 =====
class PirSensor:
    """
    PIR(HC-SR501) 래퍼 클래스
    - MotionSensor 이벤트를 받아서 retrigger 간격을 적용하고
      유효한 트리거만 콜백으로 전달
    """
    def __init__(self, pin_bcm, retrigger_gap_sec, on_valid_motion):
        """
        :param pin_bcm: PIR OUT에 연결된 BCM 핀 번호
        :param retrigger_gap_sec: 연속 트리거 최소 간격 [s]
        :param on_valid_motion: 유효 트리거 발생 시 호출할 콜백 (함수)
        """
        self._pin = pin_bcm
        self._gap = retrigger_gap_sec
        self._callback = on_valid_motion
        self._last_motion = datetime.min

        self._pir = MotionSensor(self._pin, queue_len=1, sample_rate=50, threshold=0.5)
        self._pir.when_motion = self._handle_motion

    @property
    def last_motion(self):
        return self._last_motion

    def _handle_motion(self):
        now = datetime.now()
        delta = (now - self._last_motion).total_seconds()
        if delta < self._gap:
            # 너무 빠르게 연속으로 들어오는 트리거는 무시
            return
        self._last_motion = now
        # 유효 트리거를 콜백으로 전달
        if self._callback:
            self._callback(now)


# ===== 열화상 센서 클래스 =====
class ThermalSensor:
    """
    MLX90640 열화상 수집 루프
    - PIR이 켜주면 start()
    - 30℃ 이상 픽셀 개수 카운트
    - 최근 window_sec 동안 Tmax가 danger_delta 이상 급격히 상승하면 "위기감지!"
    """
    def __init__(self,
                 threshold=THERM_THRESHOLD,
                 danger_delta=THERM_DANGER_DELTA,
                 window_sec=THERM_DANGER_WINDOW):
        self._stop = threading.Event()
        self._th = None
        self._ready = False
        self.mlx = None

        self.threshold = threshold
        self.danger_delta = danger_delta
        self.window_sec = window_sec
        self._history = []  # (time, max_temp)

    def start(self):
        """센서 초기화 및 측정 루프 시작"""
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
        frame = [0.0] * 768  # 32 x 24
        last_print = 0.0

        while not self._stop.is_set():
            if self._ready and self.mlx:
                try:
                    self.mlx.getFrame(frame)
                    now = time.time()

                    # 1) 30℃ 이상 픽셀 개수 & 온도 통계
                    hot_pixels = [t for t in frame if t >= self.threshold]
                    hot_count = len(hot_pixels)

                    tmin = min(frame)
                    tmax = max(frame)
                    # 가운데 픽셀 (12행, 16열)
                    tcenter = frame[12 * 32 + 16]

                    # 2) Tmax 히스토리 갱신
                    self._history.append((now, tmax))
                    self._history = [(t, v) for (t, v) in self._history
                                     if now - t <= self.window_sec]
                    hist_vals = [v for (_, v) in self._history]
                    hist_min = min(hist_vals) if hist_vals else tmax

                    # 3) 급격한 상승 감지
                    if tmax - hist_min >= self.danger_delta:
                        print(f"[THERM][위기감지!] {self.window_sec:.0f}초 이내 "
                              f"Tmax가 {hist_min:.1f}→{tmax:.1f}°C로 급격히 상승")

                    # 4) 1초에 한 번 상태 출력
                    if now - last_print >= 1.0:
                        print(
                            f"[THERM] hot≥{self.threshold:.1f}℃: {hot_count:3d}개 | "
                            f"Tmin={tmin:.1f}°C, Tcenter={tcenter:.1f}°C, Tmax={tmax:.1f}°C"
                        )
                        last_print = now

                except Exception as e:
                    print(f"[THERM] 읽기 실패: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.25)

    def stop(self):
        """측정 루프 정지 및 정리"""
        self._stop.set()
        if self._th:
            self._th.join(timeout=2)
        self._ready = False
        self.mlx = None
        self._history.clear()
        print("[THERM] 정지")


# ===== mmWave 레이더 센서 클래스 =====
class MmWaveSensor:
    """
    mmWave(MR60BHA1 등) UART 수신 루프
    - PIR이 켜주면 start()
    - 센서에서 오는 데이터에서 "호흡/주파수" 값을 추출한다고 가정
    - 최근 window_sec 동안의 평균과 비교해 큰 오차가 나면 "위기감지!"
    """

    def __init__(self,
                 port="/dev/serial0",
                 baud=115200,
                 window_sec=MMW_WINDOW_SEC,
                 danger_delta=MMW_DANGER_DELTA,
                 vmin=MMW_MIN_VALID,
                 vmax=MMW_MAX_VALID):
        self.port = port
        self.baud = baud
        self._stop = threading.Event()
        self._th = None
        self.ser = None
        self._ready = False

        self.window_sec = window_sec
        self.danger_delta = danger_delta
        self.vmin = vmin
        self.vmax = vmax
        self._history = []  # (time, value)

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

    def _parse_resp_value(self, data: bytes):
        """
        mmWave 모듈에서 오는 raw bytes에서
        '호흡/주파수' 값을 추출하는 함수.

        ★ 현재는 예시용 더미 구현입니다. ★
        실제 센서 프로토콜(데이터시트)을 보고
        이 부분만 수정하면 됩니다.

        여기서는 그냥:
        - 데이터 길이를 이용해 가짜 값 생성 (디버깅용)
        """
        if not data:
            return None
        # 예시: 데이터 길이를 2로 나눈 값을 "호흡수"라고 가정 (더미)
        fake_val = len(data) / 2.0
        return fake_val

    def _loop(self):
        last_print = 0.0

        while not self._stop.is_set():
            if self._ready and self.ser:
                try:
                    data = self.ser.read(128)
                    if not data:
                        continue

                    # 1) 원시 수신 로그 (필요시)
                    # print(f"[MMW] {len(data)} bytes 수신")

                    # 2) 호흡/주파수 값 추출
                    val = self._parse_resp_value(data)
                    if val is None:
                        continue

                    now = time.time()

                    # 유효 범위 체크 (너무 이상한 값은 무시)
                    if not (self.vmin <= val <= self.vmax):
                        print(f"[MMW] 무시되는 값(범위 밖): {val:.1f}")
                        continue

                    # 3) 히스토리 갱신
                    self._history.append((now, val))
                    self._history = [(t, v) for (t, v) in self._history
                                     if now - t <= self.window_sec]
                    vals = [v for (_, v) in self._history]
                    avg = sum(vals) / len(vals) if vals else val

                    # 4) 평균과의 큰 오차 감지
                    if abs(val - avg) >= self.danger_delta:
                        print(f"[MMW][위기감지!] 최근 평균 {avg:.1f} 대비 "
                              f"{val:.1f}로 큰 오차 발생")

                    # 5) 1초에 한 번 상태 출력
                    if now - last_print >= 1.0:
                        print(f"[MMW] 최근 {self.window_sec:.0f}s 평균={avg:.1f}, "
                              f"현재={val:.1f}")
                        last_print = now

                except Exception as e:
                    print(f"[MMW] 수신오류: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.25)

    def stop(self):
        self._stop.set()
        if self._th:
            self._th.join(timeout=2)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self._ready = False
        self._history.clear()
        print("[MMW] 정지")


# ===== 전체 시스템 컨트롤러 =====
class PetTriggerController:
    """
    PIR → mmWave + Thermal 센서 ON/OFF를 관리하는 상위 컨트롤러
    - PIR은 PirSensor 클래스로 이벤트만 전달
    - 센서 작동/정지는 이 컨트롤러가 관리
    """
    def __init__(self):
        # PIR 래퍼 (유효 트리거 콜백 = self._on_valid_pir_motion)
        self.pir = PirSensor(
            pin_bcm=PIR_PIN_BCM,
            retrigger_gap_sec=RETRIGGER_GAP_SEC,
            on_valid_motion=self._on_valid_pir_motion
        )

        # 전원/EN 제어용 GPIO
        self.mmw_pw   = PowerSwitch(MMW_PWR_PIN_BCM,   ACTIVE_HIGH_POWER)
        self.therm_pw = PowerSwitch(THERM_PWR_PIN_BCM, ACTIVE_HIGH_POWER)

        # 센서 작업 객체
        self.mmw   = MmWaveSensor()
        self.therm = ThermalSensor()

        # 상태 관리
        self.sensors_on = False
        self.last_motion = datetime.min
        self._lock = threading.Lock()

    # PIR에서 유효 움직임 감지 시 호출되는 콜백
    def _on_valid_pir_motion(self, when: datetime):
        with self._lock:
            self.last_motion = when
            if not self.sensors_on:
                print("[SYS] 반려동물/대상 감지 → 센서 ON")
                self._power_on_and_start()

    def _power_on_and_start(self):
        """센서 전원 ON + 웜업 후 측정 시작"""
        # 전원/EN ON
        self.mmw_pw.on()
        self.therm_pw.on()

        # mmWave 먼저 웜업
        time.sleep(WARMUP_MMW_SEC)
        if USE_MMWAVE:
            self.mmw.start()

        # 남은 시간만큼 더 기다렸다가 열화상 시작
        remain = max(WARMUP_THERM_SEC - WARMUP_MMW_SEC, 0)
        time.sleep(remain)
        if USE_THERMAL:
            self.therm.start()

        self.sensors_on = True

    def _power_off_and_stop(self):
        """센서 측정 중지 + 전원 OFF"""
        if USE_THERMAL:
            self.therm.stop()
        if USE_MMWAVE:
            self.mmw.stop()

        self.therm_pw.off()
        self.mmw_pw.off()

        self.sensors_on = False
        print("[SYS] 무감지 타임아웃 → 센서 OFF")

    def run(self):
        print("=== Pet Trigger Controller (PIR → mmWave + Thermal) ===")
        print(f"PIR={PIR_PIN_BCM}, MMW_PWR={MMW_PWR_PIN_BCM}, "
              f"THERM_PWR={THERM_PWR_PIN_BCM}, ACTIVE_HIGH={ACTIVE_HIGH_POWER}")
        print("대기 중… (반려동물/대상 감지 시 자동으로 켜짐)")

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

    # systemd 등에서 SIGTERM 받을 때도 정상 종료하기 위해
    def _sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm)

    ctrl.run()


if __name__ == "__main__":
    main()
