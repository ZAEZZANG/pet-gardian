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

- 추가 요구사항 반영:
    · PIR은 항상 켜져 있음 (절대 OFF 안 함)
    · 센서가 ON 된 뒤,
        · 열화상: Tmax < THERM_THRESHOLD (30℃ 이하, hot pixel 없음)
        · mmWave: 현재 값 <= 최근 평균값
        · 두 센서 모두에서 [위기감지!]가 발생하지 않은 “안전 상태”가
          INACTIVITY_SEC 동안 연속 유지되면
          → mmWave + 열화상 OFF (PIR은 계속 대기)

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
INACTIVITY_SEC     = 30     # 이 시간 동안 "안전 상태"가 유지되면 mmWave+열화상 OFF
RETRIGGER_GAP_SEC  = 1.0    # PIR 연속 트리거 과민 방지 최소 간격 [s]
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
    - PIR 자체는 항상 켜져 있음 (전원 제어하지 않음)
    """
    def __init__(self, pin_bcm, retrigger_gap_sec, on_valid_motion):
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
            return
        self._last_motion = now
        if self._callback:
            self._callback(now)


# ===== 열화상 센서 클래스 =====
class ThermalSensor:
    """
    MLX90640 열화상 수집 루프
    - 30℃ 이상 픽셀 개수 카운트
    - Tmax 급상승 시 위기감지
    - last_data_time: 마지막 프레임 시각
    - last_hot: 마지막 프레임에서 hot pixel(30℃ 이상)이 있었는지 여부
    - danger_raised: 위기감지 발생 여부
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

        self.last_data_time = None
        self.last_hot = False
        self.danger_raised = False

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

        self.danger_raised = False
        self.last_hot = False
        self.last_data_time = None

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
                    self.last_data_time = now

                    hot_pixels = [t for t in frame if t >= self.threshold]
                    hot_count = len(hot_pixels)

                    tmin = min(frame)
                    tmax = max(frame)
                    tcenter = frame[12 * 32 + 16]

                    # hot 여부 업데이트 (30℃ 이상 픽셀 존재 여부)
                    self.last_hot = (hot_count > 0)

                    self._history.append((now, tmax))
                    self._history = [(t, v) for (t, v) in self._history
                                     if now - t <= self.window_sec]
                    hist_vals = [v for (_, v) in self._history]
                    hist_min = min(hist_vals) if hist_vals else tmax

                    if tmax - hist_min >= self.danger_delta:
                        self.danger_raised = True
                        print(f"[THERM][위기감지!] {self.window_sec:.0f}초 이내 "
                              f"Tmax가 {hist_min:.1f}→{tmax:.1f}°C로 급격히 상승")

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
        self._stop.set()
        if self._th:
            self._th.join(timeout=2)
        self._ready = False
        self.mlx = None
        self._history.clear()
        self.last_data_time = None
        self.last_hot = False
        self.danger_raised = False
        print("[THERM] 정지")


# ===== mmWave 레이더 센서 클래스 =====
class MmWaveSensor:
    """
    mmWave(MR60BHA1 등) UART 수신 루프
    - 호흡/주파수 값 추출 (더미 구현)
    - 평균 대비 큰 오차는 위기감지
    - last_data_time: 마지막 유효 값 시각
    - last_val: 마지막 값
    - last_avg: 최근 window 평균
    - danger_raised: 위기감지 발생 여부
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

        self.last_data_time = None
        self.last_val = None
        self.last_avg = None
        self.danger_raised = False

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

        self._history.clear()
        self.last_data_time = None
        self.last_val = None
        self.last_avg = None
        self.danger_raised = False

        self._stop.clear()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _parse_resp_value(self, data: bytes):
        """
        ★ 현재는 예시용 더미 구현 ★
        실제 센서 프로토콜에 맞게 수정하면 된다.
        """
        if not data:
            return None
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

                    val = self._parse_resp_value(data)
                    if val is None:
                        continue

                    now = time.time()

                    if not (self.vmin <= val <= self.vmax):
                        print(f"[MMW] 무시되는 값(범위 밖): {val:.1f}")
                        continue

                    self.last_data_time = now

                    self._history.append((now, val))
                    self._history = [(t, v) for (t, v) in self._history
                                     if now - t <= self.window_sec]
                    vals = [v for (_, v) in self._history]
                    avg = sum(vals) / len(vals) if vals else val

                    self.last_val = val
                    self.last_avg = avg

                    if abs(val - avg) >= self.danger_delta:
                        self.danger_raised = True
                        print(f"[MMW][위기감지!] 최근 평균 {avg:.1f} 대비 "
                              f"{val:.1f}로 큰 오차 발생")

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
        self.last_data_time = None
        self.last_val = None
        self.last_avg = None
        self.danger_raised = False
        print("[MMW] 정지")


# ===== 전체 시스템 컨트롤러 =====
class PetTriggerController:
    """
    PIR → mmWave + Thermal 센서 ON/OFF를 관리하는 상위 컨트롤러

    - PIR은 항상 켜져 있고, 움직임 감지 시 센서 ON
    - 센서가 ON 된 후, 다음 조건이 모두 충족된 "안전 상태"가
      INACTIVITY_SEC 동안 연속 유지되면 센서 OFF:
        · Thermal: 마지막 Tmax < THERM_THRESHOLD (hot pixel 없음) 이고
                   위기감지(danger_raised)도 발생 X
        · mmWave: last_val <= last_avg 이고
                  위기감지(danger_raised)도 발생 X
    """
    def __init__(self):
        self.pir = PirSensor(
            pin_bcm=PIR_PIN_BCM,
            retrigger_gap_sec=RETRIGGER_GAP_SEC,
            on_valid_motion=self._on_valid_pir_motion
        )

        self.mmw_pw   = PowerSwitch(MMW_PWR_PIN_BCM,   ACTIVE_HIGH_POWER)
        self.therm_pw = PowerSwitch(THERM_PWR_PIN_BCM, ACTIVE_HIGH_POWER)

        self.mmw   = MmWaveSensor()
        self.therm = ThermalSensor()

        self.sensors_on = False
        self._lock = threading.Lock()

        # "안전 상태"가 시작된 시각 (None이면 아직 안전 연속 구간 아님)
        self.safe_start_time = None

    def _on_valid_pir_motion(self, when: datetime):
        with self._lock:
            if not self.sensors_on:
                print("[SYS] 반려동물/대상 감지 → 센서 ON")
                self._power_on_and_start()

    def _power_on_and_start(self):
        self.mmw_pw.on()
        self.therm_pw.on()

        self.safe_start_time = None

        time.sleep(WARMUP_MMW_SEC)
        if USE_MMWAVE:
            self.mmw.start()

        remain = max(WARMUP_THERM_SEC - WARMUP_MMW_SEC, 0)
        time.sleep(remain)
        if USE_THERMAL:
            self.therm.start()

        self.sensors_on = True

    def _power_off_and_stop(self):
        if USE_THERMAL:
            self.therm.stop()
        if USE_MMWAVE:
            self.mmw.stop()

        self.therm_pw.off()
        self.mmw_pw.off()

        self.sensors_on = False
        self.safe_start_time = None
        print("[SYS] 30초간 안전 상태 유지 → 센서 OFF (PIR은 계속 대기)")

    def _check_safe_state(self, now: float) -> bool:
        """
        현재 시점에 열화상 + mmWave 모두 "안전한 상태"인지 판단.
        True면 안전, False면 비안전(위험 또는 판단불가).
        """
        therm_safe = True
        mmw_safe = True

        if USE_THERMAL:
            # 아직 데이터가 없거나 위기감지가 한 번이라도 떴으면 안전 X
            if self.therm.last_data_time is None or self.therm.danger_raised:
                therm_safe = False
            else:
                # hot pixel 없음 → Tmax < threshold
                therm_safe = (self.therm.last_hot is False)

        if USE_MMWAVE:
            if (self.mmw.last_data_time is None or
                self.mmw.danger_raised or
                self.mmw.last_val is None or
                self.mmw.last_avg is None):
                mmw_safe = False
            else:
                # 현재값 <= 평균값일 때만 "안전"
                mmw_safe = (self.mmw.last_val <= self.mmw.last_avg)

        return therm_safe and mmw_safe

    def run(self):
        print("=== Pet Trigger Controller (PIR → mmWave + Thermal) ===")
        print(f"PIR={PIR_PIN_BCM}, MMW_PWR={MMW_PWR_PIN_BCM}, "
              f"THERM_PWR={THERM_PWR_PIN_BCM}, ACTIVE_HIGH={ACTIVE_HIGH_POWER}")
        print("대기 중… (PIR은 항상 켜져 있고, 감지 시 mmWave/열화상 ON)")

        try:
            while True:
                time.sleep(0.2)
                if self.sensors_on:
                    now = time.time()
                    safe = self._check_safe_state(now)

                    if safe:
                        if self.safe_start_time is None:
                            self.safe_start_time = now
                        elif now - self.safe_start_time >= INACTIVITY_SEC:
                            with self._lock:
                                if self.sensors_on:
                                    self._power_off_and_stop()
                    else:
                        # 한 번이라도 비안전 상태가 되면 타이머 리셋
                        self.safe_start_time = None

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
