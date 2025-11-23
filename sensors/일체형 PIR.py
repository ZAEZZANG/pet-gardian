import RPi.GPIO as GPIO
import time

# ============================================
# 🔵 1. 핀 번호 & 상태 변수 설정
# ============================================

# PIR 센서가 연결된 라즈베리파이 GPIO 번호 (BCM 기준)
# 예: GPIO17 에 연결했다면 17, 실제 연결한 핀 번호로 바꿔줘야 함!
PIR_PIN = 17   

# mmWave / 열화상 센서의 "전원 상태"를 기억하는 변수
mmwave_on = False      # 현재 mmWave가 켜져 있는지 여부
thermal_on = False     # 현재 열화상 카메라가 켜져 있는지 여부

# PIR이 움직임을 감지하면, 그 시점부터 최소 얼마 동안 센서를 켜둘지
sensor_active_until = 0           # 이 시각까지는 센서를 켜두자 (유닉스 타임스탬프)
SENSOR_ACTIVE_TIME = 60           # 움직임 감지 후 60초 동안은 ON 유지 (초 단위)


# ============================================
# 🔵 2. 센서 전원 ON/OFF 함수
#     - 지금은 print만 찍고,
#       나중에 실제 전원 제어 핀 / 초기화 코드 넣으면 됨
# ============================================

def mmwave_power_on():
    """
    mmWave 센서를 켜는 함수.
    - 실제로는 전원 인가, 초기화 코드 등이 들어갈 자리.
    """
    global mmwave_on
    if not mmwave_on:
        print("[일체형 PIR] mmWave ON")
        # TODO: 👉 여기서 실제 mmWave 전원 ON / 초기화 코드 넣기
        # 예시:
        # GPIO.output(MMWAVE_EN_PIN, GPIO.HIGH)
        mmwave_on = True

def mmwave_power_off():
    """
    mmWave 센서를 끄는 함수.
    - 저전력 모드, 전원 차단 등 구현 가능.
    """
    global mmwave_on
    if mmwave_on:
        print("[일체형 PIR] mmWave OFF")
        # TODO: 👉 여기서 실제 mmWave 전원 OFF 코드 넣기
        # 예시:
        # GPIO.output(MMWAVE_EN_PIN, GPIO.LOW)
        mmwave_on = False

def thermal_power_on():
    """
    열화상 카메라를 켜는 함수.
    - I2C 초기화, 라이브러리 인스턴스 생성 등을 여기서 해도 됨.
    """
    global thermal_on
    if not thermal_on:
        print("[일체형 PIR] Thermal ON")
        # TODO: 👉 여기서 실제 열화상 카메라 ON / 초기화 코드 넣기
        thermal_on = True

def thermal_power_off():
    """
    열화상 카메라를 끄는 함수.
    - 필요하다면 전원 차단, 리소스 정리 등 수행.
    """
    global thermal_on
    if thermal_on:
        print("[일체형 PIR] Thermal OFF")
        # TODO: 👉 여기서 실제 열화상 카메라 OFF 코드 넣기
        thermal_on = False

def wake_sensors():
    """
    ✅ 일체형 PIR이 '무언가 움직임'을 감지했을 때 호출되는 함수.
       강아지든 사람이든 상관없이:
       - mmWave 센서 ON
       - 열화상 카메라 ON
       - 그리고 SENSOR_ACTIVE_TIME 동안은 최소한 켜두도록 시간 갱신
    """
    global sensor_active_until
    now = time.time()
    sensor_active_until = now + SENSOR_ACTIVE_TIME  # 지금 시각 + 60초

    mmwave_power_on()
    thermal_power_on()


# ============================================
# 🔵 3. PIR 콜백 & GPIO 초기 설정
# ============================================

def pir_callback(channel):
    """
    🟡 일체형 PIR이 '움직임 감지'했을 때 자동으로 호출되는 함수.
       - 이 코드는 위치 상관 없음 (옆 벽 중앙, 허브 앞…)
       - '시야 안에서 무언가 움직임 감지됨' → 센서 깨우기
    """
    print("[일체형 PIR] 움직임 감지 → mmWave + Thermal 깨우기")
    wake_sensors()

def setup_pir():
    """
    프로그램 시작 시, 딱 한 번만 호출해서
    - GPIO 모드 설정
    - PIR 핀 입력으로 설정
    - RISING 엣지(LOW→HIGH) 감지 시 pir_callback 호출하도록 등록
    """
    GPIO.setmode(GPIO.BCM)  # BCM 모드 사용 (핀 번호를 GPIO 번호로 씀)
    GPIO.setup(PIR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # RISING: LOW -> HIGH로 바뀌는 순간만 감지
    # bouncetime=200: 200ms 안에 튀는 신호는 무시 (디바운스)
    GPIO.add_event_detect(PIR_PIN, GPIO.RISING,
                          callback=pir_callback, bouncetime=200)
    print("[일체형 PIR] 설정 완료 (옆 벽 중앙에서 움직임 감지 대기 중)")


# ============================================
# 🔵 4. 센서 전원 유지 / OFF 관리
# ============================================

def update_sensor_power():
    """
    메인 루프에서 주기적으로 호출해서,
    - sensor_active_until 시간이 지났다면
      mmWave와 열화상 센서를 OFF 시켜주는 함수.

    👉 요약:
       - 최근에 움직임 감지한 뒤 60초 동안은 ON 유지
       - 그 이후로는 다시 OFF해서 전력 절약
    """
    now = time.time()
    if now > sensor_active_until:
        mmwave_power_off()
        thermal_power_off()
