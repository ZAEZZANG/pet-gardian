"""
thermal.py
- MLX90640 열화상 카메라 전용 모듈
- 배경 온도 대비 '핫 픽셀' 개수를 세고, 그 크기로
  none / pet / human_or_large 를 분류하는 기능 담당

필요 라이브러리(라즈베리파이에서 한 번만 설치):
    pip3 install adafruit-circuitpython-mlx90640
"""

import time

try:
    import board
    import busio
    import adafruit_mlx90640
    THERMAL_AVAILABLE = True
except ImportError:
    # PC에서 개발할 때 ImportError 나도 죽지 않게 처리
    THERMAL_AVAILABLE = False
    board = None
    busio = None
    adafruit_mlx90640 = None

# ==========================
# CONFIG (나중에 실험하면서 튜닝)
# ==========================

# 온도 기준
HOT_ABS_TEMP = 28.0          # 이 온도 이상이면 "따뜻한 물체" 후보
HOT_DELTA_TEMP = 4.0         # 배경보다 +4°C 이상이면 핫 픽셀로 취급

# 픽셀 수 기준 (대략값, 실제로 강아지/사람 찍어서 조정 필수!)
PET_PIXEL_MIN = 15           # 이 이상이면 "작은 동물(소형견/고양이)" 후보
PET_PIXEL_MAX = 250          # 이 이하까지만 반려동물로 봄
HUMAN_PIXEL_MIN = 300        # 이 이상이면 사람/큰 물체로 판단

# ==========================
# 내부 상태
# ==========================

mlx = None          # 센서 객체
mlx_frame = [0.0] * 768   # 24x32 프레임 버퍼 (1D 배열)


# ==========================
# 초기화 함수
# ==========================

def init_thermal():
    """
    MLX90640 초기화.
    - 성공: True 리턴
    - 실패 또는 PC 환경 등: False 리턴
    """
    global mlx

    if not THERMAL_AVAILABLE:
        print("[THERMAL] adafruit_mlx90640 라이브러리를 찾을 수 없습니다. (PC 환경일 수 있음)")
        return False

    if mlx is not None:
        # 이미 초기화 된 경우
        return True

    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        sensor = adafruit_mlx90640.MLX90640(i2c)
        sensor.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ
        mlx = sensor
        print("[THERMAL] MLX90640 초기화 완료.")
        return True
    except Exception as e:
        print(f"[THERMAL] 초기화 실패: {e}")
        return False


# ==========================
# 메인 측정 함수
# ==========================

def get_thermal_readings():
    """
    MLX90640 한 프레임 읽어서:
      1) 전체 픽셀에서 배경 온도(중앙값) 추정
      2) 배경 + HOT_DELTA_TEMP 이상 & HOT_ABS_TEMP 이상인 픽셀을 '핫 픽셀'로 카운트
      3) 핫 픽셀들의 평균 온도를 hot_region_temp로 리턴

    Returns:
        (hot_region_temp: float, hot_pixel_count: int)

    ※ main 코드에서는 예를 들어:
        temp, pixels = get_thermal_readings()
        entity = classify_thermal_entity(pixels)
    이런 식으로 사용하면 됨.
    """
    global mlx, mlx_frame

    if mlx is None:
        # 아직 초기화 안 된 경우
        return 0.0, 0

    try:
        mlx.getFrame(mlx_frame)
    except Exception as e:
        print(f"[THERMAL] MLX read error: {e}")
        return 0.0, 0

    temps = list(mlx_frame)

    # 1) 배경 온도 추정: 중앙값(median) 사용
    temps_sorted = sorted(temps)
    mid = len(temps_sorted) // 2
    if len(temps_sorted) % 2 == 0:
        background_temp = (temps_sorted[mid - 1] + temps_sorted[mid]) / 2.0
    else:
        background_temp = temps_sorted[mid]

    max_temp = max(temps)

    # 2) 핫 픽셀 기준 온도
    #    - 절대 기준(HOT_ABS_TEMP)
    #    - 배경 + DELTA 기준
    #    둘 중 더 높은 값을 사용
    threshold = max(HOT_ABS_TEMP, background_temp + HOT_DELTA_TEMP)

    # 3) threshold 이상인 픽셀만 hot 픽셀로 카운트
    hot_pixels = [t for t in temps if t >= threshold]
    hot_pixel_count = len(hot_pixels)

    if hot_pixels:
        hot_region_temp = sum(hot_pixels) / len(hot_pixels)
    else:
        hot_region_temp = max_temp  # 핫 픽셀이 없으면 그냥 전체 최대 온도 기준

    return hot_region_temp, hot_pixel_count


# ==========================
# 반려동물/사람 분류 함수
# ==========================

def classify_thermal_entity(hot_pixel_count: int) -> str:
    """
    핫 픽셀 개수로 대략적인 대상 분류:

    Returns:
        "none"           : 따뜻한 물체 거의 없음
        "pet"            : 반려동물(소형견/고양이) 정도 크기
        "human_or_large" : 사람/큰 물체 수준
        "unknown"        : 애매한 영역 (튜닝 필요)

    ※ 실제로는 mmWave에서 거리/호흡/크기 정보까지 합쳐서
       최종 판단하는 게 좋고, thermal 쪽은 '크기 힌트' 역할.
    """
    if hot_pixel_count < PET_PIXEL_MIN:
        return "none"
    elif PET_PIXEL_MIN <= hot_pixel_count <= PET_PIXEL_MAX:
        return "pet"
    elif hot_pixel_count >= HUMAN_PIXEL_MIN:
        return "human_or_large"
    else:
        return "unknown"


# ==========================
# 단독 테스트용 (라즈베리파이에서만)
# ==========================

if __name__ == "__main__":
    print("=== THERMAL MODULE STANDALONE TEST ===")
    ok = init_thermal()
    if not ok:
        print("열화상 센서 초기화 실패. 라즈베리파이 + MLX90640 환경인지 확인하세요.")
    else:
        try:
            while True:
                temp, pixels = get_thermal_readings()
                entity = classify_thermal_entity(pixels)
                print(f"Temp={temp:.1f}°C, hot_pixels={pixels}, entity={entity}")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n테스트 종료.")

