# utils/telegram.py
import time
import requests

# =====================
# Telegram 설정
# =====================
BOT_TOKEN = "8149636240:AAGf4IQIQjrlr4i_RqBAFeKezAWKAxhtxeI"
CHAT_ID = "8074812404"

def send_alert(msg):
    """기본 텍스트 메시지 전송 함수 (main.py에서 호출됨)"""
    url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage"
    try:
        requests.post(url, data={"chat_id": CHAT_ID, "text": msg})
        print(f"[TELEGRAM] ✔ sent → {msg}")
    except Exception as e:
        print(f"[TELEGRAM] ❌ Failed to send alert: {e}")


# =====================
# 기준 값
# =====================
NORMAL_BREATH_MIN = 15
NORMAL_BREATH_MAX = 30
WARNING_BREATH_MIN = 10
WARNING_BREATH_MAX = 40

NORMAL_TEMP_MIN = 37.5
NORMAL_TEMP_MAX = 39.0
WARNING_TEMP_MAX = 39.4

# =====================
# 상태 추적 변수
# =====================
warning_started_time = None
CRITICAL_SENT = False
WARNING_SENT = False

# 얼마나 지속될 때 "지속 경고"를 보내는지 (초)
WARNING_DURATION = 5 * 60   # 5분


# =====================
# 평가 함수 (Main에서 사용)
# =====================
def evaluate_sensor(bpm, temperature):
    """
    센서 상태 평가:
    - 정상 NORMAL
    - 경고 WARNING
    - 위험 CRITICAL
    내부에서 자동으로 send_alert를 호출함
    """
    global warning_started_time, CRITICAL_SENT, WARNING_SENT

    now = time.time()

    # -------------------------------
    # 🚨 즉시 위험 알림 조건
    # -------------------------------
    if bpm < WARNING_BREATH_MIN or bpm > WARNING_BREATH_MAX or temperature > WARNING_TEMP_MAX:
        if not CRITICAL_SENT:
            send_alert(f"🚨 긴급 이상 감지!\n호흡수: {bpm} BPM\n체온: {temperature}°C")
            CRITICAL_SENT = True
        warning_started_time = None
        return "CRITICAL"

    # -------------------------------
    # ⚠ 경고 조건 (지속 시 알림)
    # -------------------------------
    if (bpm < NORMAL_BREATH_MIN or bpm > NORMAL_BREATH_MAX or 
        temperature > NORMAL_TEMP_MAX):

        if warning_started_time is None:
            warning_started_time = now
            print("[STATUS] ⚠ 경계 상태 시작...")

        elif now - warning_started_time >= WARNING_DURATION and not WARNING_SENT:
            send_alert(
                f"⚠️ 지속적 이상 감지 (5분 유지됨)\n"
                f"호흡수: {bpm} BPM\n체온: {temperature}°C"
            )
            WARNING_SENT = True

        return "WARNING"

    # -------------------------------
    # 🟢 정상 상태 복귀
    # -------------------------------
    warning_started_time = None
    CRITICAL_SENT = False
    WARNING_SENT = False
    return "NORMAL"
