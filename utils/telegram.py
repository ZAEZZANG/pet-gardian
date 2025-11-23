#텔레그램 알림
import time
import requests

# =====================
# Telegram 설정
# =====================
BOT_TOKEN = "8149636240:AAGf4IQIQjrlr4i_RqBAFeKezAWKAxhtxeI"
CHAT_ID = "8074812404"

def send_alert(msg):
    url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage"
    try:
        requests.post(url, data={"chat_id": CHAT_ID, "text": msg})
        print(f"[TELEGRAM] ✔ sent → {msg}")
    except:
        print("[TELEGRAM] ❌ Failed to send alert")


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
# 평가 함수
# =====================
def evaluate_sensor(bpm, temperature):
    global warning_started_time, CRITICAL_SENT, WARNING_SENT

    now = time.time()

    # -------------------------------
    # 🚨 즉시 위험 알림 조건
    # -------------------------------
    if bpm < WARNING_BREATH_MIN or bpm > WARNING_BREATH_MAX or temperature > WARNING_TEMP_MAX:
        if not CRITICAL_SENT:
            send_alert(f"⚠️ 긴급 이상 감지!\n호흡수: {bpm} BPM\n체온: {temperature}°C")
            CRITICAL_SENT = True
        warning_started_time = None
        return "CRITICAL"

    # -------------------------------
    # ⚠ 경계 상태 (지속되면 알림)
    # -------------------------------
    if (bpm < NORMAL_BREATH_MIN or bpm > NORMAL_BREATH_MAX or 
        temperature > NORMAL_TEMP_MAX):

        if warning_started_time is None:
            warning_started_time = now
            print("[STATUS] ⚠ 경계 상태 시작...")

        elif now - warning_started_time >= WARNING_DURATION and not WARNING_SENT:
            send_alert(f"⚠️ 지속적 이상 감지 (5분 이상)\n호흡수: {bpm} BPM\n체온: {temperature}°C")
            WARNING_SENT = True

        return "WARNING"

    # -------------------------------
    # ✅ 정상 상태 복귀
    # -------------------------------
    warning_started_time = None
    CRITICAL_SENT = False
    WARNING_SENT = False
    return "NORMAL"

import requests
import json

BOT_TOKEN = "8149636240:AAGf4IQIQjrlr4i_RqBAFeKezAWKAxhtxeI"
CHAT_ID = "8074812404"

def send_alert(bpm, temp, severity):
    """
    severity: 'normal', 'warning', 'danger'
    """

    status_icons = {
        "normal": "🟢 정상",
        "warning": "⚠ 경고",
        "danger": "🚨 긴급"
    }

    # 텍스트 메시지
    text = f"""
🐶 **반려동물 상태 감지됨**

상태: {status_icons.get(severity, "❓ Unknown")}

📌 호흡수: {bpm} BPM
📌 체온: {temp}°C

👉 확인이 필요합니다.
"""

    # 버튼 UI 구성
    keyboard = {
        "inline_keyboard": [
            [
                {"text": "📝 기록 보기", "callback_data": "SHOW_LOG"},
                {"text": "👀 상태 확인", "callback_data": "ACKNOWLEDGE"},
                {"text": "⛔ 알림 무시", "callback_data": "IGNORE"}
            ]
        ]
    }

    payload = {
        "chat_id": CHAT_ID,
        "text": text,
        "reply_markup": json.dumps(keyboard),
        "parse_mode": "Markdown"
    }

    url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage"

    try:
        r = requests.post(url, data=payload)
        print("[TELEGRAM] ✔ 알림 전송됨")
    except Exception as e:
        print("[TELEGRAM] ❌ 전송 실패:", e)
