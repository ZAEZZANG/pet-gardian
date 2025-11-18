import time
import paho.mqtt.client as mqtt
import requests

# ===================================
# 🔵 1. 텔레그램 설정
# ===================================
BOT_TOKEN = "YOUR_TELEGRAM_TOKEN"     # ← 수정
CHAT_ID   = "YOUR_CHAT_ID"            # ← 수정

def send_telegram_message(text: str):
    url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage"
    data = {"chat_id": CHAT_ID, "text": text}
    try:
        requests.post(url, data=data, timeout=3)
    except:
        print("[텔레그램 오류]")


# ===================================
# 🔵 2. 사람 출입 / 완화모드 변수
# ===================================
person_in_room = False
relax_until = 0
warning_start_time = None

RELAX_DURATION = 20 * 60   # 20분
MQTT_TOPIC_PERSON = "room1/person"


# ===================================
# 🔵 3. MQTT 콜백 — 위성 PIR 문통과 이벤트 처리
# ===================================
def on_message(client, userdata, msg):
    global person_in_room, relax_until, warning_start_time

    payload = msg.payload.decode()
    now = time.time()

    if msg.topic == MQTT_TOPIC_PERSON:
        if payload == "door_cross":

            # 토글: 입장↔퇴장
            person_in_room = not person_in_room
            warning_start_time = None

            if person_in_room:
                # 🔵 사람 입장
                relax_until = now + RELAX_DURATION
                print("[위성PIR] 사람 입장")
                send_telegram_message(
                    "사람이 입장하였습니다. 지금부터 20분 동안은 경고 단계에서는 알리지 않고, "
                    "위험 단계일 때만 알리겠습니다."
                )
            else:
                # 🔵 사람 퇴장
                relax_until = 0
                print("[위성PIR] 사람 퇴장")
                send_telegram_message(
                    "사람이 퇴장하였습니다. 이제 정상 민감도로 모니터링합니다."
                )


# ===================================
# 🔵 4. 센서(일체형 PIR + mmWave + 열화상) 읽기 함수
#   → 나중에 네 센서 코드 넣으면 됨!
# ===================================
def read_sensors():
    """
    이 부분은 나중에:
    - 일체형 PIR 감지 여부
    - mmWave에서 호흡수/심박/움직임
    - 열화상 카메라에서 온도 추출
    전부 합쳐서 return 하면 됨.
    지금은 테스트용 더미 값.
    """
    data = {
        "resp_rate": 24.0,   # 호흡수
        "body_temp": 38.3,   # 체온
    }
    return data


# ===================================
# 🔵 5. 이상 상태 판정 함수 (경고/위험)
# ===================================
def get_abnormal_level(sensor_data):
    rr = sensor_data["resp_rate"]
    temp = sensor_data["body_temp"]

    # 위험
    if rr < 5 or rr > 40:
        return "danger"
    if temp >= 39.5:
        return "danger"

    # 경고
    if rr < 10 or rr > 30:
        return "warning"
    if temp > 39.0:
        return "warning"

    return "normal"


# ===================================
# 🔵 6. 메인 루프 (센서 읽고 → 알림)
# ===================================
def main_loop():
    global warning_start_time

    CHECK_INTERVAL = 5  # 5초 반복

    while True:
        now = time.time()

        # 센서 읽기
        data = read_sensors()
        level = get_abnormal_level(data)

        # 현재 완화모드인가?
        in_relax = person_in_room and (now < relax_until)

        # ===============================
        # 🚨 위험 상태 → 무조건 알림
        # ===============================
        if level == "danger":
            msg = f"[위험]\n호흡수: {data['resp_rate']}\n체온: {data['body_temp']}"
            print(msg)
            send_telegram_message(msg)
            warning_start_time = None

        # ===============================
        # ⚠ 경고 상태 처리
        # ===============================
        elif level == "warning":

            # 사람 없음 → 바로 알림
            if not in_relax:
                msg = f"[경고]\n호흡수: {data['resp_rate']}\n체온: {data['body_temp']}"
                print(msg)
                send_telegram_message(msg)
                warning_start_time = None

            else:
                # 사람 있음 + 완화모드 → 5분 지속되면 알림
                if warning_start_time is None:
                    warning_start_time = now

                if now - warning_start_time >= 5 * 60:
                    msg = (
                        "[경고-사람 동반 상태]\n"
                        f"호흡수: {data['resp_rate']}\n"
                        f"체온: {data['body_temp']}\n"
                        "경고 상태가 5분 넘게 지속되고 있습니다."
                    )
                    print(msg)
                    send_telegram_message(msg)
                    warning_start_time = None

        # 정상 상태 → 초기화
        else:
            warning_start_time = None

        time.sleep(CHECK_INTERVAL)


# ===================================
# 🔵 7. MQTT 연결
# ===================================
client = mqtt.Client()
client.on_message = on_message

client.connect("RASPBERRY_PI_IP", 1883, 60)   # ← 수정
client.subscribe(MQTT_TOPIC_PERSON)
client.loop_start()

# ===================================
# 🔵 실행 시작
# ===================================
main_loop()
