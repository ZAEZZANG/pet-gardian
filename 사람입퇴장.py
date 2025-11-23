#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
room_monitor.py
- 위성 PIR에서 MQTT로 전달되는 door_cross 이벤트를 받아
  사람 입·퇴장 상태(person_in_room)와 완화 모드(relax)를 관리.
- 일체형 센서(mmWave + Thermal 등)의 데이터를 읽어와
  위험/경고/정상 상태를 판정하고,
  위험/경고 상태일 때만 알림(텔레그램 등)을 보냄.

⚠ 텔레그램 설정/토큰은 여기서 제거했고,
   send_telegram_message()는 별도 모듈(telegram.py)에서 import 하도록 변경.
"""

import time
import paho.mqtt.client as mqtt

# 👉 너가 따로 만들 파일: telegram.py
#    그 안에 send_telegram_message(text: str) 함수만 구현해두면 됨.
from telegram import send_telegram_message


# ===================================
# 🔵 1. 사람 출입 / 완화모드 변수
# ===================================
person_in_room = False
relax_until = 0
warning_start_time = None

RELAX_DURATION = 20 * 60        # 20분
MQTT_TOPIC_PERSON = "room1/person"

# MQTT 서버 (라즈베리파이 IP로 설정)
MQTT_SERVER = "RASPBERRY_PI_IP"  # ← 수정
MQTT_PORT = 1883


# ===================================
# 🔵 2. MQTT 콜백 — 위성 PIR 문통과 이벤트 처리
# ===================================
def on_message(client, userdata, msg):
    """
    위성 PIR(XIAO ESP32-C3)에서 오는 문 통과 이벤트 처리.
    payload == "door_cross" 일 때 입·퇴장을 토글.
    """
    global person_in_room, relax_until, warning_start_time

    payload = msg.payload.decode()
    now = time.time()

    if msg.topic == MQTT_TOPIC_PERSON and payload == "door_cross":
        # 토글: 입장 ↔ 퇴장
        person_in_room = not person_in_room
        warning_start_time = None

        if person_in_room:
            # 🔵 사람 입장 → 완화 모드 ON
            relax_until = now + RELAX_DURATION
            print("[위성PIR] 사람 입장 → 완화 모드 20분 활성화")
            send_telegram_message(
                "사람이 입장하였습니다. 지금부터 20분 동안은 경고 단계에서는 알리지 않고, "
                "위험 단계일 때만 알리겠습니다."
            )
        else:
            # 🔵 사람 퇴장 → 완화 모드 해제
            relax_until = 0
            print("[위성PIR] 사람 퇴장 → 정상 모드로 복귀")
            send_telegram_message(
                "사람이 퇴장하였습니다. 이제 정상 민감도로 모니터링합니다."
            )


# ===================================
# 🔵 3. 센서(일체형) 읽기 함수 — 나중에 실제 코드로 교체
# ===================================
def read_sensors():
    """
    TODO:
    - 일체형 PIR + mmWave + Thermal 융합 결과로부터
      호흡수, 체온 등 필요한 값을 읽어와 dict로 반환하도록 구현.

    지금은 테스트용 더미 값.
    """
    data = {
        "resp_rate": 24.0,   # 호흡수
        "body_temp": 38.3,   # 체온
    }
    return data


# ===================================
# 🔵 4. 이상 상태 판정 함수 (경고/위험)
# ===================================
def get_abnormal_level(sensor_data):
    rr = sensor_data["resp_rate"]
    temp = sensor_data["body_temp"]

    # 🚨 위험 조건
    if rr < 5 or rr > 40:
        return "danger"
    if temp >= 39.5:
        return "danger"

    # ⚠ 경고 조건
    if rr < 10 or rr > 30:
        return "warning"
    if temp > 39.0:
        return "warning"

    return "normal"


# ===================================
# 🔵 5. 메인 루프 (센서 읽고 → 알림)
# ===================================
def main_loop():
    global warning_start_time, person_in_room, relax_until

    CHECK_INTERVAL = 5  # 5초마다 센서 체크

    while True:
        now = time.time()

        # 센서 읽기
        data = read_sensors()
        level = get_abnormal_level(data)

        # 현재 완화모드인지 확인
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
            # 사람 없음 or 완화 모드 종료 → 바로 알림
            if not in_relax:
                msg = f"[경고]\n호흡수: {data['resp_rate']}\n체온: {data['body_temp']}"
                print(msg)
                send_telegram_message(msg)
                warning_start_time = None
            else:
                # 사람 있음 + 완화모드 → 5분 이상 지속 시 한 번 알림
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

        # 정상 상태 → 경고 타이머 초기화
        else:
            warning_start_time = None

        time.sleep(CHECK_INTERVAL)


# ===================================
# 🔵 6. MQTT 연결 및 실행 시작
# ===================================

def setup_mqtt():
    client = mqtt.Client()
    client.on_message = on_message

    client.connect(MQTT_SERVER, MQTT_PORT, 60)
    client.subscribe(MQTT_TOPIC_PERSON)
    client.loop_start()

    print("[MQTT] 연결 완료, 위성 PIR 이벤트 대기 중...")
    return client


if __name__ == "__main__":
    mqtt_client = setup_mqtt()
    try:
        main_loop()
    except KeyboardInterrupt:
        print("\n[room_monitor] 종료 요청됨")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
