# sensors/integrated_pir.py

import RPi.GPIO as GPIO
import time

PIR_PIN = 17

mmwave_on = False
thermal_on = False

sensor_active_until = 0
SENSOR_ACTIVE_TIME = 60  # 움직임 감지 후 60초 유지


def mmwave_power_on():
    global mmwave_on
    if not mmwave_on:
        print("[PIR] mmWave ON")
        mmwave_on = True


def mmwave_power_off():
    global mmwave_on
    if mmwave_on:
        print("[PIR] mmWave OFF")
        mmwave_on = False


def thermal_power_on():
    global thermal_on
    if not thermal_on:
        print("[PIR] Thermal ON")
        thermal_on = True


def thermal_power_off():
    global thermal_on
    if thermal_on:
        print("[PIR] Thermal OFF")
        thermal_on = False


def wake_sensors():
    global sensor_active_until
    now = time.time()
    sensor_active_until = now + SENSOR_ACTIVE_TIME

    mmwave_power_on()
    thermal_power_on()


def pir_callback(channel):
    print("[PIR] 움직임 감지 → 센서 깨우기")
    wake_sensors()


def setup_pir():
    """🔵 main.py에서 처음 딱 1번 호출됨"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(PIR_PIN, GPIO.RISING, callback=pir_callback, bouncetime=200)
    print("[PIR] 설정 완료 (대기 중)")


def update_sensor_power():
    """🔵 main.py 루프에서 계속 호출됨"""
    now = time.time()
    if now > sensor_active_until:
        mmwave_power_off()
        thermal_power_off()
