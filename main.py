#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🔥 Integrated Pet Health Monitor (Main Controller)

- PIR: Motion trigger
- mmWave: Breathing + activity
- Thermal: Temperature + size detection

Logic:
✔ System wakes when PIR detects movement OR stable breathing exists
✔ If no motion + no breathing change → enters LOW POWER mode
"""

import time
from datetime import datetime

# ---------------- SENSOR IMPORTS ----------------
from sensors.mmwave import MmWave
from sensors.thermal import ThermalCamera  # (placeholder if not ready)
from sensors.pir import PirSensor

# ---------------- SIGNAL PROCESSING ----------------
from utils.signal_processing import (
    estimate_breathing_rate,
    compute_motion_level,
    detect_sleep
)



# ---------------- USER SETTINGS ----------------
NO_ACTIVITY_TIMEOUT = 120   # 2 min
BREATH_STILL_THRESHOLD = 5
TEMP_WARNING_THRESHOLD = 39.5  # °C (Dog fever threshold)
PET_TYPE = "dog"



# ---------------- STATE VARIABLES ----------------
system_active = False
last_activity = time.time()

bpm_history = []
motion_history = []
temperature_history = []



# ---------------- SENSOR INSTANCES ----------------
mmwave = MmWave(port="/dev/ttyAMA10")  
thermal = ThermalCamera()  
pir = PirSensor(pin=17)



# ---------------- POWER CONTROL ----------------
def wake_system():
    global system_active, last_activity
    if not system_active:
        print("\n🐶 Wake event → Sensors powering ON...")
        mmwave.on()
        thermal.on()
        system_active = True
    last_activity = time.time()


def sleep_system():
    global system_active
    if system_active:
        print("\n😴 No activity for long → Sensors OFF (Low Power Mode)")
        mmwave.off()
        thermal.off()
        system_active = False



# ---------------- PIR CALLBACK ----------------
def on_motion_detected():
    print("📍 PIR detected movement")
    wake_system()

pir.set_callback(on_motion_detected)



# ---------------- MAIN LOOP ----------------
def main():
    global last_activity, system_active

    print("\n========== PET HEALTH MONITOR RUNNING ==========")
    print("Waiting for motion...\n")

    while True:
        now = time.time()

        if system_active:

            # -------- mmWave DATA --------
            breath_raw, motion_raw = mmwave.read()

            if motion_raw is not None:
                motion_history.append(motion_raw)

            if breath_raw is not None:
                bpm = estimate_breathing_rate(mmwave.breath_signal, fs=10)
                if bpm:
                    bpm_history.append(bpm)
                    print(f"🌬 Breathing: {bpm} BPM")

                    if PET_TYPE == "dog":
                        if bpm > 40: print("⚠ Possible rapid breathing (tachypnea)")

            # -------- Thermal DATA --------
            temp, size = thermal.read()
            if temp:
                temperature_history.append(temp)
                print(f"🌡 Temp: {temp:.1f}°C | Size pixels: {size}")

                if temp > TEMP_WARNING_THRESHOLD:
                    print("🚨 Fever suspected!")

            # -------- Sleep Detection --------
            sleeping = detect_sleep(bpm_history, motion_history)

            if sleeping:
                print("💤 Sleeping (No shutdown during stable sleep)")
                last_activity = now

            # -------- Activity Timeout --------
            if (now - last_activity) > NO_ACTIVITY_TIMEOUT and not sleeping:
                sleep_system()

        else:
            print("📴 Idle - waiting...")
            time.sleep(1)

        time.sleep(0.8)



# ---------------- ENTRY POINT ----------------
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sleep_system()
        print("\nProgram stopped safely.\n")
