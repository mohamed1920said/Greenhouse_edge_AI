import csv
import time
from datetime import datetime
import requests

ESP32_IP = "172.0.0.4"          # <-- change this
URL = f"http://{ESP32_IP}/api"
CSV_FILE = "greenhouse_log.csv"
INTERVAL_SEC = 2                    # poll every 2 seconds

FIELDS = [
    "timestamp",
    "tempC",
    "hum",
    "moist",
    "lux",
    "temp_state",
    "heater_fan",
    "fan",
    "water_pump",
    "humidifiers",
    "grow_lights",
]

def flatten(payload: dict) -> dict:
    act = payload.get("act", {})
    return {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "tempC": payload.get("tempC"),
        "hum": payload.get("hum"),
        "moist": payload.get("moist"),
        "lux": payload.get("lux"),
        "temp_state": payload.get("temp_state"),
        "heater_fan": act.get("heater_fan"),
        "fan": act.get("fan"),
        "water_pump": act.get("water_pump"),
        "humidifiers": act.get("humidifiers"),
        "grow_lights": act.get("grow_lights"),
    }

def ensure_header():
    try:
        with open(CSV_FILE, "r", newline="") as f:
            pass
    except FileNotFoundError:
        with open(CSV_FILE, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=FIELDS)
            w.writeheader()

def main():
    ensure_header()
    print(f"Logging {URL} -> {CSV_FILE} every {INTERVAL_SEC}s. Ctrl+C to stop.")
    while True:
        try:
            r = requests.get(URL, timeout=3)
            r.raise_for_status()
            payload = r.json()
            row = flatten(payload)

            with open(CSV_FILE, "a", newline="") as f:
                w = csv.DictWriter(f, fieldnames=FIELDS)
                w.writerow(row)

            print(row)
        except Exception as e:
            print("Error:", e)

        time.sleep(INTERVAL_SEC)

if __name__ == "__main__":
    main()