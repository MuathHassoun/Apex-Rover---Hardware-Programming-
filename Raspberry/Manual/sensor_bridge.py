import time
import serial
import requests

# ==================================================
# Apex Rover Sensor Bridge
#
# Flow:
#   Arduino Mega -> Raspberry Pi -> ESP32 -> Mobile App
#
# Mega sends automatically:
#   SENSOR:PITCH=2.30;ROLL=-1.10;UF=8.50;UR=9.20;ALERT=NONE
#
# Raspberry sends to ESP32:
#   http://192.168.4.1/sensor_update?pitch=...&roll=...&front=...&rear=...&balance=...
#
# Raspberry Pi DOES NOT control robot movement.
# ==================================================

ESP32_URL = "http://192.168.4.1"
MEGA_PORT = "/dev/ttyUSB0"

BAUD_RATE = 9600
SERIAL_TIMEOUT = 0.5
FORWARD_TIMEOUT = 0.5

def compute_balance_status(pitch, roll, alert):
    abs_pitch = abs(pitch)
    abs_roll = abs(roll)

    if alert == "TILT_DANGER":
        return "DANGER"

    if alert in ("OBSTACLE_FRONT", "OBSTACLE_REAR"):
        return "WARNING"

    if abs_pitch >= 30 or abs_roll >= 25:
        return "DANGER"

    if abs_pitch >= 15 or abs_roll >= 12:
        return "WARNING"

    return "STABLE"


def parse_sensor_line(line):
    line = line.strip()

    if not line.startswith("SENSOR:"):
        return None

    payload = line[len("SENSOR:"):]
    parts = payload.split(";")

    data = {}

    for part in parts:
        if "=" not in part:
            continue

        key, value = part.split("=", 1)
        key = key.strip().upper()
        value = value.strip()
        data[key] = value

    try:
        pitch = float(data.get("PITCH", 0.0))
        roll = float(data.get("ROLL", 0.0))

        # Mega may send UF/UR or FRONT/REAR
        front = float(data.get("UF", data.get("FRONT", -1.0)))
        rear = float(data.get("UR", data.get("REAR", -1.0)))

        alert = data.get("ALERT", "NONE").upper()

        balance = compute_balance_status(pitch, roll, alert)

        return {
            "pitch": pitch,
            "roll": roll,
            "front": front,
            "rear": rear,
            "balance": balance,
            "alert": alert,
        }

    except ValueError:
        print("[WARN] Bad SENSOR values:", line)
        return None


def send_to_esp32(sensor_data):
    try:
        response = requests.get(
            f"{ESP32_URL}/sensor_update",
            params={
                "pitch": f"{sensor_data['pitch']:.2f}",
                "roll": f"{sensor_data['roll']:.2f}",
                "front": f"{sensor_data['front']:.2f}",
                "rear": f"{sensor_data['rear']:.2f}",
                "balance": sensor_data["balance"],
            },
            timeout=FORWARD_TIMEOUT,
        )

        print("[ESP32]", response.text)

    except requests.RequestException as e:
        print("[ERROR] Could not send to ESP32:", e)


def main():
    print("====================================")
    print("Apex Rover Sensor Bridge")
    print(f"Mega Port : {MEGA_PORT}")
    print(f"ESP32 URL : {ESP32_URL}")
    print("====================================")

    while True:
        try:
            with serial.Serial(MEGA_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT) as mega:
                time.sleep(2)
                mega.reset_input_buffer()

                print("[OK] Connected to Mega")
                print("[INFO] Waiting for SENSOR lines from Mega...")

                while True:
                    raw = mega.readline()

                    if not raw:
                        continue

                    line = raw.decode(errors="ignore").strip()

                    if line:
                        print("[MEGA]", line)

                    sensor_data = parse_sensor_line(line)

                    if sensor_data is not None:
                        print("[SENSOR]", sensor_data)
                        send_to_esp32(sensor_data)

        except serial.SerialException as e:
            print("[ERROR] Mega serial problem:", e)
            print("[INFO] Retrying in 2 seconds...")
            time.sleep(2)


if __name__ == "__main__":
    main()
