
from dataclasses import dataclass
from typing import Optional, Dict

from config import MAX_SAFE_PITCH, MAX_SAFE_ROLL


@dataclass
class SensorData:
    pitch: float = 0.0
    roll: float = 0.0
    front_ultrasonic: float = 0.0
    rear_ultrasonic: float = 0.0

    def is_safe_angle(self) -> bool:
        return abs(self.pitch) <= MAX_SAFE_PITCH and abs(self.roll) <= MAX_SAFE_ROLL

    def is_front_close(self, limit_cm: float = 8.0) -> bool:
        return self.front_ultrasonic > 0 and self.front_ultrasonic <= limit_cm

    def is_rear_close(self, limit_cm: float = 8.0) -> bool:
        return self.rear_ultrasonic > 0 and self.rear_ultrasonic <= limit_cm


def parse_sensor_data(response: str) -> Optional[SensorData]:
    """
    Expected Mega response:
    DATA:PITCH=5.2;ROLL=-1.4;UF=8.5;UR=9.1
    """

    if not response.startswith("DATA:"):
        return None

    try:
        response = response.replace("DATA:", "")
        parts = response.split(";")

        values: Dict[str, float] = {}

        for part in parts:
            if "=" not in part:
                continue

            key, value = part.split("=")
            values[key.strip().upper()] = float(value.strip())

        return SensorData(
            pitch=values.get("PITCH", 0.0),
            roll=values.get("ROLL", 0.0),
            front_ultrasonic=values.get("UF", 0.0),
            rear_ultrasonic=values.get("UR", 0.0)
        )

    except Exception as e:
        print("[ERROR] Failed to parse sensor data")
        print(e)
        return None

