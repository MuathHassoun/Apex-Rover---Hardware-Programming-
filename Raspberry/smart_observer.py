
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any

from sensors import SensorData
from config import (
    SMART_PROGRESS_MIN_PITCH_DELTA,
    SMART_PROGRESS_MIN_ROLL_DELTA,
    SMART_PROGRESS_MIN_CENTER_DELTA,
    SMART_PROGRESS_MIN_YELLOW_AREA_DELTA,
    SMART_SLIP_ROLL_DELTA,
    SMART_SLIP_PITCH_DROP,
    JACK_EFFECT_MIN_PITCH_DELTA,
    JACK_EFFECT_MIN_ROLL_DELTA,
    JACK_EFFECT_DETECT_TIMEOUT_SEC,
    USE_ULTRASONIC_AS_HELPER,
    ULTRASONIC_MIN_DELTA_CM,
    ULTRASONIC_MIN_RATE_CM_PER_SEC,
    MPU_PROGRESS_WEIGHT,
    CAMERA_PROGRESS_WEIGHT,
    ULTRASONIC_PROGRESS_WEIGHT,
    SMART_PROGRESS_SCORE_THRESHOLD,
    MPU_JACK_EFFECT_WEIGHT,
    ULTRASONIC_JACK_EFFECT_WEIGHT,
    JACK_EFFECT_SCORE_THRESHOLD,
    WEAK_MPU_JACK_EFFECT_PITCH_DELTA,
    WEAK_MPU_JACK_EFFECT_ROLL_DELTA,
)


@dataclass
class MotionSnapshot:
    time: float
    pitch: float
    roll: float
    uf_ground: float
    ur_ground: float
    path_center_x: Optional[float]
    yellow_area: float
    confidence: float
    tracks_found: bool


class SmartMotionObserver:
    """
    Smart feedback observer for Apex Rover stairs mode.

    V9 rules:
    - MPU6500 pitch/roll is the main decision source.
    - Camera/yellow-track changes are the second main source.
    - Ground-facing ultrasonic values stay enabled, but only as low-priority helper clues.
    - Ultrasonic must pass a minimum delta/rate filter before it counts.
    - Ultrasonic values alone must NOT decide progress or jack success.
    """

    def __init__(self):
        self.state_name = "NONE"
        self.state_start_snapshot: Optional[MotionSnapshot] = None
        self.last_snapshot: Optional[MotionSnapshot] = None
        self.current_snapshot: Optional[MotionSnapshot] = None
        self.last_update_time = 0.0

    def reset(self):
        self.state_name = "NONE"
        self.state_start_snapshot = None
        self.last_snapshot = None
        self.current_snapshot = None
        self.last_update_time = 0.0

    def _snapshot(self, stairs: Optional[Dict[str, Any]], data: SensorData) -> MotionSnapshot:
        stairs = stairs or {}
        left_area = float(stairs.get("left_yellow_area") or 0.0)
        right_area = float(stairs.get("right_yellow_area") or 0.0)
        yellow_area = float(stairs.get("yellow_area") or (left_area + right_area))

        return MotionSnapshot(
            time=time.time(),
            pitch=float(data.pitch),
            roll=float(data.roll),
            uf_ground=float(data.front_ultrasonic),
            ur_ground=float(data.rear_ultrasonic),
            path_center_x=stairs.get("path_center_x") or stairs.get("center_x"),
            yellow_area=yellow_area,
            confidence=float(stairs.get("confidence") or 0.0),
            tracks_found=bool(stairs.get("yellow_tracks_found") or stairs.get("stairs_found")),
        )

    def mark_state_start(self, state_name: str, stairs: Optional[Dict[str, Any]], data: SensorData):
        self.state_name = state_name
        snap = self._snapshot(stairs, data)
        self.state_start_snapshot = snap
        self.last_snapshot = snap
        self.current_snapshot = snap
        self.last_update_time = snap.time

    def update(self, stairs: Optional[Dict[str, Any]], data: SensorData, state_name: Optional[str] = None):
        if state_name and state_name != self.state_name:
            self.mark_state_start(state_name, stairs, data)
            return

        snap = self._snapshot(stairs, data)
        self.last_snapshot = self.current_snapshot
        self.current_snapshot = snap
        self.last_update_time = snap.time

        if self.state_start_snapshot is None:
            self.state_start_snapshot = snap
            self.state_name = state_name or self.state_name

    def elapsed_in_state(self) -> float:
        if not self.state_start_snapshot or not self.current_snapshot:
            return 0.0
        return self.current_snapshot.time - self.state_start_snapshot.time

    def deltas_from_state_start(self) -> Dict[str, float]:
        if not self.state_start_snapshot or not self.current_snapshot:
            return {}
        start = self.state_start_snapshot
        cur = self.current_snapshot

        center_delta = 0.0
        if start.path_center_x is not None and cur.path_center_x is not None:
            center_delta = float(cur.path_center_x - start.path_center_x)

        return {
            "pitch": cur.pitch - start.pitch,
            "roll": cur.roll - start.roll,
            "uf_ground": cur.uf_ground - start.uf_ground,
            "ur_ground": cur.ur_ground - start.ur_ground,
            "center": center_delta,
            "yellow_area": cur.yellow_area - start.yellow_area,
            "confidence": cur.confidence - start.confidence,
        }

    def deltas_from_last_frame(self) -> Dict[str, float]:
        if not self.last_snapshot or not self.current_snapshot:
            return {}
        last = self.last_snapshot
        cur = self.current_snapshot
        center_delta = 0.0
        if last.path_center_x is not None and cur.path_center_x is not None:
            center_delta = float(cur.path_center_x - last.path_center_x)
        return {
            "pitch": cur.pitch - last.pitch,
            "roll": cur.roll - last.roll,
            "uf_ground": cur.uf_ground - last.uf_ground,
            "ur_ground": cur.ur_ground - last.ur_ground,
            "center": center_delta,
            "yellow_area": cur.yellow_area - last.yellow_area,
        }

    def _dt_from_last_frame(self) -> float:
        if not self.last_snapshot or not self.current_snapshot:
            return 0.001
        return max(self.current_snapshot.time - self.last_snapshot.time, 0.001)

    def ultrasonic_rate_from_last_frame(self) -> Dict[str, float]:
        """Return absolute ultrasonic change rates in cm/sec from the last frame."""
        d_last = self.deltas_from_last_frame()
        dt = self._dt_from_last_frame()
        return {
            "uf_rate": abs(d_last.get("uf_ground", 0.0)) / dt,
            "ur_rate": abs(d_last.get("ur_ground", 0.0)) / dt,
        }

    def ultrasonic_real_change_detected(self) -> bool:
        """
        Ground-facing ultrasonic helper filter.
        A reading counts only if it changes enough OR changes fast enough.
        This rejects tiny noisy values like 3.30 -> 3.35.
        """
        if not USE_ULTRASONIC_AS_HELPER:
            return False

        d_state = self.deltas_from_state_start()
        rates = self.ultrasonic_rate_from_last_frame()

        uf_delta = abs(d_state.get("uf_ground", 0.0))
        ur_delta = abs(d_state.get("ur_ground", 0.0))
        uf_rate = rates.get("uf_rate", 0.0)
        ur_rate = rates.get("ur_rate", 0.0)

        return bool(
            uf_delta >= ULTRASONIC_MIN_DELTA_CM
            or ur_delta >= ULTRASONIC_MIN_DELTA_CM
            or uf_rate >= ULTRASONIC_MIN_RATE_CM_PER_SEC
            or ur_rate >= ULTRASONIC_MIN_RATE_CM_PER_SEC
        )

    def mpu_progress_detected(self) -> bool:
        d = self.deltas_from_state_start()
        if not d:
            return False
        return bool(
            abs(d.get("pitch", 0.0)) >= SMART_PROGRESS_MIN_PITCH_DELTA
            or abs(d.get("roll", 0.0)) >= SMART_PROGRESS_MIN_ROLL_DELTA
        )

    def camera_progress_detected(self) -> bool:
        d = self.deltas_from_state_start()
        if not d:
            return False
        return bool(
            abs(d.get("center", 0.0)) >= SMART_PROGRESS_MIN_CENTER_DELTA
            or abs(d.get("yellow_area", 0.0)) >= SMART_PROGRESS_MIN_YELLOW_AREA_DELTA
        )

    def progress_score(self) -> float:
        """
        Multi-sensor progress score.
        V9 weighting:
        - MPU can decide progress by itself.
        - Camera is strong but usually needs another clue unless thresholds are tuned.
        - Ultrasonic is kept, filtered by minimum delta/rate, and added as helper only.
        """
        score = 0.0

        if self.mpu_progress_detected():
            score += MPU_PROGRESS_WEIGHT

        if self.camera_progress_detected():
            score += CAMERA_PROGRESS_WEIGHT

        if self.ultrasonic_real_change_detected():
            score += ULTRASONIC_PROGRESS_WEIGHT

        return score

    def progress_detected(self) -> bool:
        return self.progress_score() >= SMART_PROGRESS_SCORE_THRESHOLD

    def mpu_jack_effect_score(self) -> float:
        """
        MPU-only jack effect score.
        This stays as the main proof that the jack affected the robot body.
        """
        d = self.deltas_from_state_start()
        if not d:
            return 0.0

        score = 0.0
        if abs(d.get("pitch", 0.0)) >= JACK_EFFECT_MIN_PITCH_DELTA:
            score += MPU_JACK_EFFECT_WEIGHT
        if abs(d.get("roll", 0.0)) >= JACK_EFFECT_MIN_ROLL_DELTA:
            score += MPU_JACK_EFFECT_WEIGHT
        return score

    def weak_mpu_jack_effect_detected(self) -> bool:
        """
        Small MPU movement that is not enough alone, but can be supported by ultrasonic.
        """
        d = self.deltas_from_state_start()
        if not d:
            return False
        return bool(
            abs(d.get("pitch", 0.0)) >= WEAK_MPU_JACK_EFFECT_PITCH_DELTA
            or abs(d.get("roll", 0.0)) >= WEAK_MPU_JACK_EFFECT_ROLL_DELTA
        )

    def jack_effect_score(self) -> float:
        """
        Jack effect decision score.
        MPU is dominant. Ultrasonic can only help if there is at least weak MPU evidence.
        Ultrasonic alone cannot confirm jack effect.
        """
        score = self.mpu_jack_effect_score()
        if self.weak_mpu_jack_effect_detected() and self.ultrasonic_real_change_detected():
            score += ULTRASONIC_JACK_EFFECT_WEIGHT
        return score

    def mpu_jack_effect_detected(self) -> bool:
        return self.jack_effect_score() >= JACK_EFFECT_SCORE_THRESHOLD

    def jack_contact_detected(self) -> bool:
        """
        Backward-compatible name used by older main_brain code.
        In V8 this is MPU-based, not ultrasonic-based.
        """
        return self.mpu_jack_effect_detected()

    def slip_or_drift_detected(self) -> bool:
        """Detect sudden bad movement using MPU and visual loss."""
        d_last = self.deltas_from_last_frame()
        d_state = self.deltas_from_state_start()
        if not self.current_snapshot:
            return False

        sudden_roll = abs(d_last.get("roll", 0.0)) >= SMART_SLIP_ROLL_DELTA
        state_roll = abs(d_state.get("roll", 0.0)) >= SMART_SLIP_ROLL_DELTA
        sudden_pitch_drop = d_last.get("pitch", 0.0) <= -abs(SMART_SLIP_PITCH_DROP)

        return bool(sudden_roll or state_roll or sudden_pitch_drop)

    def summary(self) -> str:
        d = self.deltas_from_state_start()
        return (
            f"progress={self.progress_score():.2f} jack_score={self.jack_effect_score():.2f} "
            f"dp={d.get('pitch', 0.0):.2f} dr={d.get('roll', 0.0):.2f} "
            f"dUF={d.get('uf_ground', 0.0):.2f} dUR={d.get('ur_ground', 0.0):.2f} "
            f"dCX={d.get('center', 0.0):.1f} dYA={d.get('yellow_area', 0.0):.0f} "
            f"ultra={self.ultrasonic_real_change_detected()}"
        )

