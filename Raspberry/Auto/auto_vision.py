#!/usr/bin/env python3
"""
auto_vision.py
Lightweight vision helper for Apex Rover AUTO mode.

It uses template images from Raspberry/src/. This is intentional because you said
that boxes, stairs, robot basket, and objects will be added to src beside Auto/Manual.

Naming examples:
  source_box_1.jpg
  destination_box_1.png
  stairs_front.jpg
  object_red.png
  robot_basket.jpg
"""

import math
import os
from pathlib import Path

import cv2
import numpy as np

from auto_config import (
    FRAME_HEIGHT,
    FRAME_WIDTH,
    MAX_ACCEPTED_BOX_TOP_Y_RATIO,
    SRC_DIR,
    TEMPLATE_KEYWORDS,
    TEMPLATE_MIN_SCORE,
    TEMPLATE_SCALES,
)

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".webp", ".bmp"}


def _safe_float(x, default=0.0):
    try:
        return float(x)
    except Exception:
        return default


class TemplateLibrary:
    def __init__(self, src_dir=SRC_DIR):
        self.src_dir = Path(src_dir)
        self.templates = {key: [] for key in TEMPLATE_KEYWORDS.keys()}
        self.load()

    def load(self):
        if not self.src_dir.exists():
            return

        for path in self.src_dir.iterdir():
            if not path.is_file() or path.suffix.lower() not in IMAGE_EXTS:
                continue

            name = path.stem.lower()
            category = self.classify_name(name)
            if category is None:
                continue

            img = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if img is None:
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.templates[category].append({
                "name": path.name,
                "path": str(path),
                "image": img,
                "gray": gray,
                "w": gray.shape[1],
                "h": gray.shape[0],
            })

    def classify_name(self, name):
        # Specific names first so destination_box is not swallowed by generic box.
        ordered = ["destination_box", "source_box", "stairs", "object", "robot_basket"]
        for category in ordered:
            for kw in TEMPLATE_KEYWORDS.get(category, []):
                if kw in name:
                    return category
        return None

    def has_category(self, category):
        return bool(self.templates.get(category))

    def summary(self):
        return {k: [t["name"] for t in v] for k, v in self.templates.items()}


class VisionBrain:
    def __init__(self, status, library=None):
        self.status = status
        self.library = library or TemplateLibrary()
        self.status.event(
            phase="VISION_INIT",
            doing="Loaded vision templates from src folder",
            decision="templates loaded",
            templates=self.library.summary(),
        )

    # ------------------------------------------------------------
    # Public detection API
    # ------------------------------------------------------------
    def detect(self, frame, category, min_score=TEMPLATE_MIN_SCORE):
        if frame is None:
            return self._not_found(category, "no frame")

        if category == "source_box" and not self.library.has_category("source_box"):
            # Generic box fallback if no source-specific image exists.
            category = "source_box"

        result = self._template_detect(frame, category, min_score=min_score)

        # Fallback for object category: use contour if no template loaded/matched.
        if not result["found"] and category == "object":
            result = self._detect_colored_object_fallback(frame)

        self.status.set_detection(result)
        return result

    def is_box_height_allowed(self, detection, frame_shape):
        if not detection.get("found"):
            return False
        h = frame_shape[0]
        top_y = detection.get("bbox", [0, 0, 0, 0])[1]
        ratio = top_y / max(1, h)
        allowed = ratio >= MAX_ACCEPTED_BOX_TOP_Y_RATIO
        detection["box_top_y_ratio"] = ratio
        detection["height_allowed"] = allowed
        return allowed

    # ------------------------------------------------------------
    # Template matching
    # ------------------------------------------------------------
    def _template_detect(self, frame, category, min_score):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_h, frame_w = gray.shape[:2]
        best = None

        candidates = list(self.library.templates.get(category, []))

        # If source_box templates are absent, try generic source_box keyword may have loaded generic box.
        if category == "source_box" and not candidates:
            candidates = self._all_templates_containing("box")
        if category == "destination_box" and not candidates:
            candidates = self._all_templates_containing("box")

        if not candidates:
            return self._not_found(category, "no templates in src")

        for temp in candidates:
            tgray = temp["gray"]
            for scale in TEMPLATE_SCALES:
                tw = max(12, int(temp["w"] * scale))
                th = max(12, int(temp["h"] * scale))
                if tw >= frame_w or th >= frame_h:
                    continue
                resized = cv2.resize(tgray, (tw, th), interpolation=cv2.INTER_AREA)
                try:
                    res = cv2.matchTemplate(gray, resized, cv2.TM_CCOEFF_NORMED)
                    _, max_val, _, max_loc = cv2.minMaxLoc(res)
                except Exception:
                    continue

                if best is None or max_val > best["score"]:
                    x, y = max_loc
                    best = {
                        "found": max_val >= min_score,
                        "category": category,
                        "score": float(max_val),
                        "template": temp["name"],
                        "bbox": [int(x), int(y), int(tw), int(th)],
                        "cx": int(x + tw / 2),
                        "cy": int(y + th / 2),
                        "frame_w": int(frame_w),
                        "frame_h": int(frame_h),
                        "area_ratio": float((tw * th) / max(1, frame_w * frame_h)),
                        "reason": "template_match",
                    }

        if best is None:
            return self._not_found(category, "template match failed")
        if not best["found"]:
            best["reason"] = f"best score below threshold: {best['score']:.3f}"
        return best

    def _all_templates_containing(self, text):
        out = []
        for arr in self.library.templates.values():
            for t in arr:
                if text in t["name"].lower():
                    out.append(t)
        return out

    # ------------------------------------------------------------
    # Simple fallback for unknown colored objects inside a box.
    # ------------------------------------------------------------
    def _detect_colored_object_fallback(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Ignore very dark, very bright white, and weak saturation areas.
        mask = cv2.inRange(hsv, (0, 45, 35), (179, 255, 245))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return self._not_found("object", "no colored contour")

        h, w = frame.shape[:2]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for c in contours[:5]:
            area = cv2.contourArea(c)
            if area < 300:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if bw * bh <= 0:
                continue
            return {
                "found": True,
                "category": "object",
                "score": min(1.0, area / max(1, w * h) * 12.0),
                "template": "color_contour_fallback",
                "bbox": [int(x), int(y), int(bw), int(bh)],
                "cx": int(x + bw / 2),
                "cy": int(y + bh / 2),
                "frame_w": int(w),
                "frame_h": int(h),
                "area_ratio": float((bw * bh) / max(1, w * h)),
                "reason": "colored_object_fallback",
            }
        return self._not_found("object", "colored contours too small")

    def _not_found(self, category, reason):
        return {
            "found": False,
            "category": category,
            "score": 0.0,
            "template": None,
            "bbox": None,
            "cx": None,
            "cy": None,
            "frame_w": FRAME_WIDTH,
            "frame_h": FRAME_HEIGHT,
            "area_ratio": 0.0,
            "reason": reason,
        }
