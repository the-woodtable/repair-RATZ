"""
crack_tracker.py — thin YOLO tracking wrapper for the EDI PIPE CAM panel
=========================================================================
Reused reliability logic from webcam_detect.py (confirmation buffer +
missed-frame tolerance) so a track only reports once it's been seen
consistently, not on a single flickering frame.

Import this into control_panel_stereo2.py — it does NOT touch serial,
motors, or the GUI directly. It just takes a BGR frame in and returns a
list of confirmed detections out.
"""

from collections import defaultdict, deque

import numpy as np
from ultralytics import YOLO

CONFIRM_HITS = 3
CONFIRM_WINDOW = 5
MAX_MISSED_FRAMES = 10


class _TrackState:
    def __init__(self):
        self.hits = deque(maxlen=CONFIRM_WINDOW)
        self.missed_streak = 0
        self.label = None
        self.conf = 0.0
        self.mask_xy = None   # polygon points, image pixel coords
        self.centroid = None  # (x, y) pixel, used for distance lookup

    def mark_seen(self, label, conf, mask_xy):
        self.hits.append(1)
        self.missed_streak = 0
        self.label = label
        self.conf = conf
        self.mask_xy = mask_xy
        self.centroid = (float(mask_xy[:, 0].mean()), float(mask_xy[:, 1].mean()))

    def mark_missed(self):
        self.hits.append(0)
        self.missed_streak += 1

    @property
    def confirmed(self):
        return sum(self.hits) >= CONFIRM_HITS

    @property
    def alive(self):
        return self.missed_streak <= MAX_MISSED_FRAMES


class Detection:
    """What CrackTracker.update() returns per confirmed track."""

    __slots__ = ("track_id", "label", "conf", "mask_xy", "centroid")

    def __init__(self, track_id, label, conf, mask_xy, centroid):
        self.track_id = track_id
        self.label = label
        self.conf = conf
        self.mask_xy = mask_xy
        self.centroid = centroid


class CrackTracker:
    def __init__(self, weights_path: str, conf_threshold: float = 0.4,
                tracker: str = "botsort.yaml"):
        self.model = YOLO(weights_path)
        self.conf_threshold = conf_threshold
        self.tracker = tracker
        self._tracks: dict[int, _TrackState] = defaultdict(_TrackState)

    def update(self, frame_bgr) -> list[Detection]:
        """Call once per frame. Returns confirmed detections (may be empty)."""
        result = self.model.track(frame_bgr, conf=self.conf_threshold,
                                  tracker=self.tracker, persist=True,
                                  verbose=False)[0]
        names = result.names
        seen_ids = set()

        if result.masks is not None and result.boxes.id is not None:
            ids = result.boxes.id.int().cpu().tolist()
            for mask_xy, box, track_id in zip(result.masks.xy, result.boxes, ids):
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = names.get(cls_id, str(cls_id))
                self._tracks[track_id].mark_seen(label, conf, mask_xy.astype(int))
                seen_ids.add(track_id)

        for track_id, state in list(self._tracks.items()):
            if track_id not in seen_ids:
                state.mark_missed()
            if not state.alive:
                del self._tracks[track_id]

        out = []
        for track_id in seen_ids:
            state = self._tracks[track_id]
            if state.confirmed:
                out.append(Detection(track_id, state.label, state.conf,
                                     state.mask_xy, state.centroid))
        return out