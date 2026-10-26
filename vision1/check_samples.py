"""Self-check on recorded overhead photos: fails if detection or variant labelling breaks."""
import os, sys
import cv2
sys.path.insert(0, os.path.dirname(__file__))
from detect import detect

EXPECTED = {"01": ["VARb", "VARc", "VARa"], "06": ["VARa", "VARb", "VARc"]}  # left -> right in the pick area
root = os.path.join(os.path.dirname(__file__), "..", "samples")
for name, want in EXPECTED.items():
    boards = detect(cv2.imread(os.path.join(root, name + ".jpg")))
    got = [b["variant"] for b in boards]
    assert got == want, f"{name}.jpg: got {got}, want {want}"
    assert all(-90 <= b["yaw_deg"] < 90 for b in boards)
print("ok")
