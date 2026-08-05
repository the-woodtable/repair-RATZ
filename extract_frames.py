"""
extract_frames.py — turn recorded videos into training images
-------------------------------------------------------------
Takes every video in ~/Desktop/30.007/pipe_cam_data/recordings/ and saves every Nth
frame as a JPG into ~/Desktop/30.007/pipe_cam_data/frames/, ready to upload to an
annotation tool.

    python3 extract_frames.py           # every 5th frame (default)
    python3 extract_frames.py 10        # every 10th frame

Why every Nth: consecutive video frames are nearly identical. Annotating
near-duplicates wastes your time and teaches the model nothing new.
"""

import os
import sys
import glob

import cv2

DATA_DIR  = os.path.expanduser("~/Desktop/30.007/pipe_cam_data")
VIDEO_DIR = os.path.join(DATA_DIR, "recordings")
OUT_DIR   = os.path.join(DATA_DIR, "frames")
EVERY_N = int(sys.argv[1]) if len(sys.argv) > 1 else 5


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    videos = sorted(glob.glob(os.path.join(VIDEO_DIR, "*.mp4")))
    if not videos:
        print(f"No videos found in {VIDEO_DIR}")
        return

    total = 0
    for path in videos:
        name = os.path.splitext(os.path.basename(path))[0]
        cap = cv2.VideoCapture(path)
        i = saved = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if i % EVERY_N == 0:
                out = os.path.join(OUT_DIR, f"{name}_f{i:05d}.jpg")
                cv2.imwrite(out, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved += 1
            i += 1
        cap.release()
        total += saved
        print(f"{name}: {saved} frames")

    print(f"\n{total} images -> {OUT_DIR}")
    print("Next: upload that folder to Roboflow (or CVAT) and draw boxes.")


if __name__ == "__main__":
    main()
