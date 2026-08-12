"""
extract_frames.py — turn recorded videos into training images
-------------------------------------------------------------
Reads the .mp4 files in  ~/Desktop/30.007/pipe_cam_data/recordings/
and writes JPGs into    ~/Desktop/30.007/pipe_cam_data/frames/

The panel records AFTER applying ROTATE_DEG, so these frames are already
the right way up — nothing to correct here.

USAGE
    python3 extract_frames.py                # every 5th frame, grouped by session
    python3 extract_frames.py 10             # every 10th frame
    python3 extract_frames.py --fps 3        # 3 images per second of video
    python3 extract_frames.py 10 --per-video # one folder per .mp4 instead
    python3 extract_frames.py --flat         # old behaviour, all in one folder

--fps VERSUS A FIXED INTERVAL
    --fps is per second of VIDEO time, and the interval is worked out from
    each file's own frame rate, so it stays consistent across recordings made
    at different rates. Note the panel stamps its MP4s at REC_FPS (10), which
    is the rate it ASKS for, not necessarily the rate the cameras delivered —
    when the link is running at 6 fps the video plays fast, and "3 per second
    of video" is then fewer than 3 per second of real time. Fine for building
    a varied training set; don't use it to measure anything.

GROUPING (default)
    The panel writes left_<stamp>.mp4 and right_<stamp>.mp4 for each REC
    press, so a shared timestamp means "the same run". Those are grouped
    into one set:

        frames/set1_20260806_021816/left_..._f00000.jpg
                                    right_..._f00000.jpg
        frames/set2_20260806_021922/...

    Keeping a run together matters for annotation: images from one run share
    lighting, focus and pipe section, so you can label them consistently and
    can drop a whole bad run without hunting through a flat folder.

WHY EVERY Nth FRAME
    Consecutive video frames are nearly identical. Annotating near-duplicates
    costs you time and teaches the model nothing new. It also inflates your
    apparent dataset size while leaving real-world variety unchanged, which
    makes validation scores look better than the model actually is.
"""

import os
import re
import sys
import glob
from collections import defaultdict

import cv2

DATA_DIR  = os.path.expanduser("~/Desktop/30.007/pipe_cam_data")
VIDEO_DIR = os.path.join(DATA_DIR, "recordings")
OUT_DIR   = os.path.join(DATA_DIR, "frames")

JPEG_QUALITY = 95

# left_20260806_021816.mp4 -> ("left", "20260806_021816")
NAME_RE = re.compile(r"^(left|right)_(\d{8}_\d{6})$")


def parse_args(argv):
    every, mode, target_fps = 5, "session", None
    args = argv[1:]
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--per-video":
            mode = "video"
        elif a == "--flat":
            mode = "flat"
        elif a == "--fps":
            i += 1
            if i >= len(args):
                print("--fps needs a number, e.g. --fps 3")
                sys.exit(1)
            try:
                target_fps = float(args[i])
            except ValueError:
                print(f"--fps needs a number, got {args[i]!r}")
                sys.exit(1)
            if target_fps <= 0:
                print("--fps must be greater than 0")
                sys.exit(1)
        elif a.isdigit():
            every = int(a)
        else:
            print(f"Unknown argument: {a}")
            print(__doc__)
            sys.exit(1)
        i += 1
    if every < 1:
        print("Frame interval must be at least 1")
        sys.exit(1)
    return every, mode, target_fps


def interval_for(cap, every, target_fps, name):
    """Frames to skip: from --fps if given, else the fixed interval."""
    if target_fps is None:
        return every
    src = cap.get(cv2.CAP_PROP_FPS)
    if not src or src <= 0 or src != src:      # 0, missing, or NaN
        print(f"  !! {name}: no frame rate in the file header, "
              f"falling back to every {every}th frame")
        return every
    n = int(round(src / target_fps))
    return max(1, n)


def extract(path, out_dir, every, target_fps=None):
    """Save frames of one video into out_dir. Returns count saved."""
    name = os.path.splitext(os.path.basename(path))[0]
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"  !! could not open {name} — skipping")
        return 0
    every = interval_for(cap, every, target_fps, name)
    os.makedirs(out_dir, exist_ok=True)
    i = saved = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % every == 0:
            out = os.path.join(out_dir, f"{name}_f{i:05d}.jpg")
            cv2.imwrite(out, frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            saved += 1
        i += 1
    cap.release()
    if i == 0:
        print(f"  !! {name} decoded 0 frames — the file may be truncated")
    return saved


def main():
    every, mode, target_fps = parse_args(sys.argv)

    videos = sorted(glob.glob(os.path.join(VIDEO_DIR, "*.mp4")))
    if not videos:
        print(f"No videos found in {VIDEO_DIR}")
        print("Record some first: run the panel and press REC.")
        return

    # Group by the timestamp shared between the left and right file of a run.
    # Anything not matching the panel's naming keeps its own group so an
    # oddly-named file is never silently merged into someone else's set.
    groups = defaultdict(list)
    for path in videos:
        stem = os.path.splitext(os.path.basename(path))[0]
        m = NAME_RE.match(stem)
        groups[m.group(2) if m else stem].append(path)

    os.makedirs(OUT_DIR, exist_ok=True)
    total = 0
    if target_fps:
        print(f"{target_fps:g} frames per second of video, "
              f"grouping: {mode}\n")
    else:
        print(f"every {every}th frame, grouping: {mode}\n")

    for n, stamp in enumerate(sorted(groups), start=1):
        paths = sorted(groups[stamp])
        if mode == "flat":
            targets = [(OUT_DIR, p) for p in paths]
            label = "frames/"
        elif mode == "video":
            targets = [(os.path.join(
                OUT_DIR, os.path.splitext(os.path.basename(p))[0]), p)
                for p in paths]
            label = f"set{n} -> one folder per video"
        else:
            d = os.path.join(OUT_DIR, f"set{n}_{stamp}")
            targets = [(d, p) for p in paths]
            label = os.path.basename(d) + "/"

        print(f"set{n}  {stamp}  ({len(paths)} video"
              f"{'s' if len(paths) != 1 else ''}) -> {label}")
        for out_dir, path in targets:
            saved = extract(path, out_dir, every, target_fps)
            total += saved
            print(f"    {os.path.basename(path):32s} {saved:5d} frames")

    print(f"\n{total} images -> {OUT_DIR}")
    if total:
        print("Next: upload a set folder to Roboflow (or CVAT) and draw boxes.")
        print("Tip: annotate ONE set first and train a quick model — it tells "
              "you whether the footage is good enough before you label the rest.")


if __name__ == "__main__":
    main()
