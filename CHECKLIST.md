# EDI Pipe Cam — what to do, in order

Everything below is in `repair-RATZ/`. Run Python from the venv:

    source ~/Downloads/esp32env/bin/activate
    cd ~/Documents/GitHub/repair-RATZ

---

## The three programs

| File | Goes on | Flashed with |
|---|---|---|
| `camera_firmware/camera_firmware.ino` | BOTH ESP32-CAMs | Arduino IDE, board "AI Thinker ESP32-CAM" |
| `THIS ONE/` (PlatformIO project) | ESP32-S3 | PlatformIO: Build → Upload, then press RST |
| `control_panel_stereo2.py` | Laptop | `python3 control_panel_stereo2.py` |

**Golden rule:** both cameras are flashed from the *same file*, every time. Most
of the bugs in this project came from one camera running an older build.

Supporting tools: `stream_quality.py` (measure), `serial_debug.py` (diagnose),
`stereo_calibrate.py` (calibration), `extract_frames.py` (video → images).

---

# STAGE 0 — Flash and verify the stream

**Do this first, every session where you've changed firmware.**

1. Flash both cameras from `camera_firmware/`.
   (Disconnect U0T from the S3 first; GPIO0 to GND; reset; flash; reconnect.)
2. Build + Upload `THIS ONE/` in PlatformIO. Press RST on the S3.
3. Measure:

       python3 stream_quality.py '' 921600 60

**Pass criteria — do not continue until you see all of these:**

| Metric | Target |
|---|---|
| fps | > 15 |
| corrupt | 0% |
| gray-bottom | 0 / N |
| unaccounted | < 5% |
| per camera | both L and R present, roughly equal counts |
| left vs right | "cameras MATCH" |

If anything fails, see TROUBLESHOOTING at the bottom. Known-good reference:
**23.4 fps, 0% corrupt, 0/1403 gray-bottom.**

---

# STAGE 1 — Camera exposure (do before calibrating)

Both cameras must be exposed identically, or stereo matching and checkerboard
detection both suffer.

1. Put the rig where it will actually work (in the pipe, with your pipe lighting).
2. Run `stream_quality.py '' 921600 60` and read the **left vs right** block.
3. Tune in `camera_firmware/camera_firmware.ino`:

       #define FIXED_EXPOSURE 600   // 0-1200
       #define FIXED_GAIN     8     // 0-30

   - too dark → raise `FIXED_EXPOSURE` first (gain adds noise, exposure doesn't)
   - still dark at 1200 → then raise `FIXED_GAIN`
   - motion blur while driving → lower exposure, raise gain
4. Reflash **both** cameras. Repeat until:
   - brightness roughly 100–160
   - left vs right says **"cameras MATCH"**

Lock this in before Stage 2. Calibration must happen with the same settings
you'll record with.

---

# STAGE 2 — Stereo calibration (distance measurement)

**What it does:** measures the lens properties and the exact geometry between
the two cameras, so disparity can be converted to centimetres.

**When:** after both cameras are rigidly mounted in FINAL position.
**Redo it if:** either camera moves, is remounted, or is bumped.

### Prepare
- Print `calib.io_CHECKERBOARD_200x150_8x11_15.pdf`, tape it to something FLAT.
- **Measure one printed square with a ruler.** Printers rescale.
- In `stereo_calibrate.py` set:

      BOARD = (10, 7)     # 11x8 squares -> 10x7 INNER corners
      SQUARE_MM = 15.0    # your measured value

### Run (outside the pipe — you need room to move the board)

    python3 stereo_calibrate.py

- **SPACE** captures a pair. It only counts if the board is found in BOTH views.
- Capture **15–20 pairs**, varying: near/far (15–40 cm — the range you'll
  actually measure at), all four corners of the frame, tilted ±30°.
- Hold the board STILL for each capture (the cameras aren't frame-synced).
- **c** to calibrate. **q** to quit.

### Check the output
- **RMS error** < 0.5 px → good. Above 1.0 → recapture with more variety.
- **baseline** should match the real lens-to-lens spacing you can measure with
  a ruler. If it doesn't, `SQUARE_MM` is wrong.

Produces `stereo_calib.npz`. The panel picks it up automatically.

### Verify before trusting it
Run the panel, click a textured object in the LEFT view, compare the readout
to a tape measure at 20 / 40 / 60 cm. Within ~5–10% is normal.

---

# STAGE 3 — IMU calibration (bias)

The IMU runs a Kalman filter that estimates its own accelerometer bias
(`imu_bias` in telemetry). "Calibration" = letting it converge, then zeroing.

1. Put the robot on a **completely still** surface. Do not touch it.
2. Power on, run the panel, watch the `imu_bias` telemetry value.
3. Wait ~30 s. The value should settle and stop drifting.
4. Press **ZERO ODOM** (sends `Z`) — this zeros both step count and IMU position.

Do this at the start of each run, before driving. If `imu_bias` never settles,
the IMU is probably being vibrated (fan, bench, cable tension).

**Slip detection** uses this: if the stepper reports movement but the IMU sees
none, `slip` goes to 1 and your distance-along-pipe is overcounting.

---

# STAGE 4 — Odometry calibration (STEPS_PER_MM)

Distance *along the pipe* comes from counting stepper steps. The conversion
constant must be measured — it can't be calculated, because of wheel slip and
preload.

1. Put the robot in the pipe at a marked start point.
2. Press **ZERO ODOM**.
3. Drive FORWARD a decent distance (30–50 cm).
4. Measure the ACTUAL distance travelled with a tape measure.
5. Read `steps` from telemetry.
6. Set in `THIS ONE/src/main.cpp`:

       STEPS_PER_MM = steps / measured_mm

7. Rebuild + upload, then repeat once to confirm the `mm` readout now matches
   reality.

---

# STAGE 5 — Record crack footage

    python3 control_panel_stereo2.py

- **● REC** → MP4s to `~/Desktop/30.007/pipe_cam_data/recordings/`
- **SAVE still** / **AUTO-CAP** → PNG pairs to `~/Desktop/30.007/pipe_cam_data/dataset/`

**For YOLO training, prefer the PNG stills** — no compression artefacts, and no
near-duplicate frames to weed out. Use video when you want to review a run.

What to capture:
- Move SLOWLY. Fast motion = blur = useless training frames.
- Vary crack size, angle, distance, lighting.
- Include plenty of **crack-free** pipe surface (~25% of the set).
- Include things that *look* like cracks: seams, scratches, shadows, stains.
  These are what stop the model crying "crack" at every joint.

Aim for **150–300 usable images** before training.

---

# STAGE 6 — Videos → pictures → boxes → model

### 6a. Extract frames from video

    python3 extract_frames.py        # every 5th frame
    python3 extract_frames.py 10     # every 10th frame

Reads `~/Desktop/30.007/pipe_cam_data/recordings/*.mp4`
Writes `~/Desktop/30.007/pipe_cam_data/frames/*.jpg`

Every 5th frame because consecutive video frames are nearly identical —
annotating duplicates wastes your time and teaches the model nothing.

(If you used SAVE/AUTO-CAP instead, skip this — those are already images.)

### 6b. Annotate (draw the boxes)

Use **Roboflow** (free, browser): https://roboflow.com

1. Create Project → **Object Detection** → class name `crack`
2. Upload the `pipe_cam_data/frames` (or `pipe_cam_data/dataset`) folder
3. Draw a tight box around every crack in every image.
   Images with no cracks: mark as null / save with no boxes — **do not delete
   them**, they're the negatives that prevent false positives.
4. Generate a version: train/valid/test 70/20/10, resize 640×640,
   light augmentation (flips, small brightness). Skip the exotic ones.
5. Export → format **YOLOv8** → "show download code" → copy the snippet.

### 6c. Train (Google Colab, free GPU)

New notebook at https://colab.research.google.com →
**Runtime → Change runtime type → T4 GPU**

    # cell 1
    !pip install ultralytics roboflow

    # cell 2 — paste YOUR snippet from Roboflow
    from roboflow import Roboflow
    rf = Roboflow(api_key="XXXX")
    project = rf.workspace("you").project("pipe-cracks")
    dataset = project.version(1).download("yolov8")

    # cell 3 — train (10-30 min)
    from ultralytics import YOLO
    model = YOLO("yolov8n.pt")
    model.train(data=f"{dataset.location}/data.yaml",
                epochs=100, imgsz=320, patience=20)

    # cell 4 — download the weights
    from google.colab import files
    files.download("runs/detect/train/weights/best.pt")

Watch `mAP50` — above ~0.7 is a decent first pass.

### 6d. Use it

Put `best.pt` next to the panel, `pip install ultralytics` in your venv, and
the detection code can load it. (Detection was removed from the current
capture-only panel — say the word and it goes back in.)

---

# TROUBLESHOOTING

Run `python3 serial_debug.py` (panel closed) and match the symptom:

| Symptom | Meaning |
|---|---|
| 0 bytes | S3 not running: wrong port, sketch not uploaded, or USB CDC flag missing in `platformio.ini` |
| bytes but no frames | camera↔S3 baud mismatch (`Serial.begin` vs `CAM_BAUD`, both must be 460800) |
| only one camera id | that camera's 5V/GND/U0T, or its sensor failed to init |
| high unaccounted % | framing desync — S3 side |
| gray-bottom frames | bytes lost before the S3 forwarded them |
| camera LED blinks 1×/sec forever | sensor init failed → reseat that camera's ribbon |
| works then dies, gets hot | POWER. Stop, check polarity and current draw |

**The decisive test** when you can't tell whether a problem is the camera or
everything after it — wire one camera straight to a USB-serial adapter:

    python3 stream_quality.py /dev/cu.usbserial-XXXX 460800 60

Clean there = camera is fine, problem is downstream. This test is what found
the real bug after two days of chasing the wrong layer.

---

# WHERE THE DATA GOES

Everything the panel and tools capture lands under one folder:

    ~/Desktop/30.007/pipe_cam_data/
        recordings/       REC button -> left_*.mp4 / right_*.mp4
        dataset/          SAVE still / AUTO-CAP -> *_L.png / *_R.png
        calib_pairs/      CALIB PAIR button
        frames/           extract_frames.py output (video -> jpg)
        quality_samples/  stream_quality.py sample frames

If you ever want to move it, change `DATA_DIR` at the top of
`control_panel_stereo2.py`, `stream_quality.py` and `extract_frames.py` —
all other paths are derived from it.

---

## Stereo calibration — deferred to final assembly

Calibration describes exactly where the cameras are. Moving one voids it,
and the failure is SILENT: you get plausible-looking distances that are
simply wrong. So this is the last step, after the cameras are rigidly
mounted and will not move again.

### Already settled (don't re-derive)

| Setting | Value | Where |
|---|---|---|
| Board | 11x8 squares = **(10, 7)** inner corners | `stereo_calibrate.py` |
| Square size | **19.5 mm** (measured with a ruler) | `stereo_calibrate.py` |
| Rotation | `ROTATE_DEG = (270, 90)` | must match in panel AND calibrator |

First attempt used 15.0 mm and produced a 95.9 mm baseline for a rig whose
lenses are 125 mm apart — wrong by exactly 19.5/15.0. The printed baseline
vs a ruler is the self-check; use it every time.

### Known constraints at final assembly

Frames are 240 px wide in the disparity direction (the 90-degree rotation
puts the short axis there), and:

    nearest measurable = fx * baseline / numDisparities

* `numDisparities` cannot exceed ~40% of frame width. SGBM cannot match the
  leftmost `numDisparities` columns, and at 128 the dead zone swallowed the
  centre crosshair and the readout died entirely. **96 is the practical max.**
* With the current 125 mm lens gap: nearest = 50 cm. Below 20 cm the two
  fields of view do not intersect at all.

### Three ways to measure closer, in order of cost

1. **Narrow the baseline** — free, no CPU, no lost pixels. 60 mm gap -> 24 cm.
2. **`alpha` in `stereoRectify`** — currently `alpha=0`, which crops the frame
   to fully-valid pixels only and throws away real FOV given the 4 mm vertical
   and 7 mm depth offset between cameras. Try `alpha=0.5`; costs black borders.
3. **Wider lens** — 60 deg would roughly halve the near limit. Do NOT go to
   90 deg+ without more resolution: a 1 mm crack at 30 cm is 1.27 px today and
   0.40 px at 90 deg, i.e. undetectable. Ranging would improve, detection would die.
