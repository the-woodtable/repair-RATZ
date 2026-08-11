# Camera orientation — every place to change it

Three independent problems, three different fixes. They do not interact, so
diagnose one at a time.

| What you see | What to change |
|---|---|
| Both views **upside down** | `ROTATE_DEG` — swap the two numbers |
| Views **crossed** (left camera in right pane) | `main.cpp` pin swap — **one** place only |
| Text reads **backwards** | `MIRROR_H` |

After ANY change:

```
python3 check_project.py
```

It compares all the copies and fails if they disagree.

---

## 1. ROTATE_DEG — rotation (no reflash, restart the panel)

`(left, right)`, each 0 / 90 / 180 / 270 clockwise. **Four files, all must match.**

| File | Line | Note |
|---|---|---|
| `showcase_cv.py` | 32 | what RATpanel actually uses |
| `rat88_panel.py` | 81 | fallback, only used if cv2 is missing |
| `control_panel_stereo2.py` | 77 | the other panel |
| `stereo_calibrate.py` | 54 | **must match or every distance is wrong** |

**To flip both cameras 180°: swap the two numbers.** That works because our two
values are 180° apart, so exchanging them adds 180 to each:

```
(270, 90)  <->  (90, 270)
```

To rotate only ONE camera, change only its number. Index 0 = left, 1 = right.

---

## 2. MIRROR_H — horizontal mirror (no reflash, restart the panel)

`(left, right)`, 0 or 1. Applied AFTER rotation, so it means "flip what you
are looking at, left to right".

Same four files, same line-mates:

| File | Line |
|---|---|
| `showcase_cv.py` | 43 |
| `rat88_panel.py` | 82 |
| `control_panel_stereo2.py` | 105 |
| `stereo_calibrate.py` | 65 |

⚠️ **Both cameras or neither — never one of each.** Mirroring one gives the two
views opposite handedness, and no real-world point can be matched between them.
That makes stereo depth impossible, not merely inaccurate. `check_project.py`
treats a mismatch as a hard failure.

**Rotation cannot substitute for a mirror.** Rotations preserve handedness; a
mirror reverses it. No combination of the four rotations equals a mirror.

Tell them apart by holding up a page of text:

* upside down but reads normally → rotation
* right way up but reads backwards → mirror

---

## 3. Left / right swap — which camera is which (NEEDS A REFLASH)

`THIS ONE/src/main.cpp` line 30:

```cpp
int CAML_RX = 18;   // left cam  -> S3 GPIO 18
int CAMR_RX = 16;   // right cam -> S3 GPIO 16
```

Swap the two numbers, reflash the S3. Doing it here rather than in a panel
means both panels, the calibrator and any recordings all agree.

**Change EITHER the code OR the wires — never both.** Two swaps cancel out,
which is how we ended up going in circles.

**Swapping ids also swaps which camera gets which rotation**, because rotation
belongs to the physical camera and how it is mounted. So a left/right swap
normally needs `ROTATE_DEG` swapped too.

### The ground-truth test

Cover the **left** lens with your hand:

* left pane goes dark → correct
* right pane goes dark → crossed

This is the only check that does not depend on reasoning about the mounts.

---

## Before calibrating

Whatever orientation you settle on must be live when you run
`stereo_calibrate.py`, and must not change afterwards. Calibration bakes in the
orientation it saw. Change it later and every distance shifts, with no warning
and no error — just plausible numbers that are wrong.

Treat these constants as frozen once calibration is done.
