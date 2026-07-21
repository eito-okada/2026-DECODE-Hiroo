# Webcam calibration for FTC (OpenCV)

Your webcam has no built-in FTC calibration, so the AprilTag pose solver falls back to
a rough guess. These scripts measure the real lens intrinsics with OpenCV and give you
values to paste into `TeamCode/src/main/res/xml/teamwebcamcalibrations.xml`.

The calibration is a property of the **lens + sensor + resolution**, so you can do all of
this with the webcam plugged into your **laptop** — the result transfers to the robot.

## 0. One-time setup

Already done — OpenCV lives in a self-contained virtualenv at `tools/camera_calibration/.venv`
(it is git-ignored; nothing was installed into your system Python). Run the scripts with
`.venv/bin/python` and you never need to "activate" anything.

If you ever need to recreate it:

```bash
cd tools/camera_calibration
python3 -m venv .venv
.venv/bin/python -m pip install opencv-python numpy
```

Print a chessboard pattern and tape it flat to something rigid (clipboard, cardboard):
OpenCV's classic 9×6 **inner-corner** board (a 10×7 grid of squares) —
https://github.com/opencv/opencv/blob/4.x/doc/pattern.png — print it "fit to page".
"9×6 inner corners" is the default the scripts expect; count the *interior* crossings,
not the squares.

## 1. Capture images

Plug in the webcam and run (defaults: camera 0, 640×480, 9×6 board):

```bash
cd tools/camera_calibration
.venv/bin/python capture_chessboard.py --camera 0 --width 640 --height 480 --cols 9 --rows 6
```

On macOS the first run pops a **camera permission** prompt for your terminal app — allow it,
then re-run. If the window opens on the built-in FaceTime camera instead of your webcam,
try `--camera 1`.

- **Match the resolution you use on the robot** (FTC default is 640×480). A calibration
  is only valid at its capture resolution.
- Press `SPACE` to save a frame (only saves when the whole board is detected), or `a` to
  auto-capture as you move the board. Get **15–25** shots: board near/far, tilted left/
  right/up/down, and in the **corners** of the frame (that's what pins down distortion).
- Images land in `tools/camera_calibration/images/`.

## 2. Compute the calibration

```bash
.venv/bin/python calibrate.py --images images --cols 9 --rows 6 --square 25
```

`--square` is the square edge length (mm is fine). It does **not** affect fx/fy/cx/cy or
distortion, so it needn't be exact — keep it consistent. Output includes:

- **RMS reprojection error** — aim for **< 0.7 px** (< 0.4 is great). If it's high, re-shoot
  the worst images it lists and add more corner/tilt variety.
- A ready-to-paste `<Camera>` XML block, **and** a one-line `setLensIntrinsics(...)` alternative.

## 3. Plug the numbers in

### Preferred: the XML (includes distortion correction)

Edit `TeamCode/src/main/res/xml/teamwebcamcalibrations.xml` and drop the printed `<Camera>`
block inside `<Calibrations> ... </Calibrations>`. Replace `vid`/`pid` with your webcam's
real USB IDs:

- **macOS:** `system_profiler SPUSBDataType` → find your camera → "Vendor ID" / "Product ID".
- **Linux:** `lsusb` → the `ID xxxx:yyyy` pair (vid:pid).
- **Windows:** Device Manager → the camera → Details → "Hardware Ids" (`VID_xxxx&PID_yyyy`).
- Or start any Vision OpMode and read the Robot Controller **logcat** — it logs the opened
  camera's vid/pid.

Use hex form, e.g. `vid="0x046D" pid="0x0825"`. ⚠️ Once *any* calibration exists for a
camera, the SDK only offers resolutions in the calibrated aspect ratios — so calibrate at
the resolution you actually run (640×480), and add more `<Calibration>` blocks if you use
others.

### Quick alternative: code

In `BlueGoalTracker.initVision()` (and/or `AprilTagAuto`), add the printed line to the
builder — no vid/pid needed, but it does **not** apply distortion correction:

```java
aprilTag = new AprilTagProcessor.Builder()
        .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
        .setLensIntrinsics(fx, fy, cx, cy)   // <-- from calibrate.py
        .build();
```

## 4. Verify on the robot

Start a Vision OpMode with a DECODE tag at a known distance. Compare `ftcPose.range`
(and `x`/`y`) to a tape-measure reading — good calibration lands within a couple percent.
For our turret feature specifically, better calibration mainly tightens `bearing`, which is
what the odometry heading correction uses.
