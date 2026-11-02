"""
Constants for the borescope + RSI synchronized capture pipeline.

All settings derived from:
  - demo3_probe_params.py / demo4_stream_test.py findings on NTG100
  - RSI_EthernetConfig.xml (KRC RSI send/receive schema)
  - init_recorder_svo_csv.py (15-min chunking + power management precedent)
"""
import os
import cv2


# DSHOW indices to record in one session, one BoreScope each. 2026-09-15: both NTG100
# units (bore1 = idx 1, bore2 = idx 2 on win-laptop; verify with dual_preview.py, the two
# units enumerate by USB port, not by plug-in order). A single index = legacy one-camera
# session with unsuffixed file names. run.py --cam-index overrides this for a session.
CAM_INDICES      = [1, 2]
CAM_BACKEND      = cv2.CAP_DSHOW
RES_W, RES_H     = 1280, 720
VIDEO_FOURCC     = "mp4v"
FPS_NOMINAL      = 30.0
SEGMENT_MINUTES  = 15

SAFE_CONTROLS = {}

# ELP-USBGS1200P01 bead camera (Round 1, desktop issue I4; elp.py). Its own profile; the NTG100
# constants above stay as they are. Selected by DSHOW name + usb-instance substring, never by
# index: both units share the name and USB serial 01.00.00 (reference_elp_gs1200_sensor.md sec. 2).
# `controls` are applied in this order after every open. DSHOW: AUTO_EXPOSURE 0.25 = manual,
# EXPOSURE -7 = 2^-7 s = 7.8 ms, gain 0 = firmware default, white balance in Kelvin after AUTO_WB 0.
# "WB_KELVIN" is not an OpenCV name: the DSHOW property behind the UVC WhiteBalance control depends
# on the OpenCV build (laptop cv2 4.13: CAP_PROP_WB_TEMPERATURE; desktop build: WHITE_BALANCE_BLUE_U),
# so elp.py uses whichever of the two reads back a value.
# `fps` is the open-time request and the mp4's nominal rate; with 60 requested the firmware
# delivered 78-89 fps on 09-28/29, so every received frame is written and the csv holds the
# real receive times. Frames are stored raw. `rot180` False (desktop 10-01): the raw frame is
# already the operator's view (image right = -X, down = +Y) and the intrinsics were fitted on raw
# frames, so never rotate before undistort.
ELP_DEFAULT = False                  # run.py --elp / --no-elp
ELP_PROFILE = {
    "name": "Global Shutter Camera",
    "instance": "7&325db0ba",        # run.py --elp-instance overrides; "" = any (one ELP only)
    "res": (1920, 1200),
    "fps": 60,
    "controls": [
        ("CAP_PROP_AUTO_EXPOSURE", 0.25),
        ("CAP_PROP_EXPOSURE", -7),
        ("CAP_PROP_GAIN", 0),
        ("CAP_PROP_AUTO_WB", 0),
        ("WB_KELVIN", 4600),
    ],
    "rot180": False,
    # capture -> writer buffer, 6.9 MB per frame. 256 = 3.2 s at 80 fps (1.8 GB only while full): covers the 1.7 / 2.2 s
    # writer stalls of 10-05 (32 = 0.4 s dropped 137 / 189 frames); the writer drains at ~336 fps (laptop, 10-06).
    "queue_frames": 256,
    # Storage. "MJPG" (default since 10-05, desktop 10-03 patch): OpenCV's own MJPEG encoder into .avi,
    # JPEG quality set after open (constructor params are ignored). mp4v at ~26 kB/frame showed 8x8
    # blocking and wiped bed texture / weak bead edges. "mp4v" = the 10-02 format, one-word switch back.
    # MJPG segments must stay short: cv2 4.13's AVI writer aborts the whole process in release() once a
    # file passes ~4.29 GB (laptop test 10-05; 2.41 GB fine). 60 s at 77 fps = 4,620 frames.
    "fourcc": "MJPG",
    "jpeg_quality": 95,
    "segment_s": {"MJPG": 60, "mp4v": SEGMENT_MINUTES * 60},
}

RSI_HOST                 = "0.0.0.0"
RSI_PORT                 = 59152
RSI_DISCONNECT_TIMEOUT_S = 2.0
CSV_FLUSH_INTERVAL       = 200

_THIS_DIR     = os.path.dirname(os.path.abspath(__file__))
# Walk up to the folder named Y4F, so the recorder runs from any depth under it
Y4F_ROOT = _THIS_DIR
while os.path.basename(Y4F_ROOT) != "Y4F":
    if os.path.dirname(Y4F_ROOT) == Y4F_ROOT:
        raise RuntimeError(f"no folder named Y4F above {_THIS_DIR}")
    Y4F_ROOT = os.path.dirname(Y4F_ROOT)
RSI_PRINTING_ROOT = os.path.dirname(Y4F_ROOT)
VIDEO_OUTPUT_DIR  = os.path.join(Y4F_ROOT, "data", "raw_video")
CSV_OUTPUT_DIR    = os.path.join(Y4F_ROOT, "data", "roboter")

# Changed 2026-09-08: default is now True -- start borescope recording only once
# the first RSI packet arrives, so every recorded frame has a synced pose from
# frame 0 (no manual pre-roll trimming needed before offline merge_asof analysis).
# Trade-off vs the old False default: no "record everything, trim later" safety
# margin -- accepted deliberately in favor of analysis convenience.
WAIT_FOR_RSI_DEFAULT = True
STATUS_PRINT_INTERVAL_S = 5.0
