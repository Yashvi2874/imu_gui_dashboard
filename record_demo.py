"""
Captures a demo video of the IMU dashboard.

Requirements:
    pip install opencv-python pillow pyautogui

Usage:
    1. Run the dashboard first:   python main.py
    2. In a second terminal:      python record_demo.py

Output: demo.avi (30 seconds, ~15 fps) in the current directory.
Press Ctrl+C to stop early.
"""

import time
import cv2
import numpy as np
import pyautogui

OUTPUT_FILE = "demo.avi"
DURATION    = 60        # seconds
FPS         = 15
FRAME_DELAY = 1.0 / FPS


def record():
    screen_w, screen_h = pyautogui.size()
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    out    = cv2.VideoWriter(OUTPUT_FILE, fourcc, FPS, (screen_w, screen_h))

    print(f"Recording {DURATION}s → {OUTPUT_FILE}  (Ctrl+C to stop early)")
    start = time.time()

    try:
        while time.time() - start < DURATION:
            frame_start = time.time()

            screenshot = pyautogui.screenshot()
            frame = cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)
            out.write(frame)

            elapsed = time.time() - frame_start
            sleep_for = FRAME_DELAY - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        print("Stopped early.")

    out.release()
    print(f"Saved: {OUTPUT_FILE}")


if __name__ == "__main__":
    record()
