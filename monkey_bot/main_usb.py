#!/usr/bin/env python3
"""
USB camera variant of main.py — uses a normal OpenCV VideoCapture device
instead of depthai. Keeps the same detection and serial command behavior.
"""
# Attempt to import cv2; if unavailable, try to install opencv-python and re-import.
try:
    import cv2
except Exception:
    import sys
    import subprocess
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "opencv-python"])
        import importlib
        cv2 = importlib.import_module("cv2")
    except Exception as e:
        raise ImportError(
            "The 'cv2' module (OpenCV) is required but could not be imported or installed automatically. "
            "Please install it manually with: pip install opencv-python"
        ) from e

import numpy as np
import time
import math
import serial
from utils.arguments import initialize_argparser


def detect_colors_hsv(frame, draw=False):
    """
    Detect red and blue colors in the frame using HSV color space
    Returns the frame with outlines drawn around detected colors and the steering value
    """
    # Convert to BGR format if needed (grayscale)
    if len(frame.shape) == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    # Get frame dimensions
    frame_height, frame_width = frame.shape[:2]
    frame_center_x = frame_width // 2

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define HSV ranges for red and blue
    red_lower1 = np.array([0, 120, 70])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 120, 70])
    red_upper2 = np.array([180, 255, 255])

    blue_lower = np.array([100, 150, 50])
    blue_upper = np.array([130, 255, 255])

    # Create masks for each color
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

    # Morphology to clean masks
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    red_centroid = None
    blue_centroid = None

    for contour in red_contours:
        area = cv2.contourArea(contour)
        if area > 500:
            if draw:
                cv2.drawContours(frame, [contour], -1, (0, 0, 255), 3)
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, "RED", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if draw:
                    cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                if red_centroid is None:
                    red_centroid = (cx, cy)

    for contour in blue_contours:
        area = cv2.contourArea(contour)
        if area > 500:
            if draw:
                cv2.drawContours(frame, [contour], -1, (255, 0, 0), 3)
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(frame, "BLUE", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if draw:
                    cv2.circle(frame, (cx, cy), 7, (255, 0, 0), -1)
                if blue_centroid is None:
                    blue_centroid = (cx, cy)

    steering_value = None
    if red_centroid and blue_centroid:
        midpoint_x = (red_centroid[0] + blue_centroid[0]) // 2
        midpoint_y = (red_centroid[1] + blue_centroid[1]) // 2
        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width // 2
        steering_value = (midpoint_x - frame_center_x) / (frame_width / 2)
        if draw:
            cv2.line(frame, red_centroid, blue_centroid, (0, 255, 0), 3)
            cv2.circle(frame, (midpoint_x, midpoint_y), 10, (0, 255, 0), -1)
            cv2.putText(frame, "MIDPOINT", (midpoint_x - 50, midpoint_y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame_height), (255, 255, 0), 2)
    return frame if draw else None, steering_value


def clamp(val, lo, hi):
    return lo if val < lo else hi if val > hi else val


def main():
    _, args = initialize_argparser()

    print("[INFO] USB camera mode: using OpenCV VideoCapture")

    # Open video capture (default index 0)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise SystemExit("Could not open default camera (index 0).")

    # Try to set a reasonable resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, args.fps_limit)

    # Serial setup for Arduino
    ser = None
    try:
        ser = serial.Serial(args.serial_port, args.baud_rate, timeout=0.1)
        time.sleep(2.0)
        print(f"Opened serial {args.serial_port} @ {args.baud_rate}")
    except Exception as e:
        print(f"WARNING: Could not open serial port {args.serial_port}: {e}")

    def send_turn_command(steering):
        if ser is None:
            return
        reason = ""
        if steering is None:
            m1 = 0
            m2 = 0
            reason = "no-detections -> stop"
        else:
            s = -steering if args.invert_turn else steering
            if abs(s) < args.deadzone:
                m1 = 0
                m2 = 0
                reason = "deadzone -> stop"
            else:
                pwm = round(args.kp_turn * s)
                if pwm != 0 and abs(pwm) < args.min_turn_pwm:
                    pwm = args.min_turn_pwm if pwm > 0 else -args.min_turn_pwm
                pwm = clamp(int(pwm), -args.max_turn_pwm, args.max_turn_pwm)

                forward_bias = getattr(args, 'forward_bias', 30)
                m1 = -pwm + forward_bias
                m2 = pwm + forward_bias
                overall_limit = args.max_turn_pwm + abs(int(forward_bias))
                m1 = clamp(int(m1), -overall_limit, overall_limit)
                m2 = clamp(int(m2), -overall_limit, overall_limit)
                reason = f"turn={'RIGHT' if s>0 else 'LEFT'} pwm={abs(pwm)} fwd={forward_bias}"

        try:
            cmd = f"{m1},{m2}\n"
            ser.write(cmd.encode())
        except Exception:
            pass

    send_interval = 1.0 / max(1, args.command_rate_hz)
    last_send = 0.0

    try:
        while True:
            start = time.time()
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            _, steering_value = detect_colors_hsv(frame, draw=False)

            # Minimal SSH-friendly logging
            if steering_value is not None:
                direction = (
                    "CENTERED" if abs(steering_value) < 0.05 else ("LEFT" if steering_value < 0 else "RIGHT")
                )
                print(f"[STEER] value={steering_value:+.3f} dir={direction}")

            now = time.time()
            if now - last_send >= send_interval:
                send_turn_command(steering_value)
                last_send = now

            # Respect FPS limit
            elapsed = time.time() - start
            frame_time = 1.0 / max(1, args.fps_limit)
            if elapsed < frame_time:
                time.sleep(frame_time - elapsed)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            if ser:
                ser.write(b"0,0\n")
                time.sleep(0.05)
                ser.close()
        except Exception:
            pass
        cap.release()


if __name__ == '__main__':
    main()
