import depthai as dai
import cv2
import numpy as np
import time
import serial
from utils.arguments import initialize_argparser

# --- helper functions ---
def clamp(val, lo, hi):
    return lo if val < lo else hi if val > hi else val

def detect_colors_hsv(frame, draw=False):
    """Detect red and blue colors in the frame using HSV color space."""
    if len(frame.shape) == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    frame_height, frame_width = frame.shape[:2]
    frame_center_x = frame_width // 2
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    red_lower1, red_upper1 = np.array([0, 120, 70]), np.array([10, 255, 255])
    red_lower2, red_upper2 = np.array([170, 120, 70]), np.array([180, 255, 255])
    blue_lower, blue_upper = np.array([100, 150, 50]), np.array([130, 255, 255])

    red_mask = cv2.bitwise_or(cv2.inRange(hsv, red_lower1, red_upper1),
                              cv2.inRange(hsv, red_lower2, red_upper2))
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel), cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel), cv2.MORPH_OPEN, kernel)

    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    red_centroid, blue_centroid = None, None
    for contour in red_contours:
        if cv2.contourArea(contour) > 500:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                red_centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                break
    for contour in blue_contours:
        if cv2.contourArea(contour) > 500:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                blue_centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                break

    steering_value = None
    if red_centroid and blue_centroid:
        midpoint_x = (red_centroid[0] + blue_centroid[0]) // 2
        steering_value = (midpoint_x - frame_center_x) / (frame_width / 2)
    return frame if draw else None, steering_value

# --- parse CLI arguments ---
_, args = initialize_argparser()

# --- build simple pipeline ---
pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(640, 400)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam.setFps(args.fps_limit if args.fps_limit else 30)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("color")
cam.preview.link(xout.input)

# --- open device with this pipeline ---
if args.device:
    device_info = dai.DeviceInfo(args.device)
    device_context = dai.Device(pipeline, device_info)
else:
    device_context = dai.Device(pipeline)

with device_context as device:
    print("Device Information:", device.getDeviceInfo())
    color_queue = device.getOutputQueue(name="color", maxSize=1, blocking=False)
    print("Color detection queue created successfully")

    # Serial setup
    ser = None
    try:
        ser = serial.Serial(args.serial_port, args.baud_rate=115200, timeout=0.1)
        time.sleep(2.0)
        print(f"Opened serial {args.serial_port} @ {args.baud_rate}")
    except Exception as e:
        print(f"WARNING: Could not open serial port {args.serial_port}: {e}")

    def send_turn_command(steering):
        if ser is None:
            return
        if steering is None:
            m1 = m2 = 0
        else:
            s = -steering if args.invert_turn else steering
            if abs(s) < args.deadzone:
                m1 = m2 = 0
            else:
                pwm = round(args.kp_turn * s)
                if pwm != 0 and abs(pwm) < args.min_turn_pwm:
                    pwm = args.min_turn_pwm if pwm > 0 else -args.min_turn_pwm
                pwm = clamp(int(pwm), -args.max_turn_pwm, args.max_turn_pwm)
                forward_bias = getattr(args, 'forward_bias', 30)
                overall_limit = args.max_turn_pwm + abs(int(forward_bias))
                m1 = clamp(int(-pwm + forward_bias), -overall_limit, overall_limit)
                m2 = clamp(int(pwm + forward_bias), -overall_limit, overall_limit)
        try:
            ser.write(f"{m1},{m2}\n".encode())
        except Exception:
            pass

    print("Starting headless color-based centering. Press Ctrl+C to stop.")
    frames = 0
    last_fps_log = cv2.getTickCount()
    last_steer = None
    last_send = 0.0
    send_interval = 1.0 / max(1, args.command_rate_hz)

    try:
        while True:
            in_frame = color_queue.tryGet()
            if in_frame is None:
                time.sleep(0.005)
                continue

            frame = in_frame.getCvFrame()
            _, steering_value = detect_colors_hsv(frame, draw=False)
            if steering_value is not None:
                direction = "CENTERED" if abs(steering_value) < 0.05 else ("LEFT" if steering_value < 0 else "RIGHT")
                print(f"[STEER] value={steering_value:+.3f} dir={direction}")
                last_steer = (steering_value, direction)
            else:
                print("[STEER] N/A (both objects not detected)")

            now = time.time()
            if now - last_send >= send_interval:
                send_turn_command(steering_value)
                last_send = now

            frames += 1
            tick_now = cv2.getTickCount()
            elapsed = (tick_now - last_fps_log) / cv2.getTickFrequency()
            if elapsed >= 1.0:
                fps = frames / elapsed
                if last_steer is not None:
                    sv, dr = last_steer
                    print(f"[STATUS] fps={fps:.1f} | steer={sv:+.3f} ({dr})")
                else:
                    print(f"[STATUS] fps={fps:.1f} | steer=N/A")
                frames = 0
                last_fps_log = tick_now

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

print("Color centering stopped.")