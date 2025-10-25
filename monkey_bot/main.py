#update 2:52 version
import depthai as dai
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

#test
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
    # Convert to BGR format if needed (from NV12)
    if len(frame.shape) == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    
    # Get frame dimensions
    frame_height, frame_width = frame.shape[:2]
    frame_center_x = frame_width // 2
    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define HSV ranges for red and blue
    # Red needs two ranges as it wraps around 0
    red_lower1 = np.array([0, 120, 70])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 120, 70])
    red_upper2 = np.array([180, 255, 255])
    
    # Blue range
    blue_lower = np.array([100, 150, 50])
    blue_upper = np.array([130, 255, 255])
    
    # Create masks for each color
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    
    # Apply morphological operations to clean up the masks
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    
    # Find contours for red objects
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find contours for blue objects
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Variables to store centroids
    red_centroid = None
    blue_centroid = None
    
    # Draw outlines and labels for red objects
    for contour in red_contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Filter small contours
            if draw:
                # Draw contour outline
                cv2.drawContours(frame, [contour], -1, (0, 0, 255), 3)
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                # Draw bounding box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                # Add label
                cv2.putText(frame, "RED", (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            
            # Draw centroid
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if draw:
                    cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                # Store the first (largest) red centroid
                if red_centroid is None:
                    red_centroid = (cx, cy)
    
    # Draw outlines and labels for blue objects
    for contour in blue_contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Filter small contours
            if draw:
                # Draw contour outline
                cv2.drawContours(frame, [contour], -1, (255, 0, 0), 3)
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                # Draw bounding box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                # Add label
                cv2.putText(frame, "BLUE", (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
            
            # Draw centroid
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if draw:
                    cv2.circle(frame, (cx, cy), 7, (255, 0, 0), -1)
                # Store the first (largest) blue centroid
                if blue_centroid is None:
                    blue_centroid = (cx, cy)
    
    # Calculate midpoint and steering value
    steering_value = None
    if red_centroid and blue_centroid:
        # Calculate midpoint between red and blue
        midpoint_x = (red_centroid[0] + blue_centroid[0]) // 2
        midpoint_y = (red_centroid[1] + blue_centroid[1]) // 2
        
        if draw:
            # Draw line between red and blue centroids
            cv2.line(frame, red_centroid, blue_centroid, (0, 255, 0), 3)
            # Draw the midpoint
            cv2.circle(frame, (midpoint_x, midpoint_y), 10, (0, 255, 0), -1)
            cv2.putText(frame, "MIDPOINT", (midpoint_x - 50, midpoint_y - 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # Draw vertical line at frame center for reference
            cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame_height), (255, 255, 0), 2)
            cv2.putText(frame, "CENTER", (frame_center_x - 40, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Calculate steering value
        # Normalize to -1.0 to 1.0 range based on frame width
        steering_value = (midpoint_x - frame_center_x) / (frame_width / 2)
        
        if draw:
            # Draw steering indicator
            steering_text = f"Steering: {steering_value:.3f}"
            color = (0, 255, 0) if abs(steering_value) < 0.1 else (0, 165, 255)
            cv2.putText(frame, steering_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            # Draw direction arrow
            arrow_start = (frame_center_x, frame_height - 50)
            arrow_end = (int(frame_center_x + steering_value * 100), frame_height - 50)
            cv2.arrowedLine(frame, arrow_start, arrow_end, color, 3, tipLength=0.3)
    
    # Add detection count info
    red_count = len([c for c in red_contours if cv2.contourArea(c) > 500])
    blue_count = len([c for c in blue_contours if cv2.contourArea(c) > 500])
    
    if draw:
        info_text = f"Red: {red_count} | Blue: {blue_count}"
        cv2.putText(frame, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    return frame if draw else None, steering_value

def clamp(val, lo, hi):
    return lo if val < lo else hi if val > hi else val

_, args = initialize_argparser()

print("[INFO] Headless mode: no GUI windows, SSH-friendly logs only.")

device = dai.Device(dai.DeviceInfo(args.device)) if args.device else dai.Device()
print("Device Information: ", device.getDeviceInfo())

cam_features = {}
for cam in device.getConnectedCameraFeatures():
    cam_features[cam.socket] = (cam.width, cam.height)

with dai.Pipeline(device) as pipeline:
    print("Creating pipeline (headless)...")

    camera_sensors = device.getConnectedCameraFeatures()
    color_detection_output = None

    for sensor in camera_sensors:
        cam = pipeline.create(dai.node.Camera).build(sensor.socket)

        request_resolution = (
            (sensor.width, sensor.height)
            if sensor.width <= 1280 and sensor.height <= 720
            else (1280, 720)
        )  # keep moderate resolution for Pi

        # Only need BGR frames for color detection; no encoders/visualizers
        if sensor.socket == dai.CameraBoardSocket.CAM_A and color_detection_output is None:
            color_detection_output = cam.requestOutput(
                request_resolution, dai.ImgFrame.Type.BGR888p, fps=args.fps_limit
            )

    # Prepare output queue for color detection BEFORE starting the pipeline
    color_queue = None
    if color_detection_output:
        color_queue = color_detection_output.createOutputQueue(maxSize=1, blocking=False)
        print("Color detection queue created successfully")

    print("Pipeline created.")

    # Serial setup for Arduino
    ser = None
    try:
        ser = serial.Serial(args.serial_port, args.baud_rate, timeout=0.1)
        # Small delay for Arduino reset on serial open
        time.sleep(2.0)
        print(f"Opened serial {args.serial_port} @ {args.baud_rate}")
    except Exception as e:
        print(f"WARNING: Could not open serial port {args.serial_port}: {e}")

    def send_turn_command(steering):
        """Map steering (-1..1) to differential turn PWM. No forward motion.
        Contract:
        - Input: steering float in [-1,1], None means no data.
        - Output: writes "m1,m2\n" over serial when available.
        - Behavior: left/right wheels spin opposite directions to rotate in place.
        """
        if ser is None:
            # silently skip when no serial port is available
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
                # turning component (use round instead of truncating int)
                pwm = round(args.kp_turn * s)
                # enforce minimum torque once outside deadzone
                if pwm != 0 and abs(pwm) < args.min_turn_pwm:
                    pwm = args.min_turn_pwm if pwm > 0 else -args.min_turn_pwm
                pwm = clamp(int(pwm), -args.max_turn_pwm, args.max_turn_pwm)

                # small constant forward bias to always move slightly forward while turning
                forward_bias = getattr(args, 'forward_bias', 30)

                # compute motor outputs: turning is differential, forward_bias adds same sign to both
                # keep sign conventions: m1 = right motor, m2 = left motor
                m1 = -pwm + forward_bias
                m2 = pwm + forward_bias
                # clamp to safe overall range (allow forward_bias on top of max_turn_pwm)
                overall_limit = args.max_turn_pwm + abs(int(forward_bias))
                m1 = clamp(int(m1), -overall_limit, overall_limit)
                m2 = clamp(int(m2), -overall_limit, overall_limit)
                reason = f"turn={'RIGHT' if s>0 else 'LEFT'} pwm={abs(pwm)} fwd={forward_bias}"

        try:
            cmd = f"{m1},{m2}\n"
            ser.write(cmd.encode())
            # keep logs minimal for SSH-friendly behavior
        except Exception:
            # ignore serial write errors during runtime
            pass

    # Main loop
    pipeline.start()
    print("Starting headless color-based centering. Press Ctrl+C to stop.")

    frames = 0
    last_fps_log = cv2.getTickCount()
    last_steer = None

    last_send = 0.0
    send_interval = 1.0 / max(1, args.command_rate_hz)

    try:
        while pipeline.isRunning():
            if not color_queue:
                time.sleep(0.05)
                continue

            in_frame = color_queue.tryGet()
            if in_frame is None:
                # No frame ready yet
                time.sleep(0.005)
                continue

            frame = in_frame.getCvFrame()
            _, steering_value = detect_colors_hsv(frame, draw=False)

            # Print steering info (SSH-friendly)
            if steering_value is not None:
                direction = (
                    "CENTERED" if abs(steering_value) < 0.05 else ("LEFT" if steering_value < 0 else "RIGHT")
                )
                print(f"[STEER] value={steering_value:+.3f} dir={direction}")
                last_steer = (steering_value, direction)
            else:
                print("[STEER] N/A (both objects not detected)")

            now = time.time()
            if now - last_send >= send_interval:
                send_turn_command(steering_value)
                last_send = now

            # Periodic FPS/status log
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
        # Stop motors on exit for safety
        try:
            if 'ser' in locals() and ser:
                ser.write(b"0,0\n")
                time.sleep(0.05)
                ser.close()
        except Exception:
            pass

print("Color centering stopped.")