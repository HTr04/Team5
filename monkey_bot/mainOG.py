import depthai as dai
import cv2
import numpy as np
from utils.arguments import initialize_argparser

def detect_colors_hsv(frame):
    """
    Detect red and blue colors in the frame using HSV color space
    Returns the processed frame and the steering value
    """
    if len(frame.shape) == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    
    frame_height, frame_width = frame.shape[:2]
    frame_center_x = frame_width // 2
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # HSV ranges for red
    red_lower1 = np.array([0, 120, 70])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 120, 70])
    red_upper2 = np.array([180, 255, 255])
    
    # HSV range for blue
    blue_lower = np.array([100, 150, 50])
    blue_upper = np.array([130, 255, 255])
    
    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, red_lower1, red_upper1),
        cv2.inRange(hsv, red_lower2, red_upper2)
    )
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    red_centroid = None
    blue_centroid = None

    # Process red contours
    for contour in red_contours:
        if cv2.contourArea(contour) > 500:
            M = cv2.moments(contour)
            if M["m00"] != 0 and red_centroid is None:
                red_centroid = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))

    # Process blue contours
    for contour in blue_contours:
        if cv2.contourArea(contour) > 500:
            M = cv2.moments(contour)
            if M["m00"] != 0 and blue_centroid is None:
                blue_centroid = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
    
    # Compute steering
    steering_value = None
    if red_centroid and blue_centroid:
        midpoint_x = (red_centroid[0] + blue_centroid[0]) // 2
        steering_value = (midpoint_x - frame_center_x) / (frame_width / 2)
    
    return frame, steering_value

# Initialize arguments
_, args = initialize_argparser()

# Force headless mode
debug_viz = False
print("[INFO] Headless mode: no GUI windows, SSH-friendly logs only.")

# Connect to DepthAI device
device = dai.Device(dai.DeviceInfo(args.device)) if args.device else dai.Device()
print("Device Information:", device.getDeviceInfo())

# Prepare camera pipeline
with dai.Pipeline(device) as pipeline:
    color_detection_output = None
    camera_sensors = device.getConnectedCameraFeatures()

    for sensor in camera_sensors:
        cam = pipeline.create(dai.node.Camera).build(sensor.socket)
        request_resolution = (
            (sensor.width, sensor.height)
            if sensor.width <= 1920 and sensor.height <= 1080
            else (1920, 1080)
        )
        if sensor.socket == dai.CameraBoardSocket.CAM_A and color_detection_output is None:
            color_detection_output = cam.requestOutput(
                request_resolution, dai.ImgFrame.Type.BGR888p, fps=args.fps_limit
            )

    # Create output queue
    color_queue = None
    if color_detection_output:
        color_queue = color_detection_output.createOutputQueue(maxSize=1, blocking=False)
        print("Color detection queue created successfully")

    pipeline.start()
    print("Starting color detection. Press Ctrl+C to quit.")

    frames = 0
    last_fps_log = cv2.getTickCount()
    last_steer = None

    try:
        while pipeline.isRunning() and color_queue:
            in_frame = color_queue.tryGet()
            if in_frame:
                frame = in_frame.getCvFrame()
                processed_frame, steering_value = detect_colors_hsv(frame)

                # Print steering info
                if steering_value is not None:
                    direction = "CENTERED" if abs(steering_value) < 0.05 else ("LEFT" if steering_value < 0 else "RIGHT")
                    print(f"[STEER] value={steering_value:+.3f} dir={direction}")
                    last_steer = (steering_value, direction)
                else:
                    print("[STEER] N/A (both objects not detected)")

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
        print("Color detection stopped by user.")