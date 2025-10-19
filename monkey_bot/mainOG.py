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

import numpy as np
from utils.arguments import initialize_argparser

def detect_colors_hsv(frame):
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
                cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                # Store the first (largest) red centroid
                if red_centroid is None:
                    red_centroid = (cx, cy)
    
    # Draw outlines and labels for blue objects
    for contour in blue_contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Filter small contours
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
        
        # Draw steering indicator
        steering_text = f"Steering: {steering_value:.3f}"
        color = (0, 255, 0) if abs(steering_value) < 0.1 else (0, 165, 255)  # Green if centered, orange otherwise
        cv2.putText(frame, steering_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        # Draw direction arrow
        arrow_start = (frame_center_x, frame_height - 50)
        arrow_end = (int(frame_center_x + steering_value * 100), frame_height - 50)
        cv2.arrowedLine(frame, arrow_start, arrow_end, color, 3, tipLength=0.3)
    
    # Add detection count info
    red_count = len([c for c in red_contours if cv2.contourArea(c) > 500])
    blue_count = len([c for c in blue_contours if cv2.contourArea(c) > 500])
    
    info_text = f"Red: {red_count} | Blue: {blue_count}"
    cv2.putText(frame, info_text, (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    return frame, steering_value

def process_frame_from_encoder(encoder_packet):
    """
    Decode H264 packet to get raw frame for processing
    """
    # This is a workaround since we're getting H264 encoded data
    # In practice, you might need to decode the H264 stream
    # For now, we'll skip frames that can't be processed
    return None

_, args = initialize_argparser()

# Headless by default (no windows/visualizer) unless explicitly debugging
debug_viz = getattr(args, "debug_viz", False)
if debug_viz:
    visualizer = dai.RemoteConnection(httpPort=8082)
    print("[INFO] Debug visualization ENABLED: windows and remote stream may open.")
else:
    visualizer = None
    print("[INFO] Headless mode: no GUI windows, SSH-friendly logs only.")
device = dai.Device(dai.DeviceInfo(args.device)) if args.device else dai.Device()
print("Device Information: ", device.getDeviceInfo())

cam_features = {}
for cam in device.getConnectedCameraFeatures():
    cam_features[cam.socket] = (cam.width, cam.height)

# Create HSV tuning UI only when debugging locally
if debug_viz:
    cv2.namedWindow('HSV Tuning')
    cv2.createTrackbar('H Min', 'HSV Tuning', 0, 180, lambda x: None)
    cv2.createTrackbar('H Max', 'HSV Tuning', 10, 180, lambda x: None)
    cv2.createTrackbar('S Min', 'HSV Tuning', 120, 255, lambda x: None)
    cv2.createTrackbar('S Max', 'HSV Tuning', 255, 255, lambda x: None)
    cv2.createTrackbar('V Min', 'HSV Tuning', 70, 255, lambda x: None)
    cv2.createTrackbar('V Max', 'HSV Tuning', 255, 255, lambda x: None)

with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    output_queues = {}
    camera_sensors = device.getConnectedCameraFeatures()
    color_detection_output = None
    
    for sensor in camera_sensors:
        cam = pipeline.create(dai.node.Camera).build(sensor.socket)

        request_resolution = (
            (sensor.width, sensor.height)
            if sensor.width <= 1920 and sensor.height <= 1080
            else (1920, 1080)
        )  # limit frame size to 1080p
        
        # For the RGB camera, also create a raw output for color detection
        if sensor.socket == dai.CameraBoardSocket.CAM_A and color_detection_output is None:
            # Request BGR output for color detection
            color_detection_output = cam.requestOutput(
                request_resolution, dai.ImgFrame.Type.BGR888p, fps=args.fps_limit
            )
        
        # Create NV12 output and encoder only when visualizing remotely
        if debug_viz:
            cam_out = cam.requestOutput(
                request_resolution, dai.ImgFrame.Type.NV12, fps=args.fps_limit
            )

            encoder = pipeline.create(dai.node.VideoEncoder)
            encoder.setDefaultProfilePreset(
                args.fps_limit, dai.VideoEncoderProperties.Profile.H264_MAIN
            )
            cam_out.link(encoder.input)

            visualizer.addTopic(sensor.socket.name, encoder.out, "images")

    # Get the output queue for color detection BEFORE starting the pipeline
    color_queue = None
    if color_detection_output:
        color_queue = color_detection_output.createOutputQueue(maxSize=1, blocking=False)
        print("Color detection queue created successfully")

    print("Pipeline created.")

    pipeline.start()
    if debug_viz and visualizer is not None:
        visualizer.registerPipeline(pipeline)
    
    print("Starting color detection. Headless=" + ("False" if debug_viz else "True") + 
        ". Press Ctrl+C to quit." + (" Use 'q' in window/remote to exit." if debug_viz else ""))
    screenshot_counter = 0
    last_fps_log = cv2.getTickCount()
    frames = 0
    last_steer = None
    
    while pipeline.isRunning():
        # Process frames for color detection if we have a queue
        if color_queue:
            in_frame = color_queue.tryGet()
            if in_frame:
                # Get the frame
                frame = in_frame.getCvFrame()
                
                # Apply color detection and get steering value
                processed_frame, steering_value = detect_colors_hsv(frame)
                
                # Print steering value to terminal (SSH-friendly)
                if steering_value is not None:
                    # Determine direction
                    if abs(steering_value) < 0.05:
                        direction = "CENTERED"
                    elif steering_value < 0:
                        direction = "LEFT"
                    else:
                        direction = "RIGHT"
                    print(f"[STEER] value={steering_value:+.3f} dir={direction}")
                    last_steer = (steering_value, direction)
                else:
                    print("[STEER] N/A (both objects not detected)")

                # Optional GUI only in debug mode
                if debug_viz:
                    # Display the processed frame locally
                    cv2.imshow('Color Detection - Red/Blue', processed_frame)
                    # Handle keyboard input
                    local_key = cv2.waitKey(1) & 0xFF
                    if local_key == ord('s'):
                        filename = f'color_detection_{screenshot_counter:04d}.png'
                        cv2.imwrite(filename, processed_frame)
                        print(f"Screenshot saved as {filename}")
                        screenshot_counter += 1
                    elif local_key == ord('q'):
                        break

                # Periodic FPS/status log for SSH sessions
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
        
        # Check for remote visualizer key press only in debug mode
        if debug_viz and visualizer is not None:
            key = visualizer.waitKey(1)
            if key == ord("q"):
                print("Got q key from the remote connection!")
                break
if debug_viz:
    cv2.destroyAllWindows()
print("Color detection stopped.")