# --- create device (optionally from args.device) ---
if args.device:
    device_info = dai.DeviceInfo(args.device)
    device = dai.Device(device_info)
else:
    device = dai.Device()

print("Device Information: ", device.getDeviceInfo())

# --- build a simple pipeline that streams BGR frames to host ---
pipeline = dai.Pipeline()

# Color camera (works on OAK-D / CAM_A). If you only have a mono cam, switch to dai.node.MonoCamera.
cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(640, 400)                # modest resolution for Pi
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam.setFps(args.fps_limit if args.fps_limit else 30)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("color")
cam.preview.link(xout.input)

# --- open device with this pipeline ---
with dai.Device(pipeline) as device:
    print("Pipeline created and started.")

    # Output queue for color frames
    color_queue = device.getOutputQueue(name="color", maxSize=1, blocking=False)
    print("Color detection queue created successfully")

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
            m1 = 0; m2 = 0; reason = "no-detections -> stop"
        else:
            s = -steering if args.invert_turn else steering
            if abs(s) < args.deadzone:
                m1 = 0; m2 = 0; reason = "deadzone -> stop"
            else:
                pwm = round(args.kp_turn * s)
                if pwm != 0 and abs(pwm) < args.min_turn_pwm:
                    pwm = args.min_turn_pwm if pwm > 0 else -args.min_turn_pwm
                pwm = clamp(int(pwm), -args.max_turn_pwm, args.max_turn_pwm)
                forward_bias = getattr(args, 'forward_bias', 30)
                m1 = clamp(int(-pwm + forward_bias), -(args.max_turn_pwm+abs(int(forward_bias))), (args.max_turn_pwm+abs(int(forward_bias))))
                m2 = clamp(int( pwm + forward_bias), -(args.max_turn_pwm+abs(int(forward_bias))), (args.max_turn_pwm+abs(int(forward_bias))))
                reason = f"turn={'RIGHT' if s>0 else 'LEFT'} pwm={abs(pwm)} fwd={forward_bias}"
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

            frame = in_frame.getCvFrame()  # BGR frame from preview
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
            if 'ser' in locals() and ser:
                ser.write(b"0,0\n")
                time.sleep(0.05)
                ser.close()
        except Exception:
            pass

print("Color centering stopped.")