#!/usr/bin/env python3
import serial
import time

# ==== CONFIG ====
PORT = "/dev/ttyACM0"   # <-- change if different
BAUD = 115200
MAX_PWM = 180           # max turning speed
FWD_PWM = 120           # forward test speed
STEP_TIME = 2.0         # seconds for each motion
PAUSE_TIME = 1.0        # pause between motions

def send_command(ser, m1, m2, label=""):
    """Send signed PWM values to both motors"""
    line = f"{m1},{m2}\n"
    ser.write(line.encode())
    ser.flush()
    print(f"[CMD] {label} -> m1={m1:+d}, m2={m2:+d}")

def main():
    print("[INFO] Motor test starting...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=0.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(2.0)  # allow Arduino reset
        print(f"[INFO] Opened serial {PORT} @ {BAUD}")
    except Exception as e:
        print(f"[ERROR] Could not open serial port {PORT}: {e}")
        return

    try:
        # --- Step 1: Forward ---
        print("\n[TEST] Moving forward (both wheels forward)")
        send_command(ser, +FWD_PWM, +FWD_PWM, "Forward")
        time.sleep(STEP_TIME)

        # --- Step 2: Stop ---
        print("[TEST] Stop motors")
        send_command(ser, 0, 0, "Stop")
        time.sleep(PAUSE_TIME)

        # --- Step 3: Full Left Turn (180°) ---
        print("[TEST] Turning LEFT 180°")
        send_command(ser, -MAX_PWM, +MAX_PWM, "Left turn")
        time.sleep(STEP_TIME)

        # --- Step 4: Stop ---
        send_command(ser, 0, 0, "Stop")
        time.sleep(PAUSE_TIME)

        # --- Step 5: Full Right Turn (180°) ---
        print("[TEST] Turning RIGHT 180°")
        send_command(ser, +MAX_PWM, -MAX_PWM, "Right turn")
        time.sleep(STEP_TIME)

        # --- Step 6: Stop ---
        send_command(ser, 0, 0, "Stop")
        print("\n[INFO] Motor test complete!")

    except KeyboardInterrupt:
        print("\n[INFO] Test interrupted by user.")
        send_command(ser, 0, 0, "Emergency stop")
    finally:
        try:
            send_command(ser, 0, 0, "Final stop")
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
