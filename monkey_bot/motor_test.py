#!/usr/bin/env python3
import time
import serial
import argparse

# ---------- SAME HELPERS AS YOUR MAIN SCRIPT ----------
def clamp(val, lo, hi):
    return lo if val < lo else hi if val > hi else val

def make_send_turn_command(ser, args):
    """
    Returns a send_turn_command(steering) function that behaves exactly
    like your earlier code: maps steering [-1..1] to (m1,m2) with
    kp/min/max/deadzone/invert_turn/forward_bias, then writes "m1,m2\\r\\n" (or \\n).
    """
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
                # turning component
                pwm = round(args.kp_turn * s)
                if pwm != 0 and abs(pwm) < args.min_turn_pwm:
                    pwm = args.min_turn_pwm if pwm > 0 else -args.min_turn_pwm
                pwm = clamp(int(pwm), -args.max_turn_pwm, args.max_turn_pwm)

                # add forward bias (both motors same sign) to bias forward motion
                forward_bias = args.forward_bias
                m1 = -pwm + forward_bias   # right motor
                m2 =  pwm + forward_bias   # left  motor

                # overall clamp allows forward_bias on top of max_turn_pwm
                overall_limit = args.max_turn_pwm + abs(int(forward_bias))
                m1 = clamp(int(m1), -overall_limit, overall_limit)
                m2 = clamp(int(m2), -overall_limit, overall_limit)
                reason = f"turn={'RIGHT' if s>0 else 'LEFT'} pwm={abs(pwm)} fwd={forward_bias}"

        try:
            line = f"{m1},{m2}\r\n" if args.crlf else f"{m1},{m2}\n"
            ser.write(line.encode())
            ser.flush()
            print(f"[CMD] m1={m1:+d}, m2={m2:+d} | {reason}")
        except Exception as e:
            print(f"[WARN] serial write failed: {e}")

    return send_turn_command

# ---------- TEST SEQUENCE ----------
def main():
    ap = argparse.ArgumentParser(description="Motor test using SAME move logic as main script.")
    ap.add_argument("--serial_port", default="/dev/ttyACM0")
    ap.add_argument("--baud_rate", type=int, default=115200)

    # SAME behavior knobs
    ap.add_argument("--kp_turn", type=int, default=180)
    ap.add_argument("--min_turn_pwm", type=int, default=25)
    ap.add_argument("--max_turn_pwm", type=int, default=200)
    ap.add_argument("--deadzone", type=float, default=0.05)
    ap.add_argument("--invert_turn", action="store_true", default=False)
    ap.add_argument("--forward_bias", type=int, default=120, help=">0 to roll forward while steering; set 0 for pure in-place turns")

    # line ending for Arduino parser
    ap.add_argument("--crlf", type=int, default=1, help="1 = send \\r\\n, 0 = send \\n")

    # test timing
    ap.add_argument("--forward_time", type=float, default=2.0, help="seconds to go straight")
    ap.add_argument("--rotate_time", type=float, default=2.0, help="seconds to rotate ~180° (tune)")
    ap.add_argument("--pause_time", type=float, default=1.0, help="pause between steps")
    ap.add_argument("--rate_hz", type=float, default=20.0, help="command refresh rate while holding a move")

    args = ap.parse_args()
    args.crlf = bool(args.crlf)

    print("[INFO] Motor test (same move logic) starting...")
    print("[ARGS]", vars(args))

    # Open serial
    ser = None
    try:
        ser = serial.Serial(
            args.serial_port,
            args.baud_rate,
            timeout=0.05,
            write_timeout=0.2,
        )
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(2.0)  # Arduino resets on open
        print(f"[INFO] Opened serial {args.serial_port} @ {args.baud_rate}")
    except Exception as e:
        print(f"[ERROR] Could not open serial port {args.serial_port}: {e}")
        return

    send_turn_command = make_send_turn_command(ser, args)

    # helper to hold a steering command for some seconds at a given refresh rate
    def hold(steering, seconds):
        period = 1.0 / max(1.0, float(args.rate_hz))
        t0 = time.time()
        while (time.time() - t0) < seconds:
            send_turn_command(steering)
            time.sleep(period)

    try:
        # 1) Forward (uses forward_bias) – steering=0 means no turn, only bias
        print("\n[TEST] FORWARD: moving straight using forward_bias")
        hold(0.0, args.forward_time)

        # Stop
        print("[TEST] STOP")
        hold(None, 0.3)
        time.sleep(args.pause_time)

        # 2) Left turn in place (~180°) – set forward_bias=0 temporarily to match “full turn”
        print("[TEST] LEFT TURN ~180° (in-place)")
        saved_bias = args.forward_bias
        args.forward_bias = 0
        hold(-1.0, args.rotate_time)  # steering=-1 is full LEFT
        args.forward_bias = saved_bias

        # Stop
        print("[TEST] STOP")
        hold(None, 0.3)
        time.sleep(args.pause_time)

        # 3) Right turn in place (~180°)
        print("[TEST] RIGHT TURN ~180° (in-place)")
        saved_bias = args.forward_bias
        args.forward_bias = 0
        hold(+1.0, args.rotate_time)  # steering=+1 is full RIGHT
        args.forward_bias = saved_bias

        # Final stop
        print("[TEST] FINAL STOP")
        hold(None, 0.5)
        print("\n[INFO] Motor test complete.")

    except KeyboardInterrupt:
        print("\n[INFO] Test interrupted by user.")
        hold(None, 0.5)
    finally:
        try:
            hold(None, 0.2)
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
