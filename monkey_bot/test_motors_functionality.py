# test_motors_functionality.py
import sys, os, time, serial, select, termios, tty

PORT = "/dev/ttyACM0"   # <- set to the SAME port you use with the camera script
BAUD = 9600          # <- set to the SAME baud as your Arduino/camera script

def send(m1, m2, ser):
    line = f"{m1},{m2}\n"
    ser.write(line.encode())
    print("->", line.strip())

def kbhit_char():
    """Return a single char if available on stdin, else None (no sudo required)."""
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

def arduino_readlines_nonblocking(ser, max_lines=5):
    """Read a few lines without blocking, useful for 'OK m1,m2' echoes."""
    ser.timeout = 0
    lines = 0
    while ser.in_waiting and lines < max_lines:
        try:
            print("<-", ser.readline().decode(errors="ignore").rstrip())
        except Exception:
            break
        lines += 1

def main():
    # set terminal to raw so we can read single keypresses
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    print("Starting motor test loop...")
    print("Press 'C' to stop.\n")
    try:
        with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
            time.sleep(2.0)  # let Arduino reset on open

            def wait_or_break(seconds):
                t0 = time.time()
                while time.time() - t0 < seconds:
                    arduino_readlines_nonblocking(ser)
                    ch = kbhit_char()
                    if ch and ch.lower() == 'c':
                        print("\n[C] pressed — stopping.")
                        return True
                    time.sleep(0.03)
                return False

            while True:
                print("Stop");          send(0, 0, ser)
                if wait_or_break(1.0): break

                print("Forward");       send(120, 120, ser)
                if wait_or_break(1.5): break

                # Use biased turns so both wheels move (helps confirm motion)
                print("Turn Right");    send(160, 80, ser)
                if wait_or_break(1.5): break

                print("Turn Left");     send(80, 160, ser)
                if wait_or_break(1.5): break

                print("Reverse");       send(-120, -120, ser)
                if wait_or_break(1.0): break

                print("Stop");          send(0, 0, ser)
                if wait_or_break(0.5): break

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    finally:
        # restore terminal no matter what
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        try:
            with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
                ser.write(b"0,0\n")
        except Exception:
            pass
        print("Motors stopped, serial closed.")

if __name__ == "__main__":
    main()