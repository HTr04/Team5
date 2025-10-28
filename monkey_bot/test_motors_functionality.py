import time, serial,keyboard

PORT = "/dev/ttyACM0"   # adjust as needed
BAUD = 115200

def send(m1, m2, ser):
  line = f"{m1},{m2}\n"
  ser.write(line.encode())
  print("->", line.strip())

with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
    time.sleep(2.0)  # let Arduino reset
    print("Press 'C' to stop the loop at any time.\n")
    print ("Starting motor test loop...")
    try:
        while True:
            # Stop
            print("Stop")
            send(0, 0, ser); time.sleep(1.0)

            # Forward moderate
            print("Forward")
            send(120, 120, ser); time.sleep(1.5)

            # Turn right (left faster than right)
            print("Turn Right")
            send(160, 0, ser); time.sleep(1.5)

            # Turn left
            print("Turn Left")
            send(0, 160, ser); time.sleep(1.5)

            # Reverse
            print("Reverse")
            send(-120, -120, ser); time.sleep(1.0)

            # Stop briefly
            print("Stop")
            send(0, 0, ser); time.sleep(0.5)

            # check for 'C' key press
            if keyboard.is_pressed('c'):
                print("\n[C] pressed — stopping loop.")
                break

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt — exiting.")
    finally:
        # safety stop on exit
        send(0, 0, ser)
        print("Motors stopped, serial closed.")