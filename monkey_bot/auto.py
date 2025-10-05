import serial
import time

# --- Setup ---
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # let Arduino reset

def send_to_arduino(m1, m2, delay=0):
    """Send motor values, optionally wait for some time."""
    cmd = f"{m1},{m2}\n"
    arduino.write(cmd.encode())
    print(f"Sent: {cmd.strip()}")
    if delay > 0:
        time.sleep(delay)

# --- Example sequence ---

# Forward full speed for 3 sec
#send_to_arduino(255, 255, delay=3)

# Stop for 1 sec
#send_to_arduino(0, 0, delay=1)

# Reverse full speed for 2 sec
#send_to_arduino(-255, -255, delay=2)

# Spin in place (left fwd, right back) for 2 sec
#send_to_arduino(255, -255, delay=2)

# Final stop
#send_to_arduino(0, 0)

#for loop square!
for (let, i == 0; i < 5; i++) {
#Forward 1  (Start of square)
send_to_arduino(255, 255, delay=.5)

# Stop for 1 sec
send_to_arduino(0, 0, delay=1)

#Left turn
send_to_arduino(255, -255, delay=.5)

# Stop for 1 sec
send_to_arduino(0, 0, delay=1)}
#Forward 1  (Start of square)
#send_to_arduino(255, 255, delay=.5)

# Stop for 1 sec
#send_to_arduino(0, 0, delay=1)

#Left turn
#send_to_arduino(255, -255, delay=.5)

# Stop for 1 sec
#send_to_arduino(0, 0, delay=1)

#Forward 2  (Start of square)
#send_to_arduino(255, 255, delay=.5)

# Stop for 1 sec
#send_to_arduino(0, 0, delay=1)

#Left turn
#send_to_arduino(255, -255, delay=.5)

#Forward 3  (Start of square)
#send_to_arduino(255, 255, delay=.5)

# Stop for 1 sec
#send_to_arduino(0, 0, delay=1)

#Left turn
#send_to_arduino(255, -255, delay=.5)

# Stop for 1 sec
#send_to_arduino(0, 0, delay=1)

#Forward 3  (Start of square)
#send_to_arduino(255, 255, delay=.5)

#Left turn
#send_to_arduino(255, -255, delay=.5)

# Stop for 1 sec
#send_to_arduino(0, 0, delay=1)

arduino.close()
print("Serial closed.")
