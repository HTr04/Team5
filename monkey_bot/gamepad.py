# ...existing code...
from evdev import InputDevice, ecodes
import serial
import time

# Open gamepad
device_path = '/dev/input/by-id/usb-Logitech_Gamepad_F310_DE247C20-event-joystick'
gamepad = InputDevice(device_path)

# Open Arduino serial (normal baud)
arduino = serial.Serial('/dev/ttyACM0', 9600, write_timeout=0)

axis_map = {
    ecodes.ABS_X: "LEFT_X",
    ecodes.ABS_Y: "LEFT_Y",
    ecodes.ABS_RX: "RIGHT_X",
    ecodes.ABS_RY: "RIGHT_Y",
    ecodes.ABS_Z: "LT",
    ecodes.ABS_RZ: "RT",
    ecodes.ABS_HAT0X: "DPAD_X",
    ecodes.ABS_HAT0Y: "DPAD_Y",
}

button_map = {
    ecodes.BTN_SOUTH: "A",
    ecodes.BTN_EAST: "B",
    ecodes.BTN_NORTH: "Y",
    ecodes.BTN_WEST: "X",
    ecodes.BTN_TL: "LB",
    ecodes.BTN_TR: "RB",
    ecodes.BTN_SELECT: "BACK",
    ecodes.BTN_START: "START",
    ecodes.BTN_THUMBL: "LEFT_STICK_PRESS",
    ecodes.BTN_THUMBR: "RIGHT_STICK_PRESS",
}

# Store current motor values
m1_val = 0
m2_val = 0
prev_cmd = None  # last command sent
last_send = 0.0
last_print = 0.0
SEND_PERIOD = 1 / 50.0   # max 50 Hz to Arduino
PRINT_PERIOD = 0.25      # print at most 4 Hz

def map_axis(val, deadzone=4000):
    """Map raw joystick value [-32768..32767] to [-255..255] with deadzone."""
    if abs(val) < deadzone:
        return 0
    return int(-val / 32768 * 255)

def try_send(m1, m2):
    """Rate-limit serial writes to avoid flooding/blocking."""
    global prev_cmd, last_send
    now = time.monotonic()
    cmd = f"{m1},{m2}\n"
    if cmd != prev_cmd and (now - last_send) >= SEND_PERIOD:
        try:
            arduino.write(cmd.encode('ascii'))
            prev_cmd = cmd
            last_send = now
        except serial.SerialTimeoutException:
            # Skip if USB buffer is busy; we'll try again on next event tick
            pass

try:
    for event in gamepad.read_loop():
        if event.type == ecodes.EV_ABS and event.code in axis_map:
            axis_name = axis_map[event.code]
            if axis_name == "RIGHT_Y":  # Motor 1
                m1_val = map_axis(event.value)
            elif axis_name == "LEFT_Y":  # Motor 2
                m2_val = map_axis(event.value)

            try_send(m1_val, m2_val)

            # Throttled printing removed to avoid any stdout activity

        elif event.type == ecodes.EV_KEY and event.code in button_map:
            state = "Pressed" if event.value else "Released"
            btn = button_map[event.code]

            # Emergency stop on "A" button press
            if btn == "A" and state == "Pressed":
                m1_val, m2_val = 0, 0
                prev_cmd = None  # force send even if same as last
                try_send(0, 0)

except KeyboardInterrupt:
    pass
finally:
    try:
        arduino.close()
    except Exception:
        pass
# ...existing code...