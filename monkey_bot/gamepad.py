from evdev import InputDevice, ecodes
import serial
import time

# Open gamepad
device_path = '/dev/input/by-id/usb-Logitech_Gamepad_F310_DE247C20-event-joystick'
gamepad = InputDevice(device_path)

# Open Arduino serial
arduino = serial.Serial('/dev/ttyACM0', 9600)

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
prev_cmd = None  # prevent flooding

def map_axis(val, deadzone=4000):
    """Map raw joystick value [-32768..32767] to [-255..255] with deadzone."""
    if abs(val) < deadzone:
        return 0
    return int(-val / 32768 * 255)

try:
    for event in gamepad.read_loop():
        if event.type == ecodes.EV_ABS and event.code in axis_map:
            axis_name = axis_map[event.code]

            if axis_name == "RIGHT_Y":  # Motor 1
                m1_val = map_axis(event.value)
            elif axis_name == "LEFT_Y":  # Motor 2
                m2_val = map_axis(event.value)

            # Send to Arduino if values changed
            cmd = f"{m1_val},{m2_val}\n"
            if cmd != prev_cmd:
                arduino.write(cmd.encode())
                prev_cmd = cmd

            time.sleep(0.02)  # throttle ~50 Hz

        elif event.type == ecodes.EV_KEY and event.code in button_map:
            # Emergency stop on "A" button press
            if button_map[event.code] == "A" and event.value:
                m1_val, m2_val = 0, 0
                cmd = f"{m1_val},{m2_val}\n"
                arduino.write(cmd.encode())
                prev_cmd = cmd

except KeyboardInterrupt:
    pass

finally:
    arduino.close()
