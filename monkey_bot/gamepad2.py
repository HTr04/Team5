from evdev import InputDevice, categorize, ecodes
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
    ecodes.BTN_NORTH: "X",
    ecodes.BTN_WEST: "Y",
    ecodes.BTN_TL: "LB",
    ecodes.BTN_TR: "RB",
    ecodes.BTN_SELECT: "BACK",
    ecodes.BTN_START: "START",
    ecodes.BTN_THUMBL: "LEFT_STICK_PRESS",
    ecodes.BTN_THUMBR: "RIGHT_STICK_PRESS",
}

print(f"Listening on {gamepad.path} ({gamepad.name})")

# Store current motor values
m1_val = 0
m2_val = 0
button = ""
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
                print(f"Motor1={m1_val}, Motor2={m2_val}")

            time.sleep(0.02)  # throttle ~50 Hz

        elif event.type == ecodes.EV_KEY and event.code in button_map:
            state = "Pressed" if event.value else "Released"
            print(f"{button_map[event.code]} {state}")

            # Emergency stop on "A" button press
            if button == "A" and state == "Pressed":
                m1_val,m2_val = 0,0
                send_to_arduino(m1_val,m2_val)
                print(f"Motor1={m1_val}, Motor2={m2_val}")

            elif button == "Y" and state == "Pressed":
                m1_val,m2_val = 255,255
                arduino.write(m1_val, m2_val)
                time.sleep(3)
                m1_val,m2_val = 0,0
                arduino.write(m1_val,m2_val)
            else:
                print("none")

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    arduino.close()
    print("Serial connection closed.")
