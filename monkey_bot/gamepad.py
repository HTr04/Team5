from evdev import InputDevice, categorize, ecodes
import serial

# Open gamepad
device_path = '/dev/input/by-id/usb-Logitech_Gamepad_F310_B92AFE6C-event-joystick'
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

print(f"Listening on {gamepad.path} ({gamepad.name})")

# Store current motor values
m1_val = 0
m2_val = 0

def map_axis(val):
    # Map -32768 → +255 and +32767 → -255
    mapped = int(-val * 255 / 32767)  
    if mapped > 255: mapped = 255
    if mapped < -255: mapped = -255
    return mapped

for event in gamepad.read_loop():
    if event.type == ecodes.EV_ABS and event.code in axis_map:
        axis_name = axis_map[event.code]

        if axis_name == "RIGHT_Y":  # Motor 1
            m1_val = map_axis(event.value)
        elif axis_name == "LEFT_Y":  # Motor 2
            m2_val = map_axis(event.value)

        # Send to Arduino whenever updated
        cmd = f"{m1_val},{m2_val}\n"
        arduino.write(cmd.encode())
        print(f"Motor1={m1_val}, Motor2={m2_val}")

    elif event.type == ecodes.EV_KEY and event.code in button_map:
        state = "Pressed" if event.value else "Released"
        print(f"{button_map[event.code]} {state}")
