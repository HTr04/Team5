from evdev import InputDevice, categorize, ecodes

device_path = '/dev/input/by-id/usb-Logitech_Gamepad_F310_B92AFE6C-event-joystick'
gamepad = InputDevice(device_path)

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

for event in gamepad.read_loop():
    if event.type == ecodes.EV_ABS and event.code in axis_map:
        print(f"{axis_map[event.code]} = {event.value}")
    elif event.type == ecodes.EV_KEY and event.code in button_map:
        state = "Pressed" if event.value else "Released"
        print(f"{button_map[event.code]} {state}")

