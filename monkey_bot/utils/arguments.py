import argparse


def initialize_argparser():
    """Initialize the argument parser for the script."""
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        "-d",
        "--device",
        help="Optional name, DeviceID or IP of the camera to connect to.",
        required=False,
        default=None,
        type=str,
    )

    parser.add_argument(
        "-fps",
        "--fps_limit",
        help="FPS limit for the model runtime.",
        required=False,
        default=30,
        type=int,
    )

    # Serial/controls for Arduino motor driver
    parser.add_argument(
        "--serial_port",
        help="Serial port to Arduino (e.g., /dev/ttyACM0, /dev/ttyUSB0)",
        required=False,
        default="/dev/ttyACM0",
        type=str,
    )

    parser.add_argument(
        "--baud_rate",
        help="Baud rate for Arduino serial.",
        required=False,
        default=9600,
        type=int,
    )

    parser.add_argument(
        "--kp_turn",
        help="Proportional gain mapping steering (-1..1) to turn PWM.",
        required=False,
        default=180.0,
        type=float,
    )

    parser.add_argument(
        "--max_turn_pwm",
        help="Max absolute PWM for turn commands (0..255).",
        required=False,
        default=200,
        type=int,
    )

    parser.add_argument(
        "--min_turn_pwm",
        help="Minimum absolute PWM applied once outside the deadzone to overcome static friction.",
        required=False,
        default=40,
        type=int,
    )

    parser.add_argument(
        "--deadzone",
        help="Deadzone around zero steering to avoid jitter (0..1).",
        required=False,
        default=0.05,
        type=float,
    )

    parser.add_argument(
        "--command_rate_hz",
        help="How often to send serial motor commands (Hz).",
        required=False,
        default=20,
        type=int,
    )

    parser.add_argument(
        "--invert_turn",
        help="Invert turning direction if the robot rotates the wrong way.",
        action="store_true",
    )

    parser.add_argument(
        "--forward_bias",
        help="Small constant forward PWM added during turns to keep slight forward motion.",
        required=False,
        default=30,
        type=int,
    )

    parser.add_argument(
        "--debug_viz",
        help="Enable drawing and extra logs for debugging (slower).",
        action="store_true",
    )

    args = parser.parse_args()

    return parser, args
