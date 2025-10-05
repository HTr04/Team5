import cv2
import numpy as np
import time
import serial

# ---------- Constants ----------
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
Frame_center_X = FRAME_WIDTH // 2
Frame_center_Y = FRAME_HEIGHT // 2
W, H = FRAME_WIDTH, FRAME_HEIGHT

# Min contour area to accept a detection  (TUNE)
MIN_CONTOUR_AREA = 800

# Stop rules
MAX_RUNTIME_SEC = 10.0
FILL_STOP_RATIO = 0.50  # stop if largest detected object covers ≥50% of frame

# Fallback offset if only one color is visible (TUNE)
K_STEER_OFFSET_PIX = 35
K_WIDTH_SCALE = 0.35  # portion of width used as virtual spacing

# Steering/forward gains (keep your logic) (TUNE)
k_turn = 0.5
k_forward = 0.01
target_size = 15000  # area target for forward term (kept from your code)

# ---------- Helpers ----------
def clean_mask(mask):
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    return mask

def find_largest_contour(mask):
    """
    Returns (detected_bool, center_x, center_y, area).
    If none found above MIN_CONTOUR_AREA, returns (False, None, None, 0).
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, None, None, 0
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_CONTOUR_AREA:
        return False, None, None, 0
    M = cv2.moments(c)
    if M["m00"] == 0:
        return False, None, None, 0
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return True, cx, cy, int(area)

# ---------- Camera ----------
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():
    raise RuntimeError("Failed to open camera")

# ---------- Serial (open ONCE) ----------
try:
    arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
    time.sleep(2.0)  # allow Arduino to reset
except Exception as e:
    print(f"[WARN] Could not open serial: {e}")
    arduino = None

t0 = time.time()

while True:
    ok, frame = cap.read()
    if not ok:
        print("[ERR] Camera frame grab failed.")
        break

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red Color ranges (two slices)
    lower_red  = np.array([0,   120, 70])
    upper_red  = np.array([10,  255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    red_mask1 = cv2.inRange(hsv, lower_red,  upper_red)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    red_mask = clean_mask(red_mask)

    # Blue Color range (no wrap)
    lower_blue = np.array([100, 150,  0])
    upper_blue = np.array([140, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_mask = clean_mask(blue_mask)

    # ---------- Find LARGEST contour for each color ----------
    red_detected, center_x_red, center_y_red, red_area   = find_largest_contour(red_mask)
    blue_detected, center_x_blue, center_y_blue, blue_area = find_largest_contour(blue_mask)

    # _________ Decide target base x _______________________
    x_target = Frame_center_X  # default if no colors detected
    obj_area = 0

    if red_detected and blue_detected:
        x_target = int(0.5 * (center_x_red + center_x_blue))
        obj_area = max(red_area, blue_area)
    elif red_detected:
        # Assume red is on LEFT, missing blue to the RIGHT
        virtual = min(center_x_red + int(K_WIDTH_SCALE * W) + K_STEER_OFFSET_PIX, W - 1)
        x_target = int(0.5 * (center_x_red + virtual))
        obj_area = red_area
    elif blue_detected:
        # Assume blue is on RIGHT, missing red to the LEFT
        virtual = max(center_x_blue - int(K_WIDTH_SCALE * W) - K_STEER_OFFSET_PIX, 0)
        x_target = int(0.5 * (virtual + center_x_blue))
        obj_area = blue_area
    else:
        # No detection -> keep center
        obj_area = 0

    # Print required x position each loop
    print("x_target:", x_target)

    # ---------- Steering logic (placeholder using x error only) ----------
    # e.g., normalized error in [-1,1]
    e = (x_target - Frame_center_X) / (W / 2.0)
    turn_cmd = k_turn * e

    # forward term toward area target (optional)
    forward_cmd = 0.0
    if obj_area > 0:
        forward_cmd = k_forward * (target_size - obj_area)

    # ---------- Send to Arduino ----------
    if arduino and arduino.writable():
        try:
            payload = f"XT:{x_target},TURN:{turn_cmd:.3f},FWD:{forward_cmd:.3f}\n"
            arduino.write(payload.encode('ascii'))
        except Exception as e:
            print(f"[WARN] Serial write failed: {e}")

    # ---------- Stop rules ----------
    # 1) Time-based stop
    if (time.time() - t0) >= MAX_RUNTIME_SEC:
        print("[INFO] Max runtime reached. Exiting.")
        break

    # 2) Fill ratio stop (largest object fills ≥ FILL_STOP_RATIO of frame)
    if (obj_area / float(W * H)) >= FILL_STOP_RATIO:
        print("[INFO] Fill stop ratio reached. Exiting.")
        break

    # Optional: show for debugging. Press 'q' to quit early.
    # cv2.imshow("frame", frame)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

    cv2.imshow("Red Track", red_mask)
    cv2.imshow("Blue Track", blue_mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ---------- Cleanup ----------
cap.release()
cv2.destroyAllWindows()
if arduino:
    try:
        arduino.close()
    except:
        pass
