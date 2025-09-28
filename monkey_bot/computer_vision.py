import cv2
import numpy as np

# Start video capture
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red color range
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask = mask1 + mask2

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    m1_val, m2_val = 0, 0  # default stop

    if contours:
        c = max(contours, key=cv2.contourArea)
        (x, y, w, h) = cv2.boundingRect(c)
        cx = x + w // 2
        cy = y + h // 2

        frame_cx = frame.shape[1] // 2

        # --- Steering based on X position ---
        error_x = cx - frame_cx  # negative = left, positive = right
        k_turn = 0.5  # steering gain (tune this)
        turn = int(k_turn * error_x)

        # --- Forward/backward based on object size ---
        target_size = 15000  # target contour area (tune this)
        area = cv2.contourArea(c)
        error_area = target_size - area
        k_forward = 0.01  # forward speed gain
        forward = int(k_forward * error_area)

        # Motor mixing (differential drive)
        m1_val = forward - turn
        m2_val = forward + turn

        # Clamp to Arduino range
        m1_val = max(-255, min(255, m1_val))
        m2_val = max(-255, min(255, m2_val))

        # Draw box + center
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

    # --- Send command to Arduino ---
    cmd = f"{m1_val},{m2_val}\n"
    arduino.write(cmd.encode())
    print("Sent:", cmd.strip())

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
arduino.close()
cv2.destroyAllWindows()