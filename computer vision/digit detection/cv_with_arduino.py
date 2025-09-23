import cv2
import numpy as np
import time
import serial
from collections import Counter

# ----- Your classify_digit function (unchanged) -----
def classify_digit(cnt, contours, hierarchy, thresh, cnt_idx, roi_frame):
    # (keep your original classify_digit code here)
    # ...
    return None

def detect_digit():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        return "ERR_NO_CAMERA"

    time.sleep(2)  # warm up
    start_time = time.time()
    detected_digits = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        current_time = time.time()
        if current_time - start_time > 5:
            break

        h, w, _ = frame.shape
        roi_size = 200
        x1, y1 = w//2 - roi_size//2, h//2 - roi_size//2
        x2, y2 = w//2 + roi_size//2, h//2 + roi_size//2

        roi_frame = frame[y1:y2, x1:x2].copy()
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        if hierarchy is not None:
            for cnt_idx, cnt in enumerate(contours):
                if hierarchy[0][cnt_idx][3] == -1:  
                    x, y, w_c, h_c = cv2.boundingRect(cnt)
                    if w_c > 20 and h_c > 20:
                        digit = classify_digit(cnt, contours, hierarchy, thresh, cnt_idx, roi_frame)
                        if digit is not None:
                            detected_digits.append(digit)

        cv2.imshow("Digit Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if detected_digits:
        return str(Counter(detected_digits).most_common(1)[0][0])
    else:
        return "NONE"

# ----- Serial communication loop -----
def main():
    ser = serial.Serial("COM3", 9600, timeout=1)  # change COM3 to your port
    print("Listening for Arduino...")

    while True:
        if ser.in_waiting > 0:
            signal = ser.readline().decode().strip()
            print(f"Received: {signal}")

            if signal == "START":
                result = detect_digit()
                print(f"Sending back: {result}")
                ser.write((result + "\n").encode())

if __name__ == "__main__":
    main()
