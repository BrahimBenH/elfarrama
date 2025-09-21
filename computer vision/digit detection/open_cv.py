import cv2
import numpy as np

def classify_digit(cnt, contours, hierarchy, thresh, cnt_idx, roi_frame):
    x, y, w, h = cv2.boundingRect(cnt)
    roi = thresh[y:y+h, x:x+w]

    # check if digit has a hole
    has_hole = any(hierarchy[0][j][3] == cnt_idx for j in range(len(hierarchy[0])))

    if has_hole:
        # 6 vs 9
        M = cv2.moments(cnt)
        cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        for j in range(len(hierarchy[0])):
            if hierarchy[0][j][3] == cnt_idx:
                Mh = cv2.moments(contours[j])
                hx, hy = int(Mh['m10']/Mh['m00']), int(Mh['m01']/Mh['m00'])
                # Draw the detected hole center
                cv2.circle(roi_frame, (hx, hy), 5, (255, 0, 0), -1)

                if hy < cy:
                    return 9
                else:
                    return 6
    else:
        # 3 vs 5 using line detection
        edges = cv2.Canny(roi, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20,
                                minLineLength=w//3, maxLineGap=5)

        vertical_left = False

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx, dy = abs(x2 - x1), abs(y2 - y1)

                if dx > dy and dy < 5:  # horizontal line
                    cv2.line(roi_frame, (x+x1, y+y1), (x+x2, y+y2), (0, 0, 255), 2)  # red
                if dy > dx and x1 < w//3 and x2 < w//3:  # vertical on left
                    vertical_left = True
                    cv2.line(roi_frame, (x+x1, y+y1), (x+x2, y+y2), (0, 255, 255), 2)  # yellow

        if vertical_left:
            return 5
        else:
            return 3

    return None


# --- Real-time camera loop ---
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape

    # Define central ROI (200x200 box in center)
    roi_size = 200
    x1 = w//2 - roi_size//2
    y1 = h//2 - roi_size//2
    x2 = w//2 + roi_size//2
    y2 = h//2 + roi_size//2

    roi_frame = frame[y1:y2, x1:x2]

    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if hierarchy is not None:
        for cnt_idx, cnt in enumerate(contours):
            if hierarchy[0][cnt_idx][3] == -1:  # external contour
                x, y, w_c, h_c = cv2.boundingRect(cnt)
                if w_c > 20 and h_c > 20:  # ignore tiny noise
                    digit = classify_digit(cnt, contours, hierarchy, thresh, cnt_idx, roi_frame)
                    if digit is not None:
                        # Draw detection relative to ROI
                        cv2.rectangle(roi_frame, (x, y), (x+w_c, y+h_c), (0, 255, 0), 2)
                        cv2.putText(roi_frame, str(digit), (x, y-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Draw yellow ROI reference box
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

    cv2.imshow("Real-Time Digit Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
