import cv2
import numpy as np
import time
from collections import Counter

def classify_digit(cnt, contours, hierarchy, thresh, cnt_idx, roi_frame):
    """
    Classifies a digit based on its contour and hierarchy.
    Distinguishes between 6 and 9 (has_hole) and 3 and 5 (line detection).
    """
    x, y, w, h = cv2.boundingRect(cnt)
    roi = thresh[y:y+h, x:x+w]

    # Check if the digit has a hole (like 0, 6, 8, 9)
    has_hole = any(hierarchy[0][j][3] == cnt_idx for j in range(len(hierarchy[0])))

    if has_hole:
        # 6 vs 9
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            return None
        cy = int(M['m01']/M['m00'])

        for j in range(len(hierarchy[0])):
            if hierarchy[0][j][3] == cnt_idx:
                Mh = cv2.moments(contours[j])
                if Mh['m00'] != 0:
                    hy = int(Mh['m01']/Mh['m00'])
                    # Draw the detected hole center for debugging
                    cv2.circle(roi_frame, (x + int(Mh['m10']/Mh['m00']), y + hy), 5, (255, 0, 0), -1)

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

                if dy > dx and x1 < w//3 and x2 < w//3:  # vertical line on the left side
                    vertical_left = True
                    # Draw detected line for debugging
                    cv2.line(roi_frame, (x+x1, y+y1), (x+x2, y+y2), (0, 255, 255), 2)  # yellow

        if vertical_left:
            return 5
        else:
            return 3

    return None

def main():
    """
    Main function to run the digit detection process with timing.
    """
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    # Delay the start of detection by 2 seconds
    print("Camera is open. Starting detection in 2 seconds...")
    time.sleep(2)
    start_time = time.time()
    detected_digits = []
    
    print("Detection started. Looking for digits for 5 seconds...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        current_time = time.time()
        if current_time - start_time > 5:
            break

        h, w, _ = frame.shape
        roi_size = 200
        x1 = w//2 - roi_size//2
        y1 = h//2 - roi_size//2
        x2 = w//2 + roi_size//2
        y2 = h//2 + roi_size//2

        roi_frame = frame[y1:y2, x1:x2].copy()  # Use .copy() to prevent modifying the original frame accidentally
        
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
                            detected_digits.append(digit)
                            # Draw detection relative to ROI
                            cv2.rectangle(roi_frame, (x, y), (x+w_c, y+h_c), (0, 255, 0), 2)
                            cv2.putText(roi_frame, str(digit), (x, y-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Display the ROI frame and the full frame with the ROI box
        frame[y1:y2, x1:x2] = roi_frame
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.imshow("Real-Time Digit Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up and print the most predicted number
    cap.release()
    cv2.destroyAllWindows()
    
    if detected_digits:
        most_common_digit = Counter(detected_digits).most_common(1)[0][0]
        print(f"The most predicted number was: {most_common_digit}")
    else:
        print("No digits were detected in the 5-second window.")

if __name__ == "__main__":
    main()