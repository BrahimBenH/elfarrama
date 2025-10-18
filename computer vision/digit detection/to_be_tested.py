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

    # Check if the digit has a complete closed hole (like 0, 6, 8, 9)
    has_complete_hole = False
    hole_contour = None
    
    for j in range(len(hierarchy[0])):
        if hierarchy[0][j][3] == cnt_idx:  # This contour is inside our main contour
            inner_cnt = contours[j]
            
            # Check if this inner contour is a complete closed circle/hole
            # Calculate circularity: 4π*area/perimeter²
            area = cv2.contourArea(inner_cnt)
            perimeter = cv2.arcLength(inner_cnt, True)
            
            if perimeter > 0 and area > 100:  # Minimum area threshold for a valid hole
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                
                # A perfect circle has circularity = 1, we allow some tolerance
                # Also check that the hole is reasonably sized relative to the outer contour
                outer_area = cv2.contourArea(cnt)
                hole_ratio = area / outer_area if outer_area > 0 else 0
                
                if circularity > 0.3 and hole_ratio > 0.05 and hole_ratio < 0.7:
                    has_complete_hole = True
                    hole_contour = inner_cnt
                    break

    if has_complete_hole and hole_contour is not None:
        # 6 vs 9
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            return None
        cy = int(M['m01']/M['m00'])

        # Use the validated hole contour
        Mh = cv2.moments(hole_contour)
        if Mh['m00'] != 0:
            hy = int(Mh['m01']/Mh['m00'])
            hx = int(Mh['m10']/Mh['m00'])
            # Draw the detected hole center for debugging
            cv2.circle(roi_frame, (x + hx, y + hy), 5, (255, 0, 0), -1)
            
            # Compare hole position relative to the main contour center
            if hy < cy:
                return 9  # Hole is in upper part
            else:
                return 6  # Hole is in lower part
    else:
        # 3 vs 5 using more strict line detection
        edges = cv2.Canny(roi, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20,
                                 minLineLength=w//3, maxLineGap=5)

        vertical_left = False
        horizontal_lines = 0
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx, dy = abs(x2 - x1), abs(y2 - y1)

                # Check for vertical line on the left side (characteristic of 5)
                if dy > dx and x1 < w//3 and x2 < w//3:
                    vertical_left = True
                    # Draw detected line for debugging
                    cv2.line(roi_frame, (x+x1, y+y1), (x+x2, y+y2), (0, 255, 255), 2)  # yellow
                
                # Count horizontal lines (characteristic of 3)
                elif dx > dy and dy < 10:  # Strong horizontal line
                    horizontal_lines += 1

        # More strict classification
        if vertical_left:
            return 5
        elif horizontal_lines >= 2:  # Digit 3 should have at least 2 horizontal segments
            # Additional check: ensure the contour has reasonable aspect ratio for a digit
            aspect_ratio = float(w) / h
            if 0.3 < aspect_ratio < 0.8:  # Typical digit proportions
                return 3
        
        # If neither condition is met strongly, don't classify (return None)
        return None

    return None

def main():
    """
    Main function to run the digit detection process with timing.
    """
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    print("Camera is open. Warming up...")
    
    # Show camera feed for 2 seconds before starting detection
    warmup_start = time.time()
    while time.time() - warmup_start < 2:
        ret, frame = cap.read()
        if ret:
            h, w, _ = frame.shape
            roi_size = 200
            x1 = w//2 - roi_size//2
            y1 = h//2 - roi_size//2
            x2 = w//2 + roi_size//2
            y2 = h//2 + roi_size//2
            
            # Show the ROI box during warmup
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(frame, "Warming up camera...", (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("Real-Time Digit Detection", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                return
    
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