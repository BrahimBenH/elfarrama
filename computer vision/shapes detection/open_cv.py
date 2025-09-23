import cv2
import numpy as np
import time

# Set a broader tolerance for a "straight" angle
STRAIGHT_ANGLE_TOLERANCE = 15

# Initialize video capture from the default camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

# Set the duration for analysis (in seconds)
analysis_duration = 5
start_time = time.time()

# Counters for classification
straight_count = 0
inclined_count = 0
total_frames = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape

    # Define a central ROI (Region of Interest)
    roi_size = 300
    x1 = w // 2 - roi_size // 2
    y1 = h // 2 - roi_size // 2
    x2 = w // 2 + roi_size // 2
    y2 = h // 2 + roi_size // 2

    roi_frame = frame[y1:y2, x1:x2]

    # Convert ROI to grayscale and apply Gaussian blur
    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Use Canny to find edges
    edges = cv2.Canny(blurred, 50, 150)

    # Use HoughLinesP to detect lines
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50,
                            minLineLength=roi_size // 4, maxLineGap=10)

    classification = "Detecting..."
    color = (255, 255, 255)  # White

    if lines is not None:
        lines_sorted = sorted(lines, key=lambda x: np.sqrt((x[0][2] - x[0][0])**2 + (x[0][3] - x[0][1])**2), reverse=True)
        detected_lines = lines_sorted[:2]
        line_count = len(detected_lines)

        if line_count == 2:
            line1 = detected_lines[0][0]
            line2 = detected_lines[1][0]

            v1 = (line1[2] - line1[0], line1[3] - line1[1])
            v2 = (line2[2] - line2[0], line2[3] - line2[1])

            dot_product = v1[0] * v2[0] + v1[1] * v2[1]
            mag1 = np.sqrt(v1[0]**2 + v1[1]**2)
            mag2 = np.sqrt(v2[0]**2 + v2[1]**2)
            
            if mag1 != 0 and mag2 != 0:
                angle_rad = np.arccos(np.clip(dot_product / (mag1 * mag2), -1.0, 1.0))
                angle_deg = np.degrees(angle_rad)
                
                # Normalize angle to be between 0 and 180
                angle_deg = min(angle_deg, 180 - angle_deg)

                # Check for straightness with broader tolerance
                if abs(angle_deg - 90) <= STRAIGHT_ANGLE_TOLERANCE:
                    straight_count += 1
                    classification = "Straight"
                    color = (0, 255, 0)  # Green
                else:
                    inclined_count += 1
                    classification = "Inclined"
                    color = (255, 0, 0)  # Blue
            
            total_frames += 1
            
            # Draw the detected lines in the ROI
            for line in detected_lines:
                x_l1, y_l1, x_l2, y_l2 = line[0]
                cv2.line(roi_frame, (x_l1, y_l1), (x_l2, y_l2), color, 2)

    # Display the current classification text and timer
    elapsed_time = time.time() - start_time
    timer_text = f"Time left: {analysis_duration - elapsed_time:.1f}s"
    cv2.putText(frame, timer_text, (x1, y1 - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, classification, (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    
    # Draw the yellow ROI reference box
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)  # Yellow
    
    cv2.imshow("Line Angle Detection", frame)

    # Break the loop if the analysis duration is over
    if elapsed_time > analysis_duration or (cv2.waitKey(1) & 0xFF == ord('q')):
        break

# --- Final Decision and Display ---
if total_frames > 0:
    straight_percentage = (straight_count / total_frames) * 100
    inclined_percentage = (inclined_count / total_frames) * 100
else:
    straight_percentage = 0
    inclined_percentage = 0
    
if straight_percentage > inclined_percentage:
    final_result = "Overall: Mostly Straight"
    final_color = (0, 255, 0)
else:
    final_result = "Overall: Mostly Inclined"
    final_color = (255, 0, 0)

result_frame = np.zeros((300, 600, 3), dtype=np.uint8)
cv2.putText(result_frame, "Analysis Complete!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
cv2.putText(result_frame, f"Straight: {straight_percentage:.2f}%", (50, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
cv2.putText(result_frame, f"Inclined: {inclined_percentage:.2f}%", (50, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
cv2.putText(result_frame, final_result, (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, final_color, 2)

cv2.imshow("Final Result", result_frame)
cv2.waitKey(0)

# Release resources
cap.release()
cv2.destroyAllWindows()