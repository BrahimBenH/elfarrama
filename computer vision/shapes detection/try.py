import cv2  # OpenCV Library
import time

def detect_shapes_in_frame(frame):
    """
    Detect shapes in a given frame and return the annotated frame
    """
    # Create a copy to avoid modifying the original frame
    image = frame.copy()
    
    # Define ROI (Region of Interest) like in digit detection
    h, w, _ = frame.shape
    roi_size = 300  # Slightly larger for shape detection
    x1 = w//2 - roi_size//2
    y1 = h//2 - roi_size//2
    x2 = w//2 + roi_size//2
    y2 = h//2 + roi_size//2
    
    # Extract ROI for processing
    roi_frame = image[y1:y2, x1:x2].copy()
    
    gray_image = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)  # Converting to gray image

    # Setting threshold value to get new image (In simpler terms: this function checks every pixel, and depending on how
    # dark the pixel is, the threshold value will convert the pixel to either black or white (0 or 1)).
    _, thresh_image = cv2.threshold(gray_image, 220, 255, cv2.THRESH_BINARY)

    # Retrieving outer-edge coordinates in the new threshold image
    contours, hierarchy = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Iterating through each contour to retrieve coordinates of each shape
    for i, contour in enumerate(contours):
        if i == 0:
            continue

        # The 2 lines below this comment will approximate the shape we want. The reason being that in certain cases the
        # shape we want might have flaws or might be imperfect, and so, for example, if we have a rectangle with a
        # small piece missing, the program will still count it as a rectangle. The epsilon value will specify the
        # precision in which we approximate our shape.
        epsilon = 0.01*cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Only process contours with reasonable size to avoid noise
        if cv2.contourArea(contour) < 100:
            continue

        # Drawing the outer-edges onto the ROI frame
        cv2.drawContours(roi_frame, [contour], -1, (0, 255, 0), 3)

        # Retrieving coordinates of the contour so that we can put text over the shape.
        x, y, w, h = cv2.boundingRect(approx)
        x_mid = int(x + (w/3))  # This is an estimation of where the middle of the shape is in terms of the x-axis.
        y_mid = int(y + (h/1.5))  # This is an estimation of where the middle of the shape is in terms of the y-axis.

        # Setting some variables which will be used to display text on the final image
        coords = (x_mid, y_mid)
        colour = (0, 255, 0)  # Green color for better visibility
        font = cv2.FONT_HERSHEY_DUPLEX

        # This is the part where we actually guess which shape we have detected. The program will look at the amount of edges
        # the contour/shape has, and then based on that result the program will guess the shape (for example, if it has 3 edges
        # then the chances that the shape is a triangle are very good.)
        #
        # You can add more shapes if you want by checking more lengths, but for the simplicity of this tutorial program I
        # have decided to only detect 5 shapes.
        if len(approx) == 3:
            cv2.putText(roi_frame, "Triangle", coords, font, 0.7, colour, 2)  # Text on the ROI frame
        elif len(approx) == 4:
            cv2.putText(roi_frame, "Quadrilateral", coords, font, 0.7, colour, 2)
        elif len(approx) == 5:
            cv2.putText(roi_frame, "Pentagon", coords, font, 0.7, colour, 2)
        elif len(approx) == 6:
            cv2.putText(roi_frame, "Hexagon", coords, font, 0.7, colour, 2)
        else:
            # If the length is not any of the above, we will guess the shape/contour to be a circle.
            cv2.putText(roi_frame, "Circle", coords, font, 0.7, colour, 2)
    
    # Put the processed ROI back into the main frame
    image[y1:y2, x1:x2] = roi_frame
    
    # Draw the yellow ROI box on the main frame
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 2)
    
    return image

def main():
    """
    Main function to run the real-time shape detection process.
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
            roi_size = 300
            x1 = w//2 - roi_size//2
            y1 = h//2 - roi_size//2
            x2 = w//2 + roi_size//2
            y2 = h//2 + roi_size//2
            
            # Show the ROI box during warmup
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(frame, "Warming up camera...", (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("Real-Time Shape Detection", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                return
    
    print("Detection started. Press 'q' to quit...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detect shapes in the current frame
        annotated_frame = detect_shapes_in_frame(frame)
        
        # Display the frame with detected shapes
        cv2.imshow("Real-Time Shape Detection", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()