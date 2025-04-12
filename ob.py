import cv2
import apriltag
import numpy as np
import time

def detect_puck_position(frame, lower_color, upper_color, homography_matrix=None):
    if frame is None:
        print("Error: Invalid frame.")
        return None, None, None
    
    # Create a copy of the frame for display purposes
    display_frame = frame.copy()
    
    # Get frame dimensions
    height, width = frame.shape[:2]
    
    # Create a mask for the puck color in the original frame
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert to HSV for better color detection
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)
    
    # Apply morphological operations to remove noise and fill gaps
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    puck_position = None
    table_position = None
    
    if contours:
        # assume largest contour is the puck
        largest_contour = max(contours, key=cv2.contourArea)
        
        min_area = 50  # Adjust this threshold based on your puck size
        if cv2.contourArea(largest_contour) > min_area:  # Filter out noise
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            puck_position = (int(x), int(y))
            
            # Draw a circle around the puck on display frame
            cv2.circle(display_frame, puck_position, int(radius), (0, 255, 0), 2)
            cv2.circle(display_frame, puck_position, 3, (0, 0, 255), -1)  # Center point
            
            # If we have the homography matrix, calculate and draw the puck in table coordinates
            if homography_matrix is not None:
                # Convert puck position to homogeneous coordinates
                point = np.array([[float(x), float(y)]], dtype=np.float32)
                
                # Apply homography to transform from camera to table coordinates
                transformed_point = cv2.perspectiveTransform(point.reshape(-1, 1, 2), homography_matrix)
                table_x, table_y = transformed_point[0][0]
                table_position = (table_x, table_y)
                
                # Create a warped view for visualization
                warped_display = cv2.warpPerspective(frame, homography_matrix, (width, height))
                
                # Draw the puck on the warped view
                warped_puck_position = (int(table_x), int(table_y))
                cv2.circle(warped_display, warped_puck_position, 10, (0, 0, 255), -1)
                
                # Show the warped view
                cv2.imshow("Table View", warped_display)
    
    return puck_position, table_position, display_frame, mask

def detect_april_tags(frame):
    """
    Detect April Tags in the frame and return their corners
    """
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Create the AprilTag detector
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    
    # Detect AprilTags
    results = detector.detect(gray)
    
    tag_corners = {}
    tag_centers = {}
    display_frame = frame.copy()
    
    for r in results:
        # Get the tag ID
        tag_id = r.tag_id
        
        # Get the tag corners
        corners = r.corners.astype(int)
        
        # Store the corners of the tag
        tag_corners[tag_id] = corners
        
        # Calculate and store center
        center = np.mean(corners, axis=0).astype(int)
        tag_centers[tag_id] = center
        
        # Draw the tag outline
        cv2.polylines(display_frame, [corners], True, (0, 255, 0), 2)
        
        # Draw the tag ID
        cv2.putText(display_frame, str(tag_id), tuple(center), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    return tag_corners, tag_centers, display_frame

def calculate_homography(tag_corners, table_dimensions):
    """
    Calculate the homography matrix from the detected April Tags
    
    Args:
        tag_corners: Dictionary of tag_id -> corner points
        table_dimensions: (width, height) of the table in the desired unit (e.g., inches)
    
    Returns:
        homography_matrix: The 3x3 homography matrix
    """
    if len(tag_corners) < 4:
        print(f"Need 4 tags, only found {len(tag_corners)}")
        return None
    
    # Assuming tag IDs 0, 1, 2, 3 are placed at:
    # ID 0: top-left, ID 1: top-right, ID 2: bottom-right, ID 3: bottom-left
    # If your tags have different IDs, adjust accordingly
    
    # Check if we have the required tags
    required_tags = [0, 1, 2, 3]  # Adjust these IDs to match your setup
    if not all(tag_id in tag_corners for tag_id in required_tags):
        print("Missing one or more of the required tags:", required_tags)
        return None
    
    # Source points (tag centers in the image)
    src_points = []
    for tag_id in required_tags:
        # Use the center of each tag
        center = np.mean(tag_corners[tag_id], axis=0)
        src_points.append(center)
    
    src_points = np.array(src_points, dtype=np.float32)
    
    # Destination points (where the tags should be in the top-down view)
    table_width, table_height = table_dimensions
    
    # Define margins from the edge (in same units as table_dimensions)
    margin = 6  # For example, 6 inches from the edge
    
    dst_points = np.array([
        [margin, margin],                          # Top-left
        [table_width - margin, margin],            # Top-right
        [table_width - margin, table_height - margin], # Bottom-right
        [margin, table_height - margin]            # Bottom-left
    ], dtype=np.float32)
    
    # Calculate the homography matrix
    homography_matrix, _ = cv2.findHomography(src_points, dst_points)
    
    return homography_matrix

def main():
    # Puck color range in HSV (adjust these for your puck color)
    # This example is for a black puck
    lower_color = np.array([0, 0, 0])
    upper_color = np.array([180, 50, 50])  # Adjusted for better black detection in HSV
    
    # Add a reference to table_dimensions in the main function scope for relative positioning
    global table_dimensions

    # Table dimensions in inches (7.5 ft x 4 ft)
    table_dimensions = (90, 48)  # 7.5 ft = 90 inches, 4 ft = 48 inches
    
    
    
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open webcam. Try changing the device index.")
        return
    
    print("Starting camera calibration and puck detection. Press 'q' to quit, 'c' to calibrate.")

    fps_start_time = time.time()
    time_start = fps_start_time
    fps_frame_count = 0
    fps = 0
    
    # Initialize homography matrix
    homography_matrix = None
    calibration_mode = True  # Start in calibration mode
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Can't receive frame. Exiting...")
            break
        
        # Calculate and display FPS
        fps_frame_count += 1
        if (time.time() - fps_start_time) > 1:
            fps = fps_frame_count / (time.time() - fps_start_time)
            fps_frame_count = 0
            fps_start_time = time.time()
        
        if calibration_mode:
            # Detect April Tags
            tag_corners, tag_centers, april_tag_frame = detect_april_tags(frame)
            
            # Display tag detection frame
            status_text = "Calibration Mode: Detecting April Tags"
            cv2.putText(april_tag_frame, status_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.putText(april_tag_frame, f"Found {len(tag_corners)} tags. Need 4 for calibration.", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.putText(april_tag_frame, "Press 'c' to calibrate when all 4 tags are visible", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Show the detected tag IDs and their positions
            y_pos = 120
            for tag_id, center in tag_centers.items():
                tag_text = f"Tag {tag_id}: {center}"
                cv2.putText(april_tag_frame, tag_text, (10, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                y_pos += 30
            
            cv2.imshow("April Tag Detection", april_tag_frame)
            
            # If we have 4 tags and user presses 'c', calculate homography
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c') and len(tag_corners) >= 4:
                homography_matrix = calculate_homography(tag_corners, table_dimensions)
                if homography_matrix is not None:
                    print("Calibration successful!")
                    print("Homography matrix:")
                    print(homography_matrix)
                    calibration_mode = False
                else:
                    print("Calibration failed. Please try again.")
            elif key == ord('q'):
                break
        else:
            # Puck detection mode
            puck_position, table_position, processed_frame, mask = detect_puck_position(
                frame, lower_color, upper_color, homography_matrix
            )
            
            # Display position information on the frame
            if puck_position:
                camera_pos_text = f"Camera Position: {puck_position}"
                cv2.putText(processed_frame, camera_pos_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if table_position:
                    # Show table coordinates
                    table_pos_text = f"Table Position: ({table_position[0]:.1f}, {table_position[1]:.1f})"
                    cv2.putText(processed_frame, table_pos_text, (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Calculate and show position relative to table center
                    table_center_x = table_dimensions[0] / 2
                    table_center_y = table_dimensions[1] / 2
                    rel_x = table_position[0] - table_center_x
                    rel_y = table_position[1] - table_center_y
                    
                    rel_pos_text = f"Rel to Center: ({rel_x:.1f}, {rel_y:.1f})"
                    cv2.putText(processed_frame, rel_pos_text, (10, 90), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Print table coordinates to console for logging/tracking
                    print(f"Time: {time.time() - time_start:.2f}, Table Position: ({table_position[0]:.1f}, {table_position[1]:.1f}) inches")
                    
                    # Calculate position relative to table center
                    table_center_x = table_dimensions[0] / 2
                    table_center_y = table_dimensions[1] / 2
                    rel_x = table_position[0] - table_center_x
                    rel_y = table_position[1] - table_center_y
                    
                    # Calculate distance from center
                    distance_from_center = np.sqrt(rel_x**2 + rel_y**2)
                    
                    # Print position relative to table center
                    print(f"Position relative to table center: ({rel_x:.1f}, {rel_y:.1f}) inches")
                    print(f"Distance from center: {distance_from_center:.1f} inches")
            else:
                cv2.putText(processed_frame, "No puck detected", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.putText(processed_frame, f"FPS: {fps:.1f}", (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            cv2.putText(processed_frame, "Press 'r' to recalibrate, 'q' to quit, 't' to tune color", (10, 150), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            # Display the mask to help with debugging color thresholds
            cv2.imshow("Mask", mask)
                
            # Display the resulting frame
            cv2.imshow("Puck Detection", processed_frame)
            
            # Break the loop when 'q' is pressed, enter calibration mode when 'r' is pressed
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                calibration_mode = True
            elif key == ord('t'):
                # Create an interactive color tuning window (could be expanded for better control)
                print("Current color thresholds:")
                print(f"Lower: {lower_color}")
                print(f"Upper: {upper_color}")
                print("Adjust in the code if needed. Press any key to continue.")
                cv2.waitKey(0)
    
    # When everything is done, release the capture
    if cap.isOpened():
        cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)

if __name__ == "__main__":
    main()