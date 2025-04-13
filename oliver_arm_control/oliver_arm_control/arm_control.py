import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointGroupCommand
from sensor_msgs.msg import JointState
import time
import cv2
import numpy as np
import apriltag

class jointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        self.jointGroupCommandPublisherNode = self.create_publisher(JointGroupCommand, '/wx250/commands/joint_group', 10)
        self.subscription = self.create_subscription(msg_type=JointState, topic='/wx250/joint_states', callback=self.jointStateCallback, qos_profile=1) 
        
        self.setup_video_capture()

        self.joint0 = 0.7
        self.jointsMoving = False
        self.jointGroupPublisher()
        self.get_logger().info("Should have published")
        self.timer = self.create_timer(0.2, self.main_loop_callback)


    def main_loop_callback(self):
        self.get_logger().info("Callback for oliver arm")
        self.cameraPass()
        if (self.jointsMoving):
            self.get_logger().info("Joints are moving, waiting")
            return
        #self.get_logger().info("Joints are not moving, continuing")
        #self.get_logger().info("Joint0 Before: " + str(self.joint0))
        self.joint0 = -self.joint0
        #self.get_logger().info("Joint0 After: " + str(self.joint0))
        self.jointGroupPublisher(joint0=self.joint0)


    def jointStateCallback(self, msg):
        # self.get_logger().type(str(msg))
        for v in msg.velocity:
            if abs(v) > 0.01:
                #self.get_logger().info("Call-back-back: Joints are moving")
                self.jointsMoving = True
                return
        #self.get_logger().info("Call-back-back: Joints are not moving")
        self.jointsMoving = False



    def jointGroupPublisher(self, joint0 =0.0, joint1=0.3, joint2=.15, joint3=1.0, joint4=0.0):
        msg = JointGroupCommand()
        msg.name='arm'

        #Placeholders for now
        msg.cmd = [joint0, joint1, joint2, joint3, joint4]
        self.get_logger().info(str(msg))
        self.jointGroupCommandPublisherNode.publish(msg)


    def cameraPass(self):
        self.get_logger().info("Starting camera pass")
        self.get_logger().info("Calibration mode: " + str(self.calibration_mode))
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Error: Can't receive frame. Exiting...")
            return
        
        if self.calibration_mode:
            # Detect April Tags
            tag_corners, tag_centers, april_tag_frame = self.detect_april_tags(frame)
                        
            # If we have 4 tags and user presses 'c', calculate homography
            if len(tag_corners) >= 4:
                self.homography_matrix = self.calculate_homography(tag_corners, self.table_dimensions)
                if self.homography_matrix is not None:
                    self.get_logger().info("Calibration successful!")
                    self.calibration_mode = False
                else:
                    self.get_logger().info("Calibration failed. Please try again.")
        else:
            self.get_logger().info("Running puck detection mode.")
            # Puck detection mode
            puck_position, table_position, processed_frame, mask = self.detect_puck_position(
                frame, self.lower_red1, self.upper_red1, self.lower_red2, self.upper_red2, self.homography_matrix
            )
            
            # Display position information on the frame
            if puck_position:
                #camera_pos_text = f"Camera Position: {puck_position}"
                
                if table_position:

                    # Show table coordinates
                    #table_pos_text = f"Table Position: ({table_position[0]:.1f}, {table_position[1]:.1f})"
                    
                    # Calculate and show position relative to table center
                    table_center_x = self.table_dimensions[0] / 2
                    table_center_y = self.table_dimensions[1] / 2
                    rel_x = table_position[0] - table_center_x
                    rel_y = table_position[1] - table_center_y
                    
                    # rel_pos_text = f"Rel to Center: ({rel_x:.1f}, {rel_y:.1f})"
                    # cv2.putText(processed_frame, rel_pos_text, (10, 90), 
                    #            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Print table coordinates to console for logging/tracking
                    # print(f"Time: {time.time() - time_start:.2f}, Table Position: ({table_position[0]:.1f}, {table_position[1]:.1f}) inches")

                    
                    # Calculate position relative to table center
                    table_center_x = self.table_dimensions[0] / 2
                    table_center_y = self.table_dimensions[1] / 2
                    rel_x = table_position[0] - table_center_x
                    rel_y = table_position[1] - table_center_y

                    if self.firstCameraPass:
                        curTime = time.time() - self.time_start
                        curX = rel_x
                        curY = rel_y
                        self.prevTime = curTime
                        self.prevX = curX
                        self.prevY = curY
                        self.firstCameraPass = False

                    curTime = time.time() - self.time_start
                    curX = rel_x
                    curY = rel_y
                
                    velocity_x = curX - self.prevX
                    velocity_y = curY - self.prevY
                    delta_t = curTime - self.prevTime if curTime - self.prevTime > 0 else 1e-6

                    speed = np.sqrt(velocity_x**2 + velocity_y**2) / delta_t

                    if speed > 8.0 and velocity_y < 0: 
                        m, b = self.trajectory_prediction(curX, curY, curTime, self.prevX, self.prevY, self.prevTime)
                        
                        yGoal = -17
                        intersectAtX = (yGoal - b) / m
                        self.get_logger().info("Intersect at X: " + str(intersectAtX))
                    else:
                        self.get_logger().info("Puck is moving away from the goal. No prediction.")

                    self.prevTime = curTime
                    self.prevX = curX
                    self.prevY = curY
            else:
                self.get_logger().info("Puck not detected.")


    def detect_puck_position(self, frame, lower_red1, upper_red1, lower_red2, upper_red2, homography_matrix=None):
        if frame is None:
            print("Error: Invalid frame.")
            return None, None, None
        
        # Create a copy of the frame for display purposes
        display_frame = frame.copy()
        
        # Get frame dimensions
        height, width = frame.shape[:2]
        
        # Create a mask for the puck color in the original frame
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert to HSV for better color detection
        # mask = cv2.inRange(hsv_frame, lower_color, upper_color)

        mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Apply morphological operations to remove noise and fill gaps
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours in the mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)    
        solid_contours = [contours[i] for i in range(len(contours)) if hierarchy[0][i][2] == -1]

        puck_position = None
        table_position = None
        
        if solid_contours:
            # assume largest contour is the puck
            largest_contour = max(contours, key=cv2.contourArea)
            # print(cv2.contourArea(largest_contour))
            min_area = 500  # Adjust this threshold based on your puck size
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
                    # cv2.imshow("Table View", warped_display)
        
        return puck_position, table_position, display_frame, mask


    def setup_video_capture(self):
        # Puck color range in HSV (adjust these for your puck color)
        # This example is for a black puck
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])

        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([179, 255, 255])

        # Table dimensions in inches (7.5 ft x 4 ft)
        self.table_dimensions = (90, 48)  # 7.5 ft = 90 inches, 4 ft = 48 inches
        
        fps_start_time = time.time()
        self.time_start = fps_start_time
        
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().info("Error: Could not open webcam. Try changing the device index.")
            return
        
        self.get_logger().info("Starting camera calibration and puck detection.")
        
        # Initialize homography matrix
        self.homography_matrix = None
        self.calibration_mode = True  # Start in calibration mode

        self.firstCameraPass = True


    def detect_april_tags(self, frame):
        """
        Detect April Tags in the frame and return their corners
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Create the AprilTag detector
        options = apriltag.DetectorOptions(families="tag16h5")
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
            # cv2.polylines(display_frame, [corners], True, (0, 255, 0), 2)
            
            # Draw the tag ID
            # cv2.putText(display_frame, str(tag_id), tuple(center), 
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        return tag_corners, tag_centers, display_frame


    def trajectory_prediction(self, x1, y1, time1, x2, y2, time2):
        """

        @param center1: (x, y) coordinates at time1
        @param time1: time stamp of first set of coordinates

        @param center2: (x,y) coordinates at time2
        @param time2: time stamp of second set of coordinates

        return m, b: parameters of the projected line 
        """

        # Placeholder, will likely need to change based on format of time1 and time2
        delta_t = time2 - time1

        # Calculate time derivatives of x and y, assuming frictionless surface
        v_x = (x2 - x1) / delta_t
        v_y = (y2 - y1) / delta_t

        # Solve for slope and y-intercept
        m = v_y / v_x
        b = m * -1 * x2 + y2

        return m, b
        

    def calculate_homography(self, tag_corners, table_dimensions):
        """
        Calculate the homography matrix from the detected April Tags
        
        Args:
            tag_corners: Dictionary of tag_id -> corner points
            table_dimensions: (width, height) of the table in the desired unit (e.g., inches)
        
        Returns:
            homography_matrix: The 3x3 homography matrix
        """
        if len(tag_corners) < 4:
            self.get_logger().info(f"Need 4 tags, only found {len(tag_corners)}")
            return None
        
        # Assuming tag IDs 0, 1, 2, 3 are placed at:
        # ID 0: top-left, ID 1: top-right, ID 2: bottom-right, ID 3: bottom-left
        # If your tags have different IDs, adjust accordingly
        
        # Check if we have the required tags
        required_tags = [0, 1, 2, 3]  # Adjust these IDs to match your setup
        if not all(tag_id in tag_corners for tag_id in required_tags):
            self.get_logger().info("Missing one or more of the required tags:", required_tags)
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
    print('Hi from oliver_arm_control.')
    rclpy.init(args=None)
    node = jointPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
