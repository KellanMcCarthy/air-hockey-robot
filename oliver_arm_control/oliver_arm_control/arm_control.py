import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointGroupCommand
from sensor_msgs.msg import JointState
import time
import cv2
import numpy as np

class jointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        self.jointGroupCommandPublisherNode = self.create_publisher(JointGroupCommand, '/wx250/commands/joint_group', 10)
        self.subscription = self.create_subscription(msg_type=JointState, topic='/wx250/joint_states', callback=self.jointStateCallback, qos_profile=1) 
        
        self.joint0 = 1.0
        self.jointsMoving = False
        self.jointGroupPublisher()
        self.get_logger().info("Should have published")
        self.timer = self.create_timer(0.2, self.main_loop_callback)


    def main_loop_callback(self):
        self.get_logger().info("Callback for oliver arm")
        #self.cameraPass()
        if (self.jointsMoving):
            self.get_logger().info("Joints are moving, waiting")
            return
        self.get_logger().info("Joints are not moving, continuing")
        self.get_logger().info("Joint0 Before: " + str(self.joint0))
        self.joint0 = -self.joint0
        self.get_logger().info("Joint0 After: " + str(self.joint0))
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
        #black puck range
        self.get_logger().info("STARTING PUCK DETECTION PASS")
        lower_color = np.array([0, 0, 0])
        upper_color = np.array([100, 100, 100])
        
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            self.get_logger().info("Error: Could not open webcam. Try changing the device index.")
            return
        
        self.get_logger().info("Starting puck detection. Press 'q' to quit.")

        
        fps_start_time = time.time()
        time_start = fps_start_time
        fps_frame_count = 0
        fps = 0
        # Beginning of old loop
        ret, frame = cap.read()
        if not ret:
            self.get_logger().info("Error: Can't receive frame. Exiting...")
        
        puck_position, processed_frame = self.detect_puck_position(frame, lower_color, upper_color)
        
        # Calculate and display FPS
        fps_frame_count += 1
        if (time.time() - fps_start_time) > 1:
            fps = fps_frame_count / (time.time() - fps_start_time)
            fps_frame_count = 0
            fps_start_time = time.time()
        
        # Display position information on the frame
        if puck_position:
            position_text = f"Position: {puck_position}, Time: {time.time() - time_start:.2f}"
            self.get_logger().info(position_text)
            cv2.putText(processed_frame, position_text, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(processed_frame, "No puck detected", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.putText(processed_frame, f"FPS: {fps:.1f}", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # Display the mask to help with debugging color thresholds
        mask = cv2.inRange(frame, lower_color, upper_color)
        cv2.imshow("newestMask", mask)

        # Display the resulting frame
        cv2.imshow("newestPuckDetection", processed_frame)
        cv2.waitKey(1)
        self.get_logger().info("ENDING PUCK DETECTION PASS")
        
        
        """# When everything is done, release the capture
        if cap.isOpened():
            cap.release()
        cv2.destroyAllWindows()
        cv2.waitKey(1)"""


    def detect_puck_position(self, frame, lower_color, upper_color):
        if frame is None:
            self.get_logger().info("Error: Invalid frame.")
            return None, None
        
        mask = cv2.inRange(frame, lower_color, upper_color)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        if contours:
            # assume largest contour is the puck
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) > 100:  # Filter out noise
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                center = (int(x), int(y))
                
                # Draw a point at the center of the puck
                cv2.circle(frame, center, 25, (0, 255, 0), -1)
                return center, frame
        
        return None, frame
        


    


def main():
    print('Hi from oliver_arm_control.')
    rclpy.init(args=None)
    node = jointPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
