import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointGroupCommand
import time

class jointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        self.jointGroupCommandPublisherNode = self.create_publisher(JointGroupCommand, '/wx250/commands/joint_group', 10)
        
        joint0 = 0.0
        self.jointGroupPublisher()
        self.get_logger().info("Should have published")
        while True:
            time.sleep(3)
            self.jointGroupPublisher(joint0=joint0)
            joint0 = joint0 + .5
            time.sleep(3)
            self.jointGroupPublisher(joint0=joint0)
            joint0 = joint0 - 1
            time.sleep(3)
            self.jointGroupPublisher(joint0=joint0)
            joint0 = 0.0


            

        """for i in range(5):
            self.get_logger().info("Publishing now allegedly")
            self.jointGroupPublisher()
            self.get_logger().info("Allegedly just published data")"""
    

    def jointGroupPublisher(self, joint0 =0.0, joint1=0.3, joint2=.15, joint3=1.0, joint4=0.0):
        msg = JointGroupCommand()
        msg.name='arm'

        #Placeholders for now
        msg.cmd = [joint0, joint1, joint2, joint3, joint4]
        self.get_logger().info(str(msg))
        self.jointGroupCommandPublisherNode.publish(msg)


    


def main():
    print('Hi from oliver_arm_control.')
    rclpy.init(args=None)
    node = jointPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
