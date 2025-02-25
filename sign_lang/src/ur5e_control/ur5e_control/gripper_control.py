from .robotiq_gripper import RobotiqGripper
import time
from std_msgs.msg import Int32
import rclpy
from rclpy.node import Node

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        self.activate_gripper()
        self.subscription = self.create_subscription(
            Int32,
            '/gripper',
            self.gripper_callback,
            10)
        
    
    def activate_gripper(self):
        ip = "163.220.51.112"
        print("Creating gripper...")
        self.gripper = RobotiqGripper()
        print("Connecting to gripper...")
        self.gripper.connect(ip, 63352)
        print("Activating gripper...")
        self.gripper.activate()
        
    def gripper_callback(self, msg):
        self.gripper.move(msg.data, 255, 10)
        

    

    def log_info(self, gripper):
        print(f"Pos: {str(gripper.get_current_position()): >3}  "
                f"Open: {gripper.is_open(): <2}  "
                f"Closed: {gripper.is_closed(): <2}  ")
        
def main(args=None):
    rclpy.init(args=args)
    gripper_node = GripperNode()
    rclpy.spin(gripper_node)
    gripper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()