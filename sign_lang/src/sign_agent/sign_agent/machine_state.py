import rclpy
from rclpy.node import Node
import os
from std_msgs.msg import Int32, Int32MultiArray, Bool
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
import math


class MachineState(Node):
    def __init__(self):
        super().__init__('image_processor_node')
        self.subscriber = self.create_subscription(Int32MultiArray, '/instruction', self.instruction_callback, 5)
            
        self.mode = 'stop'
        
        self.map_instructions_to_ee = {0: 'x', 1: 'y', 2: 'z'}
        self.map_axis = {'x': 0, 'y': 1, 'z': 2}
                
        self.map_ee_velocities = {8: 0.001, 9: -0.001, 10: 0.01, 11: -0.01, 5: 0.03, 6: -0.03}
        
        self.instruction_left = None
        self.last_instruction_left = None
        self.instruction_right = None
        self.last_instruction_right = None

        self.gripper_state = False

        self.ee_position = None
        self.ee_orientation = None
        self.new_ee_position = None
        self.new_ee_orientation = None
        self.switch = False

        #Create timers, publishers and subscribers
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.gripper_pub = self.create_publisher(Int32, '/gripper', 10)
        self.timer_gripper = self.create_timer(0.01, self.timer_gripper_callback)

        self.sub_ee = self.create_subscription(Pose, '/ee_pose', self.ee_callback, 10)
        self.pub_ee = self.create_publisher(Pose, '/new_ee_pose', 10)
        
        self.switch_pub = self.create_publisher(Bool, '/switch', 1)


        self.timer_ms = self.create_timer(0.01, self.state_transition_callback)
    
        
    def instruction_callback(self, msg: Int32MultiArray):
        #Get instructions from subscriber
        self.instruction = msg.data
        self.instruction_left = self.instruction[0]
        self.instruction_right = self.instruction[1]

        #Check if instruction has changed
        if self.instruction_right != self.last_instruction_right:
            self.last_instruction_right = self.instruction_right
            self.instruction_time_right = time.time()

        current_time = time.time()
        self.time_lapsed_right = current_time - self.instruction_time_right

    def state_transition_callback(self):

        #Update end effector pose
        
        

        if self.mode == 'stop':
            if self.instruction_right in range(0, 3) and self.time_lapsed_right >= 2:
                self.axis_selected = self.instruction_right
                self.mode = 'secure'
                self.switch = True
            elif self.instruction_left == 11 and self.instruction_right == 10:
                self.joint_selected = None
                self.axis_selected = None
                self.mode = 'free'
                self.switch = True
            else:
                self.switch = False
                self.mode = 'stop'
                

        elif self.mode == 'secure':
            
            
            self.get_logger().info(f"Moving selected axis: {self.axis_selected}")
            # Moving in End Effector mode using ee_position
            
            if self.instruction_right in range(8, 12) and self.instruction_left == 9:   
                self.new_ee_orientation = self.ee_orientation
                self.new_ee_position = self.ee_position 
                self.new_ee_position[self.axis_selected] += self.map_ee_velocities[self.instruction_right]
                
            #Moving rotation in end effector mode
            elif self.instruction_right in range(5, 7) and self.instruction_left == 9:
                euler_angle = [0, 0, 0]
                self.new_ee_orientation = self.ee_orientation
                self.new_ee_position = self.ee_position
                euler_angle[0], euler_angle[1], euler_angle[2] = self.euler_from_quaternion(self.ee_orientation[0], self.ee_orientation[1], self.ee_orientation[2], self.ee_orientation[3])
                euler_angle[self.axis_selected] += self.map_ee_velocities[self.instruction_right]
                self.new_ee_orientation = self.quaternion_from_euler(euler_angle)

            #Toggle gripper state
            elif self.instruction_right == 3 and self.time_lapsed_right >= 1 and not self.gripper_toggled:
                self.gripper_state = not self.gripper_state
                self.gripper_toggled = True
            
            #Reset gripper toggle
            elif self.instruction_right != 3:
                self.gripper_toggled = False

            #Change mode to stop
            if self.instruction_right == 12 and self.time_lapsed_right >= 0.5:
                self.mode = 'stop'
            
        elif self.mode == 'free':
            
            

            #Change ee position in IsaacSim
            if self.instruction_left in range(0, 3) and self.instruction_right in range(8,12):
                self.new_ee_orientation = self.ee_orientation
                self.new_ee_position = self.ee_position
                self.new_ee_position[self.instruction_left] += self.map_ee_velocities[self.instruction_right]
                
            #Change ee rotation in IsaacSim
            elif self.instruction_left in range(0, 3) and self.instruction_right in range(5, 7):
                euler_angle = [0, 0, 0]
                self.new_ee_orientation = self.ee_orientation
                self.new_ee_position = self.ee_position
                euler_angle[0], euler_angle[1], euler_angle[2] = self.euler_from_quaternion(self.ee_orientation[0], self.ee_orientation[1], self.ee_orientation[2], self.ee_orientation[3])
                euler_angle[self.instruction_left] += self.map_ee_velocities[self.instruction_right]
                self.new_ee_orientation = self.quaternion_from_euler(euler_angle)
            
            #Toggle gripper state
            elif self.instruction_right == 3 and self.time_lapsed_right >= 1 and not self.gripper_toggled:
                self.gripper_state = not self.gripper_state
                self.gripper_toggled = True
            
            #Reset gripper toggle
            elif self.instruction_right != 3:
                self.gripper_toggled = False

            #Change mode to stop
            if (self.instruction_right == 12) and self.time_lapsed_right >= 0.5:
                self.mode = 'stop'

        self.get_logger().info(f"Current state: {self.mode}")

    # Euler to Quaternion angles
    def quaternion_from_euler(self, euler_angle):
        c1 = math.cos(euler_angle[0] / 2)
        c2 = math.cos(euler_angle[1] / 2)
        c3 = math.cos(euler_angle[2] / 2)

        s1 = math.sin(euler_angle[0] / 2)
        s2 = math.sin(euler_angle[1] / 2)
        s3 = math.sin(euler_angle[2] / 2)

        q = [c1* s2 * s3 + s1 * c2 * c3, #x, y, z, w
            c1 * s2 * c3 - s1 * c2 * s3,
            c1 * c2 * s3 + s1 * s2 * c3, 
            c1 * c2 * c3 - s1 *s2 *s3]
        
        return q
    
    # Quaternion to Euler angles
    def threeaxisrot(self, r11, r12, r21, r31, r32, res):
        res[0] = math.atan2(r11, r12)
        res[1] = math.asin(r21)
        res[2] = math.atan2(r31, r32)
        return res

    def euler_from_quaternion(self, x, y, z, w):
        resxyz = [0, 0, 0]
        resxyz = self.threeaxisrot(r11=(-2*(y*z - w*x)), 
                                   r12=(w*w - x*x - y*y + z*z), 
                                   r21=(2*(x*z + w*y)), 
                                   r31=(-2*(x*y - w*z)), 
                                   r32=(w*w + x*x - y*y -z*z), 
                                   res=resxyz)
        return resxyz

    def timer_callback(self):

        #Publish new end effector pose
        if self.new_ee_position is not None:
            msg = Pose()
            msg.position = Point(x=self.new_ee_position[0], y=self.new_ee_position[1], z=self.new_ee_position[2])
            msg.orientation = Quaternion(x=self.new_ee_orientation[0], y=self.new_ee_orientation[1], z=self.new_ee_orientation[2], w=self.new_ee_orientation[3])
            self.pub_ee.publish(msg)
        
        #Publish switch flag 
        msg_switch = Bool()
        msg_switch.data = self.switch
        self.switch_pub.publish(msg_switch)

    #Get end effector pose
    def ee_callback(self, msg: Pose):
        self.ee_position = [msg.position.x, msg.position.y, msg.position.z]
        self.ee_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    #Publish gripper state
    def timer_gripper_callback(self):
        if self.gripper_state:
            self.gripper_pub.publish(Int32(data=255))
        else:
            self.gripper_pub.publish(Int32(data=0))
        
    
def main(args=None):
    rclpy.init(args=args)
    node = MachineState()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()