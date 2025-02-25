import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import nep
import json


class ImagePub(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.publisher = self.create_publisher(Image, '/image_publisher', 1)
        

        self.declare_parameter('device_used', 'Thinklet')
        self.flag = self.get_parameter('device_used').value

        if self.flag == 'webcam':
            self.timer = self.create_timer(0.001, self.timer_callback_webcam)
            self.cap = cv2.VideoCapture(0)
    
        elif self.flag == 'Thinklet':
            self.timer = self.create_timer(0.001, self.timer_callback)
            # Create subscriber to android camera
            self.node = nep.node("subscriber_tk")
            self.sub = self.node.new_sub("androidCamera", 'images')
       
        
        self.bridge = CvBridge()

        #Set image width and height
        self.w = 640
        self.h = 480

    def timer_callback(self):
        #Get image from android camera
        s, frame = self.sub.listen()
        if s:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            frame = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_NEAREST)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

            #Send image to /image_publisher topic
            self.publisher.publish(msg)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
    
    def timer_callback_webcam(self):
        #Get image from webcam
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_NEAREST)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

            #Send image to /image_publisher topic
            self.publisher.publish(msg)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
        else:
            self.get_logger().error('Could not read frame')


def main(args=None):
    rclpy.init(args=args)
    image_pub = ImagePub()
    try:
        rclpy.spin(image_pub)
    except KeyboardInterrupt:
        pass
    finally:
        image_pub.cap.release()
        cv2.destroyAllWindows()
        image_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()