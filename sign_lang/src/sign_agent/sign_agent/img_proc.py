import rclpy
import mediapipe as mp
from rclpy.node import Node
import cv2
import pickle
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os, glob
from std_msgs.msg import Int32MultiArray, Int32


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor_node')
        
        #Parameter for device used
        self.declare_parameter('device_used', 'webcam')
        
        #Subscribe to image topic
        self.subscriber = self.create_subscription(Image, '/image_publisher', self.image_callback, 1)

        #Publisher for instruction
        self.publisher = self.create_publisher(Int32MultiArray, '/instruction', 1)

        #MediaPipe initialization
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(static_image_mode=False, min_detection_confidence=0.9, max_num_hands=2)  

        #Machine models used
        flag = self.get_parameter('device_used').value
        
        if flag == 'Thinklet':
            model_dict_left = pickle.load(open('/home/hri/workspaces/sign/sign_lang/src/sign_agent/model/model_lf_thk.p', 'rb'))
            model_dict_right = pickle.load(open('/home/hri/workspaces/sign/sign_lang/src/sign_agent/model/model_rh_thk.p', 'rb'))
            self.model_used = "Thinklet"
        elif flag == 'webcam':
            model_dict_left = pickle.load(open('/home/hri/workspaces/sign/sign_lang/src/sign_agent/model/model_lf_web.p', 'rb'))
            model_dict_right = pickle.load(open('/home/hri/workspaces/sign/sign_lang/src/sign_agent/model/model_rh_web.p', 'rb'))
            self.model_used = "webcam"
            
        #Set up models
        self.model_R = model_dict_right['model']
        self.model_L = model_dict_left['model']

        #Set up label instructions
        self.labels_dict_right = {0: 'X', 1: 'y', 2: 'w', 3: 'o', 4: 'r', 5: 'i', 6: 'i rot', 7: 'h', 8: 'Index up',
               9: 'Index down', 10: 'Two up', 11: 'Two down', 12: 'Stop', 13: 'a', 14: 'b', 15: 'c', 16: 'f', 17: 'L', 18: 'pulgar abajo'}

        self.labels_dict_left = {0: 'X', 1: 'y', 2: 'w', 3: 'o', 4: 'Stop', 5: 'a', 6: 'b', 7: 'c', 8: 'f', 9: 'h', 10: 'L', 11: 'V', 12: 'Pulgar arriba'}
        
        

    def image_callback(self, msg: Image):
        data_aux_R = []
        x_R = []
        y_R = []
        data_aux_L = []
        x_L = []
        y_L = []
        prediction_R = [-1]
        prediction_L = [-1]

        #Convert image to OpenCV format
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        H, W, _ = frame.shape
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)
        
        #Get hand landmarks and handedness from MediaPipe
        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                hand_label = handedness.classification[0].label

                #Get landmarks from right hand and save them
                if hand_label == 'Right':
                    for i in range(len(hand_landmarks.landmark)):
                        x = hand_landmarks.landmark[i].x
                        y = hand_landmarks.landmark[i].y
                        data_aux_R.append(x)
                        data_aux_R.append(y)
                        x_R.append(x)
                        y_R.append(y)

                    #Draw landmarks and connections on the image
                    self. mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        self.mp_drawing_styles.get_default_hand_connections_style()
                    )

                #Get landmarks from left hand and save them
                elif hand_label == 'Left':
                    for i in range(len(hand_landmarks.landmark)):
                        x = hand_landmarks.landmark[i].x
                        y = hand_landmarks.landmark[i].y
                        data_aux_L.append(x)
                        data_aux_L.append(y)
                        x_L.append(x)
                        y_L.append(y)

                    self. mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        self.mp_drawing_styles.get_default_hand_connections_style()
                    )

            # Predict right hand instruction using ML model
            if x_R and y_R and len(data_aux_R) == 42:
                prediction_R = self.model_L.predict([np.asarray(data_aux_R)])
                predicted_char_R = self.labels_dict_left[int(prediction_R[0])]
                cv2.putText(frame, predicted_char_R, (30, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 200, 200), 2, cv2.LINE_AA)

            # Predict left hand instruction using ML model 
            if x_L and y_L and len(data_aux_L) == 42:
                prediction_L = self.model_R.predict([np.asarray(data_aux_L)])
                predicted_char_L = self.labels_dict_right[int(prediction_L[0])]
                cv2.putText(frame, predicted_char_L, (450, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 200, 255), 2, cv2.LINE_AA)

            #Publish instructions for each
            output = Int32MultiArray()
            output.data = [int(prediction_R[0]), int(prediction_L[0])]
            self.publisher.publish(output)
            self.get_logger().info(f'Model used: {self.model_used}')
            self.get_logger().info(f'Instrucciones: {output}')

        cv2.imshow('frame', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()