#!/usr/bin/env python3

import numpy as np
import rospy
import ros_numpy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge
from utils import *
import os
import time

# green:  0
# red:    1
# yellow: 2

class BuoyDetector:
    def __init__(self):
        self.bridge = CvBridge()

        # Load YOLO model
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../weights/weights.pt')
        rospy.loginfo(f'Loading model: {model_path}')
        self.model = YOLO(model_path)
        rospy.loginfo('Model loaded')

        # Initialize subscribers and publishers
        self.img_sub = rospy.Subscriber('/zedx/zed_node/rgb/image_rect_color', Image, self.cam_callback, queue_size=1, buff_size=2_000_000)
        self.img_pub = rospy.Publisher('/object_detector_output', Image, queue_size=10)

    def cam_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        tik = time.time()
        result = self.model(cv_image, verbose=False)[0]
        boxes = result.boxes.data.cpu().numpy()
        if np.size(boxes) > 0:
            img, centers, colors, u_1, u_2, v_1, v_2 = parse_result(cv_image, boxes)
            msg = ros_numpy.msgify(Image, img, encoding="bgr8")
        self.img_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('yolo_buoy_detector')
    detector = BuoyDetector()
    rospy.spin()