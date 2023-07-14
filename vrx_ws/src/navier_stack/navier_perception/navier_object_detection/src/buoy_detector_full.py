#!/usr/bin/python3
import rospy
import time
import numpy as np
import time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, CompressedImage
from tf import TransformListener
from tf.transformations import quaternion_matrix
from ultralytics import YOLO
from cv_bridge import CvBridge
from navier_msgs.msg import BuoyLocate
from visualization_msgs.msg import Marker
import ros_numpy
import cv2
from geometry_msgs.msg import PointStamped
import os
from utils import *

CAMERA_FRAME = 'wamv/front_camera_link_optical'
LIDAR_FRAME = 'wamv/lidar_wamv_link'
ODOM_FRAME = 'wamv/odom'

CAM_TOPIC = '/wamv/sensors/cameras/front_camera/image_raw'
CAM_INFO_TOPIC = '/wamv/sensors/cameras/front_camera/camera_info'
LIDAR_TOPIC = '/wamv/sensors/lidars/lidar_wamv/points'

DETECTIONS_TOPIC = '/wamv/sensors/cameras/front_camera/object_detector_output'
PROJECTIONS_TOPIC = '/wamv/sensors/cameras/front_camera/image_with_projections'
BUOY_LOCATIONS_TOPIC = '/buoy_locations'

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../weights/weights.pt')
TF_TIMEOUT = rospy.Duration(5)  
BB_SCALING_FACTOR = 0.3
SYNC_THRESHOLD = 0.1 # the threshold for accepting asynchronicity between lidar and camera
MIN_DETECTION_POINTS = 1

class BuoyDetector():
    def __init__(self):
        rospy.loginfo('Initializing object detector node')
        rospy.loginfo(f'Waiting for camera info: {CAM_INFO_TOPIC}')
        self.cam_info = rospy.wait_for_message(CAM_INFO_TOPIC, CameraInfo)
        rospy.loginfo('Camera info received')
        self.P = np.reshape(self.cam_info.P, [3, 4]) #camera projection matrix
        self.listener = TransformListener()
        self.bridge = CvBridge()
        self.sync_sensors = False

        rospy.loginfo(f'Waiting for transform: {CAMERA_FRAME} --> {LIDAR_FRAME}')
        self.listener.waitForTransform(CAMERA_FRAME, LIDAR_FRAME, rospy.Time(), TF_TIMEOUT)
        self.translation, self.rotation = self.listener.lookupTransform(CAMERA_FRAME, LIDAR_FRAME, rospy.Time(0))
        self.translation = np.asarray([self.translation]).T
        self.R = quaternion_matrix(self.rotation)[:3, :3] 
        rospy.loginfo('Got transform')
        rospy.loginfo(f'Waiting for transform: {ODOM_FRAME} --> {CAMERA_FRAME}')
        try:
            self.listener.waitForTransform(ODOM_FRAME, CAMERA_FRAME, rospy.Time(), TF_TIMEOUT)
        except: 
            rospy.logwarn(f'Could not get transform: {ODOM_FRAME} --> {CAMERA_FRAME}... Have you started the localization node?')
            exit(1) 
        rospy.loginfo('Got transform')

        # load the model
        rospy.loginfo(f'Loading model: {MODEL_PATH}')
        self.model = YOLO(MODEL_PATH)
        rospy.loginfo('Model loaded')

        # Initialize subscribers and publishers
        self.img_sub =                      rospy.Subscriber(CAM_TOPIC, Image, self.cam_callback, queue_size=1, buff_size=2_000_000)
        self.velo_sub =                     rospy.Subscriber(LIDAR_TOPIC, PointCloud2, self.lidar_callback, queue_size=1)
        self.detect_img_comp_pub =          rospy.Publisher(DETECTIONS_TOPIC+'/compressed', CompressedImage, queue_size=1)
        self.projection_img_pub =           rospy.Publisher(PROJECTIONS_TOPIC, Image, queue_size=10)
        self.buoy_pub =                     rospy.Publisher(BUOY_LOCATIONS_TOPIC, BuoyLocate, queue_size=10)
        self.marker_pub =                   rospy.Publisher('/visualization_marker', Marker, queue_size = 10)

        rospy.loginfo('Object detector node initialized')

    def get_projections(self, pcl2_msg):
        pcl = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcl2_msg)
        pcl = pcl.transpose() #transpose to get one point per column

        points = np.dot(self.R, pcl) + self.translation  #rotate into camera frame then translate
        points = points[:, np.where(points[2, :] >= 0)[0]] #pick out points in front of the camera
        homogeneous_points = np.vstack((points, np.ones((1, points.shape[1])))) #add homogeneous coordinates
        homogeneous_projections = np.dot(self.P, homogeneous_points) #project onto camera
        pixels = homogeneous_projections[:2] / homogeneous_projections[2:] #normalize and remove homogeneous coordinate

        #remove projections that land outside of the camera resolution
        in_bounds = (pixels[0, :] > 0) & (pixels[0, :] < self.cam_info.width-1) & (pixels[1, :] > 0) & (pixels[1, :] < self.cam_info.height-1) #get indexes within bounds of camera resolution
        pixels = (np.rint(pixels[:, in_bounds])).astype(int) #pixels that are in our cameras image
        points = points[:,  in_bounds] # corresponding points in 3d

        #stack such that each array element represents one point, and one pixel
        pixels = np.column_stack(pixels)
        points = np.column_stack(points)

        return pixels, points

    def find_closest_pixel_index(self, pixel_ref, pixels):
        #takes in a single pixel and an array of pixels
        #returns the index of the pixel in the array closest to the given pixel
        distances = np.sqrt(np.sum((pixels - pixel_ref)**2, axis=1))
        closest_index = np.argmin(distances)
        return closest_index

    def transform_point(self, source_frame, target_frame, point, timestamp=rospy.Time(0)):
        pointstamp = PointStamped()
        pointstamp.header.frame_id = source_frame
        pointstamp.header.stamp = timestamp
        pointstamp.point.x = point[0]
        pointstamp.point.y = point[1]
        pointstamp.point.z = point[2]
        
        point = self.listener.transformPoint(target_frame, pointstamp)
        point = [point.point.x, point.point.y, point.point.z]
        return point


    def set_marker_color(self, colorcode, marker):
        color_map = {0: (0, 1, 0, 1), 1: (1, 0, 0, 1), 2: (1, 1, 0, 1)} # green, red, yellow
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color_map.get(colorcode, (0, 0, 0, 1))
        return marker

    def publish_buoy_marker(self, buoy_locations):
        marker = Marker()
        marker.header.frame_id = ODOM_FRAME
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2

        # Set the scale of the marker
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Set the color of the marker
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1

        # Set the lifetime of the marker
        marker.lifetime = rospy.Duration(0.1)

        # Set the pose of the marker
        marker.pose.orientation.w = 1.0

        for i in range(len(buoy_locations.x)):
            marker.id = i
            marker.pose.position.x = buoy_locations.x[i]
            marker.pose.position.y = buoy_locations.y[i]
            marker.pose.position.z = buoy_locations.z[i]
            marker = self.set_marker_color(buoy_locations.color[i], marker)
            self.marker_pub.publish(marker)
        return
    

    def cam_callback(self, img_msg):
        self.img_msg = img_msg

    def get_compressed_img(self, img, level=50):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = CAMERA_FRAME
        msg.format = 'jpg'
        msg.data = np.array(cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), level])[1]).tostring()
        return msg

    def scale_boxes(self, u_1, u_2, v_1, v_2, scaling_factor=1):
        u_1, u_2, v_1, v_2 = np.array([u_1, u_2, v_1, v_2])
        centers = np.array([[(u_1 + u_2)/2], [(v_1 + v_2)/2]])
        delta = np.array([[(u_2 - u_1)/2], [(v_2 - v_1)/2]])
        top_right = centers + delta*scaling_factor
        bottom_left = centers - delta*scaling_factor
        return bottom_left[0, 0], top_right[0, 0], bottom_left[1, 0], top_right[1, 0]

    def lidar_callback(self, velo_msg):
        tik = time.time()
        callback_timestamp = rospy.Time(0)
        lidar_timestamp = velo_msg.header.stamp.to_sec()
        
        if self.sync_sensors:
            try:
                prev_img_msg = self.img_msg # grab the latest image
                prev_img_timestamp = prev_img_msg.header.stamp.to_sec()
            except AttributeError:
                rospy.loginfo('Got lidar message before any image message, skipping')
                return

            # Simple message filter which waits for one image ahead of the lidar and one behind and chooses the closest one
            if lidar_timestamp > prev_img_timestamp:
                while True:
                    latest_img_msg = self.img_msg
                    latest_img_timestamp = latest_img_msg.header.stamp.to_sec()
                    if latest_img_timestamp > lidar_timestamp:
                        # use the image whose timestamp is the closest to the lidar timestamp
                        if abs(latest_img_timestamp - lidar_timestamp) < abs(prev_img_timestamp - lidar_timestamp):
                            img_msg = latest_img_msg
                            img_msg_timestamp = img_msg.header.stamp.to_sec()
                        else:
                            img_msg = prev_img_msg
                            img_msg_timestamp = img_msg.header.stamp.to_sec()
                        break
                    prev_img_msg = latest_img_msg
                    prev_img_timestamp = latest_img_timestamp
            else:
                img_msg = prev_img_msg # if the most recent image is newer than the lidar message, use this image
                img_msg_timestamp = img_msg.header.stamp.to_sec()

            if abs(lidar_timestamp - img_msg_timestamp) > SYNC_THRESHOLD:
                rospy.logwarn('Timestamp difference between lidar and image is too large, skipping this detection cycle. This is not expected, please check lidar callback in source code')
                return
        else:
            img_msg = self.img_msg

        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        pixels, points = self.get_projections(velo_msg)
        
        # run YOLO and get scaled bounding boxes
        result = self.model(img, verbose=False)[0]
        boxes = result.boxes.data.cpu().numpy()    
        img, centers, colors, u_1, u_2, v_1, v_2 = parse_result(img, boxes, draw=True)
        u_1, u_2, v_1, v_2 = self.scale_boxes(u_1, u_2, v_1, v_2, BB_SCALING_FACTOR)

        tik1 = time.time()
        compressed_msg = self.get_compressed_img(img, level=50)
        print(time.time()-tik1)
        self.detect_img_comp_pub.publish(compressed_msg)

        # perform lidar-based localization
        buoy_locations = BuoyLocate()
        for i in range(len(centers) ):
            pixels_within_box_indices = [idx for idx, pixel in enumerate(pixels) if u_1[i] <= pixel[0] <= u_2[i] and v_1[i] <= pixel[1] <= v_2[i]]
            if len(pixels_within_box_indices) >= MIN_DETECTION_POINTS:
                point = np.median(points[pixels_within_box_indices], axis=0)
                point = self.transform_point(CAMERA_FRAME, ODOM_FRAME, point, timestamp=callback_timestamp)
                buoy_locations.x.append(point[0])
                buoy_locations.y.append(point[1])
                buoy_locations.z.append(point[2])
                buoy_locations.color.append(colors[i])
                self.buoy_pub.publish(buoy_locations)
            else:
                pass
        
        if len(buoy_locations.x) != 0:
            self.buoy_pub.publish(buoy_locations)
            self.publish_buoy_marker(buoy_locations)
        rospy.loginfo(f'Found {len(buoy_locations.x)} buoys in {round(time.time()-tik, 5)} seconds, failed to localize {len(centers)-len(buoy_locations.x)} buoys')


if __name__ == '__main__':
    rospy.init_node('buoy_detector')
    BuoyDetector()
    rospy.spin()