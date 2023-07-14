#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from nmea_msgs.msg import Sentence
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from datetime import datetime
import tf
import utm
import numpy as np
import time

ODOM_FRAME = 'wamv/odom'
BASE_FRAME = 'wamv/base_link'


class Localization():
    def __init__(self):
        rospy.loginfo('waiting for initial messages')
        initial_datum = rospy.wait_for_message('/wamv/sensors/gps/gps/fix', NavSatFix)
        self.initial_utm = (utm.from_latlon(initial_datum.latitude, initial_datum.longitude))
        self.initial_utm = np.asarray([self.initial_utm[0], self.initial_utm[1]])
        rospy.loginfo('got initial messages, odom publisher started')
        self.odom_msg = self.init_odom_msg()
        self.tik = time.time()
        self.displacement = [0, 0, 0]
        self.orientation = None
        self.rate = rospy.Rate(200)

        self.br          = tf.TransformBroadcaster()
        self.imu_sub     = rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, self.imu_cb, queue_size=1)
        self.pos_lla_sub = rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, self.position_cb) #subscribes to filtered lat-lon-alt message from xsens
        self.vel_sub     = rospy.Subscriber('/wamv/sensors/gps/gps/fix_velocity', Vector3Stamped, self.vel_cb) #subscribes to xsens twist msg
        self.odom_pub    = rospy.Publisher('/wamv/odometry', Odometry, queue_size=10)


    def init_odom_msg(self):
        odom_msg = Odometry()
        odom_msg.header.frame_id = ODOM_FRAME
        odom_msg.child_frame_id = BASE_FRAME
        odom_msg.header.seq = 0
        return odom_msg

    

    def imu_cb(self, msg):
        self.orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w] #unpacking to publish transform
        self.odom_msg.twist.twist.angular.x = msg.angular_velocity.x
        self.odom_msg.twist.twist.angular.y = msg.angular_velocity.y
        self.odom_msg.twist.twist.angular.z = msg.angular_velocity.z

        self.odom_msg.pose.pose.orientation = msg.orientation #copying straight from imu topic
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.seq += 1
        
        rostime = rospy.Time.now()

        self.br.sendTransform((self.displacement[0], self.displacement[1], 0),
                (self.orientation[0],self.orientation[1],self.orientation[2],self.orientation[3]),
                rostime,
                BASE_FRAME,
                ODOM_FRAME)

        self.odom_pub.publish(self.odom_msg) #Publish to the odometry topic
        self.rate.sleep()


    def position_cb(self, msg):
        self.current_utm = (utm.from_latlon(msg.latitude, msg.longitude))
        self.current_utm = np.asarray([self.current_utm[0], self.current_utm[1]])

        rostime = rospy.Time.now()
        self.displacement = self.current_utm - self.initial_utm
        
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, self.odom_msg.pose.pose.position.z,  = self.displacement[0], self.displacement[1], 0
        self.odom_msg.header.seq += 1
        self.odom_pub.publish(self.odom_msg) #Publish to the odometry topic
        self.rate.sleep()

        
    def vel_cb(self, msg):
        #transform velocities from world frame to body frame
        if self.orientation is None:
            return
        psi = euler_from_quaternion(self.orientation)[2] - np.pi/2
        R = np.array([[np.cos(psi), -np.sin(psi), 0],
                      [np.sin(psi),  np.cos(psi), 0],
                      [0,                0,       1]])
        
        vel = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        vel_fixed = R.T@vel
        self.odom_msg.twist.twist.linear.x = vel_fixed[0]
        self.odom_msg.twist.twist.linear.y = vel_fixed[1]
        self.odom_msg.twist.twist.linear.z = vel_fixed[2]


if __name__ == '__main__':
    rospy.init_node('localization_node')
    loc = Localization()
    rospy.spin()
