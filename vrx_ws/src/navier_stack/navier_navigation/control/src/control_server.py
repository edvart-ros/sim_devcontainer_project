#!/usr/bin/env python3

import rospy
import numpy as np
from navier_msgs.msg import ControlGoal, ControlCancel
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from utils.LOS import *
from controllers import HeadingPID, DynamicPositioningPID
from visualization_msgs.msg import Marker

ODOM_FRAME = 'wamv/odom'

class ControlServer():
    def __init__(self):
        self.goal = ControlGoal
        self.rate = rospy.Rate(50)
        
        self.valid_goal_types = ['cancel', 'LOS', 'DP']
        self.lookahead_max = 5
        self.lookahead_min = 0.3
        self.gamma = 0.1

        self.goal_sub = rospy.Subscriber('/control_server/goal', ControlGoal, self.goal_cb, queue_size=1)
        self.cancel_sub = rospy.Subscriber('/control_server/cancel', ControlCancel, self.cancel_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber('/wamv/odometry', Odometry, self.odom_cb, queue_size=1)
        self.cmd_force_pub = rospy.Publisher('/cmd_force', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size = 10)

        rospy.loginfo('control server started')

    def goal_cb(self, goal_msg):
        if goal_msg.type not in self.valid_goal_types:
            rospy.logwarn('goal.type is not a valid type, valid types are: ' + str(self.valid_goal_types))
            return
        if goal_msg.type == 'DP':
            if (len(goal_msg.x_list) != 1 or len(goal_msg.y_list) != 1 or len(goal_msg.psi_list) !=1 ):
                rospy.logwarn('inconsistent goal message for DP. goal state lists must be of length 1')
                return
            self.DP_controller = DynamicPositioningPID()
            self.DP_controller.set_reference(goal_msg.x_list[0], goal_msg.y_list[0], goal_msg.psi_list[0])
        
        elif goal_msg.type == 'LOS':
            if (len(goal_msg.x_list) != len(goal_msg.y_list) or len(goal_msg.x_list) == 0):
                rospy.logwarn('inconsistent goal message for LOS. goal state lists must be of equal length and not empty')
                return
            self.heading_controller = HeadingPID()

        self.goal = goal_msg
        rospy.loginfo('goal received')

    def cancel_cb(self, cancel_msg):
        self.goal = None
        rospy.loginfo('goal cancelled')
        return

    def odom_cb(self, msg):
        self.x, self.y, self.psi = parse_odometry(msg)
        if self.goal == None:
            return
        
        if self.goal.type == 'LOS':
            self.LOS()
        if self.goal.type == 'DP':
            self.DP()
        return



    def LOS(self):
        force_msg = Twist()
        x, y, psi = self.x, self.y, self.psi

        # problem setup
        pos = (x, y)
        A = ([self.goal.x_list[0], self.goal.y_list[0]])
        B = ([self.goal.x_list[1], self.goal.y_list[1]])
        
        #publish the path visualization
        marker = get_LOS_path_marker(A, B, ODOM_FRAME, rospy.Time.now())
        self.marker_pub.publish(marker)

        # perform LOS calculations
        cross_track_error = get_cross_track(A, B, pos)
        lookahead_distance = get_lookahead(cross_track_error, self.lookahead_min, self.lookahead_max, self.gamma)
        if lookahead_distance > abs(get_signed_path_distance(A, B, pos)): #never look further than the goal
            lookahead_distance = get_signed_path_distance(A, B, pos)
        path_angle = get_path_angle(A, B)
        psi_d = get_desired_heading(path_angle, cross_track_error, lookahead_distance)
        
        # update controller and get output
        self.heading_controller.set_state(psi_d, psi)
        force_msg.angular.z = self.heading_controller.get_output()


        # publish force command
        self.cmd_force_pub.publish(force_msg)
        return


    def DP(self):
        self.DP_controller.set_state(self.x, self.y, self.psi)
        self.DP_controller.set_reference(self.goal.x_list[0], self.goal.y_list[0], self.goal.psi_list[0])
        force_msg = Twist()
        u = self.DP_controller.get_output()
        force_msg.linear.x = u[0]
        force_msg.linear.y = u[1]
        force_msg.angular.z = u[2]
        #self.cmd_force_pub.publish(force_msg)
        print('state: \n', self.DP_controller.x)
        print('\nreference: \n', self.DP_controller.x_d)
        print('\nerror: \n', self.DP_controller.error)
        print('\ncontrol: \n', u)
        self.cmd_force_pub.publish(force_msg)
        return
    
if __name__ == '__main__':
    rospy.init_node('control_server')
    control_server = ControlServer()
    rospy.spin()