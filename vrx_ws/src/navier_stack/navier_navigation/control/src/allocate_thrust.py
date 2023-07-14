#!/usr/bin/python3
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from tf import TransformListener
from tf.transformations import euler_from_quaternion

BASE_FRAME = 'wamv/base_link'
REAR_LEFT_ENGINE_LINK = 'wamv/rear_left_engine_link'
REAR_RIGHT_ENGINE_LINK = 'wamv/rear_right_engine_link'

REAR_LEFT_THRUST_TOPIC = '/wamv/thrusters/rear_left_thrust_cmd'
REAR_RIGHT_THRUST_TOPIC = '/wamv/thrusters/rear_right_thrust_cmd'
FRONT_LEFT_THRUST_TOPIC = '/wamv/thrusters/front_left_thrust_cmd'
FRONT_RIGHT_THRUST_TOPIC = '/wamv/thrusters/front_right_thrust_cmd'

REAR_LEFT_ANGLE_TOPIC = '/wamv/thrusters/rear_left_thrust_angle'
REAR_RIGHT_ANGLE_TOPIC = '/wamv/thrusters/rear_right_thrust_angle'
FRONT_LEFT_ANGLE_TOPIC = '/wamv/thrusters/front_left_thrust_angle'
FRONT_RIGHT_ANGLE_TOPIC = '/wamv/thrusters/front_right_thrust_angle'

#rear left
L1_X = -2.373776 
L1_Y = 1.027135
#rear right
L2_X = -2.373776 
L2_Y = -1.027135
#front left
L3_X = 1.75
L3_Y = 1.027135
#front right
L4_X = 1.75
L4_Y = -1.027135

a = np.pi/6 #rear thruster angle
REAR_THRUSTER_ANGLE = a

# configuration for the fixed configuration
B_x = np.array([[np.cos(a), np.cos(a), -0, 0],
              [np.sin(a),-np.sin(a), -1, 1],
              [L1_X*np.sin(a)-L1_Y*np.cos(a), -L2_X*np.sin(a)-L2_Y*np.cos(a), -L3_X, L4_X]])

B_r = np.array([[1, 0, 1, 0],
                [0, 0, 0, 1],
                [-L1_Y, L1_X, -L2_Y, L2_X]])
# configuration for the fixed front thrusters
B_f = np.array([[  -0,     0],
                [  -1,     1],
                [-L3_X, L4_X]])

# configuration for azimuthing configuration
B_az = np.hstack((B_r, B_f))

THRUST_LIMIT = 200
ANGLE_LIMIT = np.pi*0.5 #bi-directional

FORCE_X_MAX_AZ = 4*THRUST_LIMIT
FORCE_Y_MAX_AZ = 4*THRUST_LIMIT
MOMENT_Z_MAX_AZ = THRUST_LIMIT * (abs(L1_X) + abs(L1_Y) + abs(L2_X) + abs(L2_Y) + abs(L3_X) + abs(L3_Y) + abs(L4_X) + abs(L4_Y))
ANGLE_TOLERANCE = np.pi/6

FORCE_X_MAX_FIXED = 2*THRUST_LIMIT*np.cos(a)
FORCE_Y_MAX_FIXED = 2*THRUST_LIMIT*np.sin(a) + 2*THRUST_LIMIT
MOMENT_Z_MAX_FIXED = THRUST_LIMIT * (abs(L1_X*np.sin(a)) + abs(L1_Y*np.cos(a)) + abs(L2_X*np.sin(a)) + abs(L2_Y*np.cos(a) + abs(L3_X) + abs(L4_X)))

class AllocateThrust():
    def __init__(self): 
        self.B_pinv_x = np.linalg.pinv(B_x) #psuedo-inverse
        self.B_pinv_az = np.linalg.pinv(B_az)
        self.thrust_limit = THRUST_LIMIT
        
        self.thruster_msg = Float32()
        self.rear_left_thrust_pub   = rospy.Publisher(REAR_LEFT_THRUST_TOPIC, Float32, queue_size = 10)
        self.rear_right_thrust_pub  = rospy.Publisher(REAR_RIGHT_THRUST_TOPIC, Float32, queue_size = 10)
        self.front_left_thrust_pub  = rospy.Publisher(FRONT_LEFT_THRUST_TOPIC, Float32, queue_size = 10)
        self.front_right_thrust_pub = rospy.Publisher(FRONT_RIGHT_THRUST_TOPIC, Float32, queue_size = 10)
        self.rear_left_angle_pub    = rospy.Publisher(REAR_LEFT_ANGLE_TOPIC, Float32, queue_size = 10)
        self.rear_right_angle_pub   = rospy.Publisher(REAR_RIGHT_ANGLE_TOPIC, Float32, queue_size = 10)
        self.front_left_angle_pub   = rospy.Publisher(FRONT_LEFT_ANGLE_TOPIC, Float32, queue_size = 10)
        self.front_right_angle_pub  = rospy.Publisher(FRONT_RIGHT_ANGLE_TOPIC, Float32, queue_size = 10)

        # get actual motor angles via tf listener
        self.listener = TransformListener()
        rospy.loginfo(f'waiting for transform from {BASE_FRAME} to {REAR_LEFT_ENGINE_LINK}')
        self.listener.waitForTransform(BASE_FRAME, REAR_LEFT_ENGINE_LINK, rospy.Time(0), timeout=rospy.Duration(5))
        rospy.loginfo('got transform')
        rospy.loginfo(f'waiting for transform from {BASE_FRAME} to {REAR_RIGHT_ENGINE_LINK}')
        self.listener.waitForTransform(BASE_FRAME, REAR_RIGHT_ENGINE_LINK, rospy.Time(0), timeout=rospy.Duration(5))
        rospy.loginfo('got transform')

        self.cmd_force_sub = rospy.Subscriber('/cmd_force', Twist, self.cmd_force_callback, queue_size=1)
        rospy.loginfo('thrust allocation node started')

    def publish_angle_commands(self, rear_left_angle, rear_right_angle):
        self.thruster_msg.data = rear_left_angle
        self.rear_left_angle_pub.publish(self.thruster_msg)
        self.thruster_msg.data = rear_right_angle
        self.rear_right_angle_pub.publish(self.thruster_msg)

    def publish_thrust_commands(self, rear_left_thrust, rear_right_thrust, front_left_thrust, front_right_thrust):
        self.thruster_msg.data = rear_left_thrust
        self.rear_left_thrust_pub.publish(self.thruster_msg)
        self.thruster_msg.data = rear_right_thrust
        self.rear_right_thrust_pub.publish(self.thruster_msg)
        self.thruster_msg.data = front_left_thrust
        self.front_left_thrust_pub.publish(self.thruster_msg)
        self.thruster_msg.data = front_right_thrust
        self.front_right_thrust_pub.publish(self.thruster_msg)

    def cmd_force_callback(self, msg):
        try:
            config = rospy.get_param('thruster_configuration')
        except:
            rospy.logwarn('failed to get thruster configuration parameter, defaulting to fixed configuration')
            config = "fixed"
        
        if config == "fixed":
            tau_desired = np.array([[FORCE_X_MAX_FIXED*msg.linear.x],[FORCE_Y_MAX_FIXED*msg.linear.y],[MOMENT_Z_MAX_FIXED*msg.angular.z]]) #generalized desired force

            f = self.B_pinv_x @ tau_desired #commanded force (least squares solution)
            if np.max(abs(f)) > self.thrust_limit:
                f = f * self.thrust_limit / np.max(abs(f))

            #scale forces to [-1,   1] for publishing to vrx thrusters
            f = f/self.thrust_limit
            self.publish_angle_commands(REAR_THRUSTER_ANGLE, -REAR_THRUSTER_ANGLE) #set fixed angle
            self.publish_thrust_commands(f[0], f[1], f[2], f[3])
            return
        
        elif config == "azimuth":
            tau_desired = np.array([[FORCE_X_MAX_AZ*msg.linear.x],[FORCE_Y_MAX_AZ*msg.linear.y],[MOMENT_Z_MAX_AZ*msg.angular.z]]) #generalized desired force
            u = self.B_pinv_az @ tau_desired
            f = np.zeros(4)
            # rear
            f[0] = np.linalg.norm(u[0:2])
            f[1] = np.linalg.norm(u[2:4])
            # bow
            f[2] = u[4]
            f[3] = u[5]
            if np.max(abs(f)) > self.thrust_limit:
                f = f * self.thrust_limit / np.max(abs(f))
            #scale forces to [-1, 1] for publishing to vrx thrusters
            f = f/self.thrust_limit
            
            angles = np.zeros(2)
            for i in range(2):
                angles[i] = np.arctan2(u[2*i+1,0], u[2*i,0])
            # if angle violated, flip direction and reverse thrust
            for i in range(len(angles)):
                if abs(angles[i]) > ANGLE_LIMIT:
                    angles[i] += np.pi 
                    angles[i] = np.arctan2(np.sin(angles[i]), np.cos(angles[i])) #keep in [-pi, pi]
                    f[i] = -1*f[i]
            
            self.publish_angle_commands(angles[0], angles[1])
            #check thruster angles before applying thrust
            left_engine_quat = self.listener.lookupTransform(BASE_FRAME, REAR_LEFT_ENGINE_LINK, rospy.Time(0))[1]
            left_engine_angle = euler_from_quaternion(left_engine_quat)[2] #get the yaw in euler
            right_engine_quat = self.listener.lookupTransform(BASE_FRAME, REAR_RIGHT_ENGINE_LINK, rospy.Time(0))[1]
            right_engine_angle = euler_from_quaternion(right_engine_quat)[2] #get the yaw in eule
            left_angle_error = left_engine_angle-angles[0]
            right_angle_error = right_engine_angle-angles[1]
            if abs(left_angle_error) > ANGLE_TOLERANCE or abs(right_angle_error) > ANGLE_TOLERANCE:
                self.publish_thrust_commands(0, 0, 0, 0)
            else:
                self.publish_thrust_commands(f[0], f[1], f[2], f[3])
            return
        
        else:
            rospy.logwarn('couldnt get a valid thruster configuration, has it been set?')
            rospy.logwarn('valid options are "fixed" and "azimuth"')
            return
if __name__ == "__main__":
    rospy.init_node('thrust_allocation_node')
    name_node = AllocateThrust()
    rospy.spin()
