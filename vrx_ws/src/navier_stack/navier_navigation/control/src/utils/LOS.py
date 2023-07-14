from nav_msgs.msg import Odometry
from typing import Tuple
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def parse_odometry(msg: Odometry) -> Tuple[float, float, float]:
    "Parses an sensor_msgs/Odometry message and returns x, y, psi"
    quat = msg.pose.pose.orientation
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    psi = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2]
    return x, y, psi

def get_cross_track(start: Tuple[float, float], end: Tuple[float, float], position: Tuple[float, float]) -> float:
    "Calculates distance from a point(position) to a line passing through two points (start, end)"
    p1 = np.array(start)
    p2 = np.array(end)
    p3 = np.array(position)
    cross_track = np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)
    return cross_track

def get_tangential_distance_to_goal(start: Tuple[float, float], end: Tuple[float, float], position: Tuple[float, float]) -> float:
    distance_to_goal = np.linalg.norm(np.array(end) - np.array(position))
    cross_track = get_cross_track(start, end, position)
    tangential_distance = np.sqrt(distance_to_goal**2 - cross_track**2)
    return tangential_distance

def get_path_angle(start: Tuple[float, float], end: Tuple[float, float]) -> float:
    "Calculates the angle of the straight line from start to end"
    return np.arctan2((end[1]-start[1]),(end[0] - start[0]))

def get_lookahead(cross_track_error: float,delta_min: float, delta_max: float, gamma: float) -> float:
    "calculates lookahead distance based on cross-track error, max and min lookahead distances, and the gamma parameter"
    lookahead = (delta_max-delta_min)*np.exp(-gamma*np.abs(cross_track_error))
    return lookahead

def get_desired_heading(path_angle: float, cross_track_error: float, lookahead_distance: float) -> float:
    "calculates the desired heading based on the path angle, cross-track error, and lookahead distance"
    return path_angle - np.arctan2(cross_track_error, lookahead_distance)


def get_signed_path_distance(line_start, line_end, point):
    A = np.array(line_start)
    B = np.array(line_end)
    P = np.array(point)
    AB = B - A
    AP = P - A
    L = np.linalg.norm(AB)
    proj = np.dot(AB, AP)/L
    D = L-proj
    return D


def debug_los_print(x, y, psi, psi_d, lookahead_distance, cross_track_error, path_angle, torque):
    # prints the LOS debug information
    print("x: ", x)
    print("y: ", y)
    print("psi: ", psi)
    print("psi_d: ", psi_d)
    print("lookahead_distance: ", lookahead_distance)
    print("cross_track_error: ", cross_track_error)
    print("path_angle: ", path_angle)
    print("torque: ", torque)
    return
    

def get_LOS_path_marker(p1, p2, frame, rostime):
    #publish a visualization line marker from x_k, y_k to x_k1, y_k1
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rostime
    marker.type = 0
    marker.scale.x = 0.03
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1
    point0 = Point()
    point1 = Point()
    point0.x, point0.y, point0.z = p1[0], p1[1], 0
    point1.x, point1.y, point1.z = p2[0], p2[1], 0
    marker.points.append(point0)
    marker.points.append(point1)
    marker.id = 100
    return marker