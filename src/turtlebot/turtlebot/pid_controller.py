import rclpy #type: ignore
from rclpy.node import Node #type:ignore
from std_msgs.msg import Float64MultiArray #type: ignore
from nav_msgs.msg import Odometry # type: ignore
from turtlesim.msg import Pose_simple# type: ignore

from math import atan2, sqrt, sin, cos

class pid_controller(Node):

    #funciton to convert the quaternion orientation and extract just the yaw in euler angles
    @staticmethod
    def quaternion_to_euler(q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        euler_angle = atan2(siny_cosp, cosy_cosp)
        return euler_angle
    @staticmethod
    def euclidean_distance_error(current_pose, goal_pose):
        """Euclidean distance between current pose and the goal."""
        try:
            return sqrt(pow((goal_pose.x - current_pose.x), 2) + pow((goal_pose.y - current_pose.y), 2))
        except TypeError as e:
            print('Error calculating distance: ', e)
            return
    @staticmethod
    def angle_error(current_pose, goal_pose):
        """Calculate the angle at which the goal is there w.r.t 0,0 and find the error wrt our pose"""
        try:
            angle_to_goal = atan2(goal_pose.y - current_pose.y , goal_pose.x - current_pose.x)
            angle_error = atan2(sin(angle_to_goal-current_pose.theta), cos(angle_to_goal-current_pose.theta))
            return  angle_error
        except TypeError as e:
            print('Error calculating angles: ',e)
            return