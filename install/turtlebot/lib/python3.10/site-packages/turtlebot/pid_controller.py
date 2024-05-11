import rclpy #type: ignore
from rclpy.node import Node #type:ignore
from std_msgs.msg import Float64MultiArray #type: ignore
from nav_msgs.msg import Odometry # type: ignore
from turtlesim.msg import Pose_simple# type: ignore

from math import atan2

class pid_controller(Node):

    #funciton to convert the quaternion orientation and extract just the yaw in euler angles
    @staticmethod
    def quaternion_to_euler(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        euler_angle = atan2(siny_cosp, cosy_cosp)
        return euler_angle