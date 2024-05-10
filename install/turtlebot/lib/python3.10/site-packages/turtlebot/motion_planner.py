from typing import Type
from matplotlib.image import resample
import rclpy # type: ignore
import sys
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Twist, Point, Quaternion, Pose # type: ignore
from nav_msgs.msg import Odometry # type: ignore
from std_msgs.msg import Float64MultiArray # type: ignore
from math import sqrt, atan2, sin, cos # type: ignore



class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.publisher = self.create_publisher(Float64MultiArray, '/reference_pose', 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.timer = self.create_timer(1.0, self.get_input)

        self.pose_x = None
        self.pose_y = None
        self.pose_theta = None

        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None

    def pose_callback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        self.pose_theta = self.quat_to_euler(data.pose.pose.orientation)
       
    
    def reached(self):
        angle_threshold = 0.01
        distance_threshold = 0.1
        if self.angle_error() < angle_threshold and self.euclidean_distance_error() <distance_threshold:
            return True
        else:
            return False

    def can_request(self):
        try:
            if self.goal_x is None and self.goal_y is None and self.goal_theta is None:
                return True
            if self.reached():
                return True
            else:
                return False
        except TypeError:
            return False
        
        return False
    #calculate the error in dist and in angle
    def euclidean_distance_error(self):
        """Euclidean distance between current pose and the goal."""
        try:
            return sqrt(pow((self.goal_x - self.pose_x), 2) + pow((self.goal_y - self.pose_y), 2))
        except TypeError:
            print('type error, value of current pose is none, please launch the bot')
            return
    
    def angle_error(self):
        """Calculate the angle at which the goal is there w.r.t 0,0 and find the error wrt our pose"""
        try:
            angle_to_goal = atan2(self.goal_y - self.pose_y , self.goal_x - self.pose_x)
            return atan2(sin(angle_to_goal-self.pose_theta), cos(angle_to_goal-self.pose_theta)) 
        except TypeError:
            print('type error, value is still none')
            return

    #funciton to convert the quaternion orientation and extract just the yaw in euler angles
    def quat_to_euler(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        angle = atan2(siny_cosp, cosy_cosp)
        return angle    
    

    def get_input(self):
        
        goal = Float64MultiArray()
        #check if it is a valid time to prompt the user for data
        if self.can_request():
                try:
                    #expect an input of two numbers seperated by space
                    input_str = input('Please enter q to quit or the destination(x y theta mode): ')
                    #if q then quit/ close the programme
                    if input_str == 'q':
                        print('Thank you for using turtlebot!')
                        sys.exit() 
                    else:
                        input_str = input_str.split()
                        self.goal_x = float(input_str[0])
                        self.goal_y = float(input_str[1])
                        self.goal_theta = float(input_str[2])
                        mode = float(input_str[3])

                        goal.data = [self.goal_x, self.goal_y, self.goal_theta, mode]
                        self.publisher.publish(goal)
                #No input or wrong input given
                except ValueError:
                    print("Please enter the input in the given format only")
                    self.get_input()
                #Only one input given
                except IndexError:
                    print("Please enter the input in the given format only")
                    self.get_input()
                
                

def main():
    rclpy.init()
    motion_planner = MotionPlanner()
    rclpy.spin(motion_planner)
    motion_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

