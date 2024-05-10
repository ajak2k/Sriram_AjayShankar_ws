import rclpy # type: ignore
import sys
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Twist, Point, Quaternion, Pose # type: ignore
from nav_msgs.msg import Odometry # type: ignore
from std_msgs.msg import Float64MultiArray # type: ignore
from math import sqrt, atan2, sin, cos # type: ignore
from tf.transformations import euler_from_quaternion  # type: ignore

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.publisher = self.create_publisher(Float64MultiArray, '/reference_pose', 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.timer = self.create_timer(1.0, self.get_input)

        self.goal = Float64MultiArray()
        
    def pose_callback(self, data):
        self.pose_x = data.pose.position.x
        self.pose_y = data.pose.position.y
        self.pose_theta = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)[2]

    #calculate the error in dist and in angle
    def euclidean_distance_error(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - self.pose_x), 2) + pow((goal_pose[1] - self.pose_y), 2))
    
    def angle_error(self, goal_pose):
        """Calculate the angle at which the goal is there w.r.t 0,0 and find the error wrt our pose"""
        angle_to_goal = atan2(goal_pose[1] - self.pose_y , goal_pose[0] - self.pose_x)
        return atan2(sin(angle_to_goal-self.pose_theta), cos(angle_to_goal-self.pose_theta)) 
    
    def reached(self, goal_pose):
        angle_threshold = 0.01
        distance_threshold = 0.1
        
        if self.angle_error(goal_pose) < angle_threshold & self.euclidean_distance_error(goal_pose) <distance_threshold:
            return True
        else:
            return False

    def get_input(self):
        
        #check if it is a valid time to prompt the user for data
        if self.isvalid():
            try:
                #expect an input of two numbers seperated by space
                input_str = input('Please enter q to quit or the destination(x y theta mode): ')
                #if q then quit/ close the programme
                if input_str == 'q':
                    sys.exit() 
                else:
                    input_str = input_str.split()
                    self.goal[0] = float(input_str[0])
                    self.goal[1] = float(input_str[1])
                    self.goal[2] = float(input_str[2])
                    self.goal[3] = float(input_str[3])
            #No input or wrong input given
            except ValueError:
                print("Please enter the input in the given format only")
                self.get_input()
            #Only one input given
            except IndexError:
                print("Please enter the input in the given format only")
                self.get_input()
        
        self.publisher.publish(self.goal)

def main():
    rclpy.init()
    motion_planner = MotionPlanner()
    rclpy.spin(motion_planner)
    motion_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

