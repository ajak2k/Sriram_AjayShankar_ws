import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class swim_node(Node):
        def __init__(self):
            super().__init__("swim_to_node")
            self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
            self.cmd_vel = Twist()

            self.pose = None
            self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
            self.timer = self.create_timer(0.1, self.figure_eight)
            self.count = 0
        

        #update the current pose from the topic    
        def pose_callback(self, data):
            if self.pose is None:
                self.starting_x = data.x
                self.starting_y = data.y
            self.pose = data

        def figure_eight(self):
            flag = 1
            turn_speed = 1.0
            linear_speed = 2*turn_speed

            if self.pose is not None:
                pose_x_err = abs(self.pose.x-self.starting_x)
                pose_y_err = abs(self.pose.y-self.starting_y)

            if (self.pose is None):
                turn_speed = turn_speed
            else:    
                if (pose_x_err < 0.1)&(pose_y_err < 0.1):#taking only the first two decimal points on the pose
                    self.count += 1
                if self.count%2 == 0:
                    flag = -1*flag
            
            self.cmd_vel.linear.x = linear_speed
            self.cmd_vel.angular.z = flag*turn_speed
            self.cmd_vel_pub.publish(self.cmd_vel)
            print("Keep Looking! The Turtle is turtling!")
            if self.pose is not None:
                print(self.starting_x, self.pose.x, pose_x_err)
                print(self.starting_y, self.pose.y, pose_y_err)
            print(self.count)
            
def main():
    rclpy.init()
    control_node = swim_node()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()