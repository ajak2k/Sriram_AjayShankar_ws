import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class swim_node(Node):
        def __init__(self): #class constructor
            super().__init__("swim_to_node")
            #create a publisher to talk to the turtlesim
            self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
            self.cmd_vel = Twist()
            
            self.pose = None
            #create a subscriber to get the pose of the turtle
            self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
            self.timer = self.create_timer(0.1, self.figure_eight)
            self.count = 0
            
        def pose_callback(self, data): #update the current pose from the topic
            #if it is running for the first time, set the obtained data as the starting point
            if self.pose is None:
                self.starting_x = data.x
                self.starting_y = data.y
            self.pose = data

        def figure_eight(self): #function to calculate command velocities and publish them
            #flag to determine when the turtle comes within the given tolerance of the starting point and reverse the anuglar velocity accordingly
            flag = 1
            turn_speed = 1.0
            linear_speed = 2*turn_speed

            #ensuring we do not call math functions on a 'None' variable
            if self.pose is not None:
                #it is important to take the abs() as if we do not the negative values (relative quadrant 4) will always satisfy the err<threshold condition
                pose_x_err = abs(self.pose.x-self.starting_x)
                pose_y_err = abs(self.pose.y-self.starting_y)

            if (self.pose is not None):   
                #As the pose retured by turtle sim contains more than 16 digits of precision, the turtle seems to never come back exactly to the same point
                #by tuning the allowed threshold of error for a given turn speed, we will be able to get the eight
                #if the threshold is too large, the count can increase multiple times and make the 8 not possible
                #if it is too small, it will not detect that it is back in the center of the eight and keep doing just a circle, hence the tuning need
                #the presision of the bot also seems to be different  in the different axis, need to investigat the sim to understand why
                if (pose_x_err < 0.095)&(pose_y_err < 0.1)&(abs(self.pose.theta) < 0.1):#taking only the first two decimal points on the pose
                    self.count += 1
                #even number will make the turtle go with -1.0 velocity and odd numbers will make it do +1.0 velocity, this helps us to
                #switch directions and make the figure 8
                if self.count%2 == 0:
                    flag = -1*flag
            
            self.cmd_vel.linear.x = linear_speed
            self.cmd_vel.angular.z = flag*turn_speed
            self.cmd_vel_pub.publish(self.cmd_vel)
            print("Keep Looking! The Turtle is turtling!")
            
def main():
    rclpy.init()
    control_node = swim_node()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()