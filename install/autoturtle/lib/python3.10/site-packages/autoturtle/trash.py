import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


#Define the classes we need
#Control node to get the user input and calculate the command to send
class my_controller(Node):
    K_P_lin = 1.5
    K_P_ang = 0.8
    dist_tolerance = 0.1
    angle_tolerance = 1

    def __init__(self):
        super().__init__('swim_to_goal')#Creade a node called swim_to_goal using the constructor of the class NODE from rclpy
        self.current_pose = Pose()
        self.dest = Pose()
        self.cmd_vel = Twist()
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel',10)# Publisher setup to send info to cmd_vel                    
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.sub_callback, 10)#subscriber setup to call the sub_callback whenever it gets a message in the topic pose
        
    #The function that will be called whenever the subcriber get a new message
    def sub_callback(self, data):
        if self.dest.x is None or self.dest.y is None:
            self.get_destination()
        else:
            self.current_pose = data #create a new variable pose that is visible to all the funcitons of this class and put the new info there
            self.controller()

    def get_destination(self):
        try:
            input_str = input('Please enter the destination(x y): ')
            input_str = input_str.split()
            self.dest.x = float(input_str[0])
            self.dest.y = float(input_str[1])
        except ValueError:
            self.get_destination()

    def controller(self):
        #Implement Control Calculations
        distance_err = math.sqrt((self.dest.x - self.current_pose.x)**2 + (self.dest.y - self.current_pose.y)**2)
        angle_to_dest = math.atan2(self.dest.y - self.current_pose.y, self.dest.x-self.current_pose.x)
        angle_err = angle_to_dest - self.current_pose.theta
        #set the control velocities
        self.cmd_vel.linear.x = self.K_P_lin * distance_err
        self.cmd_vel.angular.z = self.K_P_ang * angle_err
        #send this to the bot
        self.publisher_.publish(self.cmd_vel)
        #now check if the destination is reached
        if angle_err<self.angle_tolerance:
            self.dest.x = None
            self.dest.y = None

#we need main to start the nodes, call the controller and close the nodes
def main():
    #start the ros2 client 
    rclpy.init()
    #create the object of thecontroller
    control_node = my_controller()    
    #spin it to keep it running
    rclpy.spin(control_node)
    #shut down all nodes
    control_node.destroy_node()
    rclpy.shutdown()
    print('Thanks for Playing!')

if __name__ == '__main__':
    main()