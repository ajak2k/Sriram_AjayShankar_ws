import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
import time

TRAVEL_TIME = 1.5 #The time within which we need the tutle to be at the destination, unit seconds

#Define the classes we need
#Control node to get the user input and calculate the command to send
class my_controller(Node):
    def __init__(self):
        super().__init__('swim_to_goal')#Creade a node called swim_to_goal using the constructor of the class NODE from rclpy
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel',10)# Publisher setup to send info to cmd_vel                    
        self.subscriber_ = self.create_subscription(Pose, 'turtle1/pose', self.sub_callback, 10)#subscriber setup to call the sub_callback whenever it gets a message in the topic pose
        self.controller()#Go to the function to control the bot

    #The function that will be called whenever the subcriber get a new message
    def sub_callback(self, data):
        self.curent_pose = data #create a new variable pose that is visible to all the funcitons of this class and put the new info there

    #Calculate the desired linear velocities for the given destination coordinates
    def calc_velocity(destination, self):
        cmd_velocity = Twist()
        pose_err = destination - self.current_pose
        cmd_velocity.linear.x = pose_err.x / TRAVEL_TIME
        cmd_velocity.linear.y = pose_err.y / TRAVEL_TIME
        return cmd_velocity
    
    def controller(self):
        dest = Pose() #variable to store the destination as a pose data type; make it easy to compare with current locaiton
        while True:
            print('Please enter the destination(x y):')
            dest.x = float(sys.argv[1])
            dest.y = float(sys.argv[2])
            self.publisher_.publish(self.calc_velocity(dest))
            time.sleep(0.01)

            if dest == self.current_pose:
                break
            #end if
        #end while 


            

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