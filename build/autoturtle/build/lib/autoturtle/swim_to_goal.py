import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rcl_interfaces.msg import Log
import math
import sys

kp_x = 5
kp_ang = 10
distance_tolerance = 0.1
angle_tolerance = 0.01
err_count = 0

class swim_to_goal(Node):
    def __init__(self):
        super().__init__("swim_to_goal")
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        #subscribe to rosout to see if the turtle is in distress
        self.collision_sub = self.create_subscription(Log, '/rosout', self.collision_callback, 10)
        #timer to keep calling the go to goal function and have the turtle controlled every 0.1 seconds
        self.timer = self.create_timer(0.1, self.go_to_goal)
        #variables to hold the current state in pose and the destination in goal
        self.pose = Pose()
        self.goal = None

    #Additional function that listens to /rosout to see if the turtle has hit a wall and reset the goal; 
    #I understand this is not part of the assignment I was curious and just did it
    def collision_callback(self, data):
        global err_count
        if data.msg.startswith("Oh no! I hit the wall!"):
            err_count = err_count + 1
        if err_count>20:
            self.goal = None
            print("The turtle hit a wall T-T | Goal has been reset")
            err_count = 0

    #update the current location from the         
    def pose_callback(self, data):
        self.pose = data

    #Function to get the input destination from the user and handle the related errors
    def get_input(self):
        self.goal = Pose()
        try:
            #expect an input of two numbers seperated by space
            input_str = input('Please enter q to quit or the destination(x y): ')
            #if q then quit/ close the programme
            if input_str == 'q':
                sys.exit() 
            else:
                input_str = input_str.split()
                self.goal.x = float(input_str[0])
                self.goal.y = float(input_str[1])
        #No input or wrong input given
        except ValueError:
            print("Please enter the input in the X Y format only")
            self.get_input()
        #Only one input given
        except IndexError:
            print("Please enter the input in the X Y format only")
            self.get_input()

    def go_to_goal(self):
        new_vel = Twist()
        #at the start or when the turtle is reset, get the goal from the user
        if self.goal == None:
            self.get_input()
        # find the distance from the current point to the destination using euclidian distance
        distance_to_goal = math.sqrt( (self.goal.x - self.pose.x)**2  + (self.goal.y - self.pose.y)**2 )
        angle_to_goal =math.atan2(self.goal.y - self.pose.y , self.goal.x - self.pose.x)

        # use atan2(sin/cos) to get the angle in a regularized fashion, source: lecture material 
        angle_error = math.atan2(math.sin(angle_to_goal-self.pose.theta), math.cos(angle_to_goal-self.pose.theta))
        
        if abs(angle_error) >= angle_tolerance:
            new_vel.angular.z = kp_ang * angle_error
        else :
            if( distance_to_goal ) >= distance_tolerance:
                new_vel.linear.x = kp_x * distance_to_goal
            else :
                new_vel.linear.x= 0.0
                print("Goal Reached ")
                self.get_input()

        self.cmd_vel_pub.publish(new_vel)

def main():
    rclpy.init()
    controller_node = swim_to_goal()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()