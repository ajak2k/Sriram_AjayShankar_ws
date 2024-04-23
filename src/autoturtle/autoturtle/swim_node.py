import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class swim_node(Node):
        def __init__(self):
            super().__init__("swim_to_goal")
            self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
            self.cmd_vel = Twist()
            #move it just a bit off the starting position of 5.44,5.44
            self.cmd_vel.linear.x = 0.1
            self.cmd_vel_pub.publish(self.cmd_vel)
            self.pose = Pose()
            self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
            self.timer = self.create_timer(0.1, self.figure_eight)

        #update the current pose from the topic    
        def pose_callback(self, data):
            self.pose = data

        def figure_eight(self):
            self.cmd_vel.linear.x = 1.0
            turn_speed = 0.5
            turning_point_y = 6

            if self.pose.y > turning_point_y:
                self.cmd_vel.angular.z = turn_speed
            elif self.pose.y <= turning_point_y:
                self.cmd_vel.angular.z = -turn_speed 
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