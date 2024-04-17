import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Twist # type: ignore
import sys
import termios
import tty

class my_teleop(Node):
    def __init__(self):
        super().__init__('my_teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.run_teleop()

    def get_key(self):
        # Set up terminal settings for raw input
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(sys.stdin.fileno())
        try:
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run_teleop(self):
        # Display control instructions
        print("Control the turtle using keyboard:")
        print("Use 'W' for forward, 'S' for backward, 'A' for left, 'D' for right")
        print("Press 'Q' to quit")

        twist = Twist()

        while True:
            key = self.get_key()
            
            if key == 'q':
                print("Exiting teleop...")
                break
            elif key == 'w':
                twist.linear.x = 0.5
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -0.5
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            elif key == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -0.5
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            # Publish the Twist message
            self.publisher_.publish(twist)

        # Stop the turtle when quitting
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main():
    # Initialize rclpy
    rclpy.init()
    
    # Create the teleop node
    teleop_node = my_teleop()

    # Keep the node running
    rclpy.spin(teleop_node)

    # Shutdown rclpy
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
