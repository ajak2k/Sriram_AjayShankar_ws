import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import curses

#define our publisher class
class my_publisher(Node):
    #construstor to set up the publishing method or mention the topic to publish to
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        #launch the funciton to control the bot
        self.controller()
        
    def controller(self):
        #initialize curses 
        stdscr = curses.initscr()
        curses.def_shell_mode()
        curses.noecho()
        curses.cbreak()
        stdscr.nodelay(True)
        
        twist = Twist()

        while True:
            #Prompt the user
            stdscr.addstr(0,0,"Please use the WASD keys to control the bot or Q to quit the program", curses.A_BOLD)
            #need to use try as .getkey() returns an exception when no input is given
            try:           
                key = stdscr.getkey()
                if key =='w' or key =='W':
                    twist.linear.z = 1.0
                    twist.angular.z= 0.0
                elif key == 'a' or key =='A':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                elif key == 's' or key =='S':
                    twist.linear.x = -1.0
                    twist.angular.z = 0.0
                elif key == 'd' or key == 'D':
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5
                elif key == 'q' or key == 'Q':
                    break
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
            except curses.error:
                # no input
                twist.linear.x = 0.0
                twist.angular.z = 0.0      
            self.publisher_.publish(twist)    
            stdscr.refresh()
        #End of while loop
        #terminate curses
        curses.nocbreak()
        curses.echo()
        curses.reset_shell_mode()
        curses.endwin()
    #end of function
#end of class           

#Main Function

def main():
    #start the ros2 client 
    rclpy.init()
    #create the object of the publisher
    pub = my_publisher()
    #spin it to keep it running
    rclpy.spin(pub)
    #shut it down
    pub.destroy_node()
    rclpy.shutdown()
    print('Thanks for Playing!')
#Starting point into the code
if __name__ == '__main__':
    main()
