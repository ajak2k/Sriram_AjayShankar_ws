import sys
import rclpy #type: ignore
from rclpy.node import Node #type:ignore
from std_msgs.msg import Float64MultiArray #type: ignore
from nav_msgs.msg import Odometry # type: ignore
from turtlesim.msg import Pose as Pose_simple# type: ignore
from turtlebot.pid_controller import pid_controller

class motion_planner(Node):
    #init for the class
    def __init__(self):
        print('############################## Initializing Motion Planner ##################################')
        super().__init__('motion_planner')
        #create pub to publish the input obtained from the user
        self.reference_pose_publisher = self.create_publisher(Float64MultiArray, 'reference_pose', 10)
        #create sub to get the current pose from odom
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.pose_update, 10)

        #Class variables
        self.current_pose = Pose_simple()
        self.goal = Pose_simple()
        self.pid_mode = -1

        self.distance_threshold = 0.01
        self.angle_threshold = 0.1

        #get the first set of inputs from the user
        self.get_input()

        #from now, check if the goal is reached and only then get inputs
        self.timer = self.create_timer(1.0, self.timer_callback)

       
#create a callback function for odom subscriber
    #this should check if the goal is reached
        #if yes, then call the user prompt
        #if no, print 'waiting for bot to reach the goal' and continue
    def pose_update(self, odom):
        #print('############################## Pose Update ##################################')
        "' Here we downsample the current locations to just the first 3 decimal points '"
        self.current_pose.x = float(int(odom.pose.pose.position.x*1000)/1000)
        self.current_pose.y = float(int(odom.pose.pose.position.y*1000)/1000)
        self.current_pose.theta = float(int(pid_controller.quaternion_to_euler(odom.pose.pose.orientation)*1000)/1000)

#create a function to read the input, parse it into the float array and publish it
    def get_input(self):
        print('############################## User Input ##################################')
        msg = Float64MultiArray()

        try:
            #expect an input of two numbers seperated by space
            input_str = input('Please enter q to quit or the destination [x y angle pid_mode]: ')
            #if q then quit/ close the programme
            if input_str == 'q':
                sys.exit() 
            else:
                input_str = input_str.split()
                self.goal.x = float(input_str[0])
                self.goal.y = float(input_str[1])
                self.goal.theta = float(input_str[2])
                self.pid_mode = float(input_str[3])
                #ensuring we have the right inputs
                if self.pid_mode not in [0.0,1.0]:
                    print('Please enter a valid pid_mode value of 0 or 1')
                    self.get_input()

        #No input or wrong input given
        except ValueError:
            print("Please enter the input in the [x y angle pid_mode] format only")
            self.get_input()
        #Only one input given
        except IndexError:
            print("Please enter all the parameters [x y angle pid_mode] ")
            self.get_input()
        
        msg.data = [self.goal.x, self.goal.y, self.goal.theta, self.pid_mode]
        self.reference_pose_publisher.publish(msg)

#a funciton to check if the bot has reached it's destination, if yes it will get new input
    def timer_callback(self):
        #print('############################## Timer Callback ##################################')
        if self.status_check():
            self.get_input()
        else:
            print('The Turtle is turtling, kindly be patient they are a bit old :)')

#now we need a function that checks if it is okay to get the next set of inputs from the user based on the bot status
    def status_check(self):
        #print('############################## Status Check ##################################')
        distance_error = pid_controller.euclidean_distance_error(self.current_pose, self.goal)
        angle_error = pid_controller.angle_error(self.current_pose, self.goal)

        if abs(angle_error) < self.angle_threshold and abs(distance_error)< self.distance_threshold:
            print('Goal reached!')
            return True
        else:
            return False

#main
#create the object, initialize it, spin it destroy it
def main(args=None):
    print('############################## Motion Planner 0.3.0 ##################################')
    rclpy.init(args=args)
    motion_planner_object = motion_planner()
    try:
        rclpy.spin(motion_planner_object)
    except Exception as e:
        motion_planner_object.get_logger().error('Error in Motion Planner: %s' % str(e))
    finally:
        motion_planner_object.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
