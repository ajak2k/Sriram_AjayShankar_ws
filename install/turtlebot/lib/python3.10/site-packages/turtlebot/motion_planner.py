import sys
import rclpy #type: ignore
from rclpy.node import Node #type:ignore
from std_msgs.msg import Float64MultiArray #type: ignore
import matplotlib as plt
import numpy as np

class motion_planner(Node):
    #init for the class
    def __init__(self):
        print('############################## Initializing Motion Planner ##################################')
        super().__init__('motion_planner')
        #create pub to publish the input obtained from the user
        self.start_goal_publisher = self.create_publisher(Float64MultiArray, 'start_goal', 10)
        #create sub to get the current pose from odom
        self.trajectory_subscriber = self.create_subscription(Float64MultiArray, 'trajectory', self.update_trajectory, 10)
        #get the input for the first time, the next times will be done when the trajectory is received
        self.get_input()

    #the callback function to parse the received trajectory, print it and call the next set of inputs
    def update_trajectory(self, msg):
        self.plot_trajectory(msg.info)
    
    def plot_trajectory(self, path):
        plt.imshow(self.current_map, cmap='gray')
        path = np.array(path)
        plt.plot(path[:, 1], path[:, 0], 'r')
        plt.show()

#create a function to read the input, parse it into the float array and publish it
    def get_input(self):
        print('############################## User Input ##################################')
        msg = Float64MultiArray()

        try:
            #expect an input of two numbers seperated by space
            input_str = input('Please enter q to quit  or the start point and the goal point [x y x y]: ')
            #if q then quit/ close the programme
            if input_str == 'q':
                sys.exit() 
            else:
                input_str = input_str.split()
                self.start = [float(input_str[0]), float(input_str[1])] 
                self.goal = [float(input_str[2]), float(input_str[3])]
        #No input or wrong input given
        except ValueError:
            print("Please enter the input in the [x y x y] format only")
            self.get_input()
        #Only one input given
        except IndexError:
            print("Please enter all the parameters [x y x y] ")
            self.get_input()
        
        msg.data = [self.start[0], self.start[1], self.goal[0], self.goal[1]]
        self.start_goal_publisher.publish(msg)


#main
#create the object, initialize it, spin it destroy it
def main(args=None):
    print('############################## Turtlebot/Motion Planner 0.4.0 ##################################')
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