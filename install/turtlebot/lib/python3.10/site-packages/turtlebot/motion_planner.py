import rclpy #type: ignore
from rclpy.node import Node #type:ignore
from std_msgs.msg import Float64MultiArray #type: ignore
from nav_msgs.msg import Odometry #type: ignore
from turtlesim.msg import Pose as Pose_simple #type: ignore
from turtlebot.pid_controller import pid_controller
import time


class motion_planner(Node):
    
    def __init__(self):
        super().__init__('motion_planner')
        self.get_logger().info('################### Initializing Motion Planner ###################')

        #Subscribers
        #trajectory -> list of points to follow from rrt_node
        #odom -> current pose of the robot from gazebo
        #target_pose -> target pose to reach from the user
        self.create_subscription(Float64MultiArray, '/trajectory', self.update_trajectory, 10)
        self.create_subscription(Odometry, '/odom', self.current_pose_update, 10)
        self.create_subscription(Float64MultiArray, '/target_pose', self.target_pose_callback, 10)

        #Publishers
        #start_goal -> start(current point) and goal(from target_pose) points for the planner
        #reference_pose -> reference/set point pose for the pid controller (each point in the trajectory)
        self.start_goal_publisher = self.create_publisher(Float64MultiArray, 'start_goal', 10)
        self.reference_pose_publisher = self.create_publisher(Float64MultiArray, 'reference_pose', 10)

        #Class variables
        self.current_pose:Pose_simple = Pose_simple()
        self.target_pose:Pose_simple = Pose_simple()
        self.trajectory:list = []

        #status flags
        self.target_pose_received:bool = False
        self.current_pose_received:bool = False
        self.trajectory_received:bool = False
        self.start_goal_published:bool = False
        self.PID_running:bool = False

        #PID parameters
        self.distance_threshold = 0.1
        self.angle_threshold = 0.1

        print('Please publish the target_pose...')

        self.timer = self.create_timer(0.1, self.timer_callback)

    #subscripion callback functions
    def target_pose_callback(self, msg):
        #get the target pose from the user and store it locally and then call the control functions
        self.target_pose.x = msg.data[0]
        self.target_pose.y = msg.data[1]
        self.target_pose.theta = 1.0 #set to 1 to indicate the message is received
        self.target_pose_received = True

    def current_pose_update(self, msg):
        #get the current pose from gazebo and store it locally
        #print('############################## Pose Update ##################################') '"
        self.current_pose.x = float(int(msg.pose.pose.position.x*100000)/100000)
        self.current_pose.y = float(int(msg.pose.pose.position.y*100000)/100000)
        self.current_pose.theta = float(int(pid_controller.quaternion_to_euler(msg.pose.pose.orientation)*100000)/100000)
        self.current_pose_received = True
        #print(f'Current Pose: {self.current_pose.x}, {self.current_pose.y}, {self.current_pose.theta}')

    def update_trajectory(self, msg):
        #get the trajectory from the rrt_node and store it locally
        #print('############################## Trajectory Update ##################################')
        self.trajectory = msg.data
        
        if self.trajectory == []:
            #self.get_logger().info('No Trajectory received, please enter new target_pose...')
            self.trajectory_received = False
        else:
            self.trajectory_received = True
    
    #a timer callback function to check if the current pose and the target pose information is available and call the control function
    def timer_callback(self):
        if self.target_pose_received and self.current_pose_received:
            self.control()
        else:
            self.get_logger().info('Waiting for the target and current pose...')
    
    
    #a function to check if the target_pose and current_pose is available and are different
        #if yes, then publish the start_goal to rrt for the trajectory
         #then loop throught the trajectory and publish each point as reference_pose to pid controller
            #wait for the bot to reach the point to a threshold before publishing the next reference for the controller
         #once the trajectory is done, reset all points and wait for the next target_pose
        #else, wait for the target_pose and current_pose to be available 
    def control(self):
        self.get_logger().info('############################## Control ##################################')
        start_goal_msg = Float64MultiArray()
        reference_pose_msg = Float64MultiArray()

        if self.target_pose_received and self.current_pose_received is False: #check if the target pose is received
            self.get_logger().info('Waiting for the target and current pose...')
            return

        if self.start_goal_published == False:
            #publish the start and goal points to the rrt node if not already done and wait for response
            start_goal_msg.data = [self.current_pose.x, self.current_pose.y, self.target_pose.x, self.target_pose.y]
            self.start_goal_publisher.publish(start_goal_msg)
            self.get_logger().info('Start and Goal points published to RRT node...')
            self.start_goal_published = True
            return

        if self.trajectory_received == False:
            self.get_logger().info('Waiting for the trajectory...')
            return 
        
        self.get_logger().info('Trajectory received...')

        if self.PID_running == False:
            self.PID_running = True
            self.get_logger().info('Starting PID controller...')
            #keep program control here till we go through all the points in the trajectory i.e., the bot reaches the target            
            i:int = 2 #set i to 2 to skip the first two points in the trajectory as that is the start point
            print(len(self.trajectory))
            print(self.trajectory)
            while i< len(self.trajectory):
                #publish the reference pose to the PID controller
                self.get_logger().info(f'Publishing reference pose {i}...')
                reference_pose_x = self.trajectory[i]
                reference_pose_y = self.trajectory[i+1]
                reference_pose_msg.data = [reference_pose_x, reference_pose_y, 0.0, 0.0] #set the theta to 0 as we are not using it and set PID mode to 0 by default
                self.reference_pose_publisher.publish(reference_pose_msg) #send the first reference pose to the PID controller
                #wait for the bot to reach the reference point
                while self.goal_reached_status() == False:
                    #self.get_logger().info('Turtle is Turtling, Please be patient...')
                    time.sleep(0.1)
                    continue
                self.get_logger().info('Turtle reached the reference point...')
                i+=2

            self.get_logger().info('Trajectory completed...')
            #reset all flags to wait for the next target_pose        
            self.PID_running = False
            self.start_goal_published = False
            self.target_pose_received = False
            self.current_pose_received = False
            self.trajectory_received = False
            self.get_logger().info('############################## Control Complete ##################################')
            return

        self.get_logger().error('Error in control logic...')    
        return
                

    
    
    #helper functions to deal with the control logics

    #identify if the pid controller is close to the target
    def goal_reached_status(self):
        #print('############################## Status Check ##################################')
        distance_error = pid_controller.euclidean_distance_error(self.current_pose, self.target_pose)
        angle_error = pid_controller.angle_error(self.current_pose, self.target_pose)

        if abs(distance_error) > self.distance_threshold and abs(angle_error) > self.angle_threshold:
            return False
        else:
            print('Goal reached!')
            return True
                
        

def main():
    print('################### Turtlebot/Motion Planner 0.4.0 ###################')
    rclpy.init(args=None)
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