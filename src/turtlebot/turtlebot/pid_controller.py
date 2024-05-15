import rclpy #type: ignore
from rclpy.node import Node #type:ignore
from std_msgs.msg import Float64MultiArray #type: ignore
from nav_msgs.msg import Odometry # type: ignore
from turtlesim.msg import Pose as Pose_simple# type: ignore
from geometry_msgs.msg import Twist#type:ignore

from math import atan2, pi, sqrt, sin, cos

class pid_controller(Node):
    def __init__(self):
        print('############################## Initializing PID Controller ##################################')
        super().__init__('pid_controller')
        #create publisher for command velocity
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        #create subscriptions to odometry and motion planner
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.pose_update, 10)
        self.reference_pose_subscriber = self.create_subscription(Float64MultiArray, 'reference_pose', self.get_ref_pose, 10)

        
        #Class variables
        self.pose = Pose_simple()
        self.goal = Pose_simple()
        self.pid_mode = -1

        
        #test info
        self.goal.x = 3.0
        self.goal.y = -4.0
        self.goal.theta = 2.0
        self.pid_mode = 1
        
        #PID parameters
        self.distance_threshold = 0.1
        self.angle_threshold = 0.1
        self.orientation_threshold = 0.5

        #this is a flag to identify if the goal x,y is reached or not
        #if the flag is set, the x,y is reched and the orientation controller will be triggered
        #flag is reset once orientation goal is also reched and the pid mode is reset to indicate end
        self.flag = 0
        #added line for git examples

        #error memory
        #linear
        self.linear_previous_error = 0.0
        self.linear_error_accumulator = 0.0
        #angle
        self.angular_previous_error = 0.0
        self.angular_error_accumulator = 0.0
        # orientation
        self.orientation_previous_error = 0.0
        self.orientation_error_accumulator = 0.0
        
        #PID mode 0 gains
        # PID gains for linear velocity
        "' Trial to tune with Z-N method: Ku identified as 1.1, Tu as ~6seconds -> Kp = 0.6Ku; Ki=1.2Ku/Tu; Kd = 0.075KuTu '"
        self.Kp_gain_linear = 1.0
        self.Ki_gain_linear = 0.5
        self.Kd_gain_linear = 0.2
        # PID gains for angular velocity
        self.Kp_gain_angular = 0.5
        self.Ki_gain_angular = 0.1 
        self.Kd_gain_angular = 0.1        
        #PID mode 1 gains
        # PID gains for linear velocity
        self.Kp_gain_linear_1 = 1.8
        self.Ki_gain_linear_1 = 0.6
        self.Kd_gain_linear_1 = 0.0
        # PID gains for angular velocity
        self.Kp_gain_angular_1 = 1.5
        self.Ki_gain_angular_1 = 0.7
        self.Kd_gain_angular_1 = 0.0
        

        # maximum speeds
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0 # rad/s


        #create timer to enact control actions
        self.timer = self.create_timer(1.0, self.timer_callback)

    def pose_update(self, odom):
        #print('############################## Pose Update ##################################')
        "' Here we downsample the current locations to just the first n decimal points '"
        self.pose.x = float(int(odom.pose.pose.position.x*100000)/100000)
        self.pose.y = float(int(odom.pose.pose.position.y*100000)/100000)
        self.pose.theta = float(int(self.quaternion_to_euler(odom.pose.pose.orientation)*100000)/100000)

    def get_ref_pose(self, msg):
        #print('############################## Reference Pose Update ##################################')
        self.goal.x = msg.data[0]
        self.goal.y = msg.data[1]
        self.goal.theta = msg.data[2]
        self.pid_mode = msg.data[3]

    def timer_callback(self):
        #print('############################## Timer Callback ##################################')
        self.controller(self.pose, self.goal, self.pid_mode)


    #funciton to convert the quaternion orientation and extract just the yaw in euler angles
    @staticmethod
    def quaternion_to_euler(q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        euler_angle = atan2(siny_cosp, cosy_cosp)
        return euler_angle
    
    @staticmethod
    def euclidean_distance_error(current_pose, goal_pose):
        """Euclidean distance between current pose and the goal."""
        try:
            return sqrt(pow((goal_pose.x - current_pose.x), 2) + pow((goal_pose.y - current_pose.y), 2))
        except TypeError as e:
            print('Error calculating distance error: ', e)
            return
        
    @staticmethod
    def angle_error(current_pose, goal_pose):
        """Calculate the angle at which the goal is there w.r.t 0,0 and find the normalized error wrt our pose"""
        try:
            angle_to_goal = atan2(goal_pose.y - current_pose.y , goal_pose.x - current_pose.x)
            angle_error = atan2(sin(angle_to_goal-current_pose.theta), cos(angle_to_goal-current_pose.theta))
            return  angle_error
        except TypeError as e:
            print('Error calculating angle error: ',e)
            return
        
    @staticmethod
    def orientation_error(current_pose, goal_pose):
        """Calculate the normalized error between the desired anglular orientation and the current orientation"""
        try:
            angle_error = atan2(sin(goal_pose.theta-current_pose.theta), cos(goal_pose.theta-current_pose.theta))
            return  angle_error
        except TypeError as e:
            print('Error calculating orientaiton error: ',e)
            return
  
    def controller(self, current_pose, goal_pose, mode):
        #print('############################## Controller Called ##################################')
        cmd_vel = Twist()
        if mode == 0:
            print('############################## PID Mode 0 Operation ##################################')
            if(self.flag == 0):
                if abs(self.angle_error(current_pose, goal_pose)) > self.angle_threshold :
                    print('Solving the Angle Problem')
                    cmd_vel.angular.z = self.P_angular(current_pose, goal_pose, mode) + self.I_angular(current_pose, goal_pose, mode) + self.D_angular(current_pose, goal_pose, mode)
                    cmd_vel.linear.x = 0.0
                    print("current angle: ", current_pose.theta, " goal angle: ",goal_pose.theta," angle error: ",self.angle_error(current_pose, goal_pose))
                else:
                    #angle goal acheived, reset the error memory and check for the linear goal
                    self.angular_previous_error = 0.0
                    self.angular_error_accumulator = 0.0
                    
                    if self.euclidean_distance_error(current_pose, goal_pose) > self.distance_threshold:
                        print('Solving the Linear Problem')
                        cmd_vel.angular.z = 0.0
                        cmd_vel.linear.x = self.P_linear(current_pose, goal_pose, mode) + self.I_linear(current_pose, goal_pose, mode) + self.D_linear(current_pose, goal_pose, mode)
                        print("current x: ", current_pose.x , " goal x: ",goal_pose.x)
                        print("current y: ", current_pose.y , " goal y: ",goal_pose.y)
                        print('distance error: ', self.euclidean_distance_error(current_pose, goal_pose))
                    else:
                        #linear goal acheived, reset the error memory and check for the orientation goal
                        self.linear_previous_error = 0.0
                        self.linear_error_accumulator = 0.0
                        self.flag = 1
            if self.flag == 1:        
                    print('Goal locaiton reached; Correcting angle to the desired input now')
                    if abs(self.orientation_error(current_pose, goal_pose)) >self.orientation_threshold:
                        print('solving the Orientation Problem')
                        cmd_vel.angular.z = self.P_orientaiton(current_pose, goal_pose) + self.I_orientation(current_pose, goal_pose) + self.D_orientation(current_pose, goal_pose)
                        cmd_vel.linear.x = 0.0
                    else:
                        #orientation goal acheived, rest the error memory and wait for new goal 
                        self.orientation_error_accumulator = 0
                        self.orientation_previous_error = 0
                        print('Goal Acheived! Waiting for motion planner instructions')
                        cmd_vel.angular.z = 0.0
                        cmd_vel.linear.x = 0.0
                        #reset pid mode to wait for new goal from motion planner
                        self.pid_mode = -1   
                        self.flag = 0  
        elif mode == 1:
            print('############################## PID Mode 1 Operation ##################################')
            if self.flag == 0:
                if abs(self.angle_error(current_pose, goal_pose)) > self.angle_threshold or self.euclidean_distance_error(current_pose, goal_pose) > self.distance_threshold:
                    print('solving the Angular and Linear Problems')
                    cmd_vel.angular.z = self.P_angular(current_pose, goal_pose, mode) + self.I_angular(current_pose, goal_pose, mode) + self.D_angular(current_pose, goal_pose, mode)
                    cmd_vel.linear.x = self.P_linear(current_pose, goal_pose, mode) + self.I_linear(current_pose, goal_pose, mode) + self.D_linear(current_pose, goal_pose, mode) 
                else:
                    #reset error histories
                    self.linear_previous_error = 0.0
                    self.angular_previous_error = 0.0
                    self.angular_error_accumulator = 0.0
                    self.linear_error_accumulator = 0.0 
                    self.flag = 1
            if self.flag == 1:    
                print('Goal location reached; Correcting orientaion to the desired input now')
                if abs(self.orientation_error(current_pose, goal_pose)) >self.angle_threshold:
                    print('solving the Orientation Problem')
                    cmd_vel.angular.z = self.P_orientaiton(current_pose, goal_pose) + self.I_orientation(current_pose, goal_pose) + self.D_orientation(current_pose, goal_pose)
                    cmd_vel.linear.x = 0.0
                else:
                    print('Goal Reached!, waiting for new goal')
                    cmd_vel.angular.z = 0.0
                    cmd_vel.linear.x = 0.0  
                    self.orientation_error_accumulator = 0
                    self.orientation_previous_error = 0
                    self.flag ==0
                    self.pid_mode = -1
        else:
            print('No Valid Operation Mode, will wait for input')    
            cmd_vel.angular.z = 0.0
            cmd_vel.linear.x = 0.0
        #limit the max velocity of the bot to prevent run away
        cmd_vel.linear.x = max(min(cmd_vel.linear.x, self.max_linear_speed), -self.max_linear_speed)
        cmd_vel.angular.z = max(min(cmd_vel.angular.z, self.max_angular_speed), -self.max_angular_speed)
        self.cmd_vel_publisher.publish(cmd_vel)   

    ######################################## Angular Controller ##############################################
    def P_angular(self, current_pose, goal_pose, mode):
        if mode == 0:
            action = self.Kp_gain_angular * self.angle_error(current_pose, goal_pose)
        if mode == 1:
            action = self.Kp_gain_angular_1 * self.angle_error(current_pose, goal_pose)    
        return action
    def I_angular(self, current_pose, goal_pose, mode):
        self.angular_error_accumulator += self.angle_error(current_pose, goal_pose)
        if mode == 0:
            action = self.Ki_gain_angular * self.angular_error_accumulator
        if mode == 1:
            action = self.Ki_gain_angular_1 * self.angular_error_accumulator
        return action
    def D_angular(self, current_pose, goal_pose, mode):    
        if mode == 0:
            action = self.Kd_gain_angular * (self.angle_error(current_pose, goal_pose) - self.angular_previous_error)
        if mode == 1:
            action = self.Kd_gain_angular_1 * (self.angle_error(current_pose, goal_pose) - self.angular_previous_error)    
        self.angular_previous_error = self.angle_error(current_pose, goal_pose)
        return action
    
    ######################################## Orientaiton Controller ##########################################
    def P_orientaiton(self, current_pose, goal_pose):
        action = self.Kp_gain_angular * self.orientation_error(current_pose, goal_pose)
        return action
    def I_orientation(self, current_pose, goal_pose):
        self.orientation_error_accumulator += self.orientation_error(current_pose, goal_pose)
        action = self.Ki_gain_angular * self.orientation_error_accumulator
        return action
    def D_orientation(self, current_pose, goal_pose):    
        action = self.Kd_gain_angular * (self.orientation_error(current_pose, goal_pose) - self.orientation_previous_error)
        self.orientation_previous_error = self.orientation_error(current_pose, goal_pose)
        return action
    
    ######################################## Linear Controller ###############################################
    def P_linear(self, current_pose, goal_pose, mode):
        if mode == 0:
            action = self.Kp_gain_linear * self.euclidean_distance_error(current_pose, goal_pose)
        if mode ==1:
            action = self.Kp_gain_linear_1 * self.euclidean_distance_error(current_pose, goal_pose)
        return action
    def I_linear(self, current_pose, goal_pose, mode):
        self.linear_error_accumulator += self.euclidean_distance_error(current_pose, goal_pose)
        if mode ==0:
            action = self.Ki_gain_linear * self.linear_error_accumulator
        if mode == 1:
            action = self.Ki_gain_linear_1 * self.linear_error_accumulator
        return action
    def D_linear(self, current_pose, goal_pose, mode):
        if mode ==0:
            action = self.Kd_gain_linear * (self.euclidean_distance_error(current_pose, goal_pose) - self.linear_previous_error)
        if mode ==1:
            action = self.Kd_gain_linear_1 * (self.euclidean_distance_error(current_pose, goal_pose) - self.linear_previous_error)
        self.linear_previous_error = self.euclidean_distance_error(current_pose, goal_pose)
        return action
    

def main():
    print('############################## TurtleBot/PID Controller 0.3.0 ##################################')
    rclpy.init()
    pid_controller_object = pid_controller()
    try:
        rclpy.spin(pid_controller_object)
    except Exception as e:
        pid_controller_object.get_logger().error('Error in PID Controller: %s' % str(e))
    finally:
        pid_controller_object.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()