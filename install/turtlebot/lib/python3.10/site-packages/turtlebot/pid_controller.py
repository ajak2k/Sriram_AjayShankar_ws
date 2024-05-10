import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from math import sqrt, atan2, sin, cos
from tf.transformations import euler_from_quaternion

class pid_controller(Node):
    def __init__(self):
        #create a node with the name 'pid_controller'
        super().__init__("pid_controller")
        
        #create a publisher to publish to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #subscribe to /odom for bot info, /reference_pose for goal/destinatin info and pid mode
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.ref_pose_sub = self.create_subscription(Float64MultiArray, '/reference_pose', self.ref_pose_callback, 10)
        
        #timer to keep calling the go to goal function and have the turtle controlled every 0.1 seconds
        self.timer = self.create_timer(0.1, self.get_to_goal)

        ############### AGENT'S POSE in the gazebo environment #######################
        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0

        ############### AGENT'S Goal from motion planner #############################
        self.goal = None

        ###############Controller mode for the PIDs ##################################
        self.mode = None

        ############### Time discretization for the PID  #############################
        self.time_discr = 0.001

        ############### PID parameters for linear velocity ############################
        self.error_prior_linear = 0
        self.integral_prior_linear = 0
        self.Kp_gain_linear = 1
        self.Ki_gain_linear = 0.1
        self.Kd_gain_linear = 0

        ############### PID parameters for angular velocity ############################
        self.error_prior_angular = 0
        self.integral_prior_angular = 0
        self.Kp_gain_angular = 1
        self.Ki_gain_angular = 0.1
        self.Kd_gain_angular = 0

    #update the pose of the bot in gazebo
    def pose_callback(self, data):
        self.pose_x = data.pose.position.x
        self.pose_y = data.pose.position.y
        self.pose_thetha = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)[2]

        
    #update the reference/goal of the bot from the motion planner
    def ref_pose_callback(self, data):
        self.goal_pose_x = data[0]
        self.goal_pose_y = data[1]
        self.goal_pose_theta = data [2]
        self.mode = data[3]
        self.goal = [self.goal_pose_x, self.goal_pose_y, self.goal_pose_theta]

    #calculate the error in dist and in angle
    def euclidean_distance_error(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - self.pose_x), 2) + pow((goal_pose[1] - self.pose_y), 2))
    
    def angle_error(self, goal_pose):
        """Calculate the angle at which the goal is there w.r.t 0,0 and find the error wrt our pose"""
        angle_to_goal = atan2(goal_pose[1] - self.pose_y , goal_pose[0] - self.pose_x)
        return atan2(sin(angle_to_goal-self.pose_theta), cos(angle_to_goal-self.pose_theta)) 
          
    #get the controller outputs linear, angular, P,I,D      

    #linear controller
    def proportional_linear_vel(self, goal_pose):
        return self.Kp_gain_linear * self.euclidean_distance_error(goal_pose)
    
    def integral_linear_vel(self, goal_pose):
        self.integral_prior_linear+= self.euclidean_distance_error(goal_pose)
        return self.Ki_gain_linear*self.integral_prior_linear
    
    def differential_linear_vel(self, goal_pose):
        action = self.Kd_gain_linear*(self.euclidean_distance_error(goal_pose)-self.error_prior_linear)
        self.error_prior_linear = self.euclidean_distance_error(goal_pose)
        return action

    #anglular controller
    def proportional_angular_vel(self, goal_pose):
        return self.Kp_gain_angular * self.angle_error(goal_pose)
    
    def intergal_anugular_vel(self, goal_pose):
        self.error_prior_angular+=self.angle_error(goal_pose)
        return self.Ki_gain_angular*self.integral_prior_angular
    
    def differential_angular_vel(self, goal_pose):
        action = self.Kd_gain_angular*(self.angle_error(goal_pose) - self.error_prior_angular)
        self.error_prior_angular = self.angle_error(goal_pose)
        return action

    #get the goal and loop till goal is reached depending on the mode
    def get_to_goal(self):
        command_vel = Twist()
        distance_tolerance = 0.1
        angle_tolerance = 0.01
        
        #set velocities to 0 is no goal is there
        if self.goal is None:
            command_vel.angular.z = 0
            command_vel.linear.x = 0

        #check the mode of the PID, do angular then linear if 0
        if self.mode == 0:
            #create command vel if the error>allowed value
            if self.angle_error(self.goal) > angle_tolerance:
                command_vel.angular.z = self.proportional_angular_vel(self.goal) + self.intergal_anugular_vel(self.goal) + self.differential_angular_vel(self.goal)
                command_vel.linear.x = 0
            else:
                #reset the error memory and goal once the desired angle is acheived
                self.error_prior_angular = 0
                self.integral_prior_angular = 0
                self.goal_pose_theta = 0
                if self.euclidean_distance_error(self.goal) > distance_tolerance:
                    command_vel.angular.z = 0
                    command_vel.linear.x = self.proportional_linear_vel(self.goal) + self.integral_linear_vel(self.goal) + self.differential_linear_vel(self.goal)
                    
                else:
                    print('Goal Reached!')                    
                   
                    #reset the linear error memory and goals once it is also reached
                    self.integral_prior_linear = 0
                    self.error_prior_linear = 0
                    self.goal_pose_x = None
                    self.goal_pose_y = None
        
        
        
        #if the mode is 1, try to launch both the controllers at the same time
        elif self.mode == 1:

            if self.angle_error(self.goal)>angle_tolerance:
                command_vel.angular.z = self.proportional_angular_vel(self.goal) + self.intergal_anugular_vel(self.goal) + self.differential_angular_vel(self.goal)
                
            else:
                #reset the error memory once the desired angle is acheived
                self.error_prior_angular = 0
                self.integral_prior_angular = 0
                self.goal_theta = None

            if self.euclidean_distance_error > distance_tolerance:
                command_vel.linear.x = self.proportional_linear_vel(self.goal) + self.integral_linear_vel(self.goal) + self.differential_linear_vel(self.goal)
                
            else:
                #reset the linear error memory once it is also reached
                self.integral_prior_linear = 0
                self.error_prior_linear = 0
                self.goal_x = None
                self.goal_y = None
        else:
            print("mode error! mode = %i"/self.mode)  

        if self.goal_x is None & self.goal_y is None % self.goal_theta is None:
            print('Reached Goal!')
            self.goal = None 

        self.cmd_vel_pub.publish(command_vel) 
        
def main():
    #initialize rclypy, create the object, ask it to go to the goal, close when done
    rclpy.init()
    pid = pid_controller()
    rclpy.spin(pid)
    pid.destroy_node()
    rclpy.shutdown()      

if __name__ == '__main__':
    main()