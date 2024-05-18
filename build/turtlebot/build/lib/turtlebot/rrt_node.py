import rclpy #type:ignore
from rclpy.node import Node #type:ignore
from nav_msgs.msg import OccupancyGrid #type:ignore
from std_msgs.msg import Float64MultiArray #type:ignore
from rclpy.qos import QoSProfile, QoSDurabilityPolicy #type:ignore
import numpy as np
import matplotlib.pyplot as plt
from turtlebot.rrt import find_path_RRT, map_img
import cv2

class rrt_node(Node):
    def __init__(self):
        print('############################## Initializing RRT Node ##################################')
        super().__init__('rrt_node')
        qos_profile = self.get_qos_profile()
        #create a subscription to /map and update the global variable current_map
        self.current_map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        #create subscription to /start_goal to get start x,y and goal x,y
       # self.start_goal_sub = self.create_subscription(Float64MultiArray, '/start_goal', self.start_goal_update, 10)
        #create publisher to /trajectory
        self.trajectory_publisher = self.create_publisher(Float64MultiArray, '/trajectory', 10)
        
        

        #Class Variables
        self.current_map = np.empty((0, 0))
        self.start = [0.1, 0.1] #start point x,y
        self.goal = [1, 1] #goal point x,y
        self.origin = [-100, -100] #default value for origin
        self.resolution = -1.0

    def get_qos_profile(self):
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return qos
        

    def map_callback(self, msg):
        #read the /map 1D data and change it into a 2D array, get sizes from msg.info.height and width
        #taken from the map_reader_template.py
        print("Received map message")
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        print("Width:", width)
        print("Height:", height)
        print("Resolution:", self.resolution)
        self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        print(f"Origin: {self.origin[0]} , {self.origin[1]}")
        
        # Convert 1D occupancy grid data to 2D numpy array
        data = msg.data
        self.current_map = np.array(data).reshape((height, width))
        self.compute_trajectory(self.start, self.goal)
        

        
        
    def plot_map(self, data):
        # Plot the occupancy grid
        plt.imshow(data, cmap='binary', origin='lower')
        plt.colorbar()
        plt.title('Occupancy Grid')
        plt.xlabel('X (cells)')
        plt.ylabel('Y (cells)')
        plt.show()

    def plot_path(self, path):
        # Plot the trajectory on the map
        plt.imshow(self.current_map, cmap='binary', origin='lower')
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], 'r')
        plt.scatter(path[:, 0], path[:, 1], c='r', s=10)
        plt.title('Computed Path')
        plt.xlabel('X (cells)')
        plt.ylabel('Y (cells)')
        plt.show()


    def start_goal_update(self, msg):
        print('############################## Updating Start and Goal positions ##################################')
        #read the data and get the Float64Array data into xy, xy
        self.start = [msg.data[0], msg.data[1]]
        self.goal = [msg.data[2], msg.data[3]]
        #call compute trajectory get the list and publish it to /trajectory
        self.compute_trajectory(self.start, self.goal)


    def compute_trajectory(self, start, goal):
        print('############################## Computing Trajectory, Please wait ##################################')
        if self.current_map.size == 0:
            self.get_logger().error("Map data is not yet received.")
            return
        #functino to convert coordinates to index
        start_index = self.coordinates_to_index(start)
        goal_index = self.coordinates_to_index(goal)
        # Ensure start and goal indexes are within the map boundaries
        if not self.is_within_bounds(start_index):
            self.get_logger().error("Start index out of bounds.")
            return
        if not self.is_within_bounds(goal_index):
            self.get_logger().error("Goal index out of bounds.")
            return
        #pass the index to find_path_rrt and get the list of points
        path , graph = find_path_RRT (start_index ,goal_index , cv2.cvtColor(map_img (self.current_map), cv2.COLOR_GRAY2BGR)[:: -1])
        #publish computed trajectory 
        print('############################## Computation Complete ##################################')
        trajectory_msg = Float64MultiArray()
        trajectory_msg.data = np.array(path).flatten()
        self.trajectory_publisher.publish(trajectory_msg)   
        

    def coordinates_to_index(self, coordinate):
        print('############################## Converting Coordinates to Indices ##################################')
        #convert the goal coordiantes in the map to the points in the grid i.e. the index of the map 2d array
        x_index = int((coordinate[0] - self.origin[0]) / self.resolution)
        y_index = int((coordinate[1] - self.origin[1]) / self.resolution)
        index = [x_index, y_index]
        print('indices: ' ,index[0], index[1])
        return index
    
    
    def is_within_bounds(self, index):
         # Check if the given index is within the bounds of the map
        return True




def main(args = None):
    print('############################## Turtlebot/RRT Node 0.4.0 ##################################')
    rclpy.init(args = args)
    rrt_object = rrt_node()
    try:
        rclpy.spin(rrt_object)     
    except Exception as e:
        rrt_object.get_logger().error('Error in RRT Node: %s' % str(e))
    finally:    
        rrt_object.destroy_node()
        rclpy.shutdown()   

if __name__ == '__main__':
    main()
