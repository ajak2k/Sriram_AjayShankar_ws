import rclpy #type:ignore
from rclpy.node import Node #type:ignore
from nav_msgs.msg import OccupancyGrid #type:ignore
from rclpy.qos import QoSProfile, QoSDurabilityPolicy #type:ignore
import numpy as np
import matplotlib.pyplot as plt


class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        qos_profile = self.get_qos_profile()
        self.subscription = self.create_subscription(OccupancyGrid,'/map',self.map_callback, qos_profile)

    def get_qos_profile(self):
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return qos
        
    def map_callback(self, msg):
        print("Received map message")
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        print("Width:", width)
        print("Height:", height)
        print("Resolution:", resolution)
        data = msg.data

        # Convert 1D occupancy grid data to 2D numpy array
        current_map = np.array(data).reshape((height, width))

        # Plot the occupancy grid
        plt.imshow(current_map, cmap='binary', origin='lower')
        plt.colorbar()
        plt.title('Occupancy Grid')
        plt.xlabel('X (cells)')
        plt.ylabel('Y (cells)')
        plt.show()

def main():
    rclpy.init()
    map_subscriber = MapSubscriber()
    rclpy.spin(map_subscriber)
    map_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
