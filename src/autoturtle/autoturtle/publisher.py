import rclpy # type: ignore
from rclpy.node import Node # type: ignore


#define our publisher class
class my_publisher(Node):
    def __init__(self):
        super.__init__('my_publisher')
         
        

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


#Starting point into the code
if __name__ == '__main__':
    main()
