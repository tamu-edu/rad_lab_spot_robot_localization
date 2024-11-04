import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import random

class GPSHeadingPublisher(Node):
    def __init__(self):
        super().__init__('gps_heading_publisher')
        
        # Publishers for NavSatFix and Float64 messages
        self.gps_publisher = self.create_publisher(NavSatFix, '/polaris/gps', 10)
        self.heading_publisher = self.create_publisher(Float64, '/polaris/heading', 10)
        
        # Timer to publish messages at a regular interval
        self.timer = self.create_timer(1.0, self.publish_data)  # Publish every second

    def publish_data(self):
        # Create and populate NavSatFix message
        gps_msg = NavSatFix()
        gps_msg.latitude = 30.6405192
        gps_msg.longitude = -96.4872006
        gps_msg.altitude = 54.385
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.gps_publisher.publish(gps_msg)

        # Create and publish Float64 message for heading
        heading_msg = Float64()
        heading_msg.data = -1.589

        # Log and publish the heading data
        self.heading_publisher.publish(heading_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSHeadingPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

