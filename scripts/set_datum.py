#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_localization.srv import SetDatum
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64
from tf_transformations import quaternion_from_euler

class DatumServiceClient(Node):
    def __init__(self):
        super().__init__('datum_service_client')

        # Initialize member variables
        self.polaris_sensor_check_passed = False
        self.polaris_lat = 0.0
        self.polaris_long = 0.0
        self.polaris_heading = 0.0
        self.datum_set_once = False

        self.client = self.create_client(SetDatum, '/datum')
        self.polaris_sensor_check_passed_subscriber = self.create_subscription(Bool, '/polaris/sensor_check_passed', self.polaris_sensor_check_callback, 10)
        self.polaris_gps_subscriber = self.create_subscription(NavSatFix, '/polaris/gps', self.polaris_gps_callback, 10)
        self.polaris_heading_subscriber = self.create_subscription(Float64, '/polaris/heading', self.polaris_heading_callback, 10)
        self.status_publisher = self.create_publisher(Bool, '/dog/is_datum_node_up', 10)
        
        # Create a timer that calls timer_callback at 4 Hz (every 0.25 seconds)
        self.datum_timer = self.create_timer(0.25, self.datum_timer_callback)

    def send_request(self):
        # Call the service and register a callback for the response
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            self.get_logger().info('Service call succeeded')
        else:
            self.get_logger().error('Service call failed')
    
    def polaris_sensor_check_callback(self, msg):
        self.get_logger().info('inhere')
        if msg.data:
            self.get_logger().info('registering TRUE')
            self.polaris_sensor_check_passed = True
    
    def polaris_gps_callback(self,msg):
        self.polaris_lat = msg.latitude
        self.polaris_long = msg.longitude

    def polaris_heading_callback(self,msg):
        self.polaris_heading = msg.data
    
    def datum_timer_callback(self):
        
        is_datum_node_up_msg = Bool()
        is_datum_node_up_msg.data = True
        self.status_publisher.publish(is_datum_node_up_msg)

        if self.polaris_sensor_check_passed and not self.datum_set_once:

            # Wait until the service is available
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for /datum service...')

            # Convert to quaternion
            roll = 0.0
            pitch = 0.0
            q = quaternion_from_euler(roll, pitch, self.polaris_heading)

            # Create the request with the specified parameters
            self.request = SetDatum.Request()
            self.request.geo_pose.position.latitude = self.polaris_lat
            self.request.geo_pose.position.longitude = self.polaris_long
            self.request.geo_pose.orientation.x = q[0]
            self.request.geo_pose.orientation.y = q[1]
            self.request.geo_pose.orientation.z = q[2]
            self.request.geo_pose.orientation.w = q[3]

            self.get_logger().info('Sending request to /datum service...')
            self.send_request()

            self.datum_set_once = True


def main(args=None):
    rclpy.init(args=args)
    node = DatumServiceClient()
    try:
        rclpy.spin(node)  # Keeps the node active and listening
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
