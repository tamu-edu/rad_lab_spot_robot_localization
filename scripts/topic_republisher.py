#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, Quaternion
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Twist Subscriber and Publisher
        self.subscription_twist = self.create_subscription(
            TwistStamped,
            '/mcu/state/vel',
            self.listener_twist_callback,
            10)
        self.publisher_twist_cov = self.create_publisher(TwistWithCovarianceStamped, '/mcu/state/vel_with_covariance', 10)

        # Odom Subscriber and IMU publisher
        self.odom_sub = self.create_subscription(
            Odometry, '/gx5/nav/odom', self.listener_odom_callback, 10)
        self.imu_pub = self.create_publisher(Imu, '/gx5/imu_with_covariance', 10)

        # GPS subscriber and publisher
        self.gps_sub = self.create_subscription(
            NavSatFix, 
            '/gx5/gnss1/fix', 
            self.gps_callback, 
            10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gx5/gnss1/fix_corrected_frameid', 10)

        # TODO: testing
        self.new_gps_msg = None
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # msg = NavSatFix()

        # # Fill out the Header
        # msg.header = Header()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = "gps_frame"

        # # Fill out the GPS data (example values)
        # msg.latitude = 30.6405
        # msg.longitude = -96.4872
        # msg.altitude = 100.0
        # msg.position_covariance = [0.0] * 9
        # msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        # self.publisher_.publish(msg)

        if self.new_gps_msg == None:
            pass
        else:
            self.gps_pub.publish(self.new_gps_msg)


    def listener_twist_callback(self, msg):
        new_msg = TwistWithCovarianceStamped()
        new_msg.header = msg.header
        new_msg.twist.twist = msg.twist
        # Adding non-zero covariance
        new_msg.twist.covariance = self.fill_covariance(0.1,6)
        # Republish the modified Twist message
        self.publisher_twist_cov.publish(new_msg)

    def rotate_along_x_by_pi(self, q):
        # Create a new Quaternion message
        rotated_q = Quaternion()
        
        # Keep w and x the same, negate y and z
        rotated_q.w = q.w
        rotated_q.x = q.x
        rotated_q.y = -q.y
        rotated_q.z = -q.z
        
        return rotated_q

    def listener_odom_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.header.frame_id = 'body'
        rotated_orientation = self.rotate_along_x_by_pi(msg.pose.pose.orientation)
        imu_msg.orientation = rotated_orientation
        imu_msg.orientation_covariance = self.fill_covariance(0.1,3)
        self.imu_pub.publish(imu_msg)

    def fill_covariance(self, value, num_states):
        return np.diag([value]*num_states).flatten().tolist() # TODO: make this 3 for IMU

    def gps_callback(self, msg):
        self.new_gps_msg = NavSatFix()
        
        # keep most of the msg the same
        self.new_gps_msg = msg
        
        # change the frame_id
        self.new_gps_msg.header.frame_id = 'body'

        # self.gps_pub.publish(new_gps_msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()
    rclpy.spin(sensor_fusion_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
