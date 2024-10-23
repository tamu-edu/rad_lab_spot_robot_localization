#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, Quaternion
import numpy as np
import tf_transformations
import math

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
        self.enu_heading_pub = self.create_publisher(Imu, '/gx5/enu_heading', 10)

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

    def transform_quaternion(self,input_quaternion):

        # Convert input Quaternion to tuple (x, y, z, w)
        quat_in = [input_quaternion.x, input_quaternion.y, input_quaternion.z, input_quaternion.w]
        
        # Create a rotation quaternion for pi/2 radians around Z-axis
        rotation_quat = tf_transformations.quaternion_from_euler(0, 0, -math.pi / 2)
        
        # Multiply the rotation quaternion by the input quaternion
        quat_out = tf_transformations.quaternion_multiply(rotation_quat, quat_in)
        
        # Normalize the output quaternion
        quat_out = tf_transformations.unit_vector(quat_out)
        
        # Convert back to geometry_msgs.msg.Quaternion
        output_quaternion = Quaternion()
        output_quaternion.x = quat_out[0]
        output_quaternion.y = quat_out[1]
        output_quaternion.z = quat_out[2]
        output_quaternion.w = quat_out[3]
        
        # Negate the x, y, z components
        left_handed_quaternion = Quaternion()
        left_handed_quaternion.x = -output_quaternion.x
        left_handed_quaternion.y = -output_quaternion.y
        left_handed_quaternion.z = -output_quaternion.z
        left_handed_quaternion.w = output_quaternion.w

        return left_handed_quaternion


    def rotate_along_x_by_pi(self, q):
        # Create a new Quaternion message
        rotated_q = Quaternion()
        
        # Keep w and x the same, negate y and z
        rotated_q.w = q.w
        rotated_q.x = q.y
        rotated_q.y = q.x
        rotated_q.z = -q.z
        
        return rotated_q

    def listener_odom_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.header.frame_id = 'body'
        # rotated_orientation = self.rotate_along_x_by_pi(msg.pose.pose.orientation)
        imu_msg.orientation = msg.pose.pose.orientation
        imu_msg.orientation_covariance = self.fill_covariance(0.1,3)
        self.imu_pub.publish(imu_msg)

        enu_imu_msg = Imu()
        enu_imu_msg.header = msg.header
        enu_imu_msg.header.frame_id = 'body'
        enu_imu_msg.orientation = self.transform_quaternion(msg.pose.pose.orientation)
        enu_imu_msg.orientation_covariance = self.fill_covariance(0.1,3)
        self.enu_heading_pub.publish(enu_imu_msg)
        
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
