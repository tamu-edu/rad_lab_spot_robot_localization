import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # IMU1 Subscriber and Publisher
        self.subscription_imu1 = self.create_subscription(
            Imu,
            '/gx5/imu/data',
            self.listener_imu1_callback,
            10)
        self.publisher_imu1 = self.create_publisher(Imu, '/gx5/imu/data_with_covariance', 10)

        # IMU2 Subscriber and Publisher
        self.subscription_imu2 = self.create_subscription(
            Imu,
            '/mcu/state/imu',
            self.listener_imu2_callback,
            10)
        self.publisher_imu2 = self.create_publisher(Imu, '/mcu/state/imu_with_covariance', 10)

        # Twist Subscriber and Publisher
        self.subscription_twist = self.create_subscription(
            TwistStamped,
            '/mcu/state/vel',
            self.listener_twist_callback,
            10)
        self.publisher_twist_cov = self.create_publisher(TwistWithCovarianceStamped, '/mcu/state/vel_with_covariance', 10)

    def listener_imu1_callback(self, msg):
        # Modify covariance for IMU1
        msg.angular_velocity_covariance = [0.01] * 9  # Example covariance changes
        msg.linear_acceleration_covariance = [0.01] * 9  # Example covariance changes
        # Republish the modified IMU1 message
        self.publisher_imu1.publish(msg)

    def listener_imu2_callback(self, msg):
        # Modify covariance for IMU2
        msg.angular_velocity_covariance = [0.02] * 9  # Example covariance changes
        msg.linear_acceleration_covariance = [0.02] * 9  # Example covariance changes
        # Republish the modified IMU2 message
        self.publisher_imu2.publish(msg)

    def listener_twist_callback(self, msg):
        new_msg = TwistWithCovarianceStamped()
        new_msg.header = msg.header
        new_msg.twist.twist = msg.twist
        # Adding non-zero covariance
        new_msg.twist.covariance = [0.03] * 36  # Example non-zero covariance
        # Republish the modified Twist message
        self.publisher_twist_cov.publish(new_msg)

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
