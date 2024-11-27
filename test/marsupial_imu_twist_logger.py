import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
import csv
from datetime import datetime
from tf_transformations import euler_from_quaternion


class IMUTwistLogger(Node):
    def __init__(self):
        super().__init__('imu_twist_logger')
        
        # Subscriptions
        self.imu_subscription = self.create_subscription(
            Imu,
            '/gx5/imu_with_covariance',
            self.imu_callback,
            10)
        
        self.twist_subscription = self.create_subscription(
            TwistWithCovarianceStamped,
            '/mcu/state/vel_with_covariance',
            self.twist_callback,
            10)

        # CSV file setup
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = f'imu_twist_data_{timestamp}.csv'
        self.fieldnames = [
            'timestamp',
            'imu_roll', 'imu_pitch', 'imu_yaw',
            'imu_angular_velocity_x', 'imu_angular_velocity_y', 'imu_angular_velocity_z',
            'imu_linear_acceleration_x', 'imu_linear_acceleration_y', 'imu_linear_acceleration_z',
            'twist_linear_x', 'twist_linear_y', 'twist_linear_z',
            'twist_angular_x', 'twist_angular_y', 'twist_angular_z'
        ]
        
        with open(self.csv_file, mode='w') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writeheader()
        
        self.get_logger().info(f"Logging data to {self.csv_file}")

    def imu_callback(self, msg: Imu):
        # Extract quaternion and convert to Euler angles
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Store data in a dictionary
        self.imu_data = {
            'imu_roll': roll,
            'imu_pitch': pitch,
            'imu_yaw': yaw,
            'imu_angular_velocity_x': msg.angular_velocity.x,
            'imu_angular_velocity_y': msg.angular_velocity.y,
            'imu_angular_velocity_z': msg.angular_velocity.z,
            'imu_linear_acceleration_x': msg.linear_acceleration.x,
            'imu_linear_acceleration_y': msg.linear_acceleration.y,
            'imu_linear_acceleration_z': msg.linear_acceleration.z,
        }

    def twist_callback(self, msg: TwistWithCovarianceStamped):
        twist_data = {
            'twist_linear_x': msg.twist.twist.linear.x,
            'twist_linear_y': msg.twist.twist.linear.y,
            'twist_linear_z': msg.twist.twist.linear.z,
            'twist_angular_x': msg.twist.twist.angular.x,
            'twist_angular_y': msg.twist.twist.angular.y,
            'twist_angular_z': msg.twist.twist.angular.z,
        }

        # Log data to CSV when twist data is received (ensure imu_data exists)
        if hasattr(self, 'imu_data'):
            row = {
                'timestamp': self.get_clock().now().to_msg().sec,
                **self.imu_data,
                **twist_data
            }
            
            with open(self.csv_file, mode='a') as file:
                writer = csv.DictWriter(file, fieldnames=self.fieldnames)
                writer.writerow(row)
            
            self.get_logger().info(f"Logged data row at timestamp {row['timestamp']}")


def main(args=None):
    rclpy.init(args=args)
    imu_twist_logger = IMUTwistLogger()
    
    try:
        rclpy.spin(imu_twist_logger)
    except KeyboardInterrupt:
        imu_twist_logger.get_logger().info("Shutting down logger.")
    finally:
        imu_twist_logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
