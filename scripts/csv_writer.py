#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry  # Import for Odometry
from tf_transformations import euler_from_quaternion  # Import for quaternion to euler conversion
import csv
import os
import numpy as np

class McuStateSubscriber(Node):
    def __init__(self):
        super().__init__('mcu_state_subscriber')

        # Parameters to change
        add_extension = False
        extension = "_imu_all_false"

        # Initialize time as a member variable
        self.current_time = self.get_clock().now()
        csv_extension = '.csv'

        # Create the CSV file for velocity data if it doesn't exist
        self.vel_csv_file = 'vel_data.csv'
        if not os.path.exists(self.vel_csv_file):
            with open(self.vel_csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Write the header row for velocity data
                writer.writerow(['time', 'vx', 'vy', 'vz', 'vroll', 'vpitch', 'vyaw'])

        # Create the CSV file for IMU data if it doesn't exist
        self.imu_csv_file = 'imu_data.csv'
        if not os.path.exists(self.imu_csv_file):
            with open(self.imu_csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Write the header row for IMU data
                writer.writerow(['time', 'roll', 'pitch', 'yaw', 'angular_velocity_x', 'angular_velocity_y', 
                                 'angular_velocity_z', 'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'])
                
        # Create the CSV file for odometry data if it doesn't exist
        base_odom_file = 'odom_data'
        if add_extension:
            self.odom_csv_file = base_odom_file + extension + csv_extension
        else:
            self.odom_csv_file = base_odom_file + csv_extension

        if not os.path.exists(self.odom_csv_file):
            with open(self.odom_csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Write the header row for odometry data
                writer.writerow(['time', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw', 'vroll', 'vpitch', 'vyaw', "x_cov", "y_cov", 'z_cov', 'roll_cov', 'pitch_cov', 'yaw_cov', 'vx_cov', 'vy_cov', 'vz_cov', 'vroll_cov', 'vpitch_cov', 'vyaw_cov'])


        # Subscription to /mcu/state/vel (TwistStamped)
        self.vel_subscriber = self.create_subscription(
            TwistStamped,
            '/mcu/state/vel',
            self.vel_callback,
            10
        )

        # Subscription to /mcu/state/imu (Imu)
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/mcu/state/imu',
            self.imu_callback,
            10
        )

        # Subscription to /odometry/filtered (Odometry)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

    def update_time(self):
        # Update the current time
        self.current_time = self.get_clock().now()

    def vel_callback(self, msg: TwistStamped):
        # Update the time
        self.update_time()

        # Extract linear velocity components (vx, vy, vz)
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z

        # Extract angular velocity components (vroll, vpitch, vyaw)
        vroll = msg.twist.angular.x
        vpitch = msg.twist.angular.y
        vyaw = msg.twist.angular.z

        # Write the data to the velocity CSV file
        with open(self.vel_csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            # Write the row with time and velocity components
            writer.writerow([self.current_time.nanoseconds, vx, vy, vz, vroll, vpitch, vyaw])

    def imu_callback(self, msg: Imu):
        # Update the time
        self.update_time()

        # Extract orientation (quaternion)
        orientation_x = msg.orientation.x
        orientation_y = msg.orientation.y
        orientation_z = msg.orientation.z
        orientation_w = msg.orientation.w

        # Convert quaternion to Euler angles
        quaternion = (orientation_x, orientation_y, orientation_z, orientation_w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Extract angular velocity
        angular_velocity_x = msg.angular_velocity.x
        angular_velocity_y = msg.angular_velocity.y
        angular_velocity_z = msg.angular_velocity.z

        # Extract linear acceleration
        linear_acceleration_x = msg.linear_acceleration.x
        linear_acceleration_y = msg.linear_acceleration.y
        linear_acceleration_z = msg.linear_acceleration.z

        # Write the data to the IMU CSV file
        with open(self.imu_csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            # Write the row with time and IMU data (Euler angles, angular velocity, and linear acceleration)
            writer.writerow([self.current_time.nanoseconds, roll, pitch, yaw, angular_velocity_x, angular_velocity_y, 
                             angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z])

    def odom_callback(self, msg: Odometry):
        # Update the time
        self.update_time()

        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extract orientation (quaternion)
        ori_x = msg.pose.pose.orientation.x
        ori_y = msg.pose.pose.orientation.y
        ori_z = msg.pose.pose.orientation.z
        ori_w = msg.pose.pose.orientation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        quaternion = (ori_x, ori_y, ori_z, ori_w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Extract linear velocity
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        # Extract angular velocity
        vroll = msg.twist.twist.angular.x
        vpitch = msg.twist.twist.angular.y
        vyaw = msg.twist.twist.angular.z

        # Extract diagonal elements from pose covariance (first 6x6 matrix)
        pose_cov = np.array(msg.pose.covariance).reshape(6, 6)
        for i in range(6):
            x_cov, y_cov, z_cov, roll_cov, pitch_cov, yaw_cov = np.diag(pose_cov)

        # Extract diagonal elements from twist covariance (first 6x6 matrix)
        twist_cov = np.array(msg.twist.covariance).reshape(6, 6)
        for i in range(6):
            vx_cov, vy_cov, vz_cov, vroll_cov, vpitch_cov, vyaw_cov = np.diag(twist_cov)
            

        # Now, x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, and vyaw are available for use
        with open(self.odom_csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            # Write the row with time, position, orientation, and velocity components
            writer.writerow([self.current_time.nanoseconds, x, y, z, vx, vy, vz, roll, pitch, yaw, vroll, vpitch, vyaw, x_cov, y_cov, z_cov, roll_cov, pitch_cov, yaw_cov, vx_cov, vy_cov, vz_cov, vroll_cov, vpitch_cov, vyaw_cov])


def main(args=None):
    rclpy.init(args=args)

    mcu_state_subscriber = McuStateSubscriber()

    rclpy.spin(mcu_state_subscriber)

    # Shutdown the node after spinning
    mcu_state_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
