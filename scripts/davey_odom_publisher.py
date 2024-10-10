#! /usr/bin/env python3

import transformations as tf
import rclpy
import time
import csv
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from numpy import cos, sin, pi

class OdometryPublisher(Node):

    def __init__(self):
        #initialize odometry publisher node
        super().__init__("odometry_publisher")

        #start a publisher to publish odometry
        self.publisher_ = self.create_publisher(Odometry, '/leg/odometry', 10)
        #self.timer = self.create_timer(0.1, self.odometry)
        self.tf_broadcaster = TransformBroadcaster(self)

        #start a subscriber to get velocity
        self.subscription = self.create_subscription(
            TwistStamped, 'mcu/state/vel', self.velocity_callback, 10)

        #initialize instance variables
        self.linear_velocity_x = 0.0
        self.linear_velocity_y = 0.0
        self.linear_velocity_z = 0.0

        self.prev_linear_velocity_x = 0.0
        self.prev_linear_velocity_y = 0.0
        self.prev_linear_velocity_z = 0.0

        self.angular_velocity_x = 0.0
        self.angular_velocity_y = 0.0
        self.angular_velocity_z = 0.0

        self.prev_angular_velocity_x = 0.0
        self.prev_angular_velocity_y = 0.0
        self.prev_angular_velocity_z = 0.0

        self.pitch = 0.0
        self.roll = 0.0 + pi
        self.yaw = 0.0 + pi

        self.position_x = 0.0 
        self.position_y = 0.0
        self.position_z = 0.0

        self.current_time = time.time()
        self.prev_time = time.time()

        #open csv file and name it
        self.csv_file = open('wheel_odometry_data.csv', mode = 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        #write the header row
        self.csv_writer.writerow(['Time', 'Position X', 'Position Y'])

    def velocity_callback(self, msg):
        #record linear velocities
        self.linear_velocity_x = msg.twist.linear.x
        self.linear_velocity_y = msg.twist.linear.y
        self.linear_velocity_z = msg.twist.linear.z

        #record angular velocities
        self.angular_velocity_x = msg.twist.angular.x
        self.angular_velocity_y = msg.twist.angular.y
        self.angular_velocity_z = msg.twist.angular.z

        self.odometry()

    def odometry(self):

        #current time calculator
        self.current_time = time.time()
        dt = self.current_time - self.prev_time

        self.position_x += (self.linear_velocity_x * cos(self.yaw) - self.linear_velocity_y * sin(self.yaw)) * dt
        self.position_y += (self.linear_velocity_x * sin(self.yaw) + self.linear_velocity_y * cos(self.yaw)) * dt
        self.position_z += self.linear_velocity_z * dt


        #write data to the csv file
        self.csv_writer.writerow([self.current_time, self.position_x, self.position_y])

        #integrate angular velocity to get r,p,w
        self.roll += self.angular_velocity_x * dt
        self.pitch += self.angular_velocity_y * dt
        self.yaw += self.angular_velocity_z * dt

        #self.yaw = (self.yaw + pi) % (2 * pi) - pi
        #self.new_yaw = self.yaw - (np.pi / 2.0)  #adjust the yaw to ENU by subtracting pi/2
        #self.new_yaw = (self.new_yaw + 2 * np.pi) % (2 * np.pi)  # Ensure yaw is within [0, 2*pi]
        #self.get_logger().info(f'Published Yaw: {self.new_yaw}')
        
        #create the new odometry message
        odom_msg = Odometry()

        #populate the message
        #position
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link' #previously body
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = self.position_z

        #linear velocity
        odom_msg.twist.twist.linear.x = self.linear_velocity_x
        odom_msg.twist.twist.linear.y = self.linear_velocity_y
        odom_msg.twist.twist.linear.z = self.linear_velocity_z

        #angular velocity
        odom_msg.twist.twist.angular.x = self.angular_velocity_x
        odom_msg.twist.twist.angular.y = self.angular_velocity_y
        odom_msg.twist.twist.angular.z = self.angular_velocity_z

        #convert Euler angles to quaternion
        #quaternion = tf.quaternion_from_euler(self.yaw, self.pitch, self.roll) 
        quaternion = tf.quaternion_from_euler(self.yaw, self.pitch, self.roll, axes='sxyz') 
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        #create 6x6 matrix w zeros and 0.01 in the diagonal
        pose_covariance = np.zeros((6,6))
        twist_covariance = np.zeros((6,6))

        #fill diagonal with 0.01
        np.fill_diagonal(pose_covariance, 0.01)
        np.fill_diagonal(twist_covariance, 0.01)

        #flatten the matrices to match expected ros format
        pose_covariance_flat = pose_covariance.flatten().tolist()
        twist_covariance_flat = twist_covariance.flatten().tolist()

        #assign odom message
        odom_msg.pose.covariance = pose_covariance_flat
        odom_msg.twist.covariance = twist_covariance_flat

        self.prev_time = self.current_time

        self.publisher_.publish(odom_msg)

        #Create and populate the transform message
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link' #previously body
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation.x = odom_msg.pose.pose.orientation.x
        transform.transform.rotation.y = odom_msg.pose.pose.orientation.y
        transform.transform.rotation.z = odom_msg.pose.pose.orientation.z
        transform.transform.rotation.w = odom_msg.pose.pose.orientation.w
        
        #Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)        

    def destroy_node(self):
        super().destroy_node()

        #close csv file
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)

    odometry_publisher = OdometryPublisher()

    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        pass

    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()