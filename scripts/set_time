#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import subprocess

class TimeSetter(Node):
    def __init__(self):
        super().__init__('time_setter')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/mcu/state/vel',  # Replace with your TwistStamped topic name
            self.time_callback,
            10
        )
        self.has_run = False  # Flag to check if the callback has already run

    def time_callback(self, msg):
        if self.has_run:
            return  # Skip further executions of the callback
        
        # Set flag to prevent further callback execution
        self.has_run = True

        # Extract seconds and nanoseconds from header's stamp
        secs = msg.header.stamp.sec

        # Convert to date format for the `date` command
        # Here, we use only seconds for simplicity, but you can handle nanoseconds if desired.
        time_str = subprocess.check_output(['date', '-d', f'@{secs}']).decode().strip()

        # Set the system time (requires sudo privileges)
        self.get_logger().info(f"Setting system time to: {time_str}")
        subprocess.call(['sudo', 'date', '-s', time_str])

def main(args=None):
    rclpy.init(args=args)
    time_setter = TimeSetter()
    rclpy.spin(time_setter)
    time_setter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

