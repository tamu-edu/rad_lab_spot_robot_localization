import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from std_msgs.msg import Bool
from apriltag_ros_msgs.msg import AprilTagDetectionArray

class MultiTopicSubscriber(Node):
    def __init__(self):
        super().__init__('multi_topic_subscriber')

        self.last_msg_time = {}
        self.msg_count = {}
        self.topic_rates = {}
        self.total_rates = {}
        self.message_counts = {}

        # Subscriptions with separate callbacks for each topic
        self.create_subscription(TwistStamped, '/mcu/state/vel', self.callback_twist_stamped, 10)
        self.create_subscription(Odometry, '/gx5/nav/odom', self.callback_odometry_gx5_nav, 10)
        self.create_subscription(NavSatFix, '/gx5/gnss1/fix', self.callback_navsatfix_gnss1, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/mcu/state/vel_with_covariance', self.callback_twist_with_covariance, 10)
        self.create_subscription(Imu, '/gx5/imu_with_covariance', self.callback_imu_with_covariance, 10)
        self.create_subscription(Imu, '/gx5/enu_heading', self.callback_enu_heading, 10)
        self.create_subscription(NavSatFix, '/gx5/gnss1/fix_corrected_frameid', self.callback_navsatfix_corrected, 10)
        self.create_subscription(Bool, '/dog/launch_localization', self.callback_launch_localization, 10)
        self.create_subscription(Odometry, '/odometry/local', self.callback_odometry_local, 10)
        self.create_subscription(Odometry, '/odometry/global', self.callback_odometry_global, 10)
        self.create_subscription(Odometry, '/odometry/gps', self.callback_odometry_gps, 10)
        self.create_subscription(BatteryState, '/mcu/state/battery', self.callback_battery_state, 10)
        self.create_subscription(AprilTagDetectionArray, '/tag_detections', self.callback_tag_detections, 10)
        self.create_subscription(Bool, '/dog/is_datum_node_up', self.callback_is_datum_node_up, 10)

    # Individual callback implementations
    def callback_twist_stamped(self, msg):
        self.update_rate('/mcu/state/vel')

    def callback_odometry_gx5_nav(self, msg):
        self.update_rate('/gx5/nav/odom')

    def callback_navsatfix_gnss1(self, msg):
        self.update_rate('/gx5/gnss1/fix')

    def callback_twist_with_covariance(self, msg):
        self.update_rate('/mcu/state/vel_with_covariance')

    def callback_imu_with_covariance(self, msg):
        self.update_rate('/gx5/imu_with_covariance')

    def callback_enu_heading(self, msg):
        self.update_rate('/gx5/enu_heading')

    def callback_navsatfix_corrected(self, msg):
        self.update_rate('/gx5/gnss1/fix_corrected_frameid')

    def callback_launch_localization(self, msg):
        self.update_rate('/dog/launch_localization')

    def callback_odometry_local(self, msg):
        self.update_rate('/odometry/local')

    def callback_odometry_global(self, msg):
        self.update_rate('/odometry/global')

    def callback_odometry_gps(self, msg):
        self.update_rate('/odometry/gps')

    def callback_battery_state(self, msg):
        self.update_rate('/mcu/state/battery')

    def callback_tag_detections(self, msg):
        self.update_rate('/tag_detections')

    def callback_is_datum_node_up(self, msg):
        self.update_rate('/dog/is_datum_node_up')

    # Update rate logic shared among callbacks
    def update_rate(self, topic_name):
        now = self.get_clock().now()

        if topic_name not in self.last_msg_time:
            self.last_msg_time[topic_name] = now
            self.msg_count[topic_name] = 0
            self.total_rates[topic_name] = 0.0
            self.message_counts[topic_name] = 0

        self.msg_count[topic_name] += 1
        time_diff = now - self.last_msg_time[topic_name]
        time_diff_sec = time_diff.nanoseconds * 1e-9

        if time_diff_sec > 0:
            rate = 1.0 / time_diff_sec
            self.topic_rates[topic_name] = rate
            self.total_rates[topic_name] += rate
            self.message_counts[topic_name] += 1

        self.last_msg_time[topic_name] = now

    def print_average_rates(self):
        print("\nAverage Message Rates:")
        for topic, total_rate in self.total_rates.items():
            message_count = self.message_counts.get(topic, 1)  # Avoid division by zero
            average_rate = total_rate / message_count if message_count > 0 else 0.0
            print(f"  {topic}: {average_rate:.2f} Hz")


def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_average_rates()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

