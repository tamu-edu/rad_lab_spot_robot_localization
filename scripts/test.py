#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped

class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer_node')

        # Subscriber to the clicked point in map frame
        self.subscription = self.create_subscription(PoseStamped, '/target_waypoint', self.listener_callback,10)
        
        # Publisher for the transformed pose in the body frame
        self.pub_goal = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        # Initialize the tf buffer and listener to listen for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def listener_callback(self, msg):
        # The input PoseStamped is in the 'map' frame
        try:
            # Wait for the transform from map to body
            transform = self.tf_buffer.lookup_transform('body', 'map', rclpy.time.Time())
            
            # Transform the clicked point pose to the body frame
            transformed_pose = do_transform_pose(msg, transform)

            # Create the PoseStamped message with the transformed pose
            transformed_pose.header.frame_id = "body" 
            
            # Publish the transformed pose
            self.pub_goal.publish(transformed_pose)

            self.get_logger().info(f"Transformed Pose: {transformed_pose.pose.position.x}, {transformed_pose.pose.position.y}")

        except Exception as e:
            self.get_logger().warn(f"Could not transform pose: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
