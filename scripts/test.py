import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs

class TransformWaypointNode(Node):
    def __init__(self):
        super().__init__('transform_waypoint_node')
        
        # Subscriber to /target_waypoint
        self.subscription = self.create_subscription(
            PoseStamped,
            '/target_waypoint',
            self.waypoint_callback,
            10
        )
        
        # Publisher for transformed goal in body frame on /move_base_simple/goal
        self.pub_goal = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def waypoint_callback(self, msg):
        try:
            # Lookup transform from map frame to body frame
            transform = self.tf_buffer.lookup_transform(
                'body',  # Target frame
                'map',   # Source frame
                rclpy.time.Time()
            )
            
            # Extract the Pose from the PoseStamped message
            pose_msg = Pose()
            pose_msg.position = msg.pose.position
            pose_msg.orientation = msg.pose.orientation

            # Transform the received PoseStamped to body frame
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_msg, transform)
            
            # Update the frame_id of the transformed pose to 'body'
            # transformed_pose.header.frame_id = 'body'
            # transformed_pose = PoseStamped()
            # transformed_pose.header.frame_id = 'body'
            # transformed_pose.pose = pose_msg
            
            # Publish the transformed pose to /move_base_simple/goal
            # self.pub_goal.publish(transformed_pose)
            self.pub_goal.publish(PoseStamped(header=Header(frame_id="body"), pose=Pose(position=Point(x=transformed_pose.position.x, y=transformed_pose.position.y))))
            # self.pub_goal.publish(PoseStamped(header=Header(frame_id="body"), pose=Pose(position=Point(x=0.5, y=0.0))))
        
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Transform failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TransformWaypointNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
