import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Topic to subscribe
            self.goal_pose_callback,
            10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',  # RViz topic for markers
            10
        )

        self.poses = []  # Store received poses
        self.marker_id = 0  # Unique ID for markers

    def goal_pose_callback(self, msg: PoseStamped):
        # Store the received pose
        self.poses.append(msg.pose)
        
        # Publish markers for nodes and edges
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()

        # Add nodes (spheres) for each pose
        for i, pose in enumerate(self.poses):
            node_marker = Marker()
            node_marker.header.frame_id = "map"  # Replace with your frame
            node_marker.header.stamp = self.get_clock().now().to_msg()
            node_marker.ns = "nodes"
            node_marker.id = i  # Unique ID for each node
            node_marker.type = Marker.SPHERE
            node_marker.action = Marker.ADD
            node_marker.pose = pose
            node_marker.scale.x = 0.1  # Adjust size
            node_marker.scale.y = 0.1
            node_marker.scale.z = 0.1
            node_marker.color.r = 1.0
            node_marker.color.g = 0.0
            node_marker.color.b = 0.0
            node_marker.color.a = 1.0
            marker_array.markers.append(node_marker)

        # Add edges (lines) between consecutive poses
        if len(self.poses) > 1:
            edge_marker = Marker()
            edge_marker.header.frame_id = "map"  # Replace with your frame
            edge_marker.header.stamp = self.get_clock().now().to_msg()
            edge_marker.ns = "edges"
            edge_marker.id = len(self.poses)  # Unique ID for edges
            edge_marker.type = Marker.LINE_STRIP
            edge_marker.action = Marker.ADD
            edge_marker.scale.x = 0.05  # Line width
            edge_marker.color.r = 0.0
            edge_marker.color.g = 0.0
            edge_marker.color.b = 1.0
            edge_marker.color.a = 1.0

            # Add points to the line
            for pose in self.poses:
                edge_marker.points.append(pose.position)

            marker_array.markers.append(edge_marker)

        # Publish the marker array
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
