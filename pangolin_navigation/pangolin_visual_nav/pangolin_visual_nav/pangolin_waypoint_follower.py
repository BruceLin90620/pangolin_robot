import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

class RealWorldPathFinder(Node):
    def __init__(self):
        super().__init__('pangolin_visual_navigation')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_nav', #Velocity command for navigation
            qos_profile=rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        )
        self.create_subscription(Path, '/visual_slam/tracking/slam_path', self.current_pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.waypoints_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 10)

        # State variables
        self.current_pose = None
        self.goal_pose = None
        self.state = "IDLE" # Default state
        self.path = []  # List of waypoints
        self.current_waypoint_index = 0
        self.angular_errors = []  # To store angular errors over time
        self.distance_errors = []  # To store distance errors over time
        self.obstacle_detected = False

        # Tolerances
        self.linear_tolerance = 0.1
        self.angular_tolerance = 0.15

        # Speed settings
        self.max_linear_speed = 1.0  
        self.max_angular_speed = 0.5  
        self.obstacle_stop_distance = 0.5
        self.safe_distance = 0.8

        # Timer to run navigation logic periodically(0.1 second)
        self.timer = self.create_timer(0.1, self.navigation_loop)

    def current_pose_callback(self, msg):
        if msg.poses:
            self.current_pose = msg.poses[-1].pose  # Get robot's current position
            self.get_logger().info(f"Current Pose: {self.current_pose.position.x}, {self.current_pose.position.y}")

    def goal_pose_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f"Goal Pose: {self.goal_pose.position.x}, {self.goal_pose.position.y}")
        if self.current_pose is not None:
            self.generate_path()
            self.state = "NAVIGATE"
        else:
            self.get_logger().warn("Current pose is not available. Cannot plan path.")

    def scan_callback(self, msg):
        self.obstacle_detected = any(
            not np.isnan(r) and msg.range_min < r < self.obstacle_stop_distance for r in msg.ranges
    )

    def generate_path(self):
        if self.current_pose is None or self.goal_pose is None:
            self.get_logger().warn("Cannot plan path. Current pose or goal pose is missing.")
            return
        self.get_logger().info("Planning path...")
        x_start, y_start = self.current_pose.position.x, self.current_pose.position.y
        x_goal, y_goal = self.goal_pose.position.x, self.goal_pose.position.y

        # Calculate the path length
        path_length = np.hypot(x_goal - x_start, y_goal - y_start)
        self.get_logger().info(f"Path length: {path_length}")

        # Define the desired spacing between waypoints
        waypoint_spacing = 0.8

        # Calculate the number of waypoints
        num_points = max(2, int(np.ceil(path_length / waypoint_spacing)))
        self.get_logger().info(f"Number of waypoints: {num_points}")

        # Generate waypoints using linear interpolation
        self.path = [
            (x_start + i * (x_goal - x_start) / (num_points - 1),
            y_start + i * (y_goal - y_start) / (num_points - 1))
            for i in range(num_points)
        ]
        self.current_waypoint_index = 0
        self.get_logger().info(f"Generated path with {len(self.path)} waypoints.")
        # Publish path and waypoints
        self.publish_path()
        self.publish_waypoints()

    def navigation_loop(self):
        if self.state == "IDLE" or self.state == "DONE":
            # Update position via callbacks but do nothing else in these states.
            return

        if self.current_pose is None or self.goal_pose is None or not self.path:
            # Ensure we have a valid pose, goal, and path before proceeding.
            return

        # Ensure we do not exceed the path length
        if self.current_waypoint_index >= len(self.path):
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info("Goal reached. Waiting for a new goal.")
            self.state = "IDLE"
            self.plot_results()
            return

        # Current waypoint to target
        x_waypoint, y_waypoint = self.path[self.current_waypoint_index]
        x, y = self.current_pose.position.x, self.current_pose.position.y
        theta = self.quaternion_to_yaw(self.current_pose.orientation)

        # Compute errors
        x_diff = x_waypoint - x
        y_diff = y_waypoint - y
        distance_to_waypoint = np.hypot(x_diff, y_diff)
        target_angle = np.arctan2(y_diff, x_diff)
        angular_error = self.normalize_angle(target_angle - theta)

        self.distance_errors.append(distance_to_waypoint)
        self.angular_errors.append(angular_error)

        # Check if the waypoint is reached
        if distance_to_waypoint <= self.linear_tolerance:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.path):
                self.publish_velocity(0.0, 0.0)
                self.get_logger().info("Goal reached. Waiting for a new goal.")
                self.state = "IDLE"
                self.plot_results()
                return
            return  # Proceed to the next iteration targeting the next waypoint.

        # Obstacle avoidance
        if self.obstacle_detected:
            self.get_logger().info("Obstacle detected! Stopping.")
            self.publish_velocity(0.0, 0.0)
            return

        # Navigation logic
        if abs(angular_error) > np.pi / 2:  # Handle large angular errors
            linear_speed = -min(self.max_linear_speed, distance_to_waypoint)  # Reverse
            angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_error / 2))
        else:
            linear_speed = min(self.max_linear_speed, distance_to_waypoint)
            angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_error))

        # Publish the computed velocity
        self.publish_velocity(linear_speed, angular_speed)


    def publish_velocity(self, linear_x, angular_z):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
        cmd_vel_msg.angular.z = -angular_z
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def publish_path(self):
        """Publish the planned path to RViz."""
        path_msg = Path()
        path_msg.header.frame_id = "map"  # Adjust according to your TF frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0  # Neutral orientation
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def publish_waypoints(self):
        """Publish waypoints as markers to RViz."""
        marker_array = MarkerArray()

        for i, point in enumerate(self.path):
            marker = Marker()
            marker.header.frame_id = "map"  # Adjust according to your TF frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.1  # Slightly above the ground
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # Adjust size
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Fully visible

            marker_array.markers.append(marker)

        self.waypoints_pub.publish(marker_array)

    def plot_results(self):
        """Plot the robot's error reductions."""
        plt.figure()
        #plt.subplot(2, 1, 2)
        plt.plot(self.distance_errors, label="Distance Error", color='blue')
        plt.plot(self.angular_errors, label="Angular Error", color='orange')
        plt.xlabel('Time Steps')
        plt.ylabel('Error')
        plt.title('Errors Over Time')
        plt.legend()
        plt.grid()
        plt.tight_layout()
        plt.show()

    @staticmethod
    def quaternion_to_yaw(orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return np.arctan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

def main(args=None):
    rclpy.init(args=args)
    node = RealWorldPathFinder()
    try:
        rclpy.spin(node)  # Keep the program running until interrupted by the user
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
    finally:
        node.destroy_node()  # Clean up the node
        rclpy.shutdown()  # Shut down ROS2

if __name__ == '__main__':
    main()
