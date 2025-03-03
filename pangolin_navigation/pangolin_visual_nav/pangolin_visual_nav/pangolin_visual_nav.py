import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import numpy as np
from visualization_msgs.msg import Marker

class RealWorldPathFinder(Node):
    def __init__(self):
        super().__init__('pangolin_visual_navigation')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_nav', #Velocity command for navigation
            qos_profile=rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/path_marker',  # Marker for RViz visualization
            qos_profile=rclpy.qos.QoSProfile(depth=1)
        )

        self.create_subscription(Path, '/visual_slam/tracking/slam_path', self.current_pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # State variables
        self.current_pose = None
        self.starting_pose = None 
        self.goal_pose = None
        self.state = "IDLE" # Default state
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
            self.current_pose = msg.poses[-1].pose #Get robot's current position

    def goal_pose_callback(self, msg):
        self.goal_pose = msg.pose
        if self.starting_pose is None:
            self.starting_pose = self.current_pose
        self.get_logger().info(f"New Goal Pose: {self.goal_pose.position.x}, {self.goal_pose.position.y}")
        self.state = "NAVIGATE"  # Reset state to NAVIGATE for the new goal

    
    def scan_callback(self, msg):
        if len(msg.ranges) == 0:
            self.get_logger().warn("Laser scan ranges are empty!")
            return

        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=msg.range_max)  # Replace NaN with max range

        # Divide scan ranges into three sections: left, center, right
        left = ranges[:int(len(ranges) // 2.5)]
        center = ranges[int(len(ranges) // 3):int(2 * len(ranges) // 3)]
        right = ranges[int(2 * len(ranges) // 2.5):]

        self.obstacle_left = np.any((left > msg.range_min) & (left < self.obstacle_stop_distance))
        self.obstacle_center = np.any((center > msg.range_min) & (center < self.obstacle_stop_distance))
        self.obstacle_right = np.any((right > msg.range_min) & (right < self.obstacle_stop_distance))


    def navigation_loop(self):
        if self.state == "IDLE" or self.state == "DONE":
            return

        if self.current_pose is None or self.goal_pose is None:
            return

        # Check if obstacles are detected
        if self.obstacle_center:
            # Obstacle in the middle - move backward
            self.publish_velocity(-0.5, 0.0)
        elif self.obstacle_left:
            # Obstacle on the left - move to the right
            self.publish_velocity(0.0, -1.0)
        elif self.obstacle_right:
            # Obstacle on the right - move to the left
            self.publish_velocity(0.0, 1.0)
        else:
            # Resume navigation
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            theta = self.quaternion_to_yaw(self.current_pose.orientation)

            x_goal = self.goal_pose.position.x
            y_goal = self.goal_pose.position.y

            x_diff = x_goal - x
            y_diff = y_goal - y
            distance_to_goal = np.hypot(x_diff, y_diff)
            target_angle = np.arctan2(y_diff, x_diff)
            angular_error = self.normalize_angle(target_angle - theta)

            if distance_to_goal <= self.linear_tolerance:
                self.publish_velocity(0.0, 0.0)
                self.get_logger().info("Goal reached! Waiting for the next goal...")
                self.state = "DONE"  # Set state to DONE after reaching goal
                return

            if abs(angular_error) > np.pi / 2:
                linear_speed = -min(self.max_linear_speed, distance_to_goal)
                angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_error / 2))
            else:
                linear_speed = min(self.max_linear_speed, distance_to_goal)
                angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_error))

            # Visualize path in RViz
            self.visualize_path_in_rviz()

            self.publish_velocity(linear_speed, angular_speed)

    def visualize_path_in_rviz(self):
        if self.starting_pose and self.goal_pose:
            marker = Marker()
            marker.header.frame_id = "map"  # Adjust the frame to match your setup
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "path"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Thickness of the line
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Alpha (opacity)

            # Add the starting point
            start_point = self.create_point(self.starting_pose.position.x, self.starting_pose.position.y, 0.0)
            # Add the goal point
            goal_point = self.create_point(self.goal_pose.position.x, self.goal_pose.position.y, 0.0)

            marker.points.append(start_point)
            marker.points.append(goal_point)

            # Publish the marker
            self.marker_pub.publish(marker)


    def publish_velocity(self, linear_x, angular_z):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
        cmd_vel_msg.angular.z = -angular_z
        self.cmd_vel_pub.publish(cmd_vel_msg)

    @staticmethod
    def quaternion_to_yaw(orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return np.arctan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    @staticmethod
    def create_point(x, y, z):
        from geometry_msgs.msg import Point
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

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

