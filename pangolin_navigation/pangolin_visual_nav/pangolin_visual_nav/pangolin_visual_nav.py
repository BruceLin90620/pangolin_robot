import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import numpy as np

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

        # State variables
        self.current_pose = None
        self.goal_pose = None
        self.state = "IDLE" # Default state
        self.angular_errors = []  # To store angular errors over time
        self.distance_errors = []  # To store distance errors over time

        # Tolerances
        self.linear_tolerance = 0.1
        self.angular_tolerance = 0.15

        # Speed settings
        self.max_linear_speed = 1.0  
        self.max_angular_speed = 0.5  

        # Timer to run navigation logic periodically(0.1 second)
        self.timer = self.create_timer(0.1, self.navigation_loop)

    def current_pose_callback(self, msg):
        if msg.poses:
            self.current_pose = msg.poses[-1].pose #Get robot's current position

    def goal_pose_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f"Goal Pose: {self.goal_pose.position.x}, {self.goal_pose.position.y}")
        self.state = "NAVIGATE"

    def navigation_loop(self):
        if self.state == "IDLE" or self.state == "DONE":
            # Still update the current position via callbacks but do nothing here.
            return

        if self.current_pose is None or self.goal_pose is None:
            return

        # Compute errors
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        theta = self.quaternion_to_yaw(self.current_pose.orientation)

        # Get goal position
        x_goal = self.goal_pose.position.x
        y_goal = self.goal_pose.position.y

        x_diff = x_goal - x
        y_diff = y_goal - y
        distance_to_goal = np.hypot(x_diff, y_diff)
        target_angle = np.arctan2(y_diff, x_diff)
        angular_error = self.normalize_angle(target_angle - theta)

        # Log errors for plotting
        self.distance_errors.append(distance_to_goal)
        self.angular_errors.append(angular_error)

        # Check if we have reached the goal
        if distance_to_goal <= self.linear_tolerance:
            # Goal reached, stop
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info("Goal reached. Waiting for a new goal.")
            self.state = "IDLE"  # Reset to IDLE to wait for the next goal
            self.plot_results()
            return

        # Navigation logic
        if abs(angular_error) > np.pi / 2:  # If the angle between current and goal position is less than pi/2, resume
            linear_speed = -min(self.max_linear_speed, distance_to_goal)  # Reverse
            angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_error / 2))
        else:
            linear_speed = min(self.max_linear_speed, distance_to_goal)
            angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_error))

        # Publish the computed velocity
        self.publish_velocity(linear_speed, angular_speed)

    def publish_velocity(self, linear_x, angular_z):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
        cmd_vel_msg.angular.z = -angular_z
        self.cmd_vel_pub.publish(cmd_vel_msg)

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
