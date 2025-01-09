import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from cv_bridge import CvBridge
import numpy as np
from apriltag import apriltag
import cv2
from DXL_motor_control import DXL_Communication
from Pangolin_Config import PangolinConfiguration, PangolinDynamixel
import time

DEGREE_TO_SERVO = 4095 / 360

class PangolinControl:
    def __init__(self):
        self.control_cmd = ControlCmd()
        self.pangolin_config = PangolinConfiguration()

        self.motor_position = np.zeros(8)
        self.leg_center_position = self.pangolin_config.leg_center_position

    def cleanup(self):
        self.control_cmd.cleanup()

    def set_head_position(self, yaw_angle: float, pitch_angle: float):
        head_motor_angle = np.array([yaw_angle, pitch_angle])
        head_motor_pos = head_motor_angle * DEGREE_TO_SERVO + self.leg_center_position[4:6]
        self.motor_position[4:6] = head_motor_pos
        self.control_cmd.motor_position_control(self.motor_position)

    def set_tail_position(self, yaw_angle: float):
        tail_motor_angle = np.array([yaw_angle])
        tail_motor_pos = tail_motor_angle * DEGREE_TO_SERVO + self.leg_center_position[6:8]
        self.motor_position[6:8] = tail_motor_pos
        self.control_cmd.motor_position_control(self.motor_position)

class ControlCmd:
    def __init__(self):
        pangolin_dynamixel = PangolinDynamixel()

        self.dynamixel = DXL_Communication(pangolin_dynamixel.DEVICE_NAME, pangolin_dynamixel.B_RATE)
        self.dynamixel.activateDXLConnection()

        head_motor_Y = self.dynamixel.createMotor('motor5', motor_number=5)
        head_motor_P = self.dynamixel.createMotor('motor6', motor_number=6)
        tail_motor_Y = self.dynamixel.createMotor('motor7', motor_number=7)

        self.head_motor_list = [head_motor_Y, head_motor_P]
        self.tail_motor_list = [tail_motor_Y]

        self.dynamixel.rebootAllMotor()
        self.dynamixel.updateMotorData()

        self.enable_all_motor()

    def cleanup(self):
        self.disable_all_motor()

    def enable_all_motor(self):
        for motor in self.head_motor_list + self.tail_motor_list:
            motor.enableMotor()
            motor.directWriteData(40, 112, 4)

    def disable_all_motor(self):
        for motor in self.head_motor_list + self.tail_motor_list:
            motor.disableMotor()

        self.dynamixel.closeHandler()

    def motor_position_control(self, position: np.array):
        motor_id = 4
        for motor in self.head_motor_list + self.tail_motor_list:
            motor.writePosition(int(position[motor_id]))
            motor_id += 1

        self.dynamixel.sentAllCmd()
        time.sleep(0.1)

class RealSenseHeadControl(Node):
    def __init__(self):
        super().__init__('realsense_head_control')

        # ROS2 Publishers
        #self.camera_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel_key', 10)

        # Initialize TF Broadcaster and Listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize Pangolin Control for head
        self.pangolin_control = PangolinControl()

        # AprilTag Detector
        self.detector = apriltag("tag36h11")

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Parameters
        self.camera_frame = "camera_link"
        self.tag0_id = 0  # ID for AprilTag used in robot following
        self.tag1_id = 1  # ID for AprilTag used in head following

        # Store last-known transform and timestamp
        self.last_transform = None
        self.last_detected_time = self.get_clock().now()

        # Detection timeout in seconds
        self.detection_timeout = 1.0

        # Timer for processing frames
        #self.timer = self.create_timer(1 / 60, self.process_camera_frame)

        # Subscribe to RealSense topic provided by Isaac ROS
        self.create_subscription(
            Image, '/image', self.camera_callback, 10
        )

        self.get_logger().info("RealSense Head Control with AprilTag Following initialized.")

    def camera_callback(self, msg):
        """Callback for receiving camera frames."""
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_camera_frame(color_image)
        except Exception as e:
            self.get_logger().error(f"Failed to process camera frame: {e}")

    def process_camera_frame(self, color_image):
        """Process camera frame and detect AprilTags."""
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray_image)

        if detections:
            for tag in detections:
                tag_id = int(tag['id'])
                center = tag['center']

                if tag_id == self.tag0_id:
                    # Robot following logic
                    self.follow_robot(center)
                    return

                elif tag_id == self.tag1_id:
                    # Head following logic
                    self.follow_head_and_tail(center,color_image)
                    return

        # Handle no detections or timeout
        current_time = self.get_clock().now()
        time_since_last_detection = (current_time - self.last_detected_time).nanoseconds * 1e-9

        if self.last_transform and time_since_last_detection < self.detection_timeout:
            self.get_logger().info("Using last-known transform for robot following.")
            self.follow_robot(use_last_transform=True)
        else:
            self.get_logger().info("No valid AprilTag detected. Stopping the robot.")
            self.publish_cmd_vel(0.0, 0.0)

    def follow_robot(self, center=None, use_last_transform=False):
        """Follow AprilTag with ID `tag0_id`."""
        if use_last_transform:
            translation = self.last_transform
        else:
            translation = self.calculate_translation(center)
            if translation is None:
                return

            # Save last-known translation
            self.last_transform = translation
            self.last_detected_time = self.get_clock().now()

        distance = translation[2]  # Depth (z-axis)
        lateral_error = translation[0]  # Horizontal offset (x-axis)

        # Proportional control for velocities
        linear_velocity = 0.5 if distance > 0.2 else 0.0
        angular_velocity = -1.0 if lateral_error > -0.4 else (1.0 if lateral_error < -0.7 else 0.0) #lateral_error=0.1

        self.publish_cmd_vel(linear_velocity, -angular_velocity)
        self.get_logger().info(
            f"Robot Following | Distance: {distance:.2f}, Lateral Error: {lateral_error:.2f}, "
            f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}"
        )

    def follow_head_and_tail(self, center, color_image):
        """Follow AprilTag with ID `tag1_id`."""
        # Adjusted for 1920x1080 resolution
        image_width = color_image.shape[1]
        image_height = color_image.shape[0]

        error_x = (center[0] - image_width / 2) / (image_width / 2)
        error_y = (center[1] - image_height / 2) / (image_height / 2)

        self.get_logger().info(f"Center X: {center[0]}, Error X: {error_x}")
        self.get_logger().info(f"Center Y: {center[1]}, Error Y: {error_y}")

        # Proportional control constants
        Kp_yaw = 35  # Increased for more responsive movement
        Kp_pitch = 33

        # Dead zone thresholds
        dead_zone_x = 0.1 #0.05
        dead_zone_y = 0.1 #0.05

        # Apply dead zone logic
        if abs(error_x) < dead_zone_x:
            error_x = 0.0
        if abs(error_y) < dead_zone_y:
            error_y = 0.0

        # Calculate adjustments with extended range
        yaw_adjustment = np.clip(Kp_yaw * error_x, -35.0, 35.0)
        pitch_adjustment = np.clip(Kp_pitch * error_y, -45.0, 45.0)

        #Smoothing adjustments
        smoothing_factor = 0.55  # Higher value for smoother transitions
        yaw_adjustment = (
            smoothing_factor * yaw_adjustment + (1 - smoothing_factor) * getattr(self, 'last_yaw_adjustment', 0.0)
        )
        pitch_adjustment = (
            smoothing_factor * pitch_adjustment + (1 - smoothing_factor) * getattr(self, 'last_pitch_adjustment', 0.0)
        )

        # Save adjustments for smoothing
        self.last_yaw_adjustment = yaw_adjustment
        self.last_pitch_adjustment = pitch_adjustment

        # Update head position
        self.pangolin_control.set_head_position(yaw_angle=-yaw_adjustment, pitch_angle=-pitch_adjustment)
        # Tail movement proportional to yaw
        tail_angle = yaw_adjustment 
        self.pangolin_control.set_tail_position(yaw_angle=tail_angle)

        # Logging for debugging
        self.get_logger().info(
            f"Head Following | Center: {center}, Error X: {error_x:.2f}, "
            f"Yaw: {-yaw_adjustment:.2f}, Pitch: {-pitch_adjustment:.2f}"
        )


    def calculate_translation(self, center):
        """Placeholder for 3D translation logic."""
        # Isaac ROS provides depth data in a separate topic. Add logic to calculate translation if needed.
        return [center[0] / 640 - 1, center[1] / 480 - 1, 1.0]  # Example values

    def publish_cmd_vel(self, linear, angular):
        """Publish cmd_vel to control the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.cmd_vel_publisher.publish(cmd_vel)

    def cleanup(self):
        """Cleanup the resources."""
        self.pangolin_control.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseHeadControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

