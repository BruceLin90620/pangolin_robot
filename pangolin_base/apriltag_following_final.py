import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import pyrealsense2 as rs
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


class ControlCmd:
    def __init__(self):
        pangolin_dynamixel = PangolinDynamixel()

        self.dynamixel = DXL_Communication(pangolin_dynamixel.DEVICE_NAME, pangolin_dynamixel.B_RATE)
        self.dynamixel.activateDXLConnection()

        head_motor_Y = self.dynamixel.createMotor('motor5', motor_number=5)
        head_motor_P = self.dynamixel.createMotor('motor6', motor_number=6)

        self.head_motor_list = [head_motor_Y, head_motor_P]

        self.dynamixel.rebootAllMotor()
        self.dynamixel.updateMotorData()

        self.enable_all_motor()

    def cleanup(self):
        self.disable_all_motor()

    def enable_all_motor(self):
        for motor in self.head_motor_list:
            motor.enableMotor()
            motor.directWriteData(40, 112, 4)

    def disable_all_motor(self):
        for motor in self.head_motor_list:
            motor.disableMotor()

        self.dynamixel.closeHandler()

    def motor_position_control(self, position: np.array):
        motor_id = 4
        for motor in self.head_motor_list:
            motor.writePosition(int(position[motor_id]))
            motor_id += 1

        self.dynamixel.sentAllCmd()
        time.sleep(0.1)


class RealSenseHeadControl(Node):
    def __init__(self):
        super().__init__('realsense_head_control')

        # ROS2 Publishers
        self.camera_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel_key', 10)

        # Initialize TF Broadcaster and Listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize Pangolin Control for head
        self.pangolin_control = PangolinControl()

        # AprilTag Detector
        self.detector = apriltag("tag36h11")

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.pipeline.start(self.config)

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
        self.timer = self.create_timer(1 / 60, self.process_frames)

        self.get_logger().info("RealSense Head Control with AprilTag Following initialized.")

    def process_frames(self):
        """Capture and process frames from the RealSense camera."""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            self.get_logger().error("Failed to capture RealSense frames.")
            self.publish_cmd_vel(0.0, 0.0)
            return

        color_image = np.asanyarray(color_frame.get_data())
        self.publish_camera_frame(color_image)

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray_image)

        if detections:
            for tag in detections:
                tag_id = int(tag['id'])
                center = tag['center']

                if tag_id == self.tag0_id:
                    # Robot following logic
                    self.follow_robot(center, depth_frame)
                    return

                elif tag_id == self.tag1_id:
                    # Head following logic
                    self.follow_head(center)
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

    def follow_robot(self, center=None, depth_frame=None, use_last_transform=False):
        """Follow AprilTag with ID `tag0_id`."""
        #self.pangolin_control.set_head_position(yaw_angle=0, pitch_angle=-20)
        if use_last_transform:
            translation = self.last_transform
        else:
            translation = self.calculate_translation(center, depth_frame)
            if translation is None:
                return

            # Save last-known translation
            self.last_transform = translation
            self.last_detected_time = self.get_clock().now()

        distance = translation[2]  # Depth (z-axis)
        lateral_error = translation[0]  # Horizontal offset (x-axis)

        # Proportional control for velocities
        linear_velocity = 0.5 if distance > 0.2 else 0.0
        angular_velocity = -1.0 if lateral_error > 0.1 else (1.0 if lateral_error < -0.1 else 0.0)

        self.publish_cmd_vel(linear_velocity, -angular_velocity)
        self.get_logger().info(
            f"Robot Following | Distance: {distance:.2f}, Lateral Error: {lateral_error:.2f}, "
            f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}"
        )

    def follow_head(self, center):
        """Follow AprilTag with ID `tag1_id`."""
        error_x = (center[0] - 1280 / 2) / (1280 / 2)  # Horizontal error
        error_y = (center[1] - 960 / 2) / (960 / 2)  # Vertical error

        # Proportional control constants
        Kp_yaw = 35  # Reduce from 40
        Kp_pitch = 35  # Reduce from 50

        # Dead zone thresholds
        dead_zone_x = 0.05  # Ignore small horizontal errors
        dead_zone_y = 0.05  # Ignore small vertical errors

        # Apply dead zone logic
        if abs(error_x) < dead_zone_x:
            error_x = 0.0
        if abs(error_y) < dead_zone_y:
            error_y = 0.0

        # Calculate adjustments with damping
        yaw_adjustment = np.clip(Kp_yaw * error_x, -35.0, 35.0)
        pitch_adjustment = np.clip(Kp_pitch * error_y, -30.0, 30.0)

        # Ensure adjustments do not cause abrupt movements
        smoothing_factor = 0.7
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

        self.get_logger().info(
            f"Head Following | Center: {center}, Yaw: {-yaw_adjustment:.2f}, Pitch: {-pitch_adjustment:.2f}"
        )


    def calculate_translation(self, center, depth_frame):
        """Calculate 3D translation for the detected AprilTag."""
        x, y = int(center[0]), int(center[1])
        depth = depth_frame.get_distance(x, y)

        if depth <= 0:
            self.get_logger().warning("Invalid depth value detected.")
            return None

        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        translation = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
        return translation

    def publish_camera_frame(self, frame):
        """Publish the camera frame as a ROS2 Image message."""
        try:
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.camera_publisher.publish(image_message)
        except Exception as e:
            self.get_logger().error(f"Failed to publish camera frame: {e}")

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
