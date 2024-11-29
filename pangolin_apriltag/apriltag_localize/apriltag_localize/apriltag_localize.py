import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Pose
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from isaac_ros_visual_slam_interfaces.srv import SetSlamPose
import numpy as np
from transforms3d import quaternions, euler
import yaml

class AprilTagLocalization(Node):
    """
    A ROS2 node for localizing a robot using AprilTag detections.
    This node subscribes to AprilTag detections, computes the robot's pose in the map frame,
    and updates the SLAM pose estimate.
    """

    def __init__(self):
        super().__init__('apriltag_localization')
        # Subscribe to AprilTag detections
        self.tag_sub = self.create_subscription(AprilTagDetectionArray, '/tag_detections', self.tag_callback, 10)
        # Create a client to update the SLAM pose
        self.vslam_client = self.create_client(SetSlamPose, 'visual_slam/set_slam_pose')
        self.load_tag_poses()
        self.setup_tf()

    def load_tag_poses(self):
        """
        Load AprilTag poses from a YAML file.
        The file path is specified as a ROS parameter.
        """
        tag_poses_file = self.declare_parameter('tag_poses_file', '/home/pangolin/pangolin_ws/src/pangolin_robot/pangolin_apriltag/apriltag_localize/config/tag_poses.yaml').value
        with open(tag_poses_file, 'r') as file:
            self.map_tag_info = yaml.safe_load(file)['tag_info']

    def setup_tf(self):
        """
        Set up TF2 components for transform operations.
        """
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Create a timer to periodically broadcast tag transforms
        self.create_timer(1.0, self.broadcast_tag_transforms)

    def broadcast_tag_transforms(self):
        """
        Broadcast static transforms for all AprilTags in the map.
        This method is called periodically to update the TF tree.
        """
        for tag_id, pose in self.map_tag_info.items():
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = f'tag_{tag_id}'
            transform.transform.translation = Vector3(x=pose[0], y=pose[1], z=pose[2])
            quat = euler.euler2quat(pose[3], pose[4], pose[5], 'sxyz')
            transform.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            self.static_broadcaster.sendTransform(transform)

    def tag_callback(self, msg):
        """
        Callback function for AprilTag detections.
        Process the first detected tag if it's within a certain distance.
        """
        if not msg.detections:
            # self.get_logger().warn("No tag detections received")
            return
        
        detection = msg.detections[0]
        detected_tag_pose = detection.pose.pose.pose
        detected_tag_id = detection.id
        detection_distance = np.hypot(detected_tag_pose.position.z, detected_tag_pose.position.x)
        
        if detection_distance < 1.0:
            self.process_detection(detected_tag_id)

    def process_detection(self, detected_tag_id):
        """
        Process a single tag detection to compute the robot's pose in the map frame.
        """
        try:
            time_now = rclpy.time.Time()
            # Look up the transform from the robot's base to the detected tag
            base_to_tag = self.tf_buffer.lookup_transform("base_link", f"tag36h11:{detected_tag_id}", time_now)
            # Look up the transform from the map to the detected tag
            map_to_tag = self.tf_buffer.lookup_transform("map", f"tag_{detected_tag_id}", time_now)
            # Compute the robot's pose in the map frame
            robot_pose_map = self.multiply_transforms(map_to_tag.transform, self.invert_transform(base_to_tag.transform))
            
            # Convert the computed transform to a Pose message
            pose = Pose()
            pose.position.x = robot_pose_map.translation.x
            pose.position.y = robot_pose_map.translation.y
            pose.position.z = robot_pose_map.translation.z
            pose.orientation = robot_pose_map.rotation

            # self.get_logger().info(f"Robot pose in map frame: {pose}")
            self.send_pose_update(pose)
        except Exception as e:
            self.get_logger().error(f'Failed to compute transform: {str(e)}')

    def send_pose_update(self, pose):
        """
        Send a pose update to the SLAM system.
        """
        request = SetSlamPose.Request()
        request.pose = pose
        self.vslam_client.call_async(request)

    def multiply_transforms(self, t1, t2):
        """
        Multiply two transforms.
        """
        m1 = self.transform_to_matrix(t1)
        m2 = self.transform_to_matrix(t2)
        result_matrix = np.dot(m1, m2)
        return self.matrix_to_transform(result_matrix)

    def transform_to_matrix(self, transform):
        """
        Convert a transform to a 4x4 transformation matrix.
        """
        translation = [transform.translation.x, transform.translation.y, transform.translation.z]
        rotation = [transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z]
        mat = np.eye(4)
        mat[:3, :3] = quaternions.quat2mat(rotation)
        mat[:3, 3] = translation
        return mat

    def matrix_to_transform(self, matrix):
        """
        Convert a 4x4 transformation matrix to a transform.
        """
        translation = matrix[:3, 3]
        rotation_matrix = matrix[:3, :3]
        quat = quaternions.mat2quat(rotation_matrix)
        transform = TransformStamped().transform
        transform.translation = Vector3(x=translation[0], y=translation[1], z=translation[2])
        transform.rotation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])
        return transform

    def invert_transform(self, transform):
        """
        Compute the inverse of a transform.
        """
        mat = self.transform_to_matrix(transform)
        inv_mat = np.linalg.inv(mat)
        return self.matrix_to_transform(inv_mat)

def main(args=None):
    """
    Main function to initialize and run the AprilTag localization node.
    """
    rclpy.init(args=args)
    node = AprilTagLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()