import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
import rclpy.time
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_geometry_msgs
# from tf_transformations import quaternion_matrix
from tf_transformations import quaternion_matrix, quaternion_from_matrix
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class AprilTagLocalization(Node):

    def __init__(self):
        super().__init__('april_tag_localization')
        # Subscribers
        self.vslam_sub = self.create_subscription(PoseWithCovarianceStamped, '/visual_slam/tracking/vo_pose_covariance', self.vslam_callback, 10)
        # self.tf_sub = self.create_subscription(TFMessage, '/tf_static', self.tf_callback, rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))


        self.apriltag_sub = self.create_subscription(AprilTagDetectionArray, '/tag_detections', self.apriltag_callback, 10)

        # Publisher
        self.corrected_pose_pub = self.create_publisher(PoseStamped, '/corrected_pose', 10)
        
        # TF Broadcaster and Listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Store latest SLAM poseQ
        self.slam_pose = None
        
        # 更新 AprilTag 地图，假设 Tag 0 在世界坐标系原点
        self.apriltag_map = {
            0: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            # 添加其他 Tag 如果需要
        }

        self.static_broadcaster = StaticTransformBroadcaster(self)

        # try:
        #     t = self.tf_buffer.lookup_transform(
        #         'map',
        #         'base_link',
        #         rclpy.time.Time(0),
        #         rclpy.time.Duration(seconds=1))
        # except:
        #     self.get_logger().info('Could not transform cam_link to base_link')

        self.timer = self.create_timer(1.0, self.on_timer)

                
    def vslam_callback(self, msg):
        pass

    def apriltag_callback(self, msg):
        pass

    def get_tag_abs_matrix(self, tag_abs_pos, tag_abs_ori):
        pass
    
    def correct_pose(self, slam_pose, tag_pose, tag_abs_pos, tag_abs_ori):
        pass
    

    def pose_to_matrix(self, pose):
        mat = np.eye(4)
        mat[:3, :3] = quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[:3, :3]
        mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return mat

    def matrix_to_pose(self, matrix):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = matrix[:3, 3]
        quat = quaternion_from_matrix(matrix)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quat
        return pose


    def publish_pose(self, pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose = pose.pose if isinstance(pose, PoseStamped) else pose
        msg.pose = pose.pose 
        self.corrected_pose_pub.publish(msg)
        self.broadcast_tf(msg)

    def broadcast_tf(self, pose_stamped):
        t = TransformStamped()
        t.header = pose_stamped.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pose_stamped.pose.position.x
        t.transform.translation.y = pose_stamped.pose.position.y
        t.transform.translation.z = pose_stamped.pose.position.z
        t.transform.rotation = pose_stamped.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()













import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from transforms3d import quaternions, euler

class AprilTagLocalization(Node):
    def __init__(self):
        super().__init__('apriltag_localization')
        
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.broadcast_static_transform()

    def broadcast_static_transform(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "tag0"
        
        static_transformStamped.transform.translation.x = -0.5
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0

        q = euler.euler2quat(-1.57, 0, -1.57, 'sxyz')
        static_transformStamped.transform.rotation.w = q[0]
        static_transformStamped.transform.rotation.x = q[1]
        static_transformStamped.transform.rotation.y = q[2]
        static_transformStamped.transform.rotation.z = q[3]
        
        self.static_broadcaster.sendTransform(static_transformStamped)
        self.get_logger().info("Broadcast static transform from map to tag0")

    def tag_callback(self, msg):
        if not msg.detections:
            self.get_logger().warn("No tag detections received")
            return
        
        detection = msg.detections[0]

        try:
            time_now = rclpy.time.Time()
            
            # Get transform from camera to detected tag
            trans_cam_to_tag = self.tf_buffer.lookup_transform(
                "camera_link",
                "tag36h11:0",
                time_now)
            self.get_logger().info(f"Camera to tag36h11:0: {self.transform_to_string(trans_cam_to_tag.transform)}")

            # Get transform from base_link to camera
            trans_base_to_cam = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_link",
                time_now)
            self.get_logger().info(f"Base to camera: {self.transform_to_string(trans_base_to_cam.transform)}")

            # Calculate transform from base_link to detected tag
            trans_base_to_tag = self.multiply_transforms(trans_base_to_cam.transform, trans_cam_to_tag.transform)
            self.get_logger().info(f"Base to tag: {self.transform_to_string(trans_base_to_tag)}")


            # Get transform from base_link to camera
            robot_base_to_cam = self.tf_buffer.lookup_transform(
                "base_link",
                "tag36h11:0",
                time_now)
            self.get_logger().info(f"Base to tag robot: {self.transform_to_string(robot_base_to_cam.transform)}")


            # Get transform from map to tag0
            trans_map_to_tag0 = self.tf_buffer.lookup_transform(
                "map",
                "tag0",
                time_now)
            self.get_logger().info(f"Map to tag0: {self.transform_to_string(trans_map_to_tag0.transform)}")
            
            # Calculate robot pose in map frame
            robot_pose_map = self.multiply_transforms(trans_map_to_tag0.transform, self.invert_transform(trans_base_to_tag))
            self.get_logger().info(f"Calculated robot pose in map frame: {self.transform_to_string(robot_pose_map)}")

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform = robot_pose_map
            
            self.tf_broadcaster.sendTransform(t)
            self.get_logger().info("Broadcast transform from map to base_link")
            
        except tf2_ros.TransformException as e:
            self.get_logger().error(f'Failed to compute transform: {str(e)}')

    def multiply_transforms(self, t1, t2):
        m1 = self.transform_to_matrix(t1)
        m2 = self.transform_to_matrix(t2)
        result_matrix = np.dot(m1, m2)
        return self.matrix_to_transform(result_matrix)

    def transform_to_matrix(self, transform):
        translation = [transform.translation.x, transform.translation.y, transform.translation.z]
        rotation = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        
        mat = np.eye(4)
        mat[:3, :3] = quaternions.quat2mat(rotation)
        mat[:3, 3] = translation
        return mat

    def matrix_to_transform(self, matrix):
        translation = matrix[:3, 3]
        rotation_matrix = matrix[:3, :3]
        quat = quaternions.mat2quat(rotation_matrix)
        
        transform = TransformStamped().transform
        transform.translation = Vector3(x=translation[0], y=translation[1], z=translation[2])
        transform.rotation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])
        return transform

    def invert_transform(self, transform):
        mat = self.transform_to_matrix(transform)
        inv_mat = np.linalg.inv(mat)
        return self.matrix_to_transform(inv_mat)

    def transform_to_string(self, transform):
        return f"Translation: ({transform.translation.x:.4f}, {transform.translation.y:.4f}, {transform.translation.z:.4f}), " \
               f"Rotation: ({transform.rotation.x:.4f}, {transform.rotation.y:.4f}, {transform.rotation.z:.4f}, {transform.rotation.w:.4f})"

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()