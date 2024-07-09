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
