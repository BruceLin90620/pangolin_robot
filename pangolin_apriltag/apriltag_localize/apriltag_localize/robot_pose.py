import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
import tf2_geometry_msgs
import numpy as np
from transforms3d.quaternions import quat2mat, mat2quat

class RobotPoseEstimator(Node):

    def __init__(self):
        super().__init__('robot_pose_estimator')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.apriltag_callback,
            10)
        
        self.timer = self.create_timer(0.1, self.broadcast_transform)
        
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        self.robot_pose = None
        self.map_to_base_transform = None

    def apriltag_callback(self, msg):
        if not msg.detections:
            return

        poses = []
        for detection in msg.detections:
            tag_pose = PoseStamped()
            tag_pose.header = msg.header
            tag_pose.pose = detection.pose.pose.pose
            
            if not tag_pose.header.frame_id:
                self.get_logger().warn("Received detection with empty frame_id, skipping...")
                continue

            try:
                # 轉換到相機框架
                camera_pose = tag_pose  # 已經在相機框架中，不需要轉換

                # 從相機框架轉換到基座框架
                base_pose = self.tf_buffer.transform(camera_pose, self.base_frame, rclpy.duration.Duration(seconds=1.0))
                
                # 從基座框架轉換到地圖框架
                map_tag_id = f"tag_{detection.id}"
                if self.tf_buffer.can_transform(self.map_frame, map_tag_id, rclpy.time.Time()):
                    tag_in_map = self.tf_buffer.lookup_transform(self.map_frame, map_tag_id, rclpy.time.Time())
                    map_pose = PoseStamped()
                    map_pose.header.frame_id = self.map_frame
                    map_pose.header.stamp = self.get_clock().now().to_msg()
                    
                    # 計算機器人在地圖中的位置
                    tag_in_map_mat = quat2mat([tag_in_map.transform.rotation.w,
                                               tag_in_map.transform.rotation.x,
                                               tag_in_map.transform.rotation.y,
                                               tag_in_map.transform.rotation.z])
                    base_in_tag_mat = quat2mat([base_pose.pose.orientation.w,
                                                base_pose.pose.orientation.x,
                                                base_pose.pose.orientation.y,
                                                base_pose.pose.orientation.z])
                    map_to_base_mat = np.dot(tag_in_map_mat, np.linalg.inv(base_in_tag_mat))
                    
                    map_to_base_quat = mat2quat(map_to_base_mat)
                    
                    map_pose.pose.position.x = tag_in_map.transform.translation.x - (map_to_base_mat[0][2] * base_pose.pose.position.x +
                                                                                     map_to_base_mat[1][2] * base_pose.pose.position.y +
                                                                                     map_to_base_mat[2][2] * base_pose.pose.position.z)
                    map_pose.pose.position.y = tag_in_map.transform.translation.y - (map_to_base_mat[0][1] * base_pose.pose.position.x +
                                                                                     map_to_base_mat[1][1] * base_pose.pose.position.y +
                                                                                     map_to_base_mat[2][1] * base_pose.pose.position.z)
                    map_pose.pose.position.z = tag_in_map.transform.translation.z - (map_to_base_mat[0][0] * base_pose.pose.position.x +
                                                                                     map_to_base_mat[1][0] * base_pose.pose.position.y +
                                                                                     map_to_base_mat[2][0] * base_pose.pose.position.z)
                    map_pose.pose.orientation.w = map_to_base_quat[0]
                    map_pose.pose.orientation.x = map_to_base_quat[1]
                    map_pose.pose.orientation.y = map_to_base_quat[2]
                    map_pose.pose.orientation.z = map_to_base_quat[3]
                    
                    poses.append(map_pose)
                    self.get_logger().info(f"map_pose {map_pose} ")

                else:
                    self.get_logger().warn(f"Cannot find transform from {self.map_frame} to {map_tag_id}")
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Transform failed: {str(e)}")

        if poses:
            self.robot_pose = self.average_poses(poses)
            self.calculate_map_to_base_transform()

    def average_poses(self, poses):
        avg_pose = PoseStamped()
        avg_pose.header.frame_id = self.map_frame
        avg_pose.header.stamp = self.get_clock().now().to_msg()
        
        position = np.mean([np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]) for p in poses], axis=0)
        avg_pose.pose.position.x = position[0]
        avg_pose.pose.position.y = position[1]
        avg_pose.pose.position.z = position[2]
        
        # 使用第一個姿態的方向（為了簡單起見）
        avg_pose.pose.orientation = poses[0].pose.orientation
        
        return avg_pose

    def calculate_map_to_base_transform(self):
        if self.robot_pose:
            map_to_base = TransformStamped()
            map_to_base.header.stamp = self.get_clock().now().to_msg()
            map_to_base.header.frame_id = self.map_frame
            map_to_base.child_frame_id = self.base_frame
            
            map_to_base.transform.translation.x = self.robot_pose.pose.position.x
            map_to_base.transform.translation.y = self.robot_pose.pose.position.y
            map_to_base.transform.translation.z = self.robot_pose.pose.position.z
            
            map_to_base.transform.rotation = self.robot_pose.pose.orientation
            
            self.map_to_base_transform = map_to_base

    def broadcast_transform(self):
        if self.map_to_base_transform:
            self.tf_broadcaster.sendTransform(self.map_to_base_transform)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()