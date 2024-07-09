import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped, PoseStamped
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
import tf2_geometry_msgs
import numpy as np

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
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_link')
        
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        self.robot_pose = PoseStamped()
        self.new_transform = None

    def apriltag_callback(self, msg):
        if not msg.detections:
            return

        poses = []
        for detection in msg.detections:
            tag_pose = PoseStamped()
            tag_pose.header = msg.header
            tag_pose.pose = detection.pose.pose.pose
            
            # 檢查 frame_id 是否有效
            if not tag_pose.header.frame_id:
                self.get_logger().warn("Received detection with empty frame_id, skipping...")
                continue

            try:
                # 首先轉換到相機框架
                camera_pose = self.tf_buffer.transform(tag_pose, self.camera_frame, rclpy.duration.Duration(seconds=0.1))
                
                self.get_logger().info(f"camera_pose: {camera_pose}")

                # 然後從相機框架轉換到基座框架
                base_pose = self.tf_buffer.transform(camera_pose, self.base_frame, rclpy.duration.Duration(seconds=0.1))

                self.get_logger().info(f"base_pose: {base_pose}")
                
                map_pose = self.tf_buffer.transform(base_pose, self.map_frame, rclpy.duration.Duration(seconds=0.5))

                self.get_logger().info(f"map_pose: {map_pose}")

                poses.append(map_pose)
                
                # if self.tf_buffer.can_transform(self.map_frame, self.base_frame, rclpy.time.Time()):
                #     # 如果存在，從基座框架轉換到地圖框架
                #     map_pose = self.tf_buffer.transform(base_pose, self.map_frame, rclpy.duration.Duration(seconds=1.0))
                #     poses.append(map_pose)
                # else:
                #     self.get_logger().warn(f"Cannot transform from {self.map_frame} to {self.base_frame}")
                #     # 如果不存在 map 到 base_link 的轉換，我們可以選擇發布一個臨時的轉換
                #     self.publish_temporary_map_to_odom_transform()
                
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"Transform failed: {str(e)}")

        if poses:
            self.robot_pose = self.average_poses(poses)
            # self.calculate_odom_to_map_transform()

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

    def calculate_odom_to_map_transform(self):
        try:
            odom_to_base = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
            map_to_base = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            
            odom_to_map = TransformStamped()
            odom_to_map.header.stamp = self.get_clock().now().to_msg()
            odom_to_map.header.frame_id = self.map_frame
            odom_to_map.child_frame_id = self.odom_frame
            
            # 計算轉換
            odom_to_map.transform.translation.x = map_to_base.transform.translation.x - odom_to_base.transform.translation.x
            odom_to_map.transform.translation.y = map_to_base.transform.translation.y - odom_to_base.transform.translation.y
            odom_to_map.transform.translation.z = map_to_base.transform.translation.z - odom_to_base.transform.translation.z
            
            # 對於旋轉，我們簡化為使用從地圖到基座的旋轉
            odom_to_map.transform.rotation = map_to_base.transform.rotation
            
            self.new_transform = odom_to_map
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Failed to calculate odom to map transform: {str(e)}")

    def broadcast_transform(self):
        if self.new_transform:
            self.tf_broadcaster.sendTransform(self.new_transform)

    def publish_temporary_map_to_odom_transform(self):
        # 發布一個臨時的 map 到 odom 的轉換
        # 這只是一個示例，實際應用中您可能需要一個更複雜的邏輯來決定這個轉換
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        t_odom_2_base_footprint = TransformStamped()
        t_odom_2_base_footprint.header.stamp = self.get_clock().now().to_msg()
        t_odom_2_base_footprint.header.frame_id = self.odom_frame
        t_odom_2_base_footprint.child_frame_id = 'base_footprint'
        t_odom_2_base_footprint.transform.translation.x = 0.0
        t_odom_2_base_footprint.transform.translation.y = 0.0
        t_odom_2_base_footprint.transform.translation.z = 0.0
        t_odom_2_base_footprint.transform.rotation.x = 0.0
        t_odom_2_base_footprint.transform.rotation.y = 0.0
        t_odom_2_base_footprint.transform.rotation.z = 0.0
        t_odom_2_base_footprint.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_odom_2_base_footprint)
        
def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()