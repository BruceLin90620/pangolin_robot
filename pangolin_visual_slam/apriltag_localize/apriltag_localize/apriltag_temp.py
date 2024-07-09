#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion
import rclpy.time
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformException
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from tf2_geometry_msgs import do_transform_pose
import numpy as np
from tf_transformations import quaternion_matrix, quaternion_from_matrix, compose_matrix, translation_from_matrix
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.affines import compose, decompose

class AprilTagCameraLocalizationNode(Node):
    def __init__(self):
        super().__init__('apriltag_camera_localization_node')
        
        # 创建转换广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建 TF buffer 和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅 AprilTag 检测结果
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'tag_detections',
            self.tag_callback,
            10)
        
        # 更新 AprilTag 地图，假设 Tag 0 在世界坐标系原点
        self.apriltag_map = {
            0: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            # 添加其他 Tag 如果需要
        }

        # 定义 tag 在 map 中的位置
        self.tag_position = Point(x=1.0, y=1.0, z=0.0)
        self.tag_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # 创建一个定时器来定期获取和广播转换
        self.create_timer(0.1, self.transform_callback)  # 10Hz

    def transform_callback(self):
        from_frame_rel = 'cam_Link'
        to_frame_rel = 'base_link'
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            
            self.get_logger().info(
                f'yes {t}')

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

    def tag_callback(self, msg):

        if msg.detections is None:
            self.get_logger().info(f'not get apriltag ')
            return
        
        poselist_base_wrt_map = []
        for detection in msg.detections:
            tag_id = detection.id

            if tag_id in self.apriltag_map:
                tag_pose = detection.pose.pose.pose
                tag_map_pos = self.apriltag_map[tag_id][:3]
                tag_map_ori = self.apriltag_map[tag_id][3:]
                
                # poselist_tag_wrt_camera = self.pose2poselist(tag_pose)


                poselist_tag_wrt_base = self.transformPose(tag_pose, 'camera_link', 'base_link')
                self.get_logger().info(f"poselist_tag_wrt_base: {poselist_tag_wrt_base}")

                # poselist_base_wrt_tag = self.invPoselist(poselist_tag_wrt_base)
                # self.get_logger().info(f"poselist_base_wrt_tag: {poselist_base_wrt_tag}")

                # poselist_base_wrt_map_single = self.transformPose(poselist_base_wrt_tag, 'base_link', 'map')

                # if poselist_base_wrt_map_single:
                #     poselist_base_wrt_map.append(poselist_base_wrt_map_single)
                #     self.get_logger().debug(f"poselist_base_wrt_map: {poselist_base_wrt_map_single}")

        pass

    def pose2poselist(self, pose):
        return [pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    

    def on_timer(self):

        from_frame_rel = 'cam_Link'
        to_frame_rel = 'base_link'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            
            self.get_logger().info(
                f'yes {t}')

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
    # def transformPose(self, pose, source_frame, target_frame):
    #     try:
    #         transform = self.tf_buffer.lookup_transform(
    #             target_frame,
    #             source_frame,
    #             rclpy.time.Time())
    #     except TransformException as ex:
    #         self.get_logger().info(
    #             f'Could not transform : {ex}')
    #         return
    #     pose_stamped = PoseStamped()
    #     pose_stamped.header.frame_id = source_frame
    #     pose_stamped.pose.position.x = pose[0]
    #     pose_stamped.pose.position.y = pose[1]
    #     pose_stamped.pose.position.z = pose[2]
    #     pose_stamped.pose.orientation.x = pose[3]
    #     pose_stamped.pose.orientation.y = pose[4]
    #     pose_stamped.pose.orientation.z = pose[5]
    #     pose_stamped.pose.orientation.w = pose[6]
    #     pose_transformed = do_transform_pose(pose_stamped.pose, transform)
    #     p = pose_transformed.position
    #     o = pose_transformed.orientation
    #     return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
    
    def transformPose(self, pose, source_frame, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform : {ex}')
            return
        pose_transformed = do_transform_pose(pose, transform)

        return pose_transformed

    def invPoselist(self, poselist):
        mat = compose(poselist[:3], quat2mat(poselist[3:]), np.ones(3))
        pos, rot, _, _ = decompose(np.linalg.inv(mat))
        return list(pos) + list(mat2quat(rot))

    def xyzquat_from_matrix(self, matrix):
        return translation_from_matrix(matrix).tolist() + quaternion_from_matrix(matrix).tolist()

    def matrix_from_xyzquat(self, arg1, arg2=None):
        return self.matrix_from_xyzquat_np_array(arg1, arg2).tolist()

    def matrix_from_xyzquat_np_array(self, arg1, arg2=None):
        if arg2 is not None:
            translate = arg1
            quaternion = arg2
        else:
            translate = arg1[0:3]
            quaternion = arg1[3:7]

        return np.dot(compose_matrix(translate=translate),quaternion_matrix(quaternion))

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagCameraLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()